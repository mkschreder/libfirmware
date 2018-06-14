/*
 * This file is part of Ninjaflight.
 *
 * Ninjaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ninjaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ninjaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(STM32F40_41xxx)
#include <stm32f4xx_flash.h>
#else
#include <stm32f10x_flash.h>
#endif

#include "config_flash.h"

#include <string.h>
#include <errno.h>

extern uint8_t __config_start;
extern uint8_t __config_end;

#if !defined(FLASH_PAGE_SIZE)
# if defined(STM32F303xC)
#  define FLASH_PAGE_SIZE				(0x800)
# elif defined(STM32F10X_MD)
#  define FLASH_PAGE_SIZE				(0x400)
# elif defined(STM32F10X_HD)
#  define FLASH_PAGE_SIZE				(0x800)
# elif defined(STM32F40_41xxx)
#  define FLASH_PAGE_SIZE				(0x1000) // 16k
# elif defined(UNIT_TEST)
#  define FLASH_PAGE_SIZE				(0x400)
# else
#  error "Flash page size not defined for target."
# endif
#endif

#define CONFIG_FLASH_SIZE (uint32_t)((uint8_t*)&__config_end - (uint8_t*)&__config_start)

void _clear_flags(void){
#if defined(STM32F303)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F40_41xxx)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#elif defined(UNIT_TEST)
    // NOP
#else
# error "Unsupported CPU"
#endif
}

int flash_write(uint32_t base, const void *data, size_t size){
	if(base >= CONFIG_FLASH_SIZE) return -EINVAL;
	if((size & 0x03) != 0) return -EFAULT;

	FLASH_Unlock();

	_clear_flags();

	for(int c = 0; c < (int)(size >> 2); c++){
		uint32_t value = ((uint32_t*)data)[c];
		if(FLASH_ProgramWord(((uintptr_t)&__config_start) + base + (uint32_t)(c << 2), value) != FLASH_COMPLETE){
			FLASH_Lock();
			return c << 2;
		}
	}
	FLASH_Lock();
	return (int)size;
}

#if defined(STM32F40_41xxx)
static int32_t _flash_sector_from_addr(uint32_t addr){
	// NOTE: this is memory map of my particular device. Will be different on different chips. 
	// TODO: make this more generic? 
	struct sector {
		uint16_t id;
		uint32_t start, end;
	} sectors[] = {
		{ FLASH_Sector_0, 0x08000000, 0x080003FFF },
		{ FLASH_Sector_1, 0x08004000, 0x080007FFF },
		{ FLASH_Sector_2, 0x08008000, 0x08000BFFF },
		{ FLASH_Sector_3, 0x0800C000, 0x08000FFFF },
		{ FLASH_Sector_4, 0x08010000, 0x08001FFFF },
		{ FLASH_Sector_5, 0x08020000, 0x08003FFFF },
		{ FLASH_Sector_6, 0x08040000, 0x08005FFFF },
		{ FLASH_Sector_7, 0x08060000, 0x08007FFFF },
		{ FLASH_Sector_8, 0x08080000, 0x08009FFFF },
		{ FLASH_Sector_9, 0x080A0000, 0x0800BFFFF },
		{ FLASH_Sector_10, 0x080C0000, 0x0800DFFFF },
		{ FLASH_Sector_11, 0x080E0000, 0x0800FFFFF }
	};
	for(size_t c = 0; c < (sizeof(sectors) / sizeof(sectors[0])); c++){
		const struct sector *s = &sectors[c];
		if(addr >= s->start && addr <= s->end) return s->id;
	}
	return -1;
}
#endif 

static int _flash_erase_page_addr(uint32_t addr){
#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
	return FLASH_ErasePage(addr);
#elif defined(STM32F40_41xxx)
	// for stm32f4 the procedure is a bit different.
	int32_t id = _flash_sector_from_addr(addr);
	if(id < 0) return -1;

	return FLASH_EraseSector((uint16_t)id, VoltageRange_3); // TODO: we assume 3.3v powering scheme
#else
	#error "Don't know how to erase pages on this MCU!"
#endif
}

int flash_erase_page(uint32_t base){
	if((base % FLASH_PAGE_SIZE != 0) || base >= CONFIG_FLASH_SIZE) return -EINVAL;

	FLASH_Unlock();

	_clear_flags();

	if(_flash_erase_page_addr(((uintptr_t)&__config_start) + base) != FLASH_COMPLETE){
		FLASH_Lock();
		return -EIO;
	}
	FLASH_Lock();
	return 0;
}

int flash_read(uint32_t base, void *data, size_t size){
	uint8_t *start = (uint8_t*)&__config_start;
	if(base >= (uintptr_t)CONFIG_FLASH_SIZE) return -EINVAL;
	memcpy(data, start + base, size);
	return (int)size;
}

size_t flash_get_page_size(void){
	return FLASH_PAGE_SIZE;
}

size_t flash_get_num_pages(void){
	return CONFIG_FLASH_SIZE / FLASH_PAGE_SIZE;
}

static int _eeprom_read(eeprom_t eeprom, uint32_t addr, void *dst, size_t size){
    (void)eeprom;
    return flash_read(addr, dst, size);
}

static int _eeprom_write(eeprom_t eeprom, uint32_t addr, const void *data, size_t size){
    (void)eeprom;
    return flash_write(addr, data, size);
}

static int _eeprom_erase_page(eeprom_t eeprom, uint32_t addr){
    (void)eeprom;
    return flash_erase_page(addr);
}

static int _eeprom_get_info(eeprom_t eeprom, struct eeprom_info *info){
    (void)eeprom;
    info->page_size = (uint16_t)flash_get_page_size();
    info->num_pages = (uint16_t)flash_get_num_pages();
	return 0;
}

static const struct eeprom_ops _eeprom_ops = {
	.read = _eeprom_read,
	.write = _eeprom_write,
	.erase_page = _eeprom_erase_page,
	.get_info = _eeprom_get_info
};

// TODO: this would typically be inside the object struct
static const struct eeprom_ops *_ops_ptr = &_eeprom_ops;

eeprom_t flash_get_eeprom_interface(void){
	return &_ops_ptr;
}


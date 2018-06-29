#include <string.h>
#include <errno.h>
#include <firmware/list.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_exti.h>
#include <stm32f30x_syscfg.h>
#include "stm32_gpio.h"
#include <thread/kernel.h>

static LIST_HEAD(ext0_list);
static LIST_HEAD(ext1_list);
static LIST_HEAD(ext2_list);
static LIST_HEAD(ext3_list);
static LIST_HEAD(ext4_list);
static LIST_HEAD(ext9_5_list);
static LIST_HEAD(ext15_10_list);

void stm32_gpio_pin_set_input_default(struct stm32_gpio_pin *self){
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = self->pin;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(self->port, &gpio);
}

void stm32_gpio_pin_set_output_default(struct stm32_gpio_pin *self){
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = self->pin;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(self->port, &gpio);
}

static int _stm32_gpio_set(gpio_pin_t pin, bool value){
	struct stm32_gpio_pin *self = container_of(pin, struct stm32_gpio_pin, ops);
	if(value){
		GPIO_SetBits(self->port, self->pin);
	} else {
		GPIO_ResetBits(self->port, self->pin);
	}
}

static bool _stm32_gpio_get(gpio_pin_t pin){
	struct stm32_gpio_pin *self = container_of(pin, struct stm32_gpio_pin, ops);
	return !!(GPIO_ReadInputData(self->port) & self->pin);
}

int _stm32_gpio_register_irq(gpio_pin_t pin, irq_trigger_mode_t mode, struct irq *irq, irq_result_t (*handler)(struct irq *self, int *woken)){
	struct stm32_gpio_pin *self = container_of(pin, struct stm32_gpio_pin, ops);
	uint8_t port_source = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	switch((uint32_t)self->port){
		case (uint32_t)GPIOA: port_source = EXTI_PortSourceGPIOA; break;
		case (uint32_t)GPIOB: port_source = EXTI_PortSourceGPIOB; break;
		case (uint32_t)GPIOC: port_source = EXTI_PortSourceGPIOC; break;
		case (uint32_t)GPIOD: port_source = EXTI_PortSourceGPIOD; break;
		case (uint32_t)GPIOE: port_source = EXTI_PortSourceGPIOE; break;
		case (uint32_t)GPIOF: port_source = EXTI_PortSourceGPIOF; break;
#ifdef GPIOG
		case (uint32_t)GPIOG: port_source = EXTI_PortSourceGPIOG; break;
#endif
		default: return -EINVAL;
	};

	uint8_t pin_idx = 0xff;
	for(uint8_t c = 0; c < 16; c++){
		if(self->pin & (1 << c)){
			SYSCFG_EXTILineConfig(port_source, c);
			pin_idx = c;
			break;
		}
	}

	if(pin_idx == 0xff) return -EINVAL; // pin not specified

	EXTI_InitTypeDef exti;
	EXTI_StructInit(&exti);
	//exti.EXTI_Line = EXTI_Line6;
	exti.EXTI_Line = self->pin;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	//exti.EXTI_Trigger = EXTI_Trigger_Rising;
	switch(mode){
		case IRQ_TRIGGER_RISING:
			exti.EXTI_Trigger = EXTI_Trigger_Rising;
			break;
		case IRQ_TRIGGER_FALLING:
			exti.EXTI_Trigger = EXTI_Trigger_Falling;
			break;
		case IRQ_TRIGGER_RISING_FALLING:
			exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
			break;
		default: return -EINVAL;
	}

	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	irq->handler = handler;

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;

	// add irq to the appropriate list
	switch(pin_idx){
		case 0: {
			nvic.NVIC_IRQChannel = EXTI0_IRQn;
			list_add(&irq->list, &ext0_list);
		} break;
		case 1: {
			nvic.NVIC_IRQChannel = EXTI1_IRQn;
			list_add(&irq->list, &ext1_list);
		} break;
#ifdef EXTI2_IRQn
		case 2: {
			nvic.NVIC_IRQChannel = EXTI2_IRQn;
			list_add(&irq->list, &ext2_list);
		} break;
#endif
		case 3: {
			nvic.NVIC_IRQChannel = EXTI3_IRQn;
			list_add(&irq->list, &ext3_list);
		} break;
		case 4: {
			nvic.NVIC_IRQChannel = EXTI4_IRQn;
			list_add(&irq->list, &ext4_list);
		} break;
		case 5 ... 9: {
			nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
			list_add(&irq->list, &ext9_5_list);
		} break;
		case 10 ... 15: {
			nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
			list_add(&irq->list, &ext15_10_list);
		} break;
	}

	NVIC_Init(&nvic);

	return 0;
}

static const struct gpio_pin_ops gpio_pin_ops = {
	.set = &_stm32_gpio_set,
	.get = &_stm32_gpio_get,
	.register_irq = &_stm32_gpio_register_irq
};

gpio_pin_t stm32_gpio_pin_get_interface(struct stm32_gpio_pin *self){
	self->ops = &gpio_pin_ops;
	return &self->ops;
}

gpio_pin_t stm32_gpio_pin_init(struct stm32_gpio_pin *self, GPIO_TypeDef *port, uint16_t pin){
	memset(self, 0, sizeof(*self));
	self->port = port;
	self->pin = pin;
	return stm32_gpio_pin_get_interface(self);
}


static inline __attribute__((always_inline)) void _handle_irq(uint32_t lines, struct list_head *list){
	struct irq *self;
	int wake = 0;
	list_for_each_entry(self, list, list){
		if(self->handler) {
			if(self->handler(self, &wake) == IRQ_HANDLED){
				// TODO: this is not currently entirely correct because we also want to support shared irqs. Need to store line number in irq struct!
				EXTI_ClearITPendingBit(lines);
				break;
			}
		}
	}
	portEND_SWITCHING_ISR(wake);
}

void EXTI0_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line0) == RESET) return;
	_handle_irq(EXTI_Line0, &ext0_list);
}

void EXTI1_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line1) == RESET) return;
	_handle_irq(EXTI_Line1, &ext1_list);
}

void EXTI2_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line2) == RESET) return;
	_handle_irq(EXTI_Line2, &ext2_list);
}

void EXTI3_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line3) == RESET) return;
	_handle_irq(EXTI_Line3, &ext3_list);
}

void EXTI4_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line4) == RESET) return;
	_handle_irq(EXTI_Line4, &ext4_list);
}

void EXTI9_5_IRQHandler(void){
	_handle_irq(EXTI_Line5 | EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9, &ext9_5_list);
}

void EXTI15_10_IRQHandler(void){
	_handle_irq(EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15, &ext15_10_list);
}



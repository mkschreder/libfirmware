#include <libfirmware/driver.h>
#include <libfirmware/serial.h>
#include <libfdt/libfdt.h>

#include <stm32f10x_tim.h>

struct stm32_enc_out {
    struct serial_device dev;
    TIM_TypeDef *hw;
    DMA_Channel_TypeDef *dma;
    uint32_t pattern_pos[4];
    uint32_t pattern_neg[4];
};

#define PIN_SET(pin) (uint32_t)(1 << (pin))
#define PIN_RESET(pin) (uint32_t)(1 << (pin + 16))

static int _serial_write(serial_port_t serial, const void *data, size_t size, uint32_t timeout){
	struct stm32_enc_out *self = container_of(serial, struct stm32_enc_out, dev.ops);
    if(size != 4) return -1;

    int32_t counts = 0;
    memcpy(&counts, data, sizeof(counts));

    if(counts == 0) {
		self->hw->ARR = 0;
    } else if(counts < 0){
		self->hw->ARR = (uint16_t)(1000000 / -counts / 2);
        DMA_Cmd(self->dma, DISABLE);
        self->dma->CMAR = (uint32_t)self->pattern_neg;
        DMA_Cmd(self->dma, ENABLE);
    } else {
		self->hw->ARR = (uint16_t)(1000000 / counts / 2);
        DMA_Cmd(self->dma, DISABLE);
        self->dma->CMAR = (uint32_t)self->pattern_pos;
        DMA_Cmd(self->dma, ENABLE);
    }

    return 4;
}

static int _serial_read(serial_port_t serial, void *data, size_t size, uint32_t timeout){
    return -1;
}

static const struct serial_ops _serial_ops = {
	.read = _serial_read,
	.write = _serial_write
};

static int _stm32_encoder_out_probe(void *fdt, int fdt_node){
	TIM_TypeDef *hw = (TIM_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "timer", (uint32_t)TIM2);
	GPIO_TypeDef *gpio = (GPIO_TypeDef*)fdt_get_int_or_default(fdt, (int)fdt_node, "gpio", (uint32_t)GPIOA);
    int pin_a = fdt_get_int_or_default(fdt, (int)fdt_node, "pin_a", 0);
    int pin_b = fdt_get_int_or_default(fdt, (int)fdt_node, "pin_b", 1);

    DMA_Channel_TypeDef *channel = NULL;

    if(hw == TIM1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        channel = DMA1_Channel5;
        printk("enc_out: TIM1, DMA1C5\n");
    } else if(hw == TIM2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        channel = DMA1_Channel2;
        printk("enc_out: TIM2, DMA1C2\n");
    } else if(hw == TIM3) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        channel = DMA1_Channel3;
        printk("enc_out: TIM3, DMA1C3\n");
    } else if(hw == TIM4) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        channel = DMA1_Channel7;
        printk("enc_out: TIM4, DMA1C7\n");
    } else if(hw == TIM5) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
        channel = DMA2_Channel2;
        printk("enc_out: TIM5, DMA2C2\n");
    } else {
        printk("enc_out: invtim\n");
        return -1;
    }

    struct stm32_enc_out *self = kzmalloc(sizeof(struct stm32_enc_out));
    self->hw = hw;
    self->dma = channel;

    // fill out the patterns for set/reset register
    self->pattern_pos[0] = PIN_RESET(pin_a) | PIN_RESET(pin_b);
    self->pattern_pos[1] = PIN_RESET(pin_b) | PIN_SET(pin_a);
    self->pattern_pos[2] = PIN_SET(pin_a) | PIN_SET(pin_b);
    self->pattern_pos[3] = PIN_RESET(pin_a) | PIN_SET(pin_b);

    self->pattern_neg[0] = PIN_RESET(pin_a) | PIN_SET(pin_b);
    self->pattern_neg[1] = PIN_SET(pin_a) | PIN_SET(pin_b);
    self->pattern_neg[2] = PIN_RESET(pin_b) | PIN_SET(pin_a);
    self->pattern_neg[3] = PIN_RESET(pin_a) | PIN_RESET(pin_b);

	TIM_TimeBaseInitTypeDef tim;
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_Prescaler = (uint16_t)(SystemCoreClock / 1000000);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 0;
	tim.TIM_ClockDivision = 0;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(hw, &tim);

	//TIM_ITConfig(hw, TIM_IT_Update, ENABLE);
    TIM_DMACmd(hw, TIM_DMA_Update, ENABLE);

    DMA_DeInit(channel);
    DMA_InitTypeDef dma;
    DMA_StructInit(&dma);
    dma.DMA_MemoryBaseAddr = (uint32_t)self->pattern_pos;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&gpio->BSRR;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = 4;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(channel, &dma);
    DMA_Cmd(channel, ENABLE);

	TIM_Cmd(hw, ENABLE);

    serial_device_init(&self->dev, fdt_node, &_serial_ops);
    serial_device_register(&self->dev);

    return 0;
}

static int _stm32_encoder_out_remove(void *fdt, int fdt_node){
    return 0;
}

DEVICE_DRIVER(stm32_enc_out, "st,stm32_enc_out", _stm32_encoder_out_probe, _stm32_encoder_out_remove)


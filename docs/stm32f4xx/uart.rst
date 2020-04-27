***********
STM32 USART
***********

This is a flexible uart driver for stm32 f4 CPUs.

Parameters
----------

.. csv-table:: Device Tree Fields
    :header: "Field", "Description"

    "compatible", "st,stm32_uart"
    "baud", "Baud rate of this device in bits per second"
    "reg", "USART1 or USART2 .. to UART8"
    "interrupt", "IRQ ID as devined in the peripheral library headers"
    "printk_port", "Set to <1> if you want prink messages to go to this port"
    "insert-cr-before-lf", "If set to <1> then a carriage return is inserted when printing"

Example configuration
---------------------

Examples below show two configurations. The first one is for a normal console port (note that gpios are set later). The second one is for an external uart port and GPIO pins are set after initializing the device.

.. code-block:: c

	serial {
		debug_uart: debug {
			compatible = "st,stm32_uart";
			baud = <921600>;
			reg = <USART1>;
			interrupt = <USART1_IRQn>;
			printk_port = <1>;
			insert-cr-before-lf = <1>;
		};
		uext_uart: uext_uart {
			compatible = "st,stm32_uart";
			baud = <921600>;
			reg = <UART8>;
			interrupt = <UART8_IRQn>;

			uext_uart_serial_pins {
				compatible = "st,stm32_gpio";
				pinctrl = <
					GPIOE GPIO_Pin_0 (GPIO_AF_UART8 | GPIO_OType_PP | GPIO_PuPd_UP | GPIO_Speed_50MHz) /* RX */
					GPIOE GPIO_Pin_1 (GPIO_AF_UART8 | GPIO_OType_PP | GPIO_PuPd_NOPULL | GPIO_Speed_50MHz) /* TX */
				>;
			};
		};
	};



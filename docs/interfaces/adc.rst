********************
ADC Device Interface
********************

This interface should be exported by any hardware ADC devices. Typically ADC will have 10, 12 or 16 bit width. Maximum width supported by this interface is 16 bits. 

.. code-block:: c

    struct adc_device_ops {
        int (*trigger)(adc_device_t dev);
        int (*read)(adc_device_t dev, unsigned int channel, uint16_t *value);
    };

    #define adc_read(dev, channel, value) (*(dev))->read(dev, channel, value)
    #define adc_trigger(dev) (*(dev))->trigger(dev)



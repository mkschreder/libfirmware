***********************
Analog Device Interface
***********************

Useful for analog devices that output a floating point value. This is different
from raw ADC value in that it can be a post processed value instead of a 12 bit
integer (as in the case of a 12 bit ADC). It is slower on systems that don't
support hardware floating point operations such as M0, M2 and M3 STM32 chips.

.. code-block:: c

    struct analog_device_ops {
        int (*write)(analog_device_t dev, unsigned int channel, float value);
        int (*read)(analog_device_t dev, unsigned int channel, float *value);
    };

    #define analog_read(dev, channel, value) ((dev)?(*(dev))->read(dev, channel, value):-EINVAL)
    #define analog_write(dev, channel, value) ((dev)?(*(dev))->write(dev, channel, value):-EINVAL)

Reading Analog Device
---------------------

.. code-block:: c

    analog_device_t dev = analog_device_find(fdt, "/path/to/device");
    if(dev){
        if(analog_device_read(dev, 0, &value) < 0){
            // fail
        } else {
            // success
        }
    }


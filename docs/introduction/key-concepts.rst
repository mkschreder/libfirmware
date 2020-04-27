**********************
LibFirmware Motivation
**********************

I originally created libfirmware as a way to share code between my firmware
projects and my linux applications. In many cases I wanted to compile the same
code for both linux and for example stm32. However this was hard to achieve
when code was dependent on many parts that only made sense to build for the
real target such as the standard peripheral library. The fact that I wanted to
write multithreaded code also complicated matters even more because now the
microcontroller targets used FreeRTOS and linux code had to be built against
pthreads (yes FreeRTOS does have a linux port but it is hediously slow and is
horrible to debug).

So I moved out all the hardware specific code into what has become libfirmware.
This library is the cross platform hardware abstraction layer which also
implements device tree support (using my custom version of libfdt) and provides
standard ways for firmware components to communicate with eachother through
abstract interfaces implemented in C. This achieves high degree of decoupling
and also makes the code easy to build for a variety of platforms.

Device Driver Model
===================

LibFirmware implements a standard way to write device drivers which is inspired
by linux where every device driver is a fairly standalone application which
usually does not expose any non-static methods. Instead it registers itself
into the global list of device drivers using a macro that automatically
registers the driver on startup.

.. code-block:: c 

    static int _mydriver_probe(void *fdt, int fdt_node) {

    }
    static int _mydriver_remove(void *fdt, int fdt_node) {

    }

    DEVICE_DRIVER(mydriver, "me,mydriver", _mydriver_probe, _mydriver_remove)

A firmware will typically use a very simple main() method which simply contains
a call to **probe_device_drivers** which in turn will walk through the device
tree and search for drivers that are capable of instantiating each device tree
section.

So let's say that your device tree contains the following code:

.. code-block:: c

    mydevinstance {
        compatible = "me,mydriver";
        myparam = <1>;
    };

When this block is parsed, the binary version of this blob will be referenced
by fdt_node parameter of the probe method. The probe method can then retreive
parameters of the confiration using libfdt functions:

.. code-block:: c

    int myparam = fdt_get_int_or_default(fdt, fdt_node, "myparam", -1);

The default is provided for the case when the parameter doesn't exist. This is
an extension to the original libfdt that I have added to make code simpler.

Referencing Devices
===================

Even your application is a node in the device tree and a device driver. So when
you want to pass a parameter to your application that will tell it which uart
to use for communication then you simply define that parameter in the device
tree block of your application:

.. code-block:: c

    myuart_label: myuart {
        ...
    };

    application {
        compatible = "my,application";
        uart = <&myuart_label>;
    };

In your application you can then search for this uart using the serial
interface functions (provided that your uart driver registers serial interfaces
of course!):

.. code-block:: c

    serial_device_t uart = serial_device_find_by_ref(fdt, fdt_node, "uart");

This way your application does not need to know whether you are using uart1 or
uart8 or some other device for serial communication. You simply provide a
reference to a generic serial device and then your application uses
**serial_write** and **serial_read** interface methods to interact with the
serial device. This is how powerful device tree is!

LibFirmware was the first firmware library after linux to use the device tree.
Making libfirmware truly unique in this regard. 


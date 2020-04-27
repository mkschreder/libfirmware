Building LibFirmware
====================

Normally cross compiling applications for different versions of linux is fairly simple. However when it comes to supporting baremetal microcontroller builds and linux builds as well things get a little more complex.

To build libfirmware for linux you simply issue the standard build sequence:

.. code-block:: bash

    mkdir -p build && cd build
    ../configure
    make
    sudo make install

However when you are building for STM32 it is better to use Rocket_ firmware
builder which already provides the correct commands for building all the
libraries. Since libfirmware also depends on libfdt_ (which also needs to be
built for the target) you need to build libfdt for the stm32 target first.

Typically you would cross compile that library and install it into a local
directory and then provide the path to that directory along with CFLAGS that
you pass to configure script.

Building for stm32f429zet6 is for example done like this:

.. code-block:: bash

    mkdir -p build && cd build
    ../configure CFLAGS="-mcpu=cortex-m4 -mthumb -specs=nosys.specs" --host=arm-none-eabi --with-target=stm32f429zet6
    make
    sudo make install



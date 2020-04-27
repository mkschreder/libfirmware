.. LibFirmware documentation master file, created by
   sphinx-quickstart on Sun Apr 26 22:08:15 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

LibFirmware MCU Device Driver Framework
=======================================

LibFirmware brings device tree support to microcontroller projects. With small
footprint, a handful of abstract interfaces and a standard way to initialize
devices you get a powerful framework for writing completely decoupled firmware
applications.

This library contains the bare metal support for various soc chips but not
support for external devices. That is implemented in libdriver_ instead.

.. _libdriver: http://swedishembedded.com/libdriver

.. toctree::
   :maxdepth: 2
   :caption: Abstract Interfaces

   introduction/key-concepts
   introduction/building

.. toctree::
   :maxdepth: 2
   :caption: Abstract Interfaces

   interfaces/adc
   interfaces/analog

.. toctree::
   :maxdepth: 2
   :caption: STM32F4xx Drivers

   stm32f4xx/uart.rst

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

.. zephyr:board:: arduino_nano_esp32

Overview
********

Introducing the Nano ESP32, a powerful addition to the Arduino ecosystem that brings the popular
ESP32-S3 to the world of Arduino and MicroPython programming. Whether you're a beginner stepping
into the world of IoT or MicroPython, or an advanced user looking to incorporate it into your next
product, the Nano ESP32 is the perfect choice. It covers all your needs to kick-start your IoT or
MicroPython project with ease.

Let's explore the key features of the Nano ESP32:

- Tiny footprint: Designed with the well-known Nano form factor in mind, this board's compact size
  makes it perfect for embedding in standalone projects.
- Wi-Fi® and Bluetooth®: Harness the power of the ESP32-S3 microcontroller, well-known in the IoT
  realm, with full Arduino support for wireless and Bluetooth® connectivity.
- Arduino and MicroPython support: Seamlessly switch between Arduino and MicroPython programming
  with a few simple steps. We even offer an introductory course for those new to the MicroPython
  world, find more information in the documentation page.
- Arduino IoT Cloud compatible: Quickly and easily create IoT projects with just a few lines of
  code. Our setup takes care of security, allowing you to monitor and control your project from
  anywhere using the Arduino IoT Cloud app. (Available starting August 2023)
- HID support: Emulate human interface devices, such as keyboards or mice, over USB, opening up new
  possibilities for interacting with your computer.

There are no more excuses to delay your exploration of IoT and MicroPython. The Nano ESP32 provides
everything you need to start creating and discovering the endless possibilities.

Hardware
********

- u-blox® NORA-W106 (ESP32-S3)
- Processor up to 240 MHz
- SRAM: 512 kB
- External Flash 128 Mbit (16 MB)
- RAM 8MB (NORA-W106-10B)
- User RGB LED
- User button
- Built-in LED Pin 13
- Built-in RGB LED pins 14-16
- Digital I/O Pins 14
- Analog input pins 8
- PWM pins 5
- External interrupts All digital pins

Supported Features
==================

.. zephyr:board-supported-hw::

Connections and IOs
===================

The `Arduino Store`_ and `Arduino Doc`_ have detailed information about board
connections. Download the `schematic`_ for more detail.

Flashing
========

First, connect the Arduino Nano esp32 board to your host computer using
the USB port to prepare it for flashing. Then build and flash your application.

Here is an example for the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: arduino_nano_esp32
   :goals: build flash

Run a serial host program to connect with your board:

.. code-block:: console

   $ minicom -D /dev/ttyACM0

You should see the following message on the console:

.. code-block:: console

   Hello World! arduino_nano_esp32

Debugging
=========

You can debug an application in the usual way.  Here is an example for the
:zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: arduino_nano_esp32
   :goals: debug

References
**********

.. target-notes::

.. _Arduino Store:
    https://store.arduino.cc/products/nano-esp32

.. _Arduino Doc:
    https://docs.arduino.cc/hardware/nano-esp32/

.. _schematic:
    https://store.arduino.cc/products/nano-esp32

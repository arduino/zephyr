.. zephyr:code-sample:: flash_read_write
   :name: Flash Read/Write
   :relevant-api: flash_interface

   Use the flash API to erase, write and read back data from flash memory.

Overview
********

This sample demonstrates using the :ref:`flash API <flash_api>` to perform
basic erase, write and read operations on the internal flash of a device.

A single generic test function (``flash_region_test``) receives the flash
device, byte offset, length and a region name, then runs the full
erase/verify/write/read/verify cycle.  The sample invokes it twice:

1. **PFM storage** -- the ``storage_partition`` in Program Flash (flash1).
2. **BFM top** -- the last page of Boot Flash Memory (flash0), exercising
   the upper address boundary.

All sizes (page size, write block size) are queried at runtime from the flash
driver, so the sample works across different devices (e.g. PIC32CK, PIC32CZ)
without modification.

Test steps
----------

For each region the following five operations are performed:

1. Erase one flash page
2. Verify the erased region reads back as the erase value
3. Write a test pattern
4. Read back the written data
5. Verify the data matches

Requirements
************

The board must have:

* A flash controller with driver support in Zephyr
* A ``storage_partition`` defined in the board devicetree

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/flash/flash_read_write
   :goals: build flash
   :compact:

Sample Output
=============

.. code-block:: console

   Flash Read/Write sample for pic32ck_sg01_cult
   ==========================

   --- PFM storage test ---

   PFM storage: flash device is ready.
   PFM storage parameters:
     Write block size : 8 bytes
     Erase value      : 0xff
     Erase block size : 4096 bytes
     Page start offset: 0x1f8000
     Page index       : 508
     Total pages      : 512
     Test offset      : 0x1f8000
     Test size        : 4096 bytes

   Test 1: PFM storage erase
     Erasing 4096 bytes at offset 0x1f8000 ...
     PFM storage erase succeeded!

   Test 2: PFM storage erase verification
     PFM storage erase verification succeeded!

   Test 3: PFM storage write
     Writing 4096 bytes (write block = 8 bytes) ...
     PFM storage write succeeded!

   Test 4: PFM storage read
     Reading back 4096 bytes ...
     PFM storage read succeeded!

   Test 5: PFM storage data verification
     PFM storage data read matches data written. Good!!

   --- BFM top test ---

   BFM top: flash device is ready.
   BFM top parameters:
     Write block size : 8 bytes
     Erase value      : 0xff
     Erase block size : 4096 bytes
     Page start offset: 0x1f000
     Page index       : 31
     Total pages      : 32
     Test offset      : 0x1f000
     Test size        : 4096 bytes

   Test 6: BFM top erase
     Erasing 4096 bytes at offset 0x1f000 ...
     BFM top erase succeeded!

   Test 7: BFM top erase verification
     BFM top erase verification succeeded!

   Test 8: BFM top write
     Writing 4096 bytes (write block = 8 bytes) ...
     BFM top write succeeded!

   Test 9: BFM top read
     Reading back 4096 bytes ...
     BFM top read succeeded!

   Test 10: BFM top data verification
     BFM top data read matches data written. Good!!

   ==========================
   All flash tests PASSED.

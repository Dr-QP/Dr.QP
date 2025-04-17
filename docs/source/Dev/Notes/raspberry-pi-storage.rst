Raspberry Pi storage
=====================

Tested on RPi 4
---------------

Built in SSD (Ultra XC10)
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   sudo dd if=/dev/zero of=/home/pi/testfile bs=1M count=1024 oflag=direct                                                                      (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 55.1041 s, 19.5 MB/s

Samsung tiny USB3 Drive (FAT)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/usbdisk/testfile bs=1M count=1024 oflag=direct                                                                (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 25.95 s, 41.4 MB/s
   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/usbdisk/testfile bs=1M count=1024 oflag=direct                                                                (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 26.7883 s, 40.1 MB/s

Samsung tiny USB3 Drive (ext4)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/usbdisk/testfile bs=1M count=1024 oflag=direct                                                                (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 21.0736 s, 51.0 MB/s
   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/usbdisk/testfile bs=1M count=1024 oflag=direct                                                                (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 21.2638 s, 50.5 MB/s
   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/usbdisk/testfile bs=1M count=1024 oflag=direct                                                                (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 20.8537 s, 51.5 MB/s

SanDisk via proper USB3 cable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/sandisk3/testfile bs=1M count=1024 oflag=direct                                                               (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 4.73075 s, 227 MB/s
   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/sandisk3/testfile bs=1M count=1024 oflag=direct                                                               (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 4.30668 s, 249 MB/s
   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/sandisk3/testfile bs=1M count=1024 oflag=direct                                                               (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 4.30062 s, 250 MB/s

Kingston DataTraveler MAX
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/usbdisk/testfile bs=1M count=1024 oflag=direct                                                                (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 5.06638 s, 212 MB/s
   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/usbdisk/testfile bs=1M count=1024 oflag=direct                                                                (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 5.07314 s, 212 MB/s
   pi@dr-qp ~> sudo dd if=/dev/zero of=/media/usbdisk/testfile bs=1M count=1024 oflag=direct                                                                (base)
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 5.06034 s, 212 MB/s

Tested on linux laptop
----------------------

Kingston DataTraveler MAX via Dock → Hub
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   ⋊> ~ sudo dd if=/dev/zero of=/media/anton/Kingston/testfile bs=1M count=1024 oflag=direct                                                               22:00:27
   Place your right index finger on the fingerprint reader
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 2.98712 s, 359 MB/s
   ⋊> ~ sudo dd if=/dev/zero of=/media/anton/Kingston/testfile bs=1M count=1024 oflag=direct                                                               22:00:55
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 2.97336 s, 361 MB/s
   ⋊> ~ sudo dd if=/dev/zero of=/media/anton/Kingston/testfile bs=1M count=1024 oflag=direct                                                               22:01:00
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 3.06077 s, 351 MB/s

Kingston DataTraveler MAX directly via USB3 adapter directly
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   ⋊> ~ sudo dd if=/dev/zero of=/media/anton/Kingston/testfile bs=1M count=1024 oflag=direct                                                               22:01:05
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 1.39293 s, 771 MB/s
   ⋊> ~ sudo dd if=/dev/zero of=/media/anton/Kingston/testfile bs=1M count=1024 oflag=direct                                                               22:01:40
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 1.35777 s, 791 MB/s
   ⋊> ~ sudo dd if=/dev/zero of=/media/anton/Kingston/testfile bs=1M count=1024 oflag=direct                                                               22:01:44
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 1.42721 s, 752 MB/s
   ⋊> ~ sudo dd if=/dev/zero of=/media/anton/Kingston/testfile bs=1M count=1024 oflag=direct                                                               22:01:47
   1024+0 records in
   1024+0 records out
   1073741824 bytes (1.1 GB, 1.0 GiB) copied, 1.38286 s, 776 MB/s

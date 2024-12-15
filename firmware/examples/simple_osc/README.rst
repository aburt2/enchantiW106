.. zephyr:code-sample:: simple-osc
   :name: Simple OSC

   Send large bundle over OSC

Overview
********

A simple script that combines zephyr's wifi shell example with sending a large OSC bundle.

Building and Running
********************

This configuration can be built for the frdm-rw612 evaluation board from NXP.

Clone the libmapper-nxp library into libdeps inorder to compile the file
libmapper-nxp: https://github.com/aburt2/libmapper-nxp


Sample console interaction
==========================

.. code-block:: console

   shell> wifi scan
   Scan requested
   shell>
   Num  | SSID                             (len) | Chan | RSSI | Sec
   1    | kapoueh!                         8     | 1    | -93  | WPA/WPA2
   2    | mooooooh                         8     | 6    | -89  | WPA/WPA2
   3    | Ap-foo blob..                    13    | 11   | -73  | WPA/WPA2
   4    | gksu                             4     | 1    | -26  | WPA/WPA2
   ----------
   Scan request done

   shell> wifi connect -s "gksu" -p SecretStuff -k 1
   Connection requested
   shell>
   Connected
   shell>

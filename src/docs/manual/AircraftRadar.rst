Aircraft Radar
==============

The Aircraft radar is optional and not activated by default since it requires
a RTLSDR device to work properly. To activate the Aircraft radar:

#. Connect the RTLSDR device to *vm1*.

   .. figure:: figs/fig301.jpg
      :scale: 100 %
      :align: center

      RTLSDR devices.

#. Download and install Dump1090 from https://github.com/antirez/dump1090.

#. Run Dump1090 as follows. If the device is detected, you will start receiving
   aircraft messages.

   .. code-block:: bash

      ubuntu@ubuntu-linux:~/dump1090$ ./dump1090 --net --interactive
      Found 1 device(s):
      0: Realtek, RTL2838UHIDIR, SN: 00000001 (currently selected)
      Found Rafael Micro R820T tuner
      Max available gain is: 49.60
      Setting gain to: 49.60
      Exact sample rate is: 2000000.052982 Hz
      Gain reported by device: 49.60
      Hex    Flight   Altitude  Speed   Lat       Lon       Track  Messages Seen   .
      --------------------------------------------------------------------------------
      406752          36000     428     0.000     0.000     300   5         2 sec

#. Edit the ROS launchfile in ``src/services/ros_packages/launch/default.launch``
   and activate the radar node. This node will try to connect to Dump190, processing
   the raw data and sending the detected aircrafts to the web application.
   After some time, you should start seeing nearby aircrafts.

   .. figure:: figs/fig302.jpg
      :scale: 100 %
      :align: center

      Nearby aircrafts detected with received ADS-B messages.

   .. HINT:: Move the map to your location, that most likely isn't the position of the simulated drone.
      Make sure that the antenna is connected to your RTLSDR device.
      If you're inside your office and cannot detect aircrafts, try to go outside.

Architecture
============

Skysense Planner's technology stack is based on Open-Source projects, including:

* `Ardupilot <http://ardupilot.org>`_: Unmanned aerial vehicle (UAV) platform, able to control autonomous multicopters, fixed-wing aircraft, traditional helicopters, ground rovers and antenna trackers.
* `Gazebo <http://gazebosim.org>`_: Simulator offering the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments.
* `MAVLink <http://qgroundcontrol.org/mavlink/start>`_: MAVLink Micro Air Vehicle Communication Protocol.
* `MAVROS <http://wiki.ros.org/mavros>`_: MAVLink extendable communication node for ROS with proxy for Ground Control Station.
* `ROS <http://www.ros.org>`_: Robot Operating System, a collection of software libraries and tools developers can use to assist in robot application development.
* `Robot Web Tools <http://robotwebtools.org/>`_: Integration between ROS and Web applications using Javascript and HTML5.
* `NASA Web World Wind <https://webworldwind.org>`_: 3D virtual globe API for HTML5 and JavaScript.
* `Dump1090 <https://github.com/antirez/dump1090>`_: ADS-B radar receiver (Mode S decoder) specifically designed for RTLSDR devices.

Skysense Planner delivers an integrated suite for planning and orchestrating complex drone missions
under safe conditions by leveraging a variety of complex components:

* Ardupilot and Gazebo are responsible for simulating a quadcopter drone.
* MAVROS connects the simulated drone to ROS using the MAVLink communication protocol.
* The Robot Web Tools connect ROS to Skysense Planner's web interface.
* Dump1090 is used to integrate ADS-B radar information.

.. raw:: html

   <div style="border-bottom: solid 1px white; overflow: hidden;">

.. container:: custom

   .. figure:: figs/fig201.png
      :scale: 60 %
      :align: left

   .. figure:: figs/fig204.png
      :scale: 80 %
      :align: left

   .. figure:: figs/fig202.png
      :scale: 80 %
      :align: left

   .. figure:: figs/fig203.png
      :scale: 80 %
      :align: left

   .. figure:: figs/fig205.png
      :scale: 60 %
      :align: left

   .. figure:: figs/fig206.png
      :scale: 60 %
      :align: left

.. raw:: html

   </div>

Overview
========

Skysense Planner is an Open-source Web Mission Planner for MAVLink-enabled Drones that allows you
to implement fully automated and scalable multi-UAS operations:

* Flight mission management: Create, modify, save and load waypoint-based flight missions for autonomous navigation.

* Automated takeoff clearance and In-flight controller: A virtual flight controller monitors your flights and, in presence of failures or errors, terminates the flight safely.

* ADS-B aircraft tracking: Monitor nearby aircrafts to avoid potential collisions.

* Geofence management: Bound your flights within a controlled area.

* Cloud-based: Accessible from Web interface and WebSocket API to orchestrate complex workflows.

.. IMPORTANT:: This first release of the Skysense Planner will setup a simulated Arducopter drone for you.
   This is the ideal setup for experimenting and testing the system. Connecting to a real drone requires
   some non-trivial script editing and it's not advised at this development stage.

.. NOTE:: Skysense Planner can be used to manage and operate any kind of MAVLink-enabled robot,
   including aerial and ground robots. However, our focus is on flying robots.

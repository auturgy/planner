First Simulated Flight
======================

#. Access Skysense Planner by pointing your browser to *vm1*'s IP address: *http://<ip address>*.
   There's no authentication. You will see the tab *Flight Data* along with the map centered on the drone's position.

   .. figure:: figs/fig100.jpg
      :scale: 85 %
      :align: center

      Flight Data tab.

   .. TIP:: The drone's state when stationing on the ground is *Status:DISARMED, Mode:STABILIZE*.

#. Select the *Geofence* tab and add the geofence vertices by clicking on the map.
   Connect the first and last waypoints to complete the procedure and set the geofence.

   .. figure:: figs/fig101.jpg
      :scale: 85 %
      :align: center

      Geofence tab.


#. Select the *Plan Mission* tab and add mission waypoints by clicking on the map.
   You cannot set waypoints outside the geofence if the geofence is already set.

   .. figure:: figs/fig102.jpg
      :scale: 85 %
      :align: center

      Plan Mission tab.

   Among the available options, You can set the Altitude of the mission waypoints (Default: 5m)
   and request to fly back to the Home waypoint once the mission is completed (Default: Activated).

#. Start the mission by clicking on a) *Set Mission* and b) *Start*. The drone will take off, reach the
   mission altitude and fly to the next waypoint. You can select the *Flight Data* panel to monitor
   the drone's live telemetry.

   .. IMPORTANT:: To stop a running mission, click on *Stop* in the Plan Mission tab.
                  The mission will be suspended immediately, giving you three options:
                  a) Cancel the mission and *Go back home*, b) Cancel the mission and land immediately, and c) continue the mission.


   .. figure:: figs/fig107.jpg
      :scale: 85 %
      :align: center

      Live telemetry during takeoff.

   .. TIP:: The drone's state will switch to *Status:ARMED, Mode:GUIDED* until the mission altitude is reached.
            Then, the drone switches to state *Status:ARMED, Mode:AUTO*.

#. The reached waypoints switch from Blue to Red. If the *Back home* was set, the drone will complete
   the mission by reaching the Home waypoint.

   .. figure:: figs/fig105.jpg
      :scale: 85 %
      :align: center

      Live telemetry during landing.

   .. TIP:: The drone's state will switch to *Status:ARMED, Mode:LAND* until the ground level is reached.
            Once the landing procedure is completed, the drone's state switches to *Status:DISARMED, Mode:STABILIZE*.

   .. figure:: figs/fig106.jpg
      :scale: 85 %
      :align: center

      Live telemetry after landing.

Congratulations! You completed your first virtual flight with Skysense Planner.

You can import and export flight missions (mission and geofence waypoints) on the *Import/Export* tab.
Exported missions do not use absolute coordinates: If you import the same mission with a drone at a different
location, the flight mission will consider your drone's position as starting position.

/********************************************************************
* Copyright (c) 2016 Skysense Inc.
* License: GNU General Public License v3
* Author: Federico Fiorini, Michele Dallachiesa
*********************************************************************/

// Enable/disable show/hide according to mission state
function show_mission_info() {
  document.getElementById("button-wrapper").style.visibility = "hidden";
  document.getElementById("mission-info").style.visibility = "visible";
  move_to('mission-param', 'mission-info');
  bottomWrapper = 'mission-info';
}

function show_mission_btns() {
  document.getElementById("button-wrapper").style.visibility = "visible";
  document.getElementById("mission-info").style.visibility = "hidden";
  move_to('mission-param', 'button-wrapper');
  bottomWrapper = 'button-wrapper';
}

function new_mission() {
  document.getElementById("new").style.display = "none";
  document.getElementById("runnig-img").style.display = "inline";
  document.getElementById("stop").style.display = "inline";
  document.getElementById("stop").disabled = false;
  document.getElementById("restart").style.display = "none";
  document.getElementById("mission-state").innerHTML = "Running mission ...";
  move_to('mission-state', 'state-span');
}

function mission_finished() {
  document.getElementById("new").style.display = "inline";
  document.getElementById("runnig-img").style.display = "none";
  document.getElementById("stop").style.display = "none";
  document.getElementById("restart").style.display = "inline";

  // Enable restart only if drone is in home position
  if (backHome == false) {
    document.getElementById("restart").disabled = true;
  } else {
    document.getElementById("restart").disabled = false;
  }

  document.getElementById("mission-state").innerHTML = "Mission completed.";
  move_to('mission-state', 'state-p');
  resizeMissionTable();
}

function mission_stopped() {
  document.getElementById("new").style.display = "inline";
  document.getElementById("runnig-img").style.display = "none";
  document.getElementById("stop").style.display = "none";
  document.getElementById("restart").style.display = "none";
  document.getElementById("mission-state").innerHTML = "Mission terminated.";
  move_to('mission-state', 'state-span');
  resizeMissionTable();
}

function mission_failed() {
  document.getElementById("new").style.display = "inline";
  document.getElementById("runnig-img").style.display = "none";
  document.getElementById("stop").style.display = "none";
  document.getElementById("restart").style.display = "inline";
  document.getElementById("mission-state").innerHTML = "Mission failed.";
  move_to('mission-state', 'state-p');
  resizeMissionTable();
}

// Set variables and flags
var backHome = true;
var currentMission = [];
var missionCancelled = false;
var lastCurrentWaypoint = 1;

// Check if max altitude is reached
function check_max_altitude() {
  var curr_altitude = document.getElementById('relative-altitude').innerHTML;
  return (parseFloat(altitude) - parseFloat(curr_altitude)) < 0.1;
}

// Check if min altitude is reached
function check_min_altitude() {
  var curr_altitude = document.getElementById('relative-altitude').innerHTML;
  return (parseFloat(curr_altitude)) < 0.05;
}

// Check if waypoints are reached
function check_waypoints() {

  if (currentMission == []) {
    return false;
  }

  // Get is_current waypoint
  currentWaypoint = get_current_waypoint();

  // Check if next waipoint is reached
  if (missionCancelled == false && currentWaypoint > lastCurrentWaypoint && currentWaypoint > 1) {

    for (var i = lastCurrentWaypoint; i < currentWaypoint; i++) {
      mark_passed_waypoint(i);
    }
    lastCurrentWaypoint = currentWaypoint;
  }

  if (currentMission[currentMission.length - 1] == undefined) {
    return false;
  }

  // Check if final waipoint is reached
  return currentMission[currentMission.length - 1].is_current == true;
}

// Get current waypoint index
function get_current_waypoint() {
  for (var i = 1; i < currentMission.length; i++) {
    if (currentMission[i].is_current == true) {
      return i;
    }
  }
}

// Set mission
function set_mission_btn() {

  waypoints = [];
  altitude = parseFloat(document.getElementById("input-altitude").value);
  backHome = document.getElementById("input-back-home").checked;
  document.getElementById("input-altitude").disabled = true;
  document.getElementById("input-back-home").disabled = true;

  updateWaypointTable(altitude);

  for (i = 0; i < coordinates.length; i++) {
    // Create waypoint
    var waypoint = new_waypoint(
      coordinates[i].latitude,
      coordinates[i].longitude,
      altitude,
      i == 0 // Set first (home) as current
    );

    // Add to list
    waypoints.push(waypoint);
  }

  // If back-home enabled: add home waypoint at the end
  if (backHome == true) {

    // Create waypoint
    var backHomeWaypoint = new_waypoint(
      coordinates[0].latitude,
      coordinates[0].longitude,
      altitude,
      false
    );
    waypoints.push(backHomeWaypoint);
  }

  // Duplicate last waypoint to use is_current value to check when reached
  var last = waypoints[waypoints.length - 1];
  waypoints.push(last);

  set_mission(waypoints);

  //document.getElementById("clear").disabled = true;
  document.getElementById("set").disabled = true;
  document.getElementById("start").disabled = false;

  disable_mission_planner();

  lastCurrentWaypoint = 1;
  missionSet = true;
  log_message('[INFO] New mission set', SUCCESS);
}

function publish_current_geofence() {

  // Clear geofence if not set
  if (geofenceSet == false) {
    publish_geofence([]);
    return;
  }

  // Pubish geofence coordinates
  var geofence = [];

  for (i = 0; i < geofenceMarks.length; i++) {
    var lat = geofenceMarks[i].position.latitude;
    var lng = geofenceMarks[i].position.longitude;

    var point = {};
    point[i] = {'lat': lat, 'lng': lng};
    geofence.push(point);
  }

  publish_geofence(geofence);
}

// Start mission
function start_mission_btn() {

  // Publish geofence
  publish_current_geofence();

  // Disable 'clear geofence' button
  document.getElementById("clear-area").disabled = true;

  new_mission();
  show_mission_info();

  // Show home marker if not showed yet
  if (homeShowed == false) {
    placemarkLayer.addRenderable(homeMark);
    missionMarks.unshift(homeMark);
    homeShowed = true;
  }

  missionRunning = true;

  // Remove and re add (to show it above the waipoints)
  remove_drone();
  drawDrone = true;

  // Failed to takeoff
  function failed_takeoff() {
    log_message('[CRITICAL] Takeoff failed', ERROR);
    failed_mission();
  }

  // Failed to arm
  function failed_arming() {
    log_message('[CRITICAL] Arming failed', ERROR);
    failed_mission();
  }

  // Failed guided mode
  function failed_guided() {
    log_message('[CRITICAL] Guided mode failed', ERROR);
    failed_mission();
  }

  // Failed mission
  function failed_mission() {
    // Stop current waiting functions
    wait_for_active = false;
    sleep(1000, function() {
      wait_for_active = true;
    });

    // Stabilize and disarm
    change_mode("STABILIZE", null, function(){
      disarm_motors();
    });

    // Set mission as failed
    mission_failed();
  }

  function finish_mission() {

    function finalize() {
      sleep(1000, function() {
        change_mode("STABILIZE", null, function(){
          disarm_motors();
          log_message('[INFO] Mission completed', SUCCESS);
        });
        mission_finished();
      });
    }

    log_message('[INFO] Last waypoint reached', SUCCESS);
    log_message('[INFO] Landing started', SUCCESS);

    sleep(1000, function() {
      land();
      wait_for(check_min_altitude, finalize);
    });
  }

  function start_mission() {
    log_message('[INFO] Takeoff completed', SUCCESS);
    sleep(1000, function() {
      change_mode("AUTO");
      wait_for(check_waypoints, finish_mission);
    });
  }

  // Mode guided
  change_mode("GUIDED", failed_guided, function() {
    // Arm motors
    arm_motors(failed_arming, function(){
      // Takeoff at given altitude
      takeoff(altitude, failed_takeoff);

      // Log and wait to reach altitude
      log_message('[INFO] Takeoff started', SUCCESS);
      wait_for(check_max_altitude, start_mission);
    });
  });

  log_message('[INFO] Mission started', SUCCESS);
}

// Restart the mission
function restart_mission_btn() {

  lastCurrentWaypoint = 1;

  // Re set marks blue
  for (i = 1; i < missionMarks.length; i++) {
    missionMarks[i].attributes.imageSource = BLUE_WAYPOINT_IMG;
  }

  // Reset waypoints
  for (i = 0; i < waypoints.length; i++) {
      waypoints[i].is_current = false;
  }
  waypoints[1].is_current = true;
  set_mission(waypoints, null, start_mission_btn);
}

// Temporary stop mission
function stop_mission_btn() {
  change_mode("ALT_HOLD");
}

// Restart current mission
function countinue_mission_btn() {
  change_mode("AUTO");
}

// Stop mission and land
function land_and_stop_btn() {

  function finalize() {
    sleep(1000, function() {
      change_mode("STABILIZE", null, function(){
        disarm_motors();
        log_message('[INFO] Mission completed', SUCCESS);
        mission_stopped();
      });
    });
  }

  log_message('[INFO] Mission cancelled', WARNING);
  log_message('[INFO] Landing started', WARNING);
  cancel_mission();
  land();

  document.getElementById("mission-state").innerHTML = "Terminating mission ...";
  document.getElementById("stop").disabled = true;

  // Stop active wait_for functions
  wait_for_active = false;

  // Wait 1 second and then re-activate wait_for
  sleep(1000, function() {
    wait_for_active = true;
    wait_for(check_min_altitude, finalize);
  });
}

// Stop mission and go back home
function back_home_and_stop_btn() {

  function stop_mission() {

    function finalize() {
      sleep(1000, function() {
        change_mode("STABILIZE", null, function(){
          disarm_motors();
          log_message('[INFO] Mission completed', SUCCESS);
          mission_stopped();
          missionCancelled = false;
        });
      });
    }

    log_message('[INFO] Landing started', WARNING);
    sleep(1000, function() {
      land();
      wait_for(check_min_altitude, finalize);
    });
  }

  log_message('[INFO] Mission cancelled', WARNING);

  // Update mission info panel
  document.getElementById("mission-state").innerHTML = "Terminating mission ...";
  document.getElementById("stop").disabled = true;

  // Set new mission to go back home
  var home_position = coordinates[0];
  var home_waypoint = new_waypoint(
    home_position.latitude,
    home_position.longitude,
    altitude,
    false
  );

  // Get current position
  var curr_lat = parseFloat(document.getElementById('latitude').innerHTML);
  var curr_lng = parseFloat(document.getElementById('longitude').innerHTML);
  var curr_waypoint = new_waypoint(
    curr_lat,
    curr_lng,
    altitude,
    true
  );

  var new_waypoints = [];
  new_waypoints[0] = curr_waypoint; // Current
  new_waypoints[1] = home_waypoint; // Home
  new_waypoints[2] = home_waypoint; // Duplicate

  // Stop active wait_for functions
  wait_for_active = false;

  // Wait 1 second and then re-activate wait_for
  sleep(1000, function() {
    wait_for_active = true;

    // Clear mission
    cancel_mission(null, function(){
      // Set back-home mission
      set_mission(new_waypoints, null, function(){
        // Mode auto
        change_mode("AUTO");
        log_message('[INFO] Going back home', WARNING);

        missionCancelled = true;
        wait_for(check_waypoints, stop_mission);
      });
    });
  });
}

// New mission
function new_mission_btn() {
  new_mission();
  clear_mission();
  clear_geofence();
}

// Export mission in text file
function export_mission_btn() {

  var missionObj = {};
  missionObj['mission_waypoints'] = {};
  missionObj['geofence'] = {};
  missionObj['back_home'] = backHome;

  var homeCoord = coordinates[0];

  // Add mission waypoints
  for (var i = 1; i < coordinates.length; i++) {
    var rel_lat = homeCoord.latitude - coordinates[i].latitude;
    var rel_lng = homeCoord.longitude - coordinates[i].longitude;
    var alt = parseFloat(altitude);

    missionObj['mission_waypoints'][i] = {'rel_lat': rel_lat, 'rel_lng': rel_lng, 'alt': alt};
  }

  // Add geofence coordinates
  for (var i = 0; i < geofenceCoordinates.length; i++) {
    var rel_lat = homeCoord.latitude - geofenceCoordinates[i].latitude;
    var rel_lng = homeCoord.longitude - geofenceCoordinates[i].longitude;

    missionObj['geofence'][i] = {'rel_lat': rel_lat, 'rel_lng': rel_lng};
  }

  download('mission', JSON.stringify(missionObj));
  log_message('[INFO] Mission exported', SUCCESS);
}

// Import mission from text file
function import_mission_btn() {
  // Reset model
  import_mission_reset();

  // Clear old mission
  clear_mission();
  clear_geofence();

  // Read file and parse it with callback
  readFile("mission-file", parse_mission_file);
}

// Parse mission file and draw mission
function parse_mission_file(content) {

  var missionObj = {};

  try {
    missionObj = JSON.parse(content);
  } catch(err) {
    console.log(err);
    $('#importErrorModal').modal('show');
    log_message('[WARNING] Mission not imported', WARNING);
    return;
  }

  // Disable/enable elements
  document.getElementById("input-altitude").disabled = false;
  document.getElementById("input-back-home").disabled = false;
  document.getElementById("clear").disabled = false;
  document.getElementById("set").disabled = false;
  document.getElementById("start").disabled = true;

  // Disable click recognizers
  disable_mission_planner();
  disable_geofence_planner();

  // Set variables
  missionSet = false;
  missionHidden = false;
  geofenceSet = false;

  log_message('[INFO] Mission imported', SUCCESS);

  var homeCoord = coordinates[0];

  // Set back-home option
  backHome = missionObj['back_home'];
  document.getElementById("input-back-home").checked = backHome;

  // Draw mission
  var mission = missionObj['mission_waypoints'];
  var mission_length = Object.keys(mission).length;

  // Set default altitude
  document.getElementById("input-altitude").value = mission[1]['alt'];

  var previousPosition = new WorldWind.Position(homeCoord.latitude, homeCoord.longitude, homeCoord.altitude);;

  for (var i = 1; i <= mission_length; i++) {
    var lat = homeCoord.latitude - mission[i]['rel_lat'];
    var lng = homeCoord.longitude - mission[i]['rel_lng'];
    var alt = mission[i]['alt'];

    var position = new WorldWind.Position(lat, lng, alt);
    coordinates.push(position);
    var marker = draw_waypoint_marker(position);
    draw_waypoint_path(previousPosition, position);

    previousPosition = position;
    previousMark = marker;

    // Add to table
    addWaypointRow(coordinates.length - 1, lat, lng, alt);
  }

  updateWaypointTable(alt);
  resizeMissionTable();

  // Draw geofence
  var geofence = missionObj['geofence'];
  var geofence_length = Object.keys(geofence).length;

  // Return if geofence is empty
  if (geofence_length == 0) {
    log_message('[INFO] Mission imported', SUCCESS);
    planMissionTab();
    return;
  }

  var previousPosition = null;

  for (var i = 0; i < geofence_length; i++) {
    var lat = homeCoord.latitude - geofence[i]['rel_lat'];
    var lng = homeCoord.longitude - geofence[i]['rel_lng'];
    var alt = geofence_altitude_view;

    var position = new WorldWind.Position(lat, lng, alt);
    geofenceCoordinates.push(position);
    draw_geofence_marker(position);

    if (previousPosition != null) {
      draw_geofence_path(previousPosition, position);
    }

    previousPosition = position;

    // Add to table
    addGeofenceRow(geofenceCoordinates.length, position.latitude, position.longitude);
  }

  resizeGeofenceTable();

  // Draw geofence
  draw_geofence_polygon();
  draw_geofence_path(geofenceCoordinates[0], previousPosition);

  geofenceSet = true;
  geofenceHidden = false;
  document.getElementById("clear-area").disabled = false;

  planMissionTab();
}

// Show import dialog modal
function import_mission_confirm() {
  document.getElementById("import-mission-confirmation").style.display = "none";
  document.getElementById("import-mission-confirmed").style.display = "block";

  // Import readfile.js after input file is displayed
  var script = document.createElement('script');
  script.src = 'js/readfile.js';
  document.head.appendChild(script);
}

// Reset dialog modal
function import_mission_reset() {
  document.getElementById("import-mission-confirmation").style.display = "block";
  document.getElementById("import-mission-confirmed").style.display = "none";
}

/********************************************************************
* Copyright (c) 2016 Skysense Inc.
* License: GNU General Public License v3
* Author: Federico Fiorini, Michele Dallachiesa
*********************************************************************/

// Connecting to ROS
// -----------------
var ros;
var change_mode;
var arm_motors;
var disarm_motors;
var takeoff;
var land;
var new_waypoint;
var set_mission;
var cancel_mission;
var listen_mission_waypoints;
var listen_global_position;
var listen_relative_altitude;
var listen_compass;
var listen_angular_velocity;
var listen_battery;
var listen_state;
var listen_radar;
var listen_mission_controller;
var publish_heartbeat;
var publish_geofence;
var started = true;

// Init ros services and topics
function init_ros() {

  // Create new connection to ros and set event listener callbacks
  ros = new ROSLIB.Ros({
    url : 'ws://' + planner_address + '/ws' 
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    log_message('[INFO] Connected to drone', SUCCESS);
    drone_status_ok();
    enable_all_tabs();
    run_forever(publish_heartbeat);

    // On connection re established after being lost
    if (!started) {
      restart_all_listeners();
    }

    started = true;
  });

  ros.on('error', function(error) {
    if (restarted) {
      console.log('Error connecting to websocket server: ', error);
      log_message('[CRITICAL] Unable to connect to drone', ERROR);
    }
    drone_not_connected();
    disable_all_tabs();
  });

  ros.on('close', function() {
    if (started) {
      console.log('Connection to websocket server closed.');
      log_message('[CRITICAL] Lost drone connection', ERROR);
    }
    drone_not_connected();
    disable_all_tabs();

    started = false;
    setTimeout(init_ros, 10000);
  });

  // Change mode service
  // -----------------

  var change_mode_service = new ROSLIB.Service({
    ros : ros,
    name : '/mavros/set_mode',
  });

  change_mode = function(mode, error_callback, success_callback) {

    // Create new request
    var request = new ROSLIB.ServiceRequest({
      custom_mode : mode
    });

    change_mode_service.callService(request, function(result) {

      if (result['success'] == false && typeof error_callback === 'function') {
        error_callback();
      } else if (result['success'] == true && typeof success_callback === 'function') {
        success_callback();
      }

      console.log('Result for service call on '
        + change_mode_service.name + ': ' + JSON.stringify(result));
    });
  }

  // Arm motors service
  // -----------------

  var arm_motors_service = new ROSLIB.Service({
    ros : ros,
    name : '/mavros/cmd/arming',
  });

  arm_motors = function(error_callback, success_callback) {

    // Create new request
    var request = new ROSLIB.ServiceRequest({
      value : true
    });

    arm_motors_service.callService(request, function(result) {

      if (result['success'] == false && typeof error_callback === 'function') {
        error_callback();
      } else if (result['success'] == true && typeof success_callback === 'function') {
        success_callback();
      }

      console.log('Result for service call on '
        + arm_motors_service.name + ': ' + JSON.stringify(result));
    });
  };

  // Disarm motors
  disarm_motors = function(error_callback, success_callback) {

    // Create new request
    var request = new ROSLIB.ServiceRequest({
      value : false
    });

    arm_motors_service.callService(request, function(result) {

      if (result['success'] == false && typeof error_callback === 'function') {
        error_callback();
      } else if (result['success'] == true && typeof success_callback === 'function') {
        success_callback();
      }

      console.log('Result for service call on '
        + arm_motors_service.name + ': ' + JSON.stringify(result));
    });
  };

  // Take off service
  // -----------------

  var takeoff_service = new ROSLIB.Service({
    ros : ros,
    name : '/mavros/cmd/takeoff',
  });

  takeoff = function(takeoff_altitude, error_callback, success_callback) {

    // Create new request
    var request = new ROSLIB.ServiceRequest({
      min_pitch : 0.0,
      yaw : 0.0,
      latitude : 0.0,
      longitude : 0.0,
      altitude : takeoff_altitude
    });

    takeoff_service.callService(request, function(result) {

      if (result['success'] == false && typeof error_callback === 'function') {
        error_callback();
      } else if (result['success'] == true && typeof success_callback === 'function') {
        success_callback();
      }

      console.log('Result for service call on '
        + takeoff_service.name + ': ' + JSON.stringify(result));
    });
  };

  // Land service
  // -----------------

  var landing_service = new ROSLIB.Service({
    ros : ros,
    name : '/mavros/cmd/land',
  });

  land = function(error_callback, success_callback) {

    // Create new request
    var request = new ROSLIB.ServiceRequest({
      min_pitch : 0.0,
      yaw : 0.0,
      latitude : 0.0,
      longitude : 0.0,
      altitude : 0.0
    });

    landing_service.callService(request, function(result) {

      if (result['success'] == false && typeof error_callback === 'function') {
        error_callback();
      } else if (result['success'] == true && typeof success_callback === 'function') {
        success_callback();
      }

      console.log('Result for service call on '
        + landing_service.name + ': ' + JSON.stringify(result));
    });
  };


  // Set waypoints service
  // ----------------------
  var set_mission_service = new ROSLIB.Service({
    ros : ros,
    name : '/mavros/mission/push',
  });

  new_waypoint = function(latitude, longitude, altitude, current) {

    var global_frame = 0;
    var mav_cmd_nav_waypoint = 16;

    return {
      frame: global_frame,
      is_current: current,
      command: mav_cmd_nav_waypoint,
      autocontinue: true,
      x_lat: latitude,
      y_long: longitude,
      z_alt: altitude
    };
  };

  set_mission = function(waypoint_list, error_callback, success_callback) {

    // Create new request
    var request = new ROSLIB.ServiceRequest({
      waypoints : waypoint_list
    });

    set_mission_service.callService(request, function(result) {

      if (result['success'] == false && typeof error_callback === 'function') {
        error_callback();
      } else if (result['success'] == true && typeof success_callback === 'function') {
        success_callback();
      }

      console.log('Result for service call on '
        + set_mission_service.name + ': ' + JSON.stringify(result));
    });
  };

  // Cancel service
  // ----------------------
  var cancel_mission_service = new ROSLIB.Service({
    ros : ros,
    name : '/mavros/mission/clear',
  });

  cancel_mission = function(error_callback, success_callback) {

    // Create new request
    var request = new ROSLIB.ServiceRequest({});

    cancel_mission_service.callService(request, function(result) {

      if (result['success'] == false && typeof error_callback === 'function') {
        error_callback();
      } else if (result['success'] == true && typeof success_callback === 'function') {
        success_callback();
      }

      console.log('Result for service call on '
        + cancel_mission_service.name + ': ' + JSON.stringify(result));
    });
  }

  // Set mission waypoints topic listener
  // ---------------------------
  var mission_waypoints_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/mavros/mission/waypoints'
  });

  function stop_listen_mission_waypoints() {
    mission_waypoints_listener.unsubscribe();
  };

  listen_mission_waypoints = function(callback, once) {

    // Once default to false
    once = (typeof once === 'undefined') ? false : once;
    mission_waypoints_listener.subscribe(function(message) {

      callback(message);

      if (once == true) {
        stop_listen_mission_waypoints();
      }
    });
  };

  // Set global position topic listener
  // ---------------------------
  var global_position_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/mavros/global_position/global'
  });

  function stop_listen_global_position() {
    global_position_listener.unsubscribe();
  };

  listen_global_position = function(callback, once) {

    // Once default to false
    once = (typeof once === 'undefined') ? false : once;
    global_position_listener.subscribe(function(message) {

      callback(message);

      if (once == true) {
        stop_listen_global_position();
      }
    });
  };

  // Set relative altitude topic listener
  // ---------------------------
  var relative_altitude_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/mavros/global_position/rel_alt'
  });

  function stop_listen_relative_altitude() {
    relative_altitude_listener.unsubscribe();
  };

  listen_relative_altitude = function(callback, once) {

    // Once default to false
    once = (typeof once === 'undefined') ? false : once;

    relative_altitude_listener.subscribe(function(message) {

      callback(message['data']);

      if (once == true) {
        stop_listen_relative_altitude();
      }
    });
  };

  // Set compass listener
  // ---------------------------
  var compass_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/mavros/global_position/compass_hdg'
  });

  function stop_listen_compass() {
    compass_listener.unsubscribe();
  };

  listen_compass = function(callback, once) {

    // Once default to false
    once = (typeof once === 'undefined') ? false : once;

    compass_listener.subscribe(function(message) {

      callback(message['data']);

      if (once == true) {
        stop_listen_compass();
      }
    });
  };

  // Set angular velocity topic listener
  // -----------------------------------
  var angular_velocity_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/mavros/imu/data_raw'
  });

  function stop_listen_angular_velocity() {
    angular_velocity_listener.unsubscribe();
  };

  listen_angular_velocity = function(callback, once) {

    // Once default to false
    once = (typeof once === 'undefined') ? false : once;

    angular_velocity_listener.subscribe(function(message) {

      callback(message['angular_velocity']);

      if (once == true) {
        stop_listen_angular_velocity();
      }
    });
  };

  // Set battery topic listener
  // ---------------------------
  var battery_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/mavros/battery'
  });

  function stop_listen_battery() {
    battery_listener.unsubscribe();
  };

  listen_battery = function(callback, once) {

    // Once default to false
    once = (typeof once === 'undefined') ? false : once;

    battery_listener.subscribe(function(message) {

      callback(message);

      if (once == true) {
        stop_listen_battery();
      }
    });
  };

  // Set state topic listener
  // ---------------------------
  var state_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/mavros/state'
  });

  function stop_listen_state() {
    state_listener.unsubscribe();
  };

  listen_state = function(callback, once) {

    // Once default to false
    once = (typeof once === 'undefined') ? false : once;

    state_listener.subscribe(function(message) {

      callback(message);

      if (once == true) {
        stop_listen_state();
      }
    });
  };

  // Set radar listener
  // ---------------------------
  var radar_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/device/radar'
  });

  function stop_listen_radar() {
    radar_listener.unsubscribe();
  };

  listen_radar = function(callback, once) {

    // Once default to false
    once = (typeof once === 'undefined') ? false : once;

    radar_listener.subscribe(function(message) {

      callback(message);

      if (once == true) {
        stop_listen_radar();
      }
    });
  };

  // Set mission controller listener
  // ---------------------------
  var mission_controller_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/device/mission/controller'
  });

  function stop_listen_mission_controller() {
    mission_controller_listener.unsubscribe();
  };

  listen_mission_controller = function(callback, once) {

    // Once default to false
    once = (typeof once === 'undefined') ? false : once;

    mission_controller_listener.subscribe(function(message) {

      callback(message);

      if (once == true) {
        stop_listen_mission_controller();
      }
    });
  };

  // Set heartbeat topic
  // ---------------------------
  var heartbeat_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/client/heartbeat',
    messageType: 'std_msgs/String'
  });
  var count = 0;

  publish_heartbeat = function() {

    var msg = new ROSLIB.Message({
      data: JSON.stringify({'count': count++})
    });

    heartbeat_topic.publish(msg);
  }

  // Set geofence topic
  // ---------------------------
  var geofence_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/client/geofence',
    messageType: 'std_msgs/String'
  });

  publish_geofence = function(coordinates) {

    var msg = new ROSLIB.Message({
      data: JSON.stringify({'coordinates': coordinates})
    });

    geofence_topic.publish(msg);
  }
}

// Connect to ros and init services and topics
init_ros();

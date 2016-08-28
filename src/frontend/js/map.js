/********************************************************************
* Copyright (c) 2016 Skysense Inc.
* License: GNU General Public License v3
* Author: Federico Fiorini, Michele Dallachiesa
*********************************************************************/

// Register an event listener to be called when the page is loaded and resized.
window.addEventListener("load", eventWindowLoaded, false);
window.addEventListener("resize", eventWindowResized, false);

// Set map zoom at 600 meters
var altitude_view = 600;

// Set global variables
var wwd = null;
var placemarkLayer = new WorldWind.RenderableLayer("Placemarks");
var canvasBackground = null;
var missionClickRecognizer = null;
var missionTapRecognizer = null;
var geofenceClickRecognizer = null;
var geofenceTapRecognizer = null;
var coordinates = [];
var geofenceCoordinates = [];
var missionMarks = [];
var missionPaths = [];
var geofenceMarks = [];
var geofencePaths = [];
var geofenceShape = [];
var homeMark = null;
var droneMark = null;
var previousMark = null;
var previousGeoMark = null;
var altitude = null;
var groundAltitude = null;
var actionState = {};
var tabState = {};
var previousState = {};
var airplanes = {};
var bottomWrapper = 'button-wrapper';

// Set flags
var drawDrone = true;
var missionRunning = false;
var homeShowed = false;
var missionError = false;
var wait_for_active = true;
var missionHidden = null;
var geofenceHidden = null;
var geofenceSet = false;
var missionSet = false;
var mapLoaded = false;
var loadMapCalled = false;
var centerDrone = false;

// Set markers altitude on map
var mission_altitude_view = 1;
var drone_altitude_view = 2;
var geofence_altitude_view = 3;
var airplanes_altitude_view = 5;

// Set constants
var SUCCESS = "success";
var WARNING = "warning";
var ERROR = "error";

// Set images
var HOME_WAYPOINT_IMG = "http://i.imgur.com/ewfoDgj.png";
var BLUE_WAYPOINT_IMG = "http://i.imgur.com/95UY1Cu.png";
var RED_WAYPOINT_IMG = "http://i.imgur.com/PBFAvsD.png";
var DRONE_IMG = "http://i.imgur.com/odfaZIK.png";
var AIRPLANE_IMG = "http://i.imgur.com/YAoGuaR.png";
var GEOFENCE_MARK_IMG = "http://i.imgur.com/iorQQa8.png";

// JQuery
$( document ).ready(function() {

  // Disable drag with right click (rotation)
  var mouseDownFlag = 0;

  $('#canvasMap').mousedown(function(event) {
    if (event.which == 3) {
      mouseDownFlag = 1;
    }
  });

  $('#canvasMap').mousemove(function(event) {
    if (mouseDownFlag == 1) {
      event.preventDefault();
    }
  });

  $('#canvasMap').mouseup(function(event) {
    if (event.which == 3) {
      mouseDownFlag = 0;
    }
  });

  // Show / hide left panel [on mobile only]
  $("#show-left-panel").click(function(e) {
    e.preventDefault();
    $(".left-panel").animate({"left": "0px"}, "slow");
    $("#message-box").animate({"left": "0px"}, "slow");
    $("#message-box-background").animate({"left": "0px"}, "slow");
    $(this).css("display", "none");
    $(this).css("left", "30%");
    $("#hide-left-panel").css("display", "inline");
    $("#hide-left-panel").animate({"left": "30%"}, "slow");
  });

  $("#hide-left-panel").click(function(e) {
    e.preventDefault();
    $(".left-panel").animate({"left": "-30%"}, "slow");
    $("#message-box").animate({"left": "-30%"}, "slow");
    $("#message-box-background").animate({"left": "-30%"}, "slow");
    $(this).css("display", "none");
    $(this).css("left", "0px");
    $("#show-left-panel").css("display", "inline");
    $("#show-left-panel").animate({"left": "0px"}, "slow");
  });
});

// When window is loaded start listining to flight data
function eventWindowLoaded() {
  flight_data();
}

// Resize canvas size on window resized
function eventWindowResized() {
  resizeMissionTable();
  resizeGeofenceTable();
  if (canvasBackground != null) {
    canvasBackground.style.width = document.getElementById('map-width').clientWidth + "px";
    canvasBackground.style.height = window.innerHeight - document.getElementById('header').clientHeight + "px";
    wwd.redraw();
  }
}

// Resize mission table height to fit in panel
function resizeMissionTable() {
  var missionTable = document.getElementById('mission-table-wrapper');
  var missionPanel = document.getElementById('mission-plan-panel').clientHeight;
  var missionInstr = document.getElementById('mission-instruction').clientHeight;
  var missionButton = document.getElementById(bottomWrapper).clientHeight;

  missionTable.style.height = missionPanel - missionInstr - missionButton - 60 + "px";
}

// Resize geofence table height to fit in panel
function resizeGeofenceTable() {
  var geofenceTable = document.getElementById('geofence-table-wrapper');
  var geofencePanel = document.getElementById('geofence-panel').clientHeight;
  var geofenceInstr = document.getElementById('geofence-instruction').clientHeight;
  var geofenceButton = document.getElementById('geofence-buttons').clientHeight;

  geofenceTable.style.height = geofencePanel - geofenceInstr - geofenceButton - 60 + "px";
}

// Load the map
function loadMap(latitude, longitude) {

  loadMapCalled = true;
  drone_status_ok();

  // Define canvas background size
  canvasBackground = document.getElementById('canvas-background');
  canvasBackground.style.width = document.getElementById('map-width').clientWidth + "px";
  canvasBackground.style.height = window.innerHeight - document.getElementById('header').clientHeight + "px";

  document.getElementById('map-buttons-wrapper').style.visibility = "visible";

  // Create a World Window for the canvas.
  wwd = new WorldWind.WorldWindow("canvasMap");
  wwd.globe = new WorldWind.Globe2D();

  // Define layers
  var layers = [
    {layer: new WorldWind.BMNGOneImageLayer(), enabled: true},
    {layer: new WorldWind.CompassLayer(), enabled: false},
    {layer: new WorldWind.CoordinatesDisplayLayer(wwd), enabled: true},
    {layer: new WorldWind.ViewControlsLayer(wwd), enabled: false}
  ];

  for (var l = 0; l < layers.length; l++) {
    layers[l].layer.enabled = layers[l].enabled;
    wwd.addLayer(layers[l].layer);
  }

  // Add map layer with transparency
  var map = new WorldWind.BingAerialLayer(null);
  map.opacity = 0.6;
  wwd.addLayer(map);

  // Add placemark renderable layer
  wwd.addLayer(placemarkLayer);

  // Adjust the Navigator to place the drone in the center of the World Window
  center_map(latitude, longitude);
  wwd.redraw();

  // Define mission click event recognizer
  function missionClickEvent(recognizer) {

    if (missionError == true) {
      return;
    }

    // Get coordinates
    var x = recognizer.clientX,
        y = recognizer.clientY;

    var pickList = wwd.pick(wwd.canvasCoordinates(x, y));
    pickList.objects.length - 1;

    // Last element will always be the map
    var obj = pickList.objects[pickList.objects.length - 1];
    var position = obj.position;

    // Push position to coordinates array
    coordinates.push(position);

    // Draw waypoint on the map
    var placemark = draw_waypoint_marker(position);

    // When second waypoint is set: change altitude of home waypoint marker to be more visible
    if (coordinates.length == 2) {
      homeMark.position.altitude = position.altitude;
      homeMark.altitudeMode = WorldWind.ABSOLUTE;
      previousMark = homeMark;
    }

    // Draw path
    draw_waypoint_path(previousMark.position, placemark.position);
    wwd.redraw();

    previousMark = placemark;
    missionHidden = false;

    // Enable buttons
    if (coordinates.length == 2) {
      document.getElementById("clear").disabled = false;
      document.getElementById("set").disabled = false;
    }

    // Add to table
    addWaypointRow(coordinates.length - 1, position.latitude, position.longitude, altitude);
    resizeMissionTable();
  }

  // Listen for mouse clicks or screen tap
  missionClickRecognizer = new WorldWind.ClickRecognizer(wwd, missionClickEvent);
  missionTapRecognizer = new WorldWind.TapRecognizer(wwd, missionClickEvent);

  // Define geofence click event recognizer
  function geofenceClickEvent(recognizer) {

    if (missionError == true) {
      return;
    }

    // Get coordinates
    var x = recognizer.clientX,
        y = recognizer.clientY;

    // Get position
    var pickList = wwd.pick(wwd.canvasCoordinates(x, y));
    var position = pickList.objects[0].position;

    // Check if clicked on first placemark and set area
    if (position == geofenceCoordinates[0]) {
      set_geofence();
      return;
    }

    // Check if clicked on placemark and return
    for (var i = 1; i < geofenceCoordinates.length; i++) {
      if (position == geofenceCoordinates[i]) {
        return;
      }
    }

    // Push position to coordinates array
    geofenceCoordinates.push(position);

    // Draw geofence marker
    var placemark = draw_geofence_marker(position);

    // Draw geofence path (only after second marker)
    if (previousGeoMark != null) {
      draw_geofence_path(previousGeoMark.position, placemark.position);
    }

    previousGeoMark = placemark;
    geofenceHidden = false;

    wwd.redraw();

    // Enable buttons
    if (geofenceCoordinates.length == 1) {
      document.getElementById("clear-area").disabled = false;
    }

    // Add to table
    addGeofenceRow(geofenceCoordinates.length, position.latitude, position.longitude);
  }

  // Listen for mouse clicks or screen tap
  geofenceClickRecognizer = new WorldWind.ClickRecognizer(wwd, geofenceClickEvent);
  geofenceTapRecognizer = new WorldWind.TapRecognizer(wwd, geofenceClickEvent);

  // Initially disable all recognizers
  missionClickRecognizer.enabled = false;
  geofenceClickRecognizer.enabled = false;
  missionTapRecognizer.enabled = false;
  geofenceTapRecognizer.enabled = false;

  mapLoaded = true;

  // Start listening to other topics
  check_airplanes();
  check_mission_controller();
  check_mission_waypoints();
  run_forever(calculateGroundAltitude)
}

// Calculate ground altitude in center of canvas
function calculateGroundAltitude() {

  var centerX = window.innerWidth - canvasBackground.width / 2;
  var centerY = window.innerHeight - canvasBackground.height / 2;
  var pickList = wwd.pick(wwd.canvasCoordinates(centerX, centerY));
  var i = pickList.objects.length - 1;
  if (pickList.objects[i] == undefined) {
    return;
  }
  var position = pickList.objects[i].position;
  groundAltitude = position.altitude;
}

// Show mission markers
function show_mission() {

  if (missionHidden == false) {
    return;
  }

  for (i = 0; i < missionMarks.length; i++) {
    placemarkLayer.addRenderable(missionMarks[i]);
  }

  for (i = 0; i < missionPaths.length; i++) {
    placemarkLayer.addRenderable(missionPaths[i]);
  }

  missionHidden = false;
  wwd.redraw();
}

// Hide mission markers
function hide_mission() {

  if (missionHidden == true) {
    return;
  }

  for (i = 0; i < missionMarks.length; i++) {
    placemarkLayer.removeRenderable(missionMarks[i]);
  }

  for (i = 0; i < missionPaths.length; i++) {
    placemarkLayer.removeRenderable(missionPaths[i]);
  }

  missionHidden = true;
  wwd.redraw();
}

// Show markers
function show_geofence() {
  if (geofenceHidden == false) {
    return;
  }

  // Show internal shape
  for (i = 0; i < geofenceShape.length; i++) {
    placemarkLayer.addRenderable(geofenceShape[i]);
  }

  // Show perimeter
  for (i = 0; i < geofenceMarks.length; i++) {
    placemarkLayer.addRenderable(geofenceMarks[i]);
  }

  for (i = 0; i < geofencePaths.length; i++) {
    placemarkLayer.addRenderable(geofencePaths[i]);
  }

  geofenceHidden = false;
  wwd.redraw();
}

// Hide mission markers
function hide_geofence() {

  if (geofenceHidden == true) {
    return;
  }

  for (i = 0; i < geofenceShape.length; i++) {
    placemarkLayer.removeRenderable(geofenceShape[i]);
  }

  for (i = 0; i < geofenceMarks.length; i++) {
    placemarkLayer.removeRenderable(geofenceMarks[i]);
  }

  for (i = 0; i < geofencePaths.length; i++) {
    placemarkLayer.removeRenderable(geofencePaths[i]);
  }

  geofenceHidden = true;
  wwd.redraw();
}

// Enable mission click recognizer event listener
function enable_mission_planner() {
  if (missionClickRecognizer != null) {
    missionClickRecognizer.enabled = true;
    missionTapRecognizer.enabled = true;
  }
}

// Disable mission click recognizer event listener
function disable_mission_planner() {
  if (missionClickRecognizer != null) {
    missionClickRecognizer.enabled = false;
    missionTapRecognizer.enabled = false;
  }
}

// Enable flight area click recognizer event listener
function enable_geofence_planner() {
  if (geofenceClickRecognizer != null) {
    geofenceClickRecognizer.enabled = true;
    geofenceTapRecognizer.enabled = true;
  }
}

// Disable flight area click recognizer event listener
function disable_geofence_planner() {
  if (geofenceClickRecognizer != null) {
    geofenceClickRecognizer.enabled = false;
    geofenceTapRecognizer.enabled = false;
  }
}

// Disable all tabs
function disable_all_tabs() {

  var navs = document.getElementsByClassName("nav-item");
  for (var l = 0; l < navs.length; l++) {
    navs[l].className = "nav-item disabled";
  }

  // Save the previous state and disable onclick
  if (JSON.stringify(tabState) === JSON.stringify({})) {
    var navs = document.getElementsByClassName("nav-link");
    for (var l = 0; l < navs.length; l++) {
      tabState[navs[l].id] = navs[l].onclick;
      navs[l].onclick = null;
    }
  }
}

// Enable all tabs
function enable_all_tabs() {

  // Get disabled tabs
  var navs = document.getElementsByClassName("nav-item disabled");

  // Remove 'disabled' class until there is none anymore
  while (true) {
    for (var l = 0; l < navs.length; l++) {
      navs[l].className = "nav-item";
    }

    navs = document.getElementsByClassName("nav-item disabled");
    if (navs.length == 0) {
      break;
    }
  }

  if (JSON.stringify(tabState) === JSON.stringify({})) {
    return;
  }

  var navs = document.getElementsByClassName("nav-link");
  for (var l = 0; l < navs.length; l++) {
    navs[l].onclick = tabState[navs[l].id];
  }

  tabState = {};
}

// Disable all possible actions in case of error
function disable_all_actions() {

  missionError = true;

  // Save the previous state
  if (JSON.stringify(actionState) === JSON.stringify({})) {
    actionState['stop'] = document.getElementById("stop").disabled;
    actionState['set'] = document.getElementById("set").disabled;
    actionState['start'] = document.getElementById("start").disabled;
    actionState['restart'] = document.getElementById("restart").disabled;
    actionState['new'] = document.getElementById("new").disabled;
    actionState['clear'] = document.getElementById("clear").disabled;
    actionState['clear-area'] = document.getElementById("clear-area").disabled;
    actionState['import-mission'] = document.getElementById("import-mission").disabled;
    actionState['export-mission'] = document.getElementById("export-mission").disabled;
  }

  document.getElementById("stop").disabled = true;
  document.getElementById("set").disabled = true;
  document.getElementById("start").disabled = true;
  document.getElementById("restart").disabled = true;
  document.getElementById("new").disabled = true;
  document.getElementById("clear").disabled = true;
  document.getElementById("clear-area").disabled = true;
  document.getElementById("import-mission").disabled = true;
  document.getElementById("export-mission").disabled = true;
}

// Re-enable actions when error finishes
function enable_all_actions() {

  missionError = false;

  if (JSON.stringify(actionState) === JSON.stringify({})) {
    return;
  }

  document.getElementById("stop").disabled = actionState['stop'];
  document.getElementById("set").disabled = actionState['set'];
  document.getElementById("start").disabled = actionState['start'];
  document.getElementById("restart").disabled = actionState['restart'];
  document.getElementById("new").disabled = actionState['new'];
  document.getElementById("clear").disabled = actionState['clear'];
  document.getElementById("clear-area").disabled = actionState['clear-area'];
  document.getElementById("import-mission").disabled = actionState['import-mission'];
  document.getElementById("export-mission").disabled = actionState['export-mission'];

  actionState = {};
}

// Clear the current mission
function clear_mission() {
  hide_mission();
  cancel_mission();

  // Re-init variables
  previousMark = null;
  homeMark = null;
  homeShowed = false;
  missionMarks = [];
  missionPaths = [];
  coordinates = [];
  missionRunning = false;
  drawDrone = false;
  missionSet = false;

  // Disable buttons
  document.getElementById("clear").disabled = true;
  document.getElementById("set").disabled = true;
  document.getElementById("start").disabled = true;
  document.getElementById("input-altitude").disabled = false;
  document.getElementById("input-back-home").disabled = false;

  // Clear table
  var old_tbody = document.getElementById("waypoint-table").tBodies[0];
  var new_tbody = document.createElement('tbody');
  old_tbody.parentNode.replaceChild(new_tbody, old_tbody);

  // Recalculate home
  draw_home_marker();

  show_mission_btns();
  enable_mission_planner();
}

// Draw on map home marker
function draw_home_marker() {

  if (homeMark != null) {
    return;
  }

  // Get current coordinates
  var lat = parseFloat(document.getElementById('latitude').innerHTML);
  var lon = parseFloat(document.getElementById('longitude').innerHTML);

  // Push position to coordinates
  var position = new WorldWind.Position(lat, lon, mission_altitude_view);
  coordinates.push(position);

  // Create placemark
  homeMark = new WorldWind.Placemark(position);
  homeMark.eyeDistanceScaling = false;
  homeMark.alwaysOnTop = true;
  homeMark.altitudeMode = WorldWind.RELATIVE_TO_GROUND;
  var placemarkAttributes = new WorldWind.PlacemarkAttributes(null);
  placemarkAttributes.imageSource = HOME_WAYPOINT_IMG;
  placemarkAttributes.depthTest = false;
  homeMark.attributes = placemarkAttributes;
  wwd.redraw();

  // Set as previous mark
  previousMark = homeMark;

  // Add to table
  addWaypointRow("Home", lat, lon, altitude);
}

// Draw drone marker
function draw_drone(latitude, longitude) {

  if (drawDrone == false) {
    return;
  }

  // Draw drone on the map
  var position = new WorldWind.Position(latitude, longitude, drone_altitude_view);

  if (droneMark == null) {
    // Create placemark
    droneMark = new WorldWind.Placemark(position);
    droneMark.eyeDistanceScaling = false;
    droneMark.altitudeMode = WorldWind.RELATIVE_TO_GROUND;
    droneMark.imageRotationReference = WorldWind.RELATIVE_TO_GLOBE;
    droneMark.alwaysOnTop = true;
    droneMark.label = "Drone #1\n\n";
    var placemarkAttributes = new WorldWind.PlacemarkAttributes(null);
    placemarkAttributes.imageSource = DRONE_IMG;

    placemarkAttributes.depthTest = false;
    droneMark.attributes = placemarkAttributes;

    // Add the placemark to the layer.
    placemarkLayer.addRenderable(droneMark);

  } else {
    // Update position
    droneMark.position = position;
  }

  wwd.redraw();
}

// Remove drone from map
function remove_drone() {
  placemarkLayer.removeRenderable(droneMark);
  drawDrone = false;
  droneMark = null;
}

// Update drone placemark rotation according to angle
function update_drone_direction(angle) {
  if (droneMark != null) {
    droneMark.imageRotation = angle;
  }
}

// Draw waypoint marker
function draw_waypoint_marker(position) {

  position.altitude = mission_altitude_view + groundAltitude;

  // Create the placemark
  var placemark = new WorldWind.Placemark(position);
  placemark.eyeDistanceScaling = false;
  placemark.altitudeMode = WorldWind.ABSOLUTE;
  placemark.alwaysOnTop = true;
  var placemarkAttributes = new WorldWind.PlacemarkAttributes(null);
  placemarkAttributes.imageSource = BLUE_WAYPOINT_IMG;

  placemarkAttributes.depthTest = false;
  placemark.attributes = placemarkAttributes;

  // Add the placemark to the layer.
  placemarkLayer.addRenderable(placemark);
  missionMarks.push(placemark);

  return placemark;
}

// Draw waypoint path
function draw_waypoint_path(pos1, pos2) {

  pos1.altitude = mission_altitude_view + groundAltitude;
  pos2.altitude = mission_altitude_view + groundAltitude;

  var path = new WorldWind.Path([pos1, pos2]);
  var pathAttributes = new WorldWind.ShapeAttributes(null);
  pathAttributes.outlineColor = WorldWind.Color.RED;
  pathAttributes.outlineWidth = 3.0;
  pathAttributes.depthTest = false;
  path.attributes = pathAttributes;
  path.altitudeMode = WorldWind.ABSOLUTE;
  path.pathType = WorldWind.LINEAR;

  placemarkLayer.addRenderable(path);
  missionPaths.push(path);
}

// Draw geofence marker
function draw_geofence_marker(position) {

  position.altitude = geofence_altitude_view + groundAltitude;

  // Create the placemark
  var placemark = new WorldWind.Placemark(position);
  placemark.eyeDistanceScaling = false;
  placemark.altitudeMode = WorldWind.ABSOLUTE;
  placemark.alwaysOnTop = true;
  var placemarkAttributes = new WorldWind.PlacemarkAttributes(null);

  placemarkAttributes.imageSource = GEOFENCE_MARK_IMG;

  placemarkAttributes.depthTest = false;
  placemark.attributes = placemarkAttributes;

  // Add the placemark to the layer.
  placemarkLayer.addRenderable(placemark);
  geofenceMarks.push(placemark);

  return placemark;
}

// Draw geofence path
function draw_geofence_path(pos1, pos2) {

  pos1.altitude = geofence_altitude_view + groundAltitude;
  pos2.altitude = geofence_altitude_view + groundAltitude;

  var path = new WorldWind.Path([pos1, pos2]);
  var pathAttributes = new WorldWind.ShapeAttributes(null);
  pathAttributes.outlineColor = new WorldWind.Color(0.33, 1, 0.42, 1);
  pathAttributes.outlineWidth = 2.0;
  pathAttributes.depthTest = false;
  path.attributes = pathAttributes;
  path.altitudeMode = WorldWind.ABSOLUTE;
  path.pathType = WorldWind.LINEAR;

  placemarkLayer.addRenderable(path);
  geofencePaths.push(path);
}

// Draw geofence polygon shape
function draw_geofence_polygon() {

  geofenceCoordinates.map(function(pos) {
    pos.altitude = geofence_altitude_view + groundAltitude;
  });

  var polygonAttributes = new WorldWind.ShapeAttributes(null);
  polygonAttributes.interiorColor = new WorldWind.Color(0.32, 1.00, 0.50, 0.5);
  polygonAttributes.drawOutline = false;
  polygonAttributes.depthTest = false;
  var polygon = new WorldWind.Polygon(geofenceCoordinates, polygonAttributes);
  polygon.altitudeMode = WorldWind.ABSOLUTE;

  geofenceShape.push(polygon);
  placemarkLayer.addRenderable(polygon);
  wwd.redraw();
}

// Draw airplane on map
function draw_airplane(uniq_id, desc, lat, lng, direction) {

  var position = new WorldWind.Position(lat, lng, airplanes_altitude_view);

  if (airplanes[uniq_id] == undefined) {

    var planeMark = new WorldWind.Placemark(position);
    planeMark.label = desc + "\n";
    planeMark.eyeDistanceScaling = false;
    planeMark.altitudeMode = WorldWind.RELATIVE_TO_GROUND;
    planeMark.imageRotation = direction;
    planeMark.imageRotationReference = WorldWind.RELATIVE_TO_GLOBE;

    var placemarkAttributes = new WorldWind.PlacemarkAttributes(null);
    placemarkAttributes.imageSource = AIRPLANE_IMG;
    placemarkAttributes.depthTest = false;
    planeMark.attributes = placemarkAttributes;

    // Save mark
    airplanes[uniq_id] = {};
    airplanes[uniq_id]['mark'] = planeMark;

    // Add the placemark to the layer.
    placemarkLayer.addRenderable(airplanes[uniq_id]['mark']);

  } else {
    airplanes[uniq_id]['mark'].position = position;
    airplanes[uniq_id]['mark'].imageRotation = direction;
  }

  // Set airplane as seen
  airplanes[uniq_id]['seen'] = 'true';
}

// Mark waypoint as passed by changing image
function mark_passed_waypoint(markerId) {

  if (missionMarks[markerId] == undefined) {
    return;
  }

  log_message('[INFO] Reached waypoint #' + markerId, SUCCESS);

  missionMarks[markerId].attributes.imageSource = RED_WAYPOINT_IMG;
}

// Center the map on the given coordinates
function center_map(latitude, longitude) {
  wwd.navigator.lookAtLocation.latitude = latitude;
  wwd.navigator.lookAtLocation.longitude = longitude;
  wwd.navigator.range = altitude_view;
}

// Update coordinate information on left panel
function update_coordinate_panel(latitude, longitude, global_altitude, gps_status_code) {

  document.getElementById('latitude').innerHTML = latitude.toFixed(5);
  document.getElementById('longitude').innerHTML = longitude.toFixed(5);
  document.getElementById('global-altitude').innerHTML = global_altitude.toFixed(2);

  var gps_status = null;
  switch (gps_status_code) {
    case 0:
      gps_status = "FIX";
      break;
    case 1:
      gps_status = "SBAS_FIX";
      break;
    case 2:
      gps_status = "GBAS_FIX";
      break;
    default:
      gps_status = "NO_FIX";
  }

  document.getElementById('gps-status').innerHTML = gps_status;
}

// Update altitude information on left panel
function update_altitude_panel(relative_altitude) {
  document.getElementById('relative-altitude').innerHTML = relative_altitude;
}

// Update battery information on left panel
function update_battery_panel(voltage, current, remaining) {
  document.getElementById('battery-voltage').innerHTML = voltage;
  document.getElementById('battery-current').innerHTML = current;
  document.getElementById('battery-percentage').innerHTML = (remaining * 100).toString() + "%";
}

// Update state information on left panel
function update_state_panel(mode, armed) {
  mode_el = document.getElementById('mode');
  mode_el.innerHTML = mode;
  mode_el.style.color = (mode == "STABILIZE") ? "green" : "orange";

  status_el = document.getElementById('status');
  status_el.innerHTML = (armed) ? "ARMED" : "DISARMED";
  status_el.style.color = (armed) ? "orange" : "green";
}

// Update pitch roll yaw information on left panel
function update_pitchrollyaw(pitch, roll, yaw) {
  document.getElementById('pitch').innerHTML = pitch.toFixed(2);
  document.getElementById('roll').innerHTML = roll.toFixed(2);
  document.getElementById('yaw').innerHTML = yaw.toFixed(2);
}

// Switch flag to center on drone on next location received
function center_on_drone_btn() {
  centerDrone = true;
}

// Show mission already running popup alert
function check_mission_running() {
  // If mission is running show popup
  if (missionRunning) {
    $('#missionRunningModal').modal('show');
  }
}

// Start getting flight data
function flight_data() {

  var loadedPanel = false;

  // Listen to global position topic
  listen_global_position(function(message) {

    // Update coordinate panel
    update_coordinate_panel(message['latitude'], message['longitude'],
     message['altitude'], message['status']['status']);

    // Show flight data panel
    if (loadedPanel == false) {
      document.getElementById("flight-data-panel").style.visibility = "visible";
      document.getElementById("message-box").style.visibility = "visible";
      document.getElementById("message-box-background").style.visibility = "visible";

      // Hide connecting
      document.getElementById('waiting-connection').style.display = "none";
      loadedPanel = true;
    }

    if (mapLoaded == false && loadMapCalled == false) {
      // Load map only first time
      loadMap(message['latitude'], message['longitude']);

      // Check if a mission is already running
      check_mission_running();

    } else if (mapLoaded == true) {
      // If map is loaded, draw drone
      draw_drone(message['latitude'], message['longitude']);
    }

    // Center on drone
    if (centerDrone == true) {
      center_map(message['latitude'], message['longitude']);
      centerDrone = false;
    }
  });

  // Listen to compass topic
  listen_compass(function(angle){
    update_drone_direction(angle);
  });

  // Listen to altitude topic
  listen_relative_altitude(function(data) {
    update_altitude_panel(data);
  });

  // Listen to battery topic
  listen_battery(function(message) {
    update_battery_panel(message['voltage'], message['current'], message['remaining']);
  });

  // Listen to state topic
  listen_state(function(message) {
    update_state_panel(message['mode'], message['armed']);
    missionRunning = message['armed'];
  });

  // Listen to angular velocity topic
  listen_angular_velocity(function(angular_velocity) {
    var roll = angular_velocity['x'];
    var pitch = angular_velocity['y'];
    var yaw = angular_velocity['z'];
    update_pitchrollyaw(pitch, roll, yaw);
  });
}

// Remove 'active' class from tabs
function removeActiveTab() {
  var actives = document.getElementsByClassName("nav-link active");
  for (var l = 0; l < actives.length; l++) {
    actives[l].className = "nav-link";
  }
}

// Switch to plan mission tab
function planMissionTab() {

  // Change active tab
  removeActiveTab();
  document.getElementById("mission-plan-tab").className = "nav-link active";

  // Change panel visibility
  document.getElementById("mission-plan-panel").style.display = "block";
  document.getElementById("flight-data-panel").style.visibility = "hidden";
  document.getElementById("geofence-panel").style.display = "none";
  document.getElementById("import-export-panel").style.display = "none";
  document.getElementById("camera-panel").style.display = "none";

  if (missionRunning == false) {

    if (missionSet == false) {
      // Enable mission planner
      enable_mission_planner();
    }
    disable_geofence_planner();

    // Draw home marker
    draw_home_marker();
  }

  show_mission();
  show_geofence();

  resizeMissionTable();
}

// Switch to flight data tab
function flightDataTab() {
  // Change active tab
  removeActiveTab();
  document.getElementById("flight-data-tab").className = "nav-link active";

  // Change panel visibility
  document.getElementById("flight-data-panel").style.visibility = "visible";
  document.getElementById("mission-plan-panel").style.display = "none";
  document.getElementById("geofence-panel").style.display = "none";
  document.getElementById("import-export-panel").style.display = "none";
  document.getElementById("camera-panel").style.display = "none";

  // Disable mission planner
  disable_mission_planner();
  disable_geofence_planner();

  // Draw drone on next location
  drawDrone = true;
}

// Switch to geofence tab
function geofenceTab() {

  // Change active tab
  removeActiveTab();
  document.getElementById("flight-area-tab").className = "nav-link active";

  // Change panel visibility
  document.getElementById("flight-data-panel").style.visibility = "hidden";
  document.getElementById("geofence-panel").style.display = "block";
  document.getElementById("mission-plan-panel").style.display = "none";
  document.getElementById("import-export-panel").style.display = "none";
  document.getElementById("camera-panel").style.display = "none";

  // Disable mission planner
  disable_mission_planner();

  // Enable flight area click recognizer
  if (geofenceSet == false && missionRunning == false) {
    enable_geofence_planner();
  }

  show_mission();
  show_geofence();

  if (missionRunning == false) {
    // Draw home marker
    draw_home_marker();
  }

  resizeGeofenceTable();
}

// Switch to import export tab
function importExportTab() {
  // Change active tab
  removeActiveTab();
  document.getElementById("import-export-tab").className = "nav-link active";

  // Change panel visibility
  document.getElementById("flight-data-panel").style.visibility = "hidden";
  document.getElementById("geofence-panel").style.display = "none";
  document.getElementById("mission-plan-panel").style.display = "none";
  document.getElementById("import-export-panel").style.display = "block";
  document.getElementById("camera-panel").style.display = "none";

  // Enable export mission button only if mission is set
  if (missionSet == false || missionError == true) {
    document.getElementById("export-mission").disabled = true;
  } else {
    document.getElementById("export-mission").disabled = false;
  }

  // Enable import mission button only if mission is not running
  if (missionRunning == true || missionError == true) {
    document.getElementById("import-mission").disabled = true;
  } else {
    document.getElementById("import-mission").disabled = false;
  }

  disable_mission_planner();
  disable_geofence_planner();
}

// Switch to video tab
function videoTab() {
  // Change active tab
  removeActiveTab();
  document.getElementById("camera-tab").className = "nav-link active";

  // Change panel visibility
  document.getElementById("flight-data-panel").style.visibility = "hidden";
  document.getElementById("geofence-panel").style.display = "none";
  document.getElementById("mission-plan-panel").style.display = "none";
  document.getElementById("import-export-panel").style.display = "none";
  document.getElementById("camera-panel").style.display = "block";
}

// Add waypoint to table
function addWaypointRow(name, latitude, longitude, altitude) {
  var table = document.getElementById("waypoint-table").tBodies[0];

  var row = table.insertRow(table.rows.length);
  var name_cell = row.insertCell(0);
  var lat_cell = row.insertCell(1);
  var lon_cell = row.insertCell(2);
  var alt_cell = row.insertCell(3);

  name_cell.innerHTML = name;
  lat_cell.innerHTML = latitude.toFixed(5);
  lon_cell.innerHTML = longitude.toFixed(5);
  alt_cell.innerHTML = '-';

  // Scroll down
  var wrapper = document.getElementById("mission-table-wrapper");
  wrapper.scrollTop = wrapper.scrollHeight;
}

// Update altitude after setting the mission
function updateWaypointTable(altitude) {
  var table = document.getElementById("waypoint-table").tBodies[0];

  // Update altitude in each row
  for (var i = 0, row; row = table.rows[i]; i++) {
    row.cells[3].innerHTML = altitude;
  }
}

// Add row to geofence table
function addGeofenceRow(count, latitude, longitude) {
  var table = document.getElementById("geofence-table").tBodies[0];

  var row = table.insertRow(table.rows.length);
  var count_cell = row.insertCell(0);
  var lat_cell = row.insertCell(1);
  var lon_cell = row.insertCell(2);

  count_cell.innerHTML = count;
  lat_cell.innerHTML = latitude.toFixed(5);
  lon_cell.innerHTML = longitude.toFixed(5);

  // Scroll down
  var wrapper = document.getElementById("geofence-table-wrapper");
  wrapper.scrollTop = wrapper.scrollHeight;
}

// Clear geofence
function clear_geofence() {
  hide_geofence();

  // Re-init variables
  previousGeoMark = null;
  geofenceMarks = [];
  geofencePaths = [];
  geofenceShape = [];
  geofenceCoordinates = [];
  geofenceSet = false;

  // Disable buttons
  document.getElementById("clear-area").disabled = true;

  // Clear table
  var old_tbody = document.getElementById("geofence-table").tBodies[0];
  var new_tbody = document.createElement('tbody');
  old_tbody.parentNode.replaceChild(new_tbody, old_tbody);

  enable_geofence_planner();
  publish_geofence([]);
}

// Set geofence
function set_geofence() {

  // Draw last path and shape
  draw_geofence_path(geofenceMarks[geofenceMarks.length - 1].position, geofenceMarks[0].position);
  draw_geofence_polygon();

  // Set geofence as set
  disable_geofence_planner();
  geofenceSet = true;

  log_message('[INFO] Geofence set', SUCCESS);
}

// Check mission controller topic and log messages
function check_mission_controller() {
  listen_mission_controller(function(message) {
    var data = JSON.parse(message['data']);

    if (JSON.stringify(data) === JSON.stringify(previousState)) {
      return;
    }

    previousState = data;

    switch (data['state']) {
      case "OK":
        // Everything ok
        if (missionError == true) {
          log_message('[OK] Everything ok', SUCCESS);
        }
        drone_status_ok();
        break;

      case "WARNING":
        // Log warnings
        for (var key in data['warnings']) {
          log_message('[WARNING] ' + data['warnings'][key], WARNING);
        }

        document.getElementById('state').style.color = "orange";
        document.getElementById('state').innerHTML = "WARNING";
        break;

      case "CRITICAL":
        // Log error
        for (var key in data['criticals']) {
          log_message('[CRITICAL] ' + data['criticals'][key], ERROR);
        }
        document.getElementById('canvas-background').style.backgroundColor = "red";
        disable_all_actions();

        document.getElementById('state').style.color = "red";
        document.getElementById('state').innerHTML = "CRITICAL";

        land_and_stop_btn(); // xxx

        break;

      default:
        break;
    }
  });
}

// Check airplanes state
function check_airplanes() {

  // Listen to radar topic
  listen_radar(function(message) {
    var data = JSON.parse(message['data']);

    // Set all airplanes as not seen yet
    for (var k in airplanes) {
      airplanes[k]['seen'] = false;
    }

    // For each plane in message,
    for (var k in data) {
      var plane = data[k];
      draw_airplane(plane['hexident'], plane['desc'], plane['lat'], plane['lon'], plane['heading']);
    }

    // Remove all airplane not seen
    for (var k in airplanes) {
      if (airplanes[k]['seen'] == false) {
        placemarkLayer.removeRenderable(airplanes[k]['mark']);
        delete airplanes[k];
      }
    }
  });
}

// Listen to mission waypoint topic and update currentMission
function check_mission_waypoints() {
  listen_mission_waypoints(function(message) {
    currentMission = message['waypoints'];
  });
}

// Run all listener functions
function restart_all_listeners() {
  flight_data();
  check_airplanes();
  check_mission_controller();
  check_mission_waypoints();
}

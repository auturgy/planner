/********************************************************************
* Copyright (c) 2016 Skysense Inc.
* License: GNU General Public License v3
* Author: Federico Fiorini, Michele Dallachiesa
*********************************************************************/

// Helpful function to sleep for given milliseconds
function sleep (time, callback) {
  setTimeout(callback, time);
};

// Helpful function to wait for condition before proceding with callback
function wait_for(condition, callback) {

  if (!wait_for_active) {
    return;
  }

  if (!condition()) {
    setTimeout(wait_for, 500, condition, callback);
  } else {
    callback();
  }
}

// Helpful function to run callback until condition is reached
function run_until(condition, callback, interval) {

  callback();

  if (!condition()) {
    setTimeout(run_until, interval, condition, callback, interval);
  } else {
    return;
  }
}

// Run a function forever every second
function run_forever(callback) {
  callback();
  setTimeout(run_forever, 1000, callback);
}

// Return a timestamp with the format "m/d/yy h:MM:ss TT"
function timestamp() {

  var now = new Date();
  var date = [ now.getMonth() + 1, now.getDate(), now.getFullYear() ];
  var time = [ now.getHours(), now.getMinutes(), now.getSeconds() ];

  // Determine AM or PM suffix based on the hour
  var suffix = ( time[0] < 12 ) ? "AM" : "PM";

  // If seconds and minutes are less than 10, add a zero
  for ( var i = 1; i < 3; i++ ) {
    if ( time[i] < 10 ) {
      time[i] = "0" + time[i];
    }
  }

  // Return the formatted string
  return date.join("/") + " " + time.join(":");
}

// Download string as file
function download(filename, text) {
  var element = document.createElement('a');
  element.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
  element.setAttribute('download', filename);
  element.style.display = 'none';
  document.body.appendChild(element);
  element.click();
  document.body.removeChild(element);
}

// Log a message in the message-box
function log_message(message, state) {

  switch (state) {
    case SUCCESS:
      var color = "green";
      break;
    case WARNING:
      var color = "orange";
      break;
    case ERROR:
      var color = "red";
      break;
    default:
      var color = "black";
  }

  var messageBox = document.getElementById("message-box");
  if (messageBox == undefined) {
    return;
  }
  var msg = "<span style='color: " + color + "'>" + timestamp() + ": " + message + "</span></br>";
  var content = messageBox.innerHTML;

  messageBox.innerHTML = content + msg;
  messageBox.scrollTop = messageBox.scrollHeight;
}

// Inform user that the drone is not connected and disable all actions
function drone_not_connected() {

  // Set state as critical
  document.getElementById('status').innerHTML = "DISCONNECTED";
  document.getElementById('state').style.color = "red";
  document.getElementById('state').innerHTML = "CRITICAL";
  document.getElementById('mode').style.color = "black";
  document.getElementById('status').style.color = "red";
  document.getElementById('canvas-background').style.backgroundColor = "blue";
  document.getElementById('waiting-connection').innerHTML = "Disconnected.";
  document.getElementById('waiting-connection').style.color = "red";
  document.getElementById('waiting-connection').style.fontSize = "15px";

  disable_all_actions();
}

// Set drone as connected
function drone_status_ok() {
  document.getElementById('canvas-background').style.backgroundColor = "black";
  document.getElementById('state').style.color = "green";
  document.getElementById('state').innerHTML = "OK";

  enable_all_actions();
}

// Helpful function to move an element to another
function move_to(elementId, newParentId) {
  // Get element
  var element = document.getElementById(elementId);

  // Delete from old parent
  var parent = element.parentNode;
  parent.removeChild(element);

  // Append to new parent as first child
  var newParent = document.getElementById(newParentId);
  newParent.insertBefore(element, newParent.firstChild);
}

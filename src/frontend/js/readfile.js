/********************************************************************
* Copyright (c) 2016 Skysense Inc.
* License: GNU General Public License v3
* Author: Federico Fiorini, Michele Dallachiesa
*********************************************************************/

// Read input file into string
function readFile(inputFileID, callback) {

  var file = document.getElementById(inputFileID).files[0];
  var reader = new FileReader();

  reader.onload = function(progressEvent) {
    callback(this.result);
  };
  reader.readAsText(file);
}

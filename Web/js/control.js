// Quadcopter ontroller settings

var maxRoll         = 20;
var maxPitch        = 20;
var yawStabilize    = true;
var yawScale        = 153;         // [0 - 255], 60% * (255 / 100%) = 153
var printPosition   = true;
var yawLimits       = 20;           // [px] Width of the are of the yaw controller where yaw = 0
var yawTrim         = true;
var yawTrimValue    = 20;
var controllerMode  = 'attitude';   // 'attitude' or 'rate'
var fsCounter       = 0;            // Failsafe counter, must increase by 1 every time EX char is sent

// Define byte indexes for joystick data in the EX characteristic
var output =    {
                    throttle        : 0,
                    rollRight       : 1,
                    rollLeft        : 2,
                    pitchForward    : 3,
                    pitchBackward   : 4,
                    yawRight        : 5,
                    yawLeft         : 6,
                    calibrate       : 10,
                    mode            : 11,
                    altitude        : 12,
                    failSafe        : 19
                };


var originalPidData = new Uint8Array(20);
var calibrateCounter = 0;

// Give input elements index corresponding to relevant bytes in characteristics
var inputMap = [    'placeholder',
                    '#roll-slave-p', '#roll-slave-i', '#roll-slave-d',
                    '#pitch-slave-p', '#pitch-slave-i', '#pitch-slave-d',
                    '#yaw-slave-p', '#yaw-slave-i', '#yaw-slave-d',
                    '#roll-master-p', '#roll-master-i', '#roll-master-d',
                    '#pitch-master-p', '#pitch-master-i', '#pitch-master-d',
                    '#yaw-master-p', '#yaw-master-i', '#yaw-master-d'];



/*  Event listeners  */

// Settings menu
addListener("#button-settings", "click", function() {
    $("#settings-container").fadeIn(400);
    $("#settings-overlay").fadeIn(400);
    select("#settings-maxRoll").value = maxRoll;
    select("#settings-maxPitch").value = maxPitch;
    select("#settings-yawScale").value = parseInt((100 * yawScale) / 255);
    select("#settings-yawLimits").value = yawLimits;
});


// 'Save' button
addListener('#button-settings-save', 'click', function() {
    maxRoll = select('#settings-maxRoll').value;
    maxPitch = select('#settings-maxPitch').value;
    yawScale = ((255 * select('#settings-yawScale').value) / 100);
    yawLimits = select('#settings-yawLimits').value;
    $("#settings-container").fadeOut(100);
    $("#settings-overlay").fadeOut(100);

    var mode = exCharVal[output.mode];
    exCharVal[output.throttle] = 0;
    exCharVal[output.mode] = mode;
    writeArrayToChar(exChar, exCharVal);
    reapplyJoystick();
});


// Common actions for settings menu buttons and overlays
$(".button-settings-back, #settings-overlay, #button-send, .overlay-container").click( function() {
    $("#settings-container, #settings-container-pid").fadeOut(100);
    $("#settings-overlay").fadeOut(100);
}).children().click(function(e) {
    return false;
});


// Function for calibrate button
addListener('#button-settings-calibrate', 'click', function() {
    exCharVal[output.calibrate] = calibrateCounter;
    calibrateCounter++;
    var el = select('#button-settings-calibrate');
    writeArrayToChar(exChar, exCharVal)
    .then( () => {
        el.style.cssText = "background-color: rgb(6, 116, 54)"
        var size = el.style.fontSize;
        el.style.fontSize = "12px";
        el.textContent = "Calibrating...";
        setTimeout(function() {
            el.style.cssText = "background-color: rgba(5, 32, 51, 1)"
            el.style.fontSize = size;
            el.textContent = "Calibrate";
        }, 3000);
    });
});

// Controller mode radio buttons
addListener('#controller-mode-attitude', 'click', function() {
    var e = select('#controller-mode-attitude');
    if(controllerMode != "attitude") {
        e.src = "img/checked.png"
        controllerMode = "attitude";
        exCharVal[output.mode] = 0;
        select('#controller-mode-rate').src = "img/unchecked.png";
    }
});

addListener('#controller-mode-rate', 'click', function() {
    var e = select('#controller-mode-rate');
    if(controllerMode != "rate") {
        e.src = "img/checked.png"
        controllerMode = "rate";
        exCharVal[output.mode] = 1;
        select('#controller-mode-attitude').src = "img/unchecked.png";
    }
});

//  Edit PID
addListener("#button-settings-pid", "click", function() {
    $("#settings-container-pid").fadeIn(200, function() {
        inputPidData();
    });
    $("#settings-overlay").fadeIn(200);
});


//  Yaw trim buttons
select(".debug-yaw-trim-input").innerHTML = yawTrimValue;

addListener(".debug-yaw-trim-plus", "touchstart", function() {
    yawTrimValue++;
    select(".debug-yaw-trim-input").innerHTML = yawTrimValue;
});

addListener(".debug-yaw-trim-minus", "touchstart", function() {
    yawTrimValue--;
    select(".debug-yaw-trim-input").innerHTML = yawTrimValue;
});


//  Altitude hold
addListener("#altitude-checkbox", "click", function(event) {
    event.preventDefault();
});

addListener("#altitude-checkbox", "touchstart", function(event) {
    var self = select("#altitude-checkbox");
    self.checked = !self.checked;
    if(self.checked == true)
        exCharVal[output.altitude] = 1;
    else
        exCharVal[output.altitude] = 0;

    writeArrayToChar(exChar, exCharVal);
});



/*  BLE functions  */

// Function called when connection is established and all characteristic
// promises are resolved
function onConnect() {

    // Connect event is fired when connected to the device, but before the
    // GATT server is connected and characteristics discovered. Uses timeout to avoid
    // errors when trying to read from the quadcopter before those steps are finished
    setTimeout(function() {

            // Read all the original PID values from the quadcopter and
            // store to originalPidData array
            readPidData()
            .then( () => {

                // Display battery level read from the quadcopter
                batteryLevel(rxCharVal[0]);
            })
    }, 1000);
}

// Get PID values from quadcopter
function readPidData() {
    return rxChar.readValue()
        .then(originalPid => {

            // Convert from dataView to Uint8Array and save original
            // PID data for possible future reset
            originalPidData = new Uint8Array(originalPid.buffer);
            rxCharVal = txCharVal = originalPidData;
            console.log("Original PID data received:", originalPidData);

            // Write original PID data to input boxes in PID menu
            for(var i = 1; i <= 18; i++) {
                select(inputMap[i]).value = originalPidData[i];
            }
        });
}


//**
//     Joystick, based on the amazing nippleJS by @yoannmoinet: http://yoannmoinet.github.io/nipplejs/
//**

/** Joystick left  **/
var joystickLeft = nipplejs.create({
    zone: select('#joystick-left'),
    mode: 'static',
    position: {left: '100px', top: '40%'},
    color: 'rgb(25, 66, 103)',
    size: 150,
    restOpacity: 0.95
});
var joystickLeftPos = joystickLeft.position;
var joystickSize = joystickLeft.options.size;
var joystickCenter = joystickSize / 2;

// Function for positioning the left joystick correctly at the bottom when released
function reapplyLeft(el) {
    select(el).style.top = "75px";
}

// Apply style to joysticks on load
(function() {
    setTimeout(function() {
        reapplyJoystick();
    }, 1500);
})();

// Doing some changes to the joystick styling on load. This function is used instead
// of making changes to the nippleJS source files. (static CSS applies to some elements, but not all)
function reapplyJoystick() {

    // Set gradient for front on left joystick
    var front_l = select(".collection_0 .front").style;
    var front_r = select(".collection_1 .front").style;
    var back_l = select(".collection_0 .back");

    front_l.background = "radial-gradient(ellipse at center,  rgba(51, 110, 163, 1) 0%, rgba(25, 66, 103, 1) 100%)";
    front_r.background = "radial-gradient(ellipse at center,  rgba(196, 2, 2, 1) 0%, rgba(145, 4, 4, 1) 100%)";
    front_l.opacity = 1;
    front_r.opacity = 1;

    // Insert borders for yaw in left joystick
    var el = document.createElement("div");
    back_l.appendChild(el);
    el.className += "back-limits";
    select('.back-limits').style.width = ((yawLimits * joystickSize) / 100) + 'px';
}

joystickLeft.on('end', function(evt, data) {
    // Executes when joystick is released

    // Set values to zero
    exCharVal[output.throttle] = 0;
    exCharVal[output.yawRight] = 0;
    exCharVal[output.yawLeft] = 0;

    // Try to send data
    if(writePermission) {

        // Setting motor values to zero, repeat until successful
        exWrite('reset')
        .then(response => {
            console.log('Quadcopter stopped: ', response)
        })
        .catch(response => {
            console.log('Stopping failed: ', response);
            console.log('Retrying...');
            exWrite('reset');
        });
    } else {
        setTimeout( function() {
            exWrite('reset');
        }, 100);
    }

    if(printPosition) {

        // Debug
        select('#debug-throttleLeft').innerHTML = '<b>Throttle</b>: 0';
        select('#debug-yawLeft').innerHTML = '<b>Yaw</b>: 0';

        reapplyLeft('.collection_0 > .front');
    }
}).on('move', function(evt, data) {

    // Executes on every new touch event from joystick

    // Throttle is scaled from 0 - 255 to fit into one byte in Uint8Array
    var throttle = parseInt(((data.distance * Math.sin(data.angle.radian) + 75) / joystickSize) * 255);

    // Yaw is defined in pixels, with the joystick offset of 75px taken into account
    // TODO - Make joystick size and offset part of the joystick objects?
    var yaw = data.distance * Math.cos(data.angle.radian) + 75;
    var yawRight = 0;
    var yawLeft = 0;
    var yawLimit = ((joystickSize / 2) / 100) * yawLimits;

    // When yawStabilize is TRUE, a vertical zone in the middle of
    // the left joystick will result in zero yaw
    if(yawStabilize) {

        // Checking if joystick is positioned outside lower and upper limits
        if(yaw <=  (joystickCenter - yawLimit)) {

            // Scale yaw values to the area outside limits
            yaw = ((joystickCenter - yawLimit - yaw) / (joystickCenter - yawLimit));
            yawDir = "left";
            yaw *= yawScale;
            yaw = parseInt(yaw);

            // Checking and applying yaw trim values
            // If the resulting yaw is negative, apply it to opposite side's output
            // and set this side's output to 0
            if(yawTrim) {
                yaw += yawTrimValue;
            }
            if(yaw < 0) {
                yawLeft = 0;
                yawRight = -1 * yaw;
            } else {
                yawLeft = yaw;
                yawRight = 0;
            }
        } else if(yaw >=  (joystickCenter + yawLimit)) {
            yaw = ((joystickCenter - yawLimit ) - (joystickSize - yaw)) / (joystickCenter - yawLimit);
            yawDir = "right";
            yaw *= yawScale;
            yaw = parseInt(yaw);
            if(yawTrim) {
                yaw -= yawTrimValue;
            }
            if(yaw < 0) {
                yaw = yawRight = 0;
                yawLeft = -1 * yaw;
            } else {
                yawRight = yaw;
                yawLeft = 0;
            }
        } else {
            if(yawTrimValue > 0) {
                yaw = yawLeft = yawTrimValue;
                yawRight = 0;
                yawDir = "left";
            } else if(yawTrimValue < 0) {
                yaw = yawRight = -1 * yawTrimValue;
                yawLeft = 0;
                yawDir = "right";
            } else {
                yaw = 0;
                yawDir = "";
            }
        }
    }

    // Storing and sending joystick data with the EX characteristic
    exCharVal[output.throttle] = parseInt(throttle);
    exCharVal[output.yawRight] = parseInt(yawRight);
    exCharVal[output.yawLeft] = parseInt(yawLeft);


    throttle = 100 * throttle / 255;

    // Try to send data
    if(writePermission) {
        exWrite();
    }

    // Debug
    if(printPosition) {
        select('#debug-throttleLeft').innerHTML = '<b>Throttle</b>: ' + throttle.toFixed(0) + '%';
        select('#debug-yawLeft').innerHTML = '<b>Yaw</b>: ' + parseInt(100 * yaw / 255) + '% ' + yawDir;
        //console.log(data);
    }
})

/** Joystick right  **/
var joystickRight = nipplejs.create({
    zone: document.getElementById('joystick-right'),
    mode: 'static',
    position: {right: '-40px', top: '40%'},
    color: 'rgba(196, 2, 2, 1)',
    size: 150,
    restOpacity: 0.9
});

joystickRightPos = joystickRight.position;

joystickRight.on('end', function(evt, data) {

    // Set values to zero
    exCharVal[output.rollRight] = 0;
    exCharVal[output.rollLeft] = 0;
    exCharVal[output.pitchForward] = 0;
    exCharVal[output.pitchBackward] = 0;

    // Try to send data
    if(writePermission) {
        exWrite();
    }

    // console.log(exCharVal);

    if(printPosition) {
        select('#debug-pitchRight').innerHTML = '<b>Pitch</b>: 0';
        select('#debug-rollRight').innerHTML = '<b>Roll</b>: 0';
        //console.log(data);
    }
}).on('move', function(evt, data) {

    // Executes on every new touch event from joystick
    var pitch = ((data.distance * Math.sin(data.angle.radian) + 75) / joystickSize) * 100;
    var roll = ((data.distance * Math.cos(data.angle.radian) + 75) / joystickSize) * 100;
    var rollRight = 0;
    var rollLeft = 0;
    var pitchForward = 0;
    var pitchBackward = 0;

    if(roll < 50){
        roll = maxRoll * (50 - roll) / 50;
        rollDir = "left";
        rollLeft = roll;
    } else if(roll > 50) {
        roll = maxRoll * (roll - 50) / 50;
        rollDir = "right";
        rollRight = roll;
    }

    if(pitch < 50){
        pitch = maxPitch * (50 - pitch) / 50;
        pitchDir = "backward";
        pitchBackward = pitch;
    } else if(pitch > 50) {
        pitch = maxPitch * (pitch - 50) / 50;
        pitchDir = "forward";
        pitchForward = pitch;
    }

    // Storing and sending joystick data with the EX characteristic
    exCharVal[output.rollRight] = parseInt(rollRight);
    exCharVal[output.rollLeft] = parseInt(rollLeft);
    exCharVal[output.pitchForward] = parseInt(pitchForward);
    exCharVal[output.pitchBackward] = parseInt(pitchBackward);

    // Try to send data
    if(writePermission) {
        exWrite();
    }


    // Debug
    if(printPosition) {
        select('#debug-pitchRight').innerHTML = '<b>Pitch</b>: ' + pitch.toFixed(0) + '&deg; '+ pitchDir;
        select('#debug-rollRight').innerHTML = '<b>Roll</b>: ' + roll.toFixed(0)  + '&deg; ' + rollDir;
        //console.log(data);
    }
})

/*  Updating PID values live  */

// Send throttle and PID data to quadcopter
addListener('#button-send', 'click', sendData);

// Send throttle and PID data to quadcopter
addListener('#button-reset', 'click', resetPid);

// Send throttle and PID data to quadcopter
addListener('#button-stop', 'click', stopQuad);


// Function to be called on connect to set input properties
function inputPidData() {

    // Enable input elements
    var inputs = document.getElementsByTagName('input');
    for( var i = 0; i < inputs.length; i++){
        inputs[i].disabled = false;
    }

    setPid(txCharVal);
}

// Function for sending both PID and controller data to quadcopter
function sendData() {

    // Get all PID input data and store to right index of charVal
    for(var i = 1; i <= 18; i++) {
        txCharVal[i] = select(inputMap[i]).value;
    }

    // Sending PID data with txChar over BLE
    writeArrayToChar(txChar, txCharVal)
    .then( () => {
        console.log("PID data sent:", txCharVal);
    })
    .catch( (error) => {
        console.log('PID data sending failed:', error)
    });
}

// Reset PID values to original
function resetPid() {
    for(var i = 1; i <= 18; i++) {
        select(inputMap[i]).value = originalPidData[i];
    }
}

// Set PID values as passed in argument
function setPid(val) {
    for(var i = 1; i <= 18; i++) {
        select(inputMap[i]).value = val[i];
    }
}

// Function to write to the EX characteristic
function exWrite(type = 'normal') {
    failSafe();
    if(type == 'reset')
        exCharVal[output.throttle] = 0;

    return writeArrayToChar(exChar, exCharVal)
    .then( response => {
        // response holds the returned promise
    });
}

function stopQuad() {
    exCharVal[0] = 0;
    writeArrayToChar(exChar, exCharVal)
    .then( () => {
        console.log("Quadcopter stopped. Data sent:", exCharVal);
    })
}

// Fail-safe implementation. Byte [19] in EX char must increase by 1 each sending
function failSafe() {
    fsCounter++;
    exCharVal[output.failSafe] = fsCounter;;
}


// Process battery level and display it
function batteryLevel(value) {
    select('#battery-level-value').textContent = value;
    select('#battery-level-unit').textContent = "%";
    select('#battery-level-container').style.display = "inline-block";
    var el = select('#battery-level-output');
    if(value >= 60)
        el.style.cssText = "background-color: rgb(13, 152, 73)"
    else if((value >= 30) && (value < 60))
        el.style.cssText = "background-color: rgb(251, 173, 0)"
    else
        el.style.cssText = "background-color: rgb(209, 6, 6)"
}

// Roll and pitch parameters should always the same
select('#roll-slave-p').addEventListener('input', function() {
    select('#pitch-slave-p').value = this.value;
});

select('#roll-slave-i').addEventListener('input', function() {
    select('#pitch-slave-i').value = this.value;
});

select('#roll-slave-d').addEventListener('input', function() {
    select('#pitch-slave-d').value = this.value;
});

select('#pitch-slave-p').addEventListener('input', function() {
    select('#roll-slave-p').value = this.value;
});

select('#pitch-slave-i').addEventListener('input', function() {
    select('#roll-slave-i').value = this.value;
});

select('#pitch-slave-d').addEventListener('input', function() {
    select('#roll-slave-d').value = this.value;
});

select('#roll-master-p').addEventListener('input', function() {
    select('#pitch-master-p').value = this.value;
});

select('#roll-master-i').addEventListener('input', function() {
    select('#pitch-master-i').value = this.value;
});

select('#roll-master-d').addEventListener('input', function() {
    select('#pitch-master-d').value = this.value;
});

select('#pitch-master-p').addEventListener('input', function() {
    select('#roll-master-p').value = this.value;
});

select('#pitch-master-i').addEventListener('input', function() {
    select('#roll-master-i').value = this.value;
});

select('#pitch-master-d').addEventListener('input', function() {
    select('#roll-master-d').value = this.value;
});

// END of joystick


    // Give input elements index corresponding to relevant bytes in characteristics
    var inputMap = [    'placeholder',
                        '#roll-slave-p', '#roll-slave-i', '#roll-slave-d',
                        '#pitch-slave-p', '#pitch-slave-i', '#pitch-slave-d',
                        '#yaw-slave-p', '#yaw-slave-i', '#yaw-slave-d',
                        '#roll-master-p', '#roll-master-i', '#roll-master-d',
                        '#pitch-master-p', '#pitch-master-i', '#pitch-master-d',
                        '#yaw-master-p', '#yaw-master-i', '#yaw-master-d'];

    // Define byte indexes for joystick data
    var output =    {
                        throttle : 0,
                        rollRight : 1,
                        rollLeft : 2,
                        pitchForward : 3,
                        pitchBackward : 4,
                        yawRight : 5,
                        yawLeft : 6
                    };

    // Get PID values from quadcopter on connect
    function readPidData() {
        return rxChar.readValue()
            .then(originalPid => {

                // Convert from dataView to Uint8Array and save original PID data for possible reset
                originalPidData = new Uint8Array(originalPid.buffer, 0, 20);
                txCharVal = originalPidData;
                console.log("Original PID data received:", originalPid);

                // Write original PID data to input boxes
                for(var i = 1; i <= 18; i++) {
                    select(inputMap[i]).value = originalPidData[i];
                }
            });
    }

    /** Button actions **/

    // Use connect() to connect to quadcopter
    if (typeof connect == 'function') {
        addListener('#button-connect', 'click', connect);
    }

    // Send throttle and PID data to quadcopter
    addListener('#button-send', 'click', sendData);

    // Send throttle and PID data to quadcopter
    addListener('#button-reset', 'click', resetPid);

    // Send throttle and PID data to quadcopter
    addListener('#button-stop', 'click', stopQuad);


    // Function to be called on connect to set input properties
    function onConnect() {

        // Read original PID values
        // readPidData();

        // Enable input elements
        var inputs = document.getElementsByTagName('input');
        for( var i = 0; i < inputs.length; i++){
            inputs[i].disabled = false;
        }

    }

    // Function for sending both PID and controller data to quadcopter
    function sendData() {

        // Get all PID input data and store to right index of charVal
        for(var i = 1; i <= 18; i++) {
            txCharVal[i] = select(inputMap[i]).value;
        }

        // Get all control input data and store to right index of exCharVal
        exCharVal[output.throttle]      = select('#throttle').value;
        exCharVal[output.rollRight]     = select('#roll-right').value;
        exCharVal[output.rollLeft]      = select('#roll-left').value;
        exCharVal[output.pitchForward]  = select('#pitch-forward').value;
        exCharVal[output.pitchBackward] = select('#pitch-backward').value;
        exCharVal[output.yawRight]      = select('#yaw-right').value;
        exCharVal[output.yawLeft]       = select('#yaw-left').value;

        // Sending PID data with rxChar over BLE
        writeArrayToChar(txChar, txCharVal)
        .then( () => {
            console.log("PID data sent:", txCharVal);

            // Sending throttle data with exChar over BLE
            writeArrayToChar(exChar, exCharVal)
            .then( () => {
                console.log("Controller data sent:", exCharVal);
            })
            .catch( (error) => {
                console.log('Control data sending failed:', error)
            });
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

    function stopQuad() {
        exCharVal[0] = 0;
        writeArrayToChar(exChar, exCharVal)
        .then( () => {
            select('#throttle').value = 0;
            console.log("Quadcopter stopped. Data sent:", exCharVal);
        })
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

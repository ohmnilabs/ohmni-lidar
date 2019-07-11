// 
// Handling responses from
// the a2m8 lidar
// 
// Hoang
// 

// importing libraries
var LidarSerial = require("./lidar_serial");

/* ======= Main Class ======= */
function LidarNode(portstr, detarr) {

    // Choose default portstr if we don't have any input
    if (portstr == null) portstr = "/dev/usb/tty1-7.1";

    // Default detarr if nothing is defined
    if (detarr == null) {
        detarr = [
            {
                direction: "front",
                theta_min: 155,
                theta_max: 205,
                autostop: {dist_warn: 1250, dist_stop: 750, handle: "front_stop"}
            },
            {
                direction: "back",
                theta_min: 345,
                theta_max: 15,
                autostop: {dist_warn: 1350, dist_stop: 850, handle: "back_stop"}
            }
        ]
    }
    this._det_lidar = detarr;

    // Stop handling on obstacle detection, on by default 
    this._collision_stop = true;

    // Front pole location going to be loaded into from calibration
    this._pole_max = 360;
    this._pole_min = 0;

    // Scan angle interval, in bit resolution i.e. 2 means 1 << 2 = 4
    this._ang_interval = 2;

    // Full revolution count, zone dependent in stead of complete resolution now for
    // faster detection time
    this._full_rev_count = 0;
    this._det_lidar.forEach((zone) => {
        let min = zone.theta_min;
        let max = zone.theta_max;
        if (min > max) max += 360;
        this._full_rev_count += ((max - min) >> this._ang_interval);
    });

    // Open the serial port first
    this._serial = new LidarSerial(this, portstr);

    // Array to store the scanned data result
    this._scanned_data = [];

    // Array to store one revolution worth of distance
    this._rev_data = {};

    // Obstacle detection tracking
    this._obstacle_warning = false;
    this._obstacle_stop = false;

    // Obstacle tracking map, clear by default
    this._obstacle_map = new Array(detarr.length).fill(0);
    this._last_obstacle_map = new Array(detarr.length).fill(0);;

    // Speed compensation value
    this._speed_compensation = 0;

    // Wait for the port to open before sending the scan message
    this._serial.on('serial_established', () => {
        console.log("Serial connection for Lidar established");
        this.set_motor_pwm(660);
        // Set motor PWM to default value on serial connection establishment
    });

    // Once we got data, start processing now
    this._serial.on('got_express_scan_data', () => {
        this.process_data();
    });
}

/* ======= Enable EventEmitter ======= */
require('util').inherits(LidarNode, require('events').EventEmitter);

/* ======= Functions ======= */

// Main monitor data monitor function?
LidarNode.prototype.monitor = function() {
    
    // Start getting data here, assuming motor is already spinning
    // from serial port establishment
    this.start_express_scan();
}

// Main data process in charge of appending data into a full revolution
LidarNode.prototype.process_data = function() {

    // Storage space for obstacle data
    this._scanned_data.forEach((cabin) => {

        // Logging for Lidar position test here
        // if (cabin.distance < 400) console.log("Found obstacle at %d degree within distance %d", cabin.angle, cabin.distance);

        // Add to obstacle zone and check against our detarr as well to allow fast obstacle detection
        this.quick_check_obs(parseInt(cabin.angle), cabin.distance);
    });

    // If we have a full resolution, pass it on to see if we can clear
    // console.log("Current length of rev data is", Object.keys(this._rev_data).length);
    if (Object.keys(this._rev_data).length >= this._full_rev_count) {

        // console.log("Got full revolution data length", Object.keys(this._rev_data).length);
        
        // Pass this over to handle
        this.full_check_obs();
    }

    // Handle obs accordingly
    this.handle_obs();

    // Clear the cabins as we don't need them anymore
    this._scanned_data = [];

}

// Quick check for obstacle and add data to zone space for clearing later
LidarNode.prototype.quick_check_obs = function(a, d) {

    // Don't proceed if there's data in this space already
    var shrinked_a = a >> this._ang_interval;
    if (!((this._rev_data[shrinked_a] > d) || (!this._rev_data[shrinked_a]))) return;

    // Iterate through the detarr
    for (let i in this._det_lidar) {

        // Because I'm lazy
        var min = this._det_lidar[i].theta_min;
        var max = this._det_lidar[i].theta_max;

        // Regular case and wrapping case causes a super long boolean condition apparently
        if ( ((min <= max) && (a >= min) && (a <= max)) || ((min > max) && (((a < 360) && (a >= min)) || (a <= max))) ) {
            
            // Save data into our zone space to be cleared later
            this._rev_data[shrinked_a] = d;

            // Will need to clear if there's obstacle within stop zone
            if (this._obstacle_map[i] == 2) continue;

            // Check for obstacle here
            if (d < this._det_lidar[i].autostop.dist_stop) {
                this._obstacle_map[i] = 2;
                this._rev_data = {}; // will need a full revolution data to clear anyway
                // console.log("Found obstacle within stop distance in %s at %d degree %d mm from Lidar with speed compensation value %d. Current obstacle state is %d", this._det_lidar[i].direction, a, d, this._speed_compensation, this._obstacle_map[i]);
            }
            else if (d < this._det_lidar[i].autostop.dist_warn) {
                this._obstacle_map[i] = 1;
                // console.log("Found obstacle within warning distance in %s at %d degree %d mm from Lidar with speed compensation value %d. Current obstacle state is %d", this._det_lidar[i].direction, a, d, this._speed_compensation, this._obstacle_map[i]);
            }
        }
    }
}

// Full revolution obstacle clear
LidarNode.prototype.full_check_obs = function() {

    // Copy data over and clear so we don't miss any data during this check
    var rev = this._rev_data;
    this._rev_data = {};
    var clear = [];

    // Don't need to do anything if there's nothing to clear
    if ((!this._obstacle_map.includes(2)) && (!this._obstacle_map.includes(1))) return;
    // console.log("Currently trying to clear obstacle map, current map is", this._obstacle_map);

    // Okay we need to check for a full revolution now
    for (let i in this._det_lidar) {

        // Can skip if already cleared
        if (this._obstacle_map[i] == 0) continue;

        // Storage variable
        let obs_signal = 0;
        let start = this._det_lidar[i].theta_min >> this._ang_interval;
        let end = this._det_lidar[i].theta_max >> this._ang_interval;
        let dist_warn = this._det_lidar[i].autostop.dist_warn;
        let dist_stop = this._det_lidar[i].autostop.dist_stop;
        let max = this._pole_max >> this._ang_interval;
        let min = this._pole_min >> this._ang_interval;

        // Process data here
        if (start > end) end += max - min;
        let cnt = 0;
        while(cnt <= (end - start)) {
            let k = start + cnt;
            if (k > max) k -= max - min;
            if (rev[k] < dist_stop) {
                // console.log("Found obstacle within stopping distance while clearing obstacle in %s at %d degree %d mm from Lidar with speed compensation value %d", this._det_lidar[i].direction, k << this._ang_interval, rev[k], this._speed_compensation);
                obs_signal = 2;
                break;  
            }
            else if ((obs_signal < 1) && (rev[k] < dist_warn)) {
                // console.log("Found obstacle within warning distance while clearing obstacle in %s at %d degree %d mm from Lidar with speed compensation value %d", this._det_lidar[i].direction, k << this._ang_interval, rev[k], this._speed_compensation);
                obs_signal = 1;
            }
            cnt++;
        }
        // Store data here
        this._obstacle_map[i] = obs_signal;
        // if (obs_signal == 0) console.log("Clear obstacle for %s", this._det_lidar[i].direction);
    }
}

// Handle obstacle here
LidarNode.prototype.handle_obs = function() {

    // Start handling here only when needed
    if (this._collision_stop) {
        for (let i in this._det_lidar) {
            if ((this._obstacle_map[i] != this._last_obstacle_map[i]) && (this._det_lidar[i].autostop.handle != null)) {
                var func = this[this._det_lidar[i].autostop.handle];
                try {
                    func.call(this, this._obstacle_map[i]);
                } catch(err) {
                    console.log("Error handling autostop action:", err);
                }
            }
        }
    }

    // Update obstacle map tracking
    this._last_obstacle_map = this._obstacle_map.slice(0);
    // console.log("Current %s obstacle map and last obstacle map %s updated", this._obstacle_map, this._last_obstacle_map);
}

// Helper function to append distance and angle to data array
LidarNode.prototype.add_data = function(a, d) {

    // If the data is zero, it is probably out of range of 8m that the lidar can see
    if (d == 0) d = 8000;
    
    // Need to do some wrapping for the angle
    if (a > 360) a -= 360;
    if (a < 0) a += 360;

    // With Dan's current mounting position, the pole will be in the 
    // angle range of 355 - 15
    if ((a > this._pole_max) || (a < this._pole_min )) return;
    
    // Everything seems about right now, append the data packet
    var cabin = {
        angle: a,
        distance: d
    };
    this._scanned_data.push(cabin);
}

// Send the start scan signal to the LIDAR
LidarNode.prototype.start_scan = function() {

    // Pass this over to serial instance
    // console.log("Sending out start scan signal");
    this._serial.start_scan();

    // Clear out previous scanned data
    this._scanned_data = [];
}

// Stop everything
LidarNode.prototype.stop = function() {

    // Stop scanning and stop motor
    this.stop_scan();
    this.set_motor_pwm(0);
}

// Send the stop scan signal to the LIDAR
LidarNode.prototype.stop_scan = function() {

    // Pass this over to serial instance
    console.log("Sending out stop scan signal");
    this._serial.stop_scan();

    // Reset data param as well
    this._obstacle_stop = false;
    this._obstacle_warning = false;
    this._scanned_data = [];
    this._rev_data = {};
    this._obstacle_map.fill(0);
    this._last_obstacle_map.fill(0);
}

// Send the hardware reset signal to the LIDAR
LidarNode.prototype.hw_reset = function() {

    // Pass this over to serial instance
    console.log("Sending out hardware reset signal");
    this._serial.hw_reset();
}

// Send the express scan signal to the LIDAR
LidarNode.prototype.start_express_scan = function() {

    // Pass this over to serial instance
    console.log("Sending out express scan signal");
    this._serial.start_express_scan();
}

// Send default turn on PWM to motor
LidarNode.prototype.set_motor_pwm = function(speed) {

    // Pass this over to serial instance
    console.log("Sending out pwm = %d to motor", speed);
    this._serial.set_motor_pwm(speed);
}

// Request device's info
LidarNode.prototype.get_info = function() {

    // Pass this over to serial instance
    console.log("Requesting device info");
    this._serial.get_info();
}

// Request device's health
LidarNode.prototype.get_health = function() {

    // Pass this over to serial instance
    console.log("Requesting device health");
    this._serial.get_health();
}

// Request device's sample rate
LidarNode.prototype.get_sample_rate = function() {

    // Pass this over to serial instance
    console.log("Requesting device sample rate");
    this._serial.get_sample_rate();
}

/* ======= Exports ======= */
module.exports = LidarNode;

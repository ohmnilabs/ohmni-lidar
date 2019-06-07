// 
// Serial port for a2m8 based on 
// android-rtc serial
// 
// Hoang
// 

const SerialPort = require('serialport');

/* ======= LIDAR command flag ======= */
const _cmd = {
    STOP: 0x25, // exit current state and enter idle state
    RESET: 0x40, // reset LIDAR core
    SCAN: 0x20, // enter slow scanning state of 2k sample rate
    EXPRESS_SCAN: 0x82, // highest 4k sample rate scanning
    FORCE_SCAN: 0x21, // ignore rotation speed scanning
    PWM: 0xF0, // PWM signal for the motor
    GET_INFO: 0x50, // request device info
    GET_HEALTH: 0x52, // request device health info
    GET_SAMPLERATE: 0x59 // request sampling time
}

/* ======= Main Class ======= */
function LidarSerial(lidar_node, portstr) {

    // Save the lidar_node class
    this._lidar_node = lidar_node;

    // Save the portname first
    this._portstr = portstr;

    // Stoarge for current port
    this._sport = null;
    this._opened = false;

    // Try and create serial port
    this.create_serial();
}

/* ======= Enable EventEmitter ======= */
require('util').inherits(LidarSerial, require('events').EventEmitter);

/* ======= Main utility functions ======= */

// Creating a serial
LidarSerial.prototype.create_serial = function() {

    // Open the port here, 115200 with 8N1 working mode from a2m8 rplidar datasheet
    console.log("Connecting to", this._portstr);
    this._sport = new SerialPort(this._portstr, {baudRate: 115200});

    // Buffer and amount of data inside buffer for incoming data
    this._buffer = Buffer.alloc(2048);
    this._buflen = 0;
    this._data_type = null;
    this._data_response_len = null;

    // What type of scan type we are going to use
    this._expect_response_cmd = null;

    // What type of respond are we expecting
    this._expect_descriptor = null;

    // Buffer storage space for first package
    this._current_packet = null;

    // Handle opened
    this._sport.on("open", () => {

        // Mark opened
        this._opened = true;
        console.log("Lidar connection established.");

        // Clear DTR flag
        this._sport.set({dtr:false}, (error) => {
            if (error != null) console.log("dtr returned " + error);
            else console.log("DTR flag cleared");
        });
        
        // Notify other process
        this.emit("serial_established");
    });

    // Handle data recieved
    this._sport.on('data', (data) => {
        // console.log("We got data:", data);
        this.handleInput(data);
    });
}

// Reset param and flush serial port after a hardware reset or stop
LidarSerial.prototype.flush_param_reset = function(donecb) {

    // Clear expecting respond cmd
    if (this._expect_response_cmd != null) this._expect_response_cmd = null;
    if (this._expect_descriptor != null) this._expect_descriptor = null;

    // Reset first scan respond packet flag
    if (!this._first_scan_respond) this._first_scan_respond = true;

    // Reset current packet buffer
    if (this._current_packet != null) this._current_packet = null;

    // Flush the serial port
    this._sport.flush((err) => {
        if (err != null) console.log("Flushing serial port failed with error:", err);
        else {
            console.log("Serial port flushed sucessfully");
            if (donecb != null) donecb();
        } 
    })
}

// Helper to get current buffer sliced to current length
LidarSerial.prototype.currentBuffer = function() {
    return this._buffer.slice(0, this._buflen);
}

// Helper to clear out buffer from start to n pos
LidarSerial.prototype.clearBuffer = function(n) {
    this._buffer.copy(this._buffer, 0, n, this._buflen);
    this._buflen -= n;
}

// Handle data coming back to us
LidarSerial.prototype.handleInput = function(data) {

    // First thing first, queue incoming data to our buffer
    data.copy(this._buffer, this._buflen);
    this._buflen += data.length;

    // Now handle descriptor
    if (this._expect_descriptor) this.handle_descriptor();
    
    // Or respond packet
    else
        this.handle_response();
}

// Handle first descriptor packet here
LidarSerial.prototype.handle_descriptor = function() {

    // Get the descriptor buffer, always 7-byte in length
    var buf = this._buffer.slice(0, 7);

    // Check first message beginning
    if ((buf[0] != 0xa5) || (buf[1] != 0x5a)) {

        // Clear this corrupted buffer
        this.clearBuffer(7);
        return;
    }

    // Check for second message beginning
    if (buf[1] != 0x5a) return;

    // Now this is tricky, the next 4 bytes will be processed as 30-bit data response length
    // and 2-bit send mode, then all of these are in little Endian which makes things even 
    // more confusing
    // Little Endian so we need to take the lower 6-bit
    var data_response_len = buf[2] + (buf[3]<<8) + (buf[4]<<16) + ((buf[5] & 0x3F)<<24);

    // Little Endian so we need to take the higher 2-bit
    var send_mode = (buf[5] & 0xC0) >> 6;
    var data_type = buf[buf.length-1];

    console.log("Recieve packet", buf, ", with data response length:", data_response_len.toString(16), ", send_mode:", send_mode.toString(16), ", data type:", data_type.toString(16));

    // Check response accordingly to scan type here
    switch(this._expect_response_cmd) {
        case _cmd.SCAN:
            if ((data_response_len != 5) || (send_mode != 1) || (data_type != 0x81)) { // yeah I don't get this either
                console.log("Got wrong descriptor packet for scan request");
            }
            else console.log("Got scan descriptor packet back");
            break;

        case _cmd.EXPRESS_SCAN:
            if ((data_response_len != 84) || (send_mode != 1) || (data_type != _cmd.EXPRESS_SCAN)) {
                console.log("Got wrong descriptor packet for express scan request");
            }
            else console.log("Got express scan descriptor packet back");
            break;

        case _cmd.GET_INFO:
            if ((data_response_len != 20) || (send_mode != 0) || (data_type != 0x04)) {
                console.log("Got wrong descriptor packet for get info request");
            }
            else console.log("Got get info descriptor packet back");
            break;

        case _cmd.GET_HEALTH:
            if ((data_response_len != 3) || (send_mode != 0) || (data_type != 0x06)) {
                console.log("Got wrong descriptor packet for get health request");
            }
            else console.log('Got get health descriptor packet back');
            break;

        case _cmd.GET_SAMPLERATE:
            if ((data_response_len != 4) || (send_mode != 0) || (data_type != 0x15)) {
                console.log("Got wrong descriptor packet for get samplerate request");
            }
            else console.log('Got sample rate descriptor packet back');
            break;
    }

    // If we get to here, we don't need to expect another descriptor packet until a new one is requested
    this._expect_descriptor = false;

    // Store the message data type for later use
    this._data_type = data_type;
    this._data_response_len = data_response_len;

    // Clear out this buffer now that we are done processing
    this.clearBuffer(7);
}

// Handle corresponding data packets here
LidarSerial.prototype.handle_response = function() {

    // Slice out the buffer we are going to process here
    var buf = this._buffer.slice(0, this._buflen);

    // Keep iterating through the buffer until we get everything
    var clearpos = 0;

    // Debug log
    // console.log("Got %d bytes of data", buf.length);

    // Process according to what we recieved from the descriptor packet
    switch (this._data_type) {
        case 0x81: // normal scan
            break;

        case _cmd.EXPRESS_SCAN:

            // Buffer space debug log
            // console.log("Lidar_serial: Current lidar buffer is:", buf.length);

            // Everything seems correct, proceed to keep processing data until we only have 1 set of 
            // incoming data only. The extra data is required for computing angle
            while(buf.length - clearpos >= this._data_response_len) {
                   
                // Get and check the sync header of the latest incoming response packet
                // console.log("Current offset value is", offset);
                var sync1 = (buf[clearpos + 0] & 0xF0) >> 4;
                var sync2 = (buf[clearpos + 1] & 0xF0) >> 4;
                if ((sync1 != 0xA) && (sync2 != 0x5)) {
                    // console.log("We got the wrong sync header:", sync1.toString(16), sync2.toString(16), ". Checking the next 2 bytes of", buf.length);
                    clearpos += 2;
                    continue;
                }

                // Sync header seems correct, check checksum next
                var rcv_cs = (buf[clearpos + 0] & 0xF) + ((buf[clearpos + 1] & 0xF) << 4);
                var cs = 0;
                for (let i = (clearpos + 2); i < (clearpos + this._data_response_len); i++) cs ^= buf[i];
                if (cs != rcv_cs) {
                    // console.log("Checksum is wrong. Expecting", cs.toString(16), "Got", rcv_cs.toString(16), ". Checking the next 2 bytes of", buf.length);
                    clearpos += 2;
                    continue;
                }

                // Drop the first data response packet as it's not correct anyway
                var start_flag = (buf[clearpos + 3] & 0x40) >> 7;
                if (start_flag) {
                    clearpos += this._data_response_len;
                    continue;  
                } 

                // Current packet is good, need to check if we have space for omega
                // calculation before proceed
                if (this._current_packet == null) {
                    this._current_packet = Buffer.alloc(this._data_response_len);
                    buf.copy(this._current_packet, 0, clearpos, clearpos + this._data_response_len);
                    clearpos += this._data_response_len;

                // Or just continue with calculation and save the next package
                } else {
                    // Everything seems to be correct, proceed to process the data here
                    var current_buf = this._current_packet.slice(0, this._data_response_len);
                    var omega = ((current_buf[2] + ((current_buf[3] & 0x7F) << 8)) / 64.0);
                    var next_omega = ((buf[clearpos + 2] + ((buf[clearpos + 3] & 0x7F) << 8)) / 64.0);

                    // Helper function to compute theta difference for angle compensation
                    var angleDiff = function(w1, w2, i) {
                        if (w1 <= w2) return (((w2 - w1) * ((i - 4)/5) ) / 32.0);
                        else return (((360 + w2 - w1) * ((i - 4)/5) ) / 32.0);
                    }

                    // Iterate throught the cabin for angle and distance value
                    for (var i = 4; i < 16; i+=5) {

                        // Retrieving distance value
                        var d1 = (current_buf[i+1] << 6) + ((current_buf[i] & 0xFC) >> 2);
                        var d2 = (current_buf[i+3] << 6) + ((current_buf[i+2] & 0xFC) >> 2);

                        // Retrieving theta value in 3-bit extra resolution, with MSB as sign but
                        var theta1_q3 = ((current_buf[i] & 0x03) << 4) + (current_buf[i+4] & 0x0F);
                        var theta1 = ((theta1_q3 & 0x1F) / 8.0);
                        if (theta1_q3 >> 5) theta1 = -theta1;
                        var theta2_q3 = ((current_buf[i+2] & 0x03) << 4) + ((current_buf[i+4] & 0xF0) >> 4);
                        var theta2 = ((theta2_q3 & 0x1F) / 8.0);
                        if (theta2_q3 >> 5) theta2 = -theta2;

                        // Push data into a2-node array
                        this._lidar_node.add_data((omega + angleDiff(omega, next_omega, i) - theta1), d1);
                        this._lidar_node.add_data((omega + angleDiff(omega, next_omega, i) - theta2), d2);
                    } 

                    // Update current packet and clean up this data chunk
                    buf.copy(this._current_packet, 0, clearpos, clearpos + this._data_response_len);
                    clearpos += this._data_response_len;
                }
            }

            // Done, signal for process
            this.emit("got_express_scan_data");
            break;

        case 0x04: // get info
            break;

        case 0x06: // get health
            break;

        case 0x15: // get samplerate
            break;
    }

    // Done processing this buffer chunk, clean up
    this.clearBuffer(clearpos);

}

// Writing buffer to serial
LidarSerial.prototype.write = function(buf) {
    
    // Sanity check before we proceed
    if ((this._portstr == null) || (this._sport == null) || (this._opened == false))
        return;

    // Proceed to write to serial port now
    // console.log("Writing buffer out:", buf);
    this._sport.write(buf);
}

// Generate message to send out
LidarSerial.prototype.gen_msg = function(cmd, data) {

    // Getting some data from input buffer
    var data_len = (data != null) ? data.length : 0;
    var buffer_len = (data != null) ? (4 + data_len) : 2;
    var buf = Buffer.alloc(buffer_len);

    // Composing buffer
    buf[0] = 0xA5; // fixed start flag
    buf[1] = cmd; // cmd flag

    // If we are sending msg with
    // no payload, return buffer now
    if (data == null) return buf;

    // Else proceed with what we are doing
    buf[2] = data_len; // payload size in bytes

    // Start copying in data
    if (data != null) data.copy(buf, 3);

    // Compute checksum
    var checksum = 0 ^ buf[0];
    for (var i = 1; i < (buf.length - 1); i++) {
        checksum = checksum ^ buf[i];
    }

    // Add checksum to buffer
    buf[buf.length - 1] = checksum;
    
    // Done
    return buf;
}

/* ======= Protocol functions ======= */

// Start express scan using high sampling rate
LidarSerial.prototype.start_express_scan = function() {

    // Before doing anything, flush out the serial port
    this.flush_param_reset(() => {

        // Save expecting respond cmd
        this._expect_response_cmd = _cmd.EXPRESS_SCAN;
        this._expect_descriptor = true;

        // Message generation
        var msg = [0, 0, 0, 0, 0];

        // Generate and write it out here
        this.write(this.gen_msg(_cmd.EXPRESS_SCAN, Buffer.from(msg)));
    });
}

// Start LIDAR slow mode sampling
LidarSerial.prototype.start_scan = function() {

    // Save expecting respond cmd
    this._expect_response_cmd = _cmd.SCAN;
    this._expect_descriptor = true;

    // Sending out now
    this.write(this.gen_msg(_cmd.SCAN, null));
}

// Stop LIDAR scanning
LidarSerial.prototype.stop_scan = function() {

    // Sending out now
    this.write(this.gen_msg(_cmd.STOP, null));

    // Reset param and flush serial port
    this.flush_param_reset();
}

// Reset LIDAR hardware
LidarSerial.prototype.hw_reset = function() {

    // Sending out now
    this.write(this.gen_msg(_cmd.RESET, null));

    // Reset param and flush serial port
    this.flush_param_reset();
}

// Request device info
LidarSerial.prototype.get_info = function() {

    // Save expecting respond cmd
    this._expect_response_cmd = _cmd.GET_INFO;
    this._expect_descriptor = true;

    // Sending out now
    this.write(this.gen_msg(_cmd.GET_INFO, null));
}

// Request device health
LidarSerial.prototype.get_health = function() {

    // Save expecting respond cmd
    this._expect_response_cmd = _cmd.GET_HEALTH;
    this._expect_descriptor = true;

    // Sending out now
    this.write(this.gen_msg(_cmd.GET_HEALTH, null));
}

// Request sample rate
LidarSerial.prototype.get_sample_rate = function() {

    // Save expecting respond cmd
    this._expect_response_cmd = _cmd.GET_SAMPLERATE;
    this._expect_descriptor = true;

    // Sending out now
    this.write(this.gen_msg(_cmd.GET_SAMPLERATE, null));
}

// Setting motor PWM
LidarSerial.prototype.set_motor_pwm = function(pwm) {

    // Limit the pwm input to 10-bit space
    if (pwm > 1023) pwm = 1023;
    if (pwm < 0) pwm = 0;

    // Message generation
    var msg = [];

    // PWM is sent as 16-bit little-endian
    msg.push(pwm & 0xff);
    msg.push((pwm >> 8) & 0xff);

    // Generate and write data here
    this.write(this.gen_msg(_cmd.PWM, Buffer.from(msg)));
}

/* ======= Export ======= */
module.exports = LidarSerial;

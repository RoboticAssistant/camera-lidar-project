// Auto-generated. Do not edit!

// (in-package create_node.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class BatteryState {
  constructor() {
    this.header = new std_msgs.msg.Header();
    this.temperature = 0;
    this.charge = 0;
    this.capacity = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type BatteryState
    // Serialize message field [header]
    bufferInfo = std_msgs.msg.Header.serialize(obj.header, bufferInfo);
    // Serialize message field [temperature]
    bufferInfo = _serializer.int8(obj.temperature, bufferInfo);
    // Serialize message field [charge]
    bufferInfo = _serializer.uint16(obj.charge, bufferInfo);
    // Serialize message field [capacity]
    bufferInfo = _serializer.uint16(obj.capacity, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type BatteryState
    let tmp;
    let len;
    let data = new BatteryState();
    // Deserialize message field [header]
    tmp = std_msgs.msg.Header.deserialize(buffer);
    data.header = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [temperature]
    tmp = _deserializer.int8(buffer);
    data.temperature = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [charge]
    tmp = _deserializer.uint16(buffer);
    data.charge = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [capacity]
    tmp = _deserializer.uint16(buffer);
    data.capacity = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'create_node/BatteryState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '481447a4e24f212e7b403e4f04aa2ac9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int8 temperature
    uint16 charge
    uint16 capacity
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

};

module.exports = BatteryState;

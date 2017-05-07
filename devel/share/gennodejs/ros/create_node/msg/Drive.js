// Auto-generated. Do not edit!

// (in-package create_node.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class Drive {
  constructor() {
    this.velocity = 0.0;
    this.radius = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type Drive
    // Serialize message field [velocity]
    bufferInfo = _serializer.float32(obj.velocity, bufferInfo);
    // Serialize message field [radius]
    bufferInfo = _serializer.float32(obj.radius, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type Drive
    let tmp;
    let len;
    let data = new Drive();
    // Deserialize message field [velocity]
    tmp = _deserializer.float32(buffer);
    data.velocity = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [radius]
    tmp = _deserializer.float32(buffer);
    data.radius = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'create_node/Drive';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cbd1637d308acca6b24c12a3db426e5e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 velocity
    float32 radius
    
    `;
  }

};

module.exports = Drive;

// Auto-generated. Do not edit!

// (in-package create_node.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class Turtle {
  constructor() {
    this.linear = 0.0;
    this.angular = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type Turtle
    // Serialize message field [linear]
    bufferInfo = _serializer.float32(obj.linear, bufferInfo);
    // Serialize message field [angular]
    bufferInfo = _serializer.float32(obj.angular, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type Turtle
    let tmp;
    let len;
    let data = new Turtle();
    // Deserialize message field [linear]
    tmp = _deserializer.float32(buffer);
    data.linear = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [angular]
    tmp = _deserializer.float32(buffer);
    data.angular = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'create_node/Turtle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9d5c2dcd348ac8f76ce2a4307bd63a13';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 linear
    float32 angular
    `;
  }

};

module.exports = Turtle;

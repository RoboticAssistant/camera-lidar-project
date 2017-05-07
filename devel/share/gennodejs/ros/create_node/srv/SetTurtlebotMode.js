// Auto-generated. Do not edit!

// (in-package create_node.srv)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetTurtlebotModeRequest {
  constructor() {
    this.mode = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type SetTurtlebotModeRequest
    // Serialize message field [mode]
    bufferInfo = _serializer.uint8(obj.mode, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type SetTurtlebotModeRequest
    let tmp;
    let len;
    let data = new SetTurtlebotModeRequest();
    // Deserialize message field [mode]
    tmp = _deserializer.uint8(buffer);
    data.mode = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'create_node/SetTurtlebotModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89b81386720be1cd0ce7a3953fcd1b19';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 mode
    
    `;
  }

};

class SetTurtlebotModeResponse {
  constructor() {
    this.valid_mode = false;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type SetTurtlebotModeResponse
    // Serialize message field [valid_mode]
    bufferInfo = _serializer.bool(obj.valid_mode, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type SetTurtlebotModeResponse
    let tmp;
    let len;
    let data = new SetTurtlebotModeResponse();
    // Deserialize message field [valid_mode]
    tmp = _deserializer.bool(buffer);
    data.valid_mode = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'create_node/SetTurtlebotModeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ef9db56bf4a60ce42049595d58c32b54';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool valid_mode
    
    `;
  }

};

module.exports = {
  Request: SetTurtlebotModeRequest,
  Response: SetTurtlebotModeResponse
};

// Auto-generated. Do not edit!

// (in-package create_node.srv)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetDigitalOutputsRequest {
  constructor() {
    this.digital_out_0 = 0;
    this.digital_out_1 = 0;
    this.digital_out_2 = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type SetDigitalOutputsRequest
    // Serialize message field [digital_out_0]
    bufferInfo = _serializer.uint8(obj.digital_out_0, bufferInfo);
    // Serialize message field [digital_out_1]
    bufferInfo = _serializer.uint8(obj.digital_out_1, bufferInfo);
    // Serialize message field [digital_out_2]
    bufferInfo = _serializer.uint8(obj.digital_out_2, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type SetDigitalOutputsRequest
    let tmp;
    let len;
    let data = new SetDigitalOutputsRequest();
    // Deserialize message field [digital_out_0]
    tmp = _deserializer.uint8(buffer);
    data.digital_out_0 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [digital_out_1]
    tmp = _deserializer.uint8(buffer);
    data.digital_out_1 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [digital_out_2]
    tmp = _deserializer.uint8(buffer);
    data.digital_out_2 = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'create_node/SetDigitalOutputsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '95ef1ce60d04abfe27bea339a6261f29';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 digital_out_0
    uint8 digital_out_1
    uint8 digital_out_2
    
    `;
  }

};

class SetDigitalOutputsResponse {
  constructor() {
    this.done = false;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type SetDigitalOutputsResponse
    // Serialize message field [done]
    bufferInfo = _serializer.bool(obj.done, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type SetDigitalOutputsResponse
    let tmp;
    let len;
    let data = new SetDigitalOutputsResponse();
    // Deserialize message field [done]
    tmp = _deserializer.bool(buffer);
    data.done = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'create_node/SetDigitalOutputsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89bb254424e4cffedbf494e7b0ddbfea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool done
    
    `;
  }

};

module.exports = {
  Request: SetDigitalOutputsRequest,
  Response: SetDigitalOutputsResponse
};

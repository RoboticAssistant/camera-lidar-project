// Auto-generated. Do not edit!

// (in-package create_node.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TurtlebotSensorState {
  constructor() {
    this.header = new std_msgs.msg.Header();
    this.bumps_wheeldrops = 0;
    this.wall = false;
    this.cliff_left = false;
    this.cliff_front_left = false;
    this.cliff_front_right = false;
    this.cliff_right = false;
    this.virtual_wall = false;
    this.motor_overcurrents = 0;
    this.dirt_detector_left = 0;
    this.dirt_detector_right = 0;
    this.remote_opcode = 0;
    this.buttons = 0;
    this.distance = 0.0;
    this.angle = 0.0;
    this.charging_state = 0;
    this.voltage = 0;
    this.current = 0;
    this.temperature = 0;
    this.charge = 0;
    this.capacity = 0;
    this.wall_signal = 0;
    this.cliff_left_signal = 0;
    this.cliff_front_left_signal = 0;
    this.cliff_front_right_signal = 0;
    this.cliff_right_signal = 0;
    this.user_digital_outputs = 0;
    this.user_digital_inputs = 0;
    this.user_analog_input = 0;
    this.charging_sources_available = 0;
    this.oi_mode = 0;
    this.song_number = 0;
    this.song_playing = false;
    this.number_of_stream_packets = 0;
    this.requested_velocity = 0;
    this.requested_radius = 0;
    this.requested_right_velocity = 0;
    this.requested_left_velocity = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type TurtlebotSensorState
    // Serialize message field [header]
    bufferInfo = std_msgs.msg.Header.serialize(obj.header, bufferInfo);
    // Serialize message field [bumps_wheeldrops]
    bufferInfo = _serializer.uint8(obj.bumps_wheeldrops, bufferInfo);
    // Serialize message field [wall]
    bufferInfo = _serializer.bool(obj.wall, bufferInfo);
    // Serialize message field [cliff_left]
    bufferInfo = _serializer.bool(obj.cliff_left, bufferInfo);
    // Serialize message field [cliff_front_left]
    bufferInfo = _serializer.bool(obj.cliff_front_left, bufferInfo);
    // Serialize message field [cliff_front_right]
    bufferInfo = _serializer.bool(obj.cliff_front_right, bufferInfo);
    // Serialize message field [cliff_right]
    bufferInfo = _serializer.bool(obj.cliff_right, bufferInfo);
    // Serialize message field [virtual_wall]
    bufferInfo = _serializer.bool(obj.virtual_wall, bufferInfo);
    // Serialize message field [motor_overcurrents]
    bufferInfo = _serializer.uint8(obj.motor_overcurrents, bufferInfo);
    // Serialize message field [dirt_detector_left]
    bufferInfo = _serializer.uint8(obj.dirt_detector_left, bufferInfo);
    // Serialize message field [dirt_detector_right]
    bufferInfo = _serializer.uint8(obj.dirt_detector_right, bufferInfo);
    // Serialize message field [remote_opcode]
    bufferInfo = _serializer.uint8(obj.remote_opcode, bufferInfo);
    // Serialize message field [buttons]
    bufferInfo = _serializer.uint8(obj.buttons, bufferInfo);
    // Serialize message field [distance]
    bufferInfo = _serializer.float64(obj.distance, bufferInfo);
    // Serialize message field [angle]
    bufferInfo = _serializer.float64(obj.angle, bufferInfo);
    // Serialize message field [charging_state]
    bufferInfo = _serializer.uint8(obj.charging_state, bufferInfo);
    // Serialize message field [voltage]
    bufferInfo = _serializer.uint16(obj.voltage, bufferInfo);
    // Serialize message field [current]
    bufferInfo = _serializer.int16(obj.current, bufferInfo);
    // Serialize message field [temperature]
    bufferInfo = _serializer.int8(obj.temperature, bufferInfo);
    // Serialize message field [charge]
    bufferInfo = _serializer.uint16(obj.charge, bufferInfo);
    // Serialize message field [capacity]
    bufferInfo = _serializer.uint16(obj.capacity, bufferInfo);
    // Serialize message field [wall_signal]
    bufferInfo = _serializer.uint16(obj.wall_signal, bufferInfo);
    // Serialize message field [cliff_left_signal]
    bufferInfo = _serializer.uint16(obj.cliff_left_signal, bufferInfo);
    // Serialize message field [cliff_front_left_signal]
    bufferInfo = _serializer.uint16(obj.cliff_front_left_signal, bufferInfo);
    // Serialize message field [cliff_front_right_signal]
    bufferInfo = _serializer.uint16(obj.cliff_front_right_signal, bufferInfo);
    // Serialize message field [cliff_right_signal]
    bufferInfo = _serializer.uint16(obj.cliff_right_signal, bufferInfo);
    // Serialize message field [user_digital_outputs]
    bufferInfo = _serializer.uint8(obj.user_digital_outputs, bufferInfo);
    // Serialize message field [user_digital_inputs]
    bufferInfo = _serializer.uint8(obj.user_digital_inputs, bufferInfo);
    // Serialize message field [user_analog_input]
    bufferInfo = _serializer.uint16(obj.user_analog_input, bufferInfo);
    // Serialize message field [charging_sources_available]
    bufferInfo = _serializer.uint8(obj.charging_sources_available, bufferInfo);
    // Serialize message field [oi_mode]
    bufferInfo = _serializer.uint8(obj.oi_mode, bufferInfo);
    // Serialize message field [song_number]
    bufferInfo = _serializer.uint8(obj.song_number, bufferInfo);
    // Serialize message field [song_playing]
    bufferInfo = _serializer.bool(obj.song_playing, bufferInfo);
    // Serialize message field [number_of_stream_packets]
    bufferInfo = _serializer.uint8(obj.number_of_stream_packets, bufferInfo);
    // Serialize message field [requested_velocity]
    bufferInfo = _serializer.int32(obj.requested_velocity, bufferInfo);
    // Serialize message field [requested_radius]
    bufferInfo = _serializer.int32(obj.requested_radius, bufferInfo);
    // Serialize message field [requested_right_velocity]
    bufferInfo = _serializer.int32(obj.requested_right_velocity, bufferInfo);
    // Serialize message field [requested_left_velocity]
    bufferInfo = _serializer.int32(obj.requested_left_velocity, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type TurtlebotSensorState
    let tmp;
    let len;
    let data = new TurtlebotSensorState();
    // Deserialize message field [header]
    tmp = std_msgs.msg.Header.deserialize(buffer);
    data.header = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [bumps_wheeldrops]
    tmp = _deserializer.uint8(buffer);
    data.bumps_wheeldrops = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [wall]
    tmp = _deserializer.bool(buffer);
    data.wall = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cliff_left]
    tmp = _deserializer.bool(buffer);
    data.cliff_left = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cliff_front_left]
    tmp = _deserializer.bool(buffer);
    data.cliff_front_left = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cliff_front_right]
    tmp = _deserializer.bool(buffer);
    data.cliff_front_right = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cliff_right]
    tmp = _deserializer.bool(buffer);
    data.cliff_right = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [virtual_wall]
    tmp = _deserializer.bool(buffer);
    data.virtual_wall = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [motor_overcurrents]
    tmp = _deserializer.uint8(buffer);
    data.motor_overcurrents = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dirt_detector_left]
    tmp = _deserializer.uint8(buffer);
    data.dirt_detector_left = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dirt_detector_right]
    tmp = _deserializer.uint8(buffer);
    data.dirt_detector_right = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [remote_opcode]
    tmp = _deserializer.uint8(buffer);
    data.remote_opcode = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [buttons]
    tmp = _deserializer.uint8(buffer);
    data.buttons = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [distance]
    tmp = _deserializer.float64(buffer);
    data.distance = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [angle]
    tmp = _deserializer.float64(buffer);
    data.angle = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [charging_state]
    tmp = _deserializer.uint8(buffer);
    data.charging_state = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [voltage]
    tmp = _deserializer.uint16(buffer);
    data.voltage = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [current]
    tmp = _deserializer.int16(buffer);
    data.current = tmp.data;
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
    // Deserialize message field [wall_signal]
    tmp = _deserializer.uint16(buffer);
    data.wall_signal = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cliff_left_signal]
    tmp = _deserializer.uint16(buffer);
    data.cliff_left_signal = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cliff_front_left_signal]
    tmp = _deserializer.uint16(buffer);
    data.cliff_front_left_signal = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cliff_front_right_signal]
    tmp = _deserializer.uint16(buffer);
    data.cliff_front_right_signal = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cliff_right_signal]
    tmp = _deserializer.uint16(buffer);
    data.cliff_right_signal = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [user_digital_outputs]
    tmp = _deserializer.uint8(buffer);
    data.user_digital_outputs = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [user_digital_inputs]
    tmp = _deserializer.uint8(buffer);
    data.user_digital_inputs = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [user_analog_input]
    tmp = _deserializer.uint16(buffer);
    data.user_analog_input = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [charging_sources_available]
    tmp = _deserializer.uint8(buffer);
    data.charging_sources_available = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [oi_mode]
    tmp = _deserializer.uint8(buffer);
    data.oi_mode = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [song_number]
    tmp = _deserializer.uint8(buffer);
    data.song_number = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [song_playing]
    tmp = _deserializer.bool(buffer);
    data.song_playing = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [number_of_stream_packets]
    tmp = _deserializer.uint8(buffer);
    data.number_of_stream_packets = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [requested_velocity]
    tmp = _deserializer.int32(buffer);
    data.requested_velocity = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [requested_radius]
    tmp = _deserializer.int32(buffer);
    data.requested_radius = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [requested_right_velocity]
    tmp = _deserializer.int32(buffer);
    data.requested_right_velocity = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [requested_left_velocity]
    tmp = _deserializer.int32(buffer);
    data.requested_left_velocity = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'create_node/TurtlebotSensorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8f8ec7fa031fc9cc88e8319cd08a785';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 OI_MODE_OFF = 0
    uint8 OI_MODE_PASSIVE = 1
    uint8 OI_MODE_SAFE = 2
    uint8 OI_MODE_FULL = 3
    
    uint8 REMOTE_LEFT = 129
    uint8 REMOTE_FORWARD = 130 
    uint8 REMOTE_RIGHT = 131 
    uint8 REMOTE_SPOT = 132 
    uint8 REMOTE_MAX = 133 
    uint8 REMOTE_SMALL = 134 
    uint8 REMOTE_MEDIUM = 135 
    uint8 REMOTE_LARGE = 136 
    uint8 REMOTE_CLEAN = 136 
    uint8 REMOTE_PAUSE = 137 
    uint8 REMOTE_POWER = 138 
    uint8 REMOTE_ARC_LEFT = 139 
    uint8 REMOTE_ARC_RIGHT = 140 
    uint8 REMOTE_DRIVE_STOP = 141 
    # Scheduling remote
    uint8 REMOTE_SEND_ALL = 142 
    uint8 REMOTE_SEEK_DOCK = 143 
    # Home base
    uint8 REMOTE_RESERVED = 240 
    uint8 REMOTE_FORCE_FIELD = 242 
    uint8 REMOTE_GREEN_BUOY = 244 
    uint8 REMOTE_GREEN_BUOY_AND_FORCE_FIELD = 246 
    uint8 REMOTE_RED_BUOY = 248 
    uint8 REMOTE_RED_BUOY_AND_FORCE_FIELD = 250 
    uint8 REMOTE_RED_BUOY_AND_GREEN_BUOY = 252 
    uint8 REMOTE_RED_BUOY_AND_GREEN_BUOY_AND_FORCE_FIELD = 254 
    uint8 REMOTE_NONE = 255
    
    uint8 CHARGING_NOT_CHARGING = 0
    uint8 CHARGING_CHARGING_RECOVERY = 1
    uint8 CHARGING_CHARGING = 2
    uint8 CHARGING_TRICKLE_CHARGING = 3
    uint8 CHARGING_WAITING = 4 
    uint8 CHARGING_CHARGING_ERROR = 5
    
    Header header
    
    uint8 bumps_wheeldrops
    bool wall
    bool cliff_left
    bool cliff_front_left
    bool cliff_front_right
    bool cliff_right
    bool virtual_wall
    uint8 motor_overcurrents
    uint8 dirt_detector_left  #roomba_only
    uint8 dirt_detector_right #roomba_only
    uint8 remote_opcode
    uint8 buttons
    float64 distance  # m
    float64 angle #radians
    uint8 charging_state
    uint16 voltage  # mV
    int16 current  # mA
    int8 temperature  # C
    uint16 charge  # mAh
    uint16 capacity  # mAh
    
    uint16 wall_signal
    uint16 cliff_left_signal
    uint16 cliff_front_left_signal
    uint16 cliff_front_right_signal
    uint16 cliff_right_signal
    uint8 user_digital_outputs
    uint8 user_digital_inputs
    uint16 user_analog_input
    uint8 charging_sources_available
    uint8 oi_mode
    uint8 song_number
    bool song_playing
    
    uint8 number_of_stream_packets
    int32 requested_velocity  # m/s
    int32 requested_radius  # m
    int32 requested_right_velocity  # m/s
    int32 requested_left_velocity  # m/s
    
    
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

// Constants for message
TurtlebotSensorState.Constants = {
  OI_MODE_OFF: 0,
  OI_MODE_PASSIVE: 1,
  OI_MODE_SAFE: 2,
  OI_MODE_FULL: 3,
  REMOTE_LEFT: 129,
  REMOTE_FORWARD: 130,
  REMOTE_RIGHT: 131,
  REMOTE_SPOT: 132,
  REMOTE_MAX: 133,
  REMOTE_SMALL: 134,
  REMOTE_MEDIUM: 135,
  REMOTE_LARGE: 136,
  REMOTE_CLEAN: 136,
  REMOTE_PAUSE: 137,
  REMOTE_POWER: 138,
  REMOTE_ARC_LEFT: 139,
  REMOTE_ARC_RIGHT: 140,
  REMOTE_DRIVE_STOP: 141,
  REMOTE_SEND_ALL: 142,
  REMOTE_SEEK_DOCK: 143,
  REMOTE_RESERVED: 240,
  REMOTE_FORCE_FIELD: 242,
  REMOTE_GREEN_BUOY: 244,
  REMOTE_GREEN_BUOY_AND_FORCE_FIELD: 246,
  REMOTE_RED_BUOY: 248,
  REMOTE_RED_BUOY_AND_FORCE_FIELD: 250,
  REMOTE_RED_BUOY_AND_GREEN_BUOY: 252,
  REMOTE_RED_BUOY_AND_GREEN_BUOY_AND_FORCE_FIELD: 254,
  REMOTE_NONE: 255,
  CHARGING_NOT_CHARGING: 0,
  CHARGING_CHARGING_RECOVERY: 1,
  CHARGING_CHARGING: 2,
  CHARGING_TRICKLE_CHARGING: 3,
  CHARGING_WAITING: 4,
  CHARGING_CHARGING_ERROR: 5,
}

module.exports = TurtlebotSensorState;

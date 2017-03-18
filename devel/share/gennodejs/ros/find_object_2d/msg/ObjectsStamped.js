// Auto-generated. Do not edit!

// (in-package find_object_2d.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObjectsStamped {
  constructor() {
    this.header = new std_msgs.msg.Header();
    this.objects = new std_msgs.msg.Float32MultiArray();
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type ObjectsStamped
    // Serialize message field [header]
    bufferInfo = std_msgs.msg.Header.serialize(obj.header, bufferInfo);
    // Serialize message field [objects]
    bufferInfo = std_msgs.msg.Float32MultiArray.serialize(obj.objects, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type ObjectsStamped
    let tmp;
    let len;
    let data = new ObjectsStamped();
    // Deserialize message field [header]
    tmp = std_msgs.msg.Header.deserialize(buffer);
    data.header = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [objects]
    tmp = std_msgs.msg.Float32MultiArray.deserialize(buffer);
    data.objects = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'find_object_2d/ObjectsStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5ec2736b62b92d101276c97e8db387b1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # objects format: 
    # [ObjectId1, objectWidth, objectHeight, h11, h12, h13, h21, h22, h23, h31, h32, h33, ObjectId2...] 
    # where h## is a 3x3 homography matrix (h31 = dx and h32 = dy, see QTransform)
    Header header
    std_msgs/Float32MultiArray objects 
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
    
    ================================================================================
    MSG: std_msgs/Float32MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float32[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

};

module.exports = ObjectsStamped;

// Auto-generated. Do not edit!

// (in-package snake_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class snake_head_rel_pos {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_rel = null;
      this.y_rel = null;
    }
    else {
      if (initObj.hasOwnProperty('x_rel')) {
        this.x_rel = initObj.x_rel
      }
      else {
        this.x_rel = 0.0;
      }
      if (initObj.hasOwnProperty('y_rel')) {
        this.y_rel = initObj.y_rel
      }
      else {
        this.y_rel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type snake_head_rel_pos
    // Serialize message field [x_rel]
    bufferOffset = _serializer.float64(obj.x_rel, buffer, bufferOffset);
    // Serialize message field [y_rel]
    bufferOffset = _serializer.float64(obj.y_rel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type snake_head_rel_pos
    let len;
    let data = new snake_head_rel_pos(null);
    // Deserialize message field [x_rel]
    data.x_rel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_rel]
    data.y_rel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'snake_control/snake_head_rel_pos';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'deda16f7231e7eaf16efd16c6f2840d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x_rel
    float64 y_rel
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new snake_head_rel_pos(null);
    if (msg.x_rel !== undefined) {
      resolved.x_rel = msg.x_rel;
    }
    else {
      resolved.x_rel = 0.0
    }

    if (msg.y_rel !== undefined) {
      resolved.y_rel = msg.y_rel;
    }
    else {
      resolved.y_rel = 0.0
    }

    return resolved;
    }
};

module.exports = snake_head_rel_pos;

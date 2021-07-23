// Auto-generated. Do not edit!

// (in-package task_assembly.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class target {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.congfig_target = null;
    }
    else {
      if (initObj.hasOwnProperty('congfig_target')) {
        this.congfig_target = initObj.congfig_target
      }
      else {
        this.congfig_target = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type target
    // Serialize message field [congfig_target]
    bufferOffset = _arraySerializer.float64(obj.congfig_target, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type target
    let len;
    let data = new target(null);
    // Deserialize message field [congfig_target]
    data.congfig_target = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.congfig_target.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'task_assembly/target';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f926a1e4c3de25050f5c6ca556f561b4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] congfig_target
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new target(null);
    if (msg.congfig_target !== undefined) {
      resolved.congfig_target = msg.congfig_target;
    }
    else {
      resolved.congfig_target = []
    }

    return resolved;
    }
};

module.exports = target;

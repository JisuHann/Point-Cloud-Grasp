// Auto-generated. Do not edit!

// (in-package task_assembly.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

let trajectory_msgs = _finder('trajectory_msgs');

//-----------------------------------------------------------

class door_open_plannerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.current_arm_joint_state = null;
      this.interpolate_path = null;
    }
    else {
      if (initObj.hasOwnProperty('current_arm_joint_state')) {
        this.current_arm_joint_state = initObj.current_arm_joint_state
      }
      else {
        this.current_arm_joint_state = new sensor_msgs.msg.JointState();
      }
      if (initObj.hasOwnProperty('interpolate_path')) {
        this.interpolate_path = initObj.interpolate_path
      }
      else {
        this.interpolate_path = new std_msgs.msg.Bool();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type door_open_plannerRequest
    // Serialize message field [current_arm_joint_state]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.current_arm_joint_state, buffer, bufferOffset);
    // Serialize message field [interpolate_path]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.interpolate_path, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type door_open_plannerRequest
    let len;
    let data = new door_open_plannerRequest(null);
    // Deserialize message field [current_arm_joint_state]
    data.current_arm_joint_state = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    // Deserialize message field [interpolate_path]
    data.interpolate_path = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.JointState.getMessageSize(object.current_arm_joint_state);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'task_assembly/door_open_plannerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b6626cbb81764e0b2868ac23680ff5bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/JointState current_arm_joint_state
    std_msgs/Bool interpolate_path
    
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
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
    string frame_id
    
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new door_open_plannerRequest(null);
    if (msg.current_arm_joint_state !== undefined) {
      resolved.current_arm_joint_state = sensor_msgs.msg.JointState.Resolve(msg.current_arm_joint_state)
    }
    else {
      resolved.current_arm_joint_state = new sensor_msgs.msg.JointState()
    }

    if (msg.interpolate_path !== undefined) {
      resolved.interpolate_path = std_msgs.msg.Bool.Resolve(msg.interpolate_path)
    }
    else {
      resolved.interpolate_path = new std_msgs.msg.Bool()
    }

    return resolved;
    }
};

class door_open_plannerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_trajectory = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_trajectory')) {
        this.joint_trajectory = initObj.joint_trajectory
      }
      else {
        this.joint_trajectory = new trajectory_msgs.msg.JointTrajectory();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type door_open_plannerResponse
    // Serialize message field [joint_trajectory]
    bufferOffset = trajectory_msgs.msg.JointTrajectory.serialize(obj.joint_trajectory, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type door_open_plannerResponse
    let len;
    let data = new door_open_plannerResponse(null);
    // Deserialize message field [joint_trajectory]
    data.joint_trajectory = trajectory_msgs.msg.JointTrajectory.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += trajectory_msgs.msg.JointTrajectory.getMessageSize(object.joint_trajectory);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'task_assembly/door_open_plannerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '00c4170f44c26e68c1de38303b04fb23';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    trajectory_msgs/JointTrajectory joint_trajectory
    
    ================================================================================
    MSG: trajectory_msgs/JointTrajectory
    Header header
    string[] joint_names
    JointTrajectoryPoint[] points
    
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
    string frame_id
    
    ================================================================================
    MSG: trajectory_msgs/JointTrajectoryPoint
    # Each trajectory point specifies either positions[, velocities[, accelerations]]
    # or positions[, effort] for the trajectory to be executed.
    # All specified values are in the same order as the joint names in JointTrajectory.msg
    
    float64[] positions
    float64[] velocities
    float64[] accelerations
    float64[] effort
    duration time_from_start
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new door_open_plannerResponse(null);
    if (msg.joint_trajectory !== undefined) {
      resolved.joint_trajectory = trajectory_msgs.msg.JointTrajectory.Resolve(msg.joint_trajectory)
    }
    else {
      resolved.joint_trajectory = new trajectory_msgs.msg.JointTrajectory()
    }

    return resolved;
    }
};

module.exports = {
  Request: door_open_plannerRequest,
  Response: door_open_plannerResponse,
  md5sum() { return 'b102701686e4ba528d2b5ce877e202a8'; },
  datatype() { return 'task_assembly/door_open_planner'; }
};

// Auto-generated. Do not edit!

// (in-package gpd_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CloudIndexed = require('../msg/CloudIndexed.js');

//-----------------------------------------------------------

let GraspConfigList = require('../msg/GraspConfigList.js');

//-----------------------------------------------------------

class detect_graspsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cloud_indexed = null;
    }
    else {
      if (initObj.hasOwnProperty('cloud_indexed')) {
        this.cloud_indexed = initObj.cloud_indexed
      }
      else {
        this.cloud_indexed = new CloudIndexed();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type detect_graspsRequest
    // Serialize message field [cloud_indexed]
    bufferOffset = CloudIndexed.serialize(obj.cloud_indexed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type detect_graspsRequest
    let len;
    let data = new detect_graspsRequest(null);
    // Deserialize message field [cloud_indexed]
    data.cloud_indexed = CloudIndexed.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CloudIndexed.getMessageSize(object.cloud_indexed);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'gpd_ros/detect_graspsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f5aafbcfa3b48e6d646c19e1a4b3b6f7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    gpd_ros/CloudIndexed cloud_indexed
    
    ================================================================================
    MSG: gpd_ros/CloudIndexed
    # This message holds a point cloud and a list of indices into the point cloud 
    # at which to sample grasp candidates.
    
    # The point cloud.
    gpd_ros/CloudSources cloud_sources
    
    # The indices into the point cloud at which to sample grasp candidates.
    std_msgs/Int64[] indices
    
    ================================================================================
    MSG: gpd_ros/CloudSources
    # This message holds a point cloud that can be a combination of point clouds 
    # from different camera sources (at least one). For each point in the cloud, 
    # this message also stores the index of the camera that produced the point.
    
    # The point cloud.
    sensor_msgs/PointCloud2 cloud
    
    # For each point in the cloud, the index of the camera that acquired the point.
    std_msgs/Int64[] camera_source
    
    # A list of camera positions at which the point cloud was acquired.
    geometry_msgs/Point[] view_points
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
    # The point cloud data may be organized 2d (image-like) or 1d
    # (unordered). Point clouds organized as 2d images may be produced by
    # camera depth sensors such as stereo or time-of-flight.
    
    # Time of sensor data acquisition, and the coordinate frame ID (for 3d
    # points).
    Header header
    
    # 2D structure of the point cloud. If the cloud is unordered, height is
    # 1 and width is the length of the point cloud.
    uint32 height
    uint32 width
    
    # Describes the channels and their layout in the binary data blob.
    PointField[] fields
    
    bool    is_bigendian # Is this data bigendian?
    uint32  point_step   # Length of a point in bytes
    uint32  row_step     # Length of a row in bytes
    uint8[] data         # Actual point data, size is (row_step*height)
    
    bool is_dense        # True if there are no invalid points
    
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
    MSG: sensor_msgs/PointField
    # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    
    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field
    
    ================================================================================
    MSG: std_msgs/Int64
    int64 data
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new detect_graspsRequest(null);
    if (msg.cloud_indexed !== undefined) {
      resolved.cloud_indexed = CloudIndexed.Resolve(msg.cloud_indexed)
    }
    else {
      resolved.cloud_indexed = new CloudIndexed()
    }

    return resolved;
    }
};

class detect_graspsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.grasp_configs = null;
    }
    else {
      if (initObj.hasOwnProperty('grasp_configs')) {
        this.grasp_configs = initObj.grasp_configs
      }
      else {
        this.grasp_configs = new GraspConfigList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type detect_graspsResponse
    // Serialize message field [grasp_configs]
    bufferOffset = GraspConfigList.serialize(obj.grasp_configs, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type detect_graspsResponse
    let len;
    let data = new detect_graspsResponse(null);
    // Deserialize message field [grasp_configs]
    data.grasp_configs = GraspConfigList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += GraspConfigList.getMessageSize(object.grasp_configs);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'gpd_ros/detect_graspsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cdff0a40c5f362fead62ab5c92e4e086';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    gpd_ros/GraspConfigList grasp_configs
    
    
    ================================================================================
    MSG: gpd_ros/GraspConfigList
    # This message stores a list of grasp configurations.
    
    # The time of acquisition, and the coordinate frame ID.
    Header header
    
    # The list of grasp configurations.
    gpd_ros/GraspConfig[] grasps
    
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
    MSG: gpd_ros/GraspConfig
    # This message describes a 2-finger grasp configuration by its 6-DOF pose,
    # consisting of a 3-DOF position and 3-DOF orientation, and the opening
    # width of the robot hand.
    
    # Position
    geometry_msgs/Point position # grasp position (bottom/base center of robot hand)
    
    # Orientation represented as three axis (R = [approach binormal axis])
    geometry_msgs/Vector3 approach # grasp approach direction
    geometry_msgs/Vector3 binormal # hand closing direction
    geometry_msgs/Vector3 axis # hand axis
    
    std_msgs/Float32 width # Required aperture (opening width) of the robot hand
    
    std_msgs/Float32 score # Score assigned to the grasp by the classifier
    
    geometry_msgs/Point sample # point at which the grasp was found
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new detect_graspsResponse(null);
    if (msg.grasp_configs !== undefined) {
      resolved.grasp_configs = GraspConfigList.Resolve(msg.grasp_configs)
    }
    else {
      resolved.grasp_configs = new GraspConfigList()
    }

    return resolved;
    }
};

module.exports = {
  Request: detect_graspsRequest,
  Response: detect_graspsResponse,
  md5sum() { return '6544a7e3669d79f6069fe9d81fee1c1e'; },
  datatype() { return 'gpd_ros/detect_grasps'; }
};

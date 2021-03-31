// Auto-generated. Do not edit!

// (in-package gpd_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CloudSources = require('./CloudSources.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class CloudSamples {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cloud_sources = null;
      this.samples = null;
    }
    else {
      if (initObj.hasOwnProperty('cloud_sources')) {
        this.cloud_sources = initObj.cloud_sources
      }
      else {
        this.cloud_sources = new CloudSources();
      }
      if (initObj.hasOwnProperty('samples')) {
        this.samples = initObj.samples
      }
      else {
        this.samples = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CloudSamples
    // Serialize message field [cloud_sources]
    bufferOffset = CloudSources.serialize(obj.cloud_sources, buffer, bufferOffset);
    // Serialize message field [samples]
    // Serialize the length for message field [samples]
    bufferOffset = _serializer.uint32(obj.samples.length, buffer, bufferOffset);
    obj.samples.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CloudSamples
    let len;
    let data = new CloudSamples(null);
    // Deserialize message field [cloud_sources]
    data.cloud_sources = CloudSources.deserialize(buffer, bufferOffset);
    // Deserialize message field [samples]
    // Deserialize array length for message field [samples]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.samples = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.samples[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CloudSources.getMessageSize(object.cloud_sources);
    length += 24 * object.samples.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gpd_ros/CloudSamples';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7acb8b5070bbe8bcd578eb420ebc4d9b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message holds a point cloud and a list of samples at which the grasp 
    # detector should search for grasp candidates.
    
    # The point cloud.
    gpd_ros/CloudSources cloud_sources
    
    # The samples, as (x,y,z) points, at which to search for grasp candidates. 
    geometry_msgs/Point[] samples
    
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
    const resolved = new CloudSamples(null);
    if (msg.cloud_sources !== undefined) {
      resolved.cloud_sources = CloudSources.Resolve(msg.cloud_sources)
    }
    else {
      resolved.cloud_sources = new CloudSources()
    }

    if (msg.samples !== undefined) {
      resolved.samples = new Array(msg.samples.length);
      for (let i = 0; i < resolved.samples.length; ++i) {
        resolved.samples[i] = geometry_msgs.msg.Point.Resolve(msg.samples[i]);
      }
    }
    else {
      resolved.samples = []
    }

    return resolved;
    }
};

module.exports = CloudSamples;

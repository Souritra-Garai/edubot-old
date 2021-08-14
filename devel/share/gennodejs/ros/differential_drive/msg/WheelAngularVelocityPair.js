// Auto-generated. Do not edit!

// (in-package differential_drive.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class WheelAngularVelocityPair {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wheel_angular_velocity_left = null;
      this.wheel_angular_velocity_right = null;
    }
    else {
      if (initObj.hasOwnProperty('wheel_angular_velocity_left')) {
        this.wheel_angular_velocity_left = initObj.wheel_angular_velocity_left
      }
      else {
        this.wheel_angular_velocity_left = 0.0;
      }
      if (initObj.hasOwnProperty('wheel_angular_velocity_right')) {
        this.wheel_angular_velocity_right = initObj.wheel_angular_velocity_right
      }
      else {
        this.wheel_angular_velocity_right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WheelAngularVelocityPair
    // Serialize message field [wheel_angular_velocity_left]
    bufferOffset = _serializer.float32(obj.wheel_angular_velocity_left, buffer, bufferOffset);
    // Serialize message field [wheel_angular_velocity_right]
    bufferOffset = _serializer.float32(obj.wheel_angular_velocity_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WheelAngularVelocityPair
    let len;
    let data = new WheelAngularVelocityPair(null);
    // Deserialize message field [wheel_angular_velocity_left]
    data.wheel_angular_velocity_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [wheel_angular_velocity_right]
    data.wheel_angular_velocity_right = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'differential_drive/WheelAngularVelocityPair';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'adff221a07855e72470c2f5460fcf2d6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 wheel_angular_velocity_left
    float32 wheel_angular_velocity_right
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WheelAngularVelocityPair(null);
    if (msg.wheel_angular_velocity_left !== undefined) {
      resolved.wheel_angular_velocity_left = msg.wheel_angular_velocity_left;
    }
    else {
      resolved.wheel_angular_velocity_left = 0.0
    }

    if (msg.wheel_angular_velocity_right !== undefined) {
      resolved.wheel_angular_velocity_right = msg.wheel_angular_velocity_right;
    }
    else {
      resolved.wheel_angular_velocity_right = 0.0
    }

    return resolved;
    }
};

module.exports = WheelAngularVelocityPair;

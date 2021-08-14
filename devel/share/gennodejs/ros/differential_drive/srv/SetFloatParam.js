// Auto-generated. Do not edit!

// (in-package differential_drive.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetFloatParamRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.val = null;
    }
    else {
      if (initObj.hasOwnProperty('val')) {
        this.val = initObj.val
      }
      else {
        this.val = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetFloatParamRequest
    // Serialize message field [val]
    bufferOffset = _serializer.float32(obj.val, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetFloatParamRequest
    let len;
    let data = new SetFloatParamRequest(null);
    // Deserialize message field [val]
    data.val = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'differential_drive/SetFloatParamRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c9ee899b5f0899afa6060c9ba611652c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 val
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetFloatParamRequest(null);
    if (msg.val !== undefined) {
      resolved.val = msg.val;
    }
    else {
      resolved.val = 0.0
    }

    return resolved;
    }
};

class SetFloatParamResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetFloatParamResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetFloatParamResponse
    let len;
    let data = new SetFloatParamResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'differential_drive/SetFloatParamResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetFloatParamResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetFloatParamRequest,
  Response: SetFloatParamResponse,
  md5sum() { return 'c9ee899b5f0899afa6060c9ba611652c'; },
  datatype() { return 'differential_drive/SetFloatParam'; }
};

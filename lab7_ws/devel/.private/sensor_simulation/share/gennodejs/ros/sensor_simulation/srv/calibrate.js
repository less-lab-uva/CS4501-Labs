// Auto-generated. Do not edit!

// (in-package sensor_simulation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class calibrateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.zero = null;
    }
    else {
      if (initObj.hasOwnProperty('zero')) {
        this.zero = initObj.zero
      }
      else {
        this.zero = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type calibrateRequest
    // Serialize message field [zero]
    bufferOffset = _serializer.bool(obj.zero, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type calibrateRequest
    let len;
    let data = new calibrateRequest(null);
    // Deserialize message field [zero]
    data.zero = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'sensor_simulation/calibrateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd9ff4c2241c03e6209e4e04727bac371';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool zero
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new calibrateRequest(null);
    if (msg.zero !== undefined) {
      resolved.zero = msg.zero;
    }
    else {
      resolved.zero = false
    }

    return resolved;
    }
};

class calibrateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.baseline = null;
    }
    else {
      if (initObj.hasOwnProperty('baseline')) {
        this.baseline = initObj.baseline
      }
      else {
        this.baseline = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type calibrateResponse
    // Serialize message field [baseline]
    bufferOffset = _serializer.float64(obj.baseline, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type calibrateResponse
    let len;
    let data = new calibrateResponse(null);
    // Deserialize message field [baseline]
    data.baseline = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'sensor_simulation/calibrateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b2d73167cca4d93dc55c783a92c0ef21';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 baseline
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new calibrateResponse(null);
    if (msg.baseline !== undefined) {
      resolved.baseline = msg.baseline;
    }
    else {
      resolved.baseline = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: calibrateRequest,
  Response: calibrateResponse,
  md5sum() { return '75b28b77c2f551fa5005ce2f520330b5'; },
  datatype() { return 'sensor_simulation/calibrate'; }
};

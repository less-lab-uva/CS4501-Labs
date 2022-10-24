// Auto-generated. Do not edit!

// (in-package simple_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class toggle_cageRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cage_on = null;
    }
    else {
      if (initObj.hasOwnProperty('cage_on')) {
        this.cage_on = initObj.cage_on
      }
      else {
        this.cage_on = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type toggle_cageRequest
    // Serialize message field [cage_on]
    bufferOffset = _serializer.bool(obj.cage_on, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type toggle_cageRequest
    let len;
    let data = new toggle_cageRequest(null);
    // Deserialize message field [cage_on]
    data.cage_on = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'simple_control/toggle_cageRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '820030dea9a8caa8250c04fcd974f946';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool cage_on
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new toggle_cageRequest(null);
    if (msg.cage_on !== undefined) {
      resolved.cage_on = msg.cage_on;
    }
    else {
      resolved.cage_on = false
    }

    return resolved;
    }
};

class toggle_cageResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type toggle_cageResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type toggle_cageResponse
    let len;
    let data = new toggle_cageResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'simple_control/toggle_cageResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new toggle_cageResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: toggle_cageRequest,
  Response: toggle_cageResponse,
  md5sum() { return 'e06196d4192ed08fc72383b1573c2db2'; },
  datatype() { return 'simple_control/toggle_cage'; }
};

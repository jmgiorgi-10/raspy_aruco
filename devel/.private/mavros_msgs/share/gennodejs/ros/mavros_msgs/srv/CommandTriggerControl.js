// Auto-generated. Do not edit!

// (in-package mavros_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class CommandTriggerControlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trigger_enable = null;
      this.cycle_time = null;
    }
    else {
      if (initObj.hasOwnProperty('trigger_enable')) {
        this.trigger_enable = initObj.trigger_enable
      }
      else {
        this.trigger_enable = false;
      }
      if (initObj.hasOwnProperty('cycle_time')) {
        this.cycle_time = initObj.cycle_time
      }
      else {
        this.cycle_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommandTriggerControlRequest
    // Serialize message field [trigger_enable]
    bufferOffset = _serializer.bool(obj.trigger_enable, buffer, bufferOffset);
    // Serialize message field [cycle_time]
    bufferOffset = _serializer.float32(obj.cycle_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommandTriggerControlRequest
    let len;
    let data = new CommandTriggerControlRequest(null);
    // Deserialize message field [trigger_enable]
    data.trigger_enable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cycle_time]
    data.cycle_time = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mavros_msgs/CommandTriggerControlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e9392eb8721dabc9986be213994357de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    bool    trigger_enable
    float32 cycle_time
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CommandTriggerControlRequest(null);
    if (msg.trigger_enable !== undefined) {
      resolved.trigger_enable = msg.trigger_enable;
    }
    else {
      resolved.trigger_enable = false
    }

    if (msg.cycle_time !== undefined) {
      resolved.cycle_time = msg.cycle_time;
    }
    else {
      resolved.cycle_time = 0.0
    }

    return resolved;
    }
};

class CommandTriggerControlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommandTriggerControlResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [result]
    bufferOffset = _serializer.uint8(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommandTriggerControlResponse
    let len;
    let data = new CommandTriggerControlResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [result]
    data.result = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mavros_msgs/CommandTriggerControlResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1cd894375e4e3d2861d2222772894fdb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    uint8 result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CommandTriggerControlResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: CommandTriggerControlRequest,
  Response: CommandTriggerControlResponse,
  md5sum() { return '7036c5f362c09a051915015871ce633d'; },
  datatype() { return 'mavros_msgs/CommandTriggerControl'; }
};

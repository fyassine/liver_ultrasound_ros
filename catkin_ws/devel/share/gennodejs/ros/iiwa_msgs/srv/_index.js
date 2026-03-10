
"use strict";

let SetPTPJointSpeedLimits = require('./SetPTPJointSpeedLimits.js')
let ConfigureControlMode = require('./ConfigureControlMode.js')
let TimeToDestination = require('./TimeToDestination.js')
let SetPTPCartesianSpeedLimits = require('./SetPTPCartesianSpeedLimits.js')
let SetSmartServoJointSpeedLimits = require('./SetSmartServoJointSpeedLimits.js')
let SetSmartServoLinSpeedLimits = require('./SetSmartServoLinSpeedLimits.js')
let SetSpeedOverride = require('./SetSpeedOverride.js')
let SetEndpointFrame = require('./SetEndpointFrame.js')
let SetWorkpiece = require('./SetWorkpiece.js')

module.exports = {
  SetPTPJointSpeedLimits: SetPTPJointSpeedLimits,
  ConfigureControlMode: ConfigureControlMode,
  TimeToDestination: TimeToDestination,
  SetPTPCartesianSpeedLimits: SetPTPCartesianSpeedLimits,
  SetSmartServoJointSpeedLimits: SetSmartServoJointSpeedLimits,
  SetSmartServoLinSpeedLimits: SetSmartServoLinSpeedLimits,
  SetSpeedOverride: SetSpeedOverride,
  SetEndpointFrame: SetEndpointFrame,
  SetWorkpiece: SetWorkpiece,
};

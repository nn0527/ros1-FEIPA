
"use strict";

let AttachmentControlCandidate = require('./AttachmentControlCandidate.js');
let AttachmentMechanismStatus = require('./AttachmentMechanismStatus.js');
let ControllerStatus = require('./ControllerStatus.js');
let AttachmentMechanismCommand = require('./AttachmentMechanismCommand.js');
let OperatorCommand = require('./OperatorCommand.js');
let SensorHealth = require('./SensorHealth.js');
let WallPerchStatus = require('./WallPerchStatus.js');
let CeilingAttachmentStatus = require('./CeilingAttachmentStatus.js');
let ControlSetpoint = require('./ControlSetpoint.js');
let SupervisorState = require('./SupervisorState.js');
let BehaviorCommand = require('./BehaviorCommand.js');

module.exports = {
  AttachmentControlCandidate: AttachmentControlCandidate,
  AttachmentMechanismStatus: AttachmentMechanismStatus,
  ControllerStatus: ControllerStatus,
  AttachmentMechanismCommand: AttachmentMechanismCommand,
  OperatorCommand: OperatorCommand,
  SensorHealth: SensorHealth,
  WallPerchStatus: WallPerchStatus,
  CeilingAttachmentStatus: CeilingAttachmentStatus,
  ControlSetpoint: ControlSetpoint,
  SupervisorState: SupervisorState,
  BehaviorCommand: BehaviorCommand,
};

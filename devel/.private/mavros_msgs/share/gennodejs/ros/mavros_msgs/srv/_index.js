
"use strict";

let CommandTriggerControl = require('./CommandTriggerControl.js')
let ParamSet = require('./ParamSet.js')
let SetMode = require('./SetMode.js')
let FileRemove = require('./FileRemove.js')
let CommandBool = require('./CommandBool.js')
let FileRename = require('./FileRename.js')
let CommandInt = require('./CommandInt.js')
let CommandLong = require('./CommandLong.js')
let FileWrite = require('./FileWrite.js')
let FileClose = require('./FileClose.js')
let FileMakeDir = require('./FileMakeDir.js')
let FileRead = require('./FileRead.js')
let StreamRate = require('./StreamRate.js')
let WaypointPull = require('./WaypointPull.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileTruncate = require('./FileTruncate.js')
let FileChecksum = require('./FileChecksum.js')
let ParamPull = require('./ParamPull.js')
let ParamGet = require('./ParamGet.js')
let FileOpen = require('./FileOpen.js')
let WaypointPush = require('./WaypointPush.js')
let CommandHome = require('./CommandHome.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileList = require('./FileList.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let ParamPush = require('./ParamPush.js')
let WaypointClear = require('./WaypointClear.js')
let CommandTOL = require('./CommandTOL.js')

module.exports = {
  CommandTriggerControl: CommandTriggerControl,
  ParamSet: ParamSet,
  SetMode: SetMode,
  FileRemove: FileRemove,
  CommandBool: CommandBool,
  FileRename: FileRename,
  CommandInt: CommandInt,
  CommandLong: CommandLong,
  FileWrite: FileWrite,
  FileClose: FileClose,
  FileMakeDir: FileMakeDir,
  FileRead: FileRead,
  StreamRate: StreamRate,
  WaypointPull: WaypointPull,
  WaypointSetCurrent: WaypointSetCurrent,
  FileTruncate: FileTruncate,
  FileChecksum: FileChecksum,
  ParamPull: ParamPull,
  ParamGet: ParamGet,
  FileOpen: FileOpen,
  WaypointPush: WaypointPush,
  CommandHome: CommandHome,
  SetMavFrame: SetMavFrame,
  FileList: FileList,
  FileRemoveDir: FileRemoveDir,
  ParamPush: ParamPush,
  WaypointClear: WaypointClear,
  CommandTOL: CommandTOL,
};

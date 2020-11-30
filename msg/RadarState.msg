#  ------------------------------------------------------------------------
#
#  Name:       RadarState
#  Id:         0x201
#  Length:     8 bytes
#  Cycle time: 50 ms
#  Senders:    ARS_ISF
#  Layout:
#
#                          Bit
#
#             7   6   5   4   3   2   1   0
#           +---+---+---+---+---+---+---+---+
#         0 |<-x|<-x|   |   |   |   |   |   |
#           +---+---+---+---+---+---+---+---+
#             |   +-- RadarState_NVMReadStatus
#             +-- RadarState_NVMwriteStatus
#           +---+---+---+---+---+---+---+---+
#         1 |<------------------------------|
#           +---+---+---+---+---+---+---+---+
#         2 |------x|<-x|<-x|<-x|<-x|<-x|   |
#           +---+---+---+---+---+---+---+---+
#                 |   |   |   |   |   +-- RadarState_Voltage_Error
#                 |   |   |   |   +-- RadarState_Temporary_Error
#                 |   |   |   +-- RadarState_Temperature_Error
#                 |   |   +-- RadarState_Interference
#                 |   +-- RadarState_Persistent_Error
#                 +-- RadarState_MaxDistanceCfg
#           +---+---+---+---+---+---+---+---+
#     B   3 |   |   |   |   |   |   |<------|
#     y     +---+---+---+---+---+---+---+---+
#     t   4 |--x|<---------x|   |<---------x|
#     e     +---+---+---+---+---+---+---+---+
#             |           |               +-- RadarState_SensorID
#             |           +-- RadarState_SortIndex
#             +-- RadarState_RadarPowerCfg
#           +---+---+---+---+---+---+---+---+
#         5 |<-----x|<-x|<-x|<-----x|<-x|   |
#           +---+---+---+---+---+---+---+---+
#                 |   |   |       |   +-- RadarState_CtrlRelayCfg
#                 |   |   |       +-- RadarState_OutputTypeCfg
#                 |   |   +-- RadarState_SendQualityCfg
#                 |   +-- RadarState_SendExtInfoCfg
#                 +-- RadarState_MotionRxState
#           +---+---+---+---+---+---+---+---+
#         6 |   |   |   |   |   |   |   |   |
#           +---+---+---+---+---+---+---+---+
#         7 |   |   |   |<---------x|   |   |
#           +---+---+---+---+---+---+---+---+
#                                 +-- RadarState_RCS_Threshold
#
#  Signal tree:
#
#    -- {root}
#       +-- RadarState_NVMwriteStatus
#       +-- RadarState_NVMReadStatus
#       +-- RadarState_MaxDistanceCfg
#       +-- RadarState_Persistent_Error
#       +-- RadarState_Interference
#       +-- RadarState_Temperature_Error
#       +-- RadarState_Temporary_Error
#       +-- RadarState_Voltage_Error
#       +-- RadarState_RadarPowerCfg
#       +-- RadarState_SortIndex
#       +-- RadarState_SensorID
#       +-- RadarState_MotionRxState
#       +-- RadarState_SendExtInfoCfg
#       +-- RadarState_SendQualityCfg
#       +-- RadarState_OutputTypeCfg
#       +-- RadarState_CtrlRelayCfg
#       +-- RadarState_RCS_Threshold
#
#  ------------------------------------------------------------------------
#
std_msgs/Header header
std_msgs/UInt8 radarstate_nvmwritestatus
std_msgs/UInt8 radarstate_nvmreadstatus
std_msgs/UInt8 radarstate_maxdistancecfg
std_msgs/UInt8 radarstate_persistent_error
std_msgs/UInt8 radarstate_interference
std_msgs/UInt8 radarstate_temperature_error
std_msgs/UInt8 radarstate_temporary_error
std_msgs/UInt8 radarstate_voltage_error
std_msgs/UInt8 radarstate_radarpowercfg
std_msgs/UInt8 radarstate_sortindex
std_msgs/UInt8 radarstate_sensorid
std_msgs/UInt8 radarstate_motionrxstate
std_msgs/UInt8 radarstate_sendextinfocfg
std_msgs/UInt8 radarstate_sendqualitycfg
std_msgs/UInt8 radarstate_outputtypecfg
std_msgs/UInt8 radarstate_ctrlrelaycfg
std_msgs/UInt8 radarstate_rcs_threshold
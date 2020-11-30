#  ------------------------------------------------------------------------
#
#  Name:       RadarConfiguration
#  Id:         0x200
#  Length:     8 bytes
#  Cycle time: 0 ms
#  Senders:    ExternalUnit
#  Layout:
#
#                          Bit
#
#             7   6   5   4   3   2   1   0
#           +---+---+---+---+---+---+---+---+
#         0 |<-x|<-x|<-x|<-x|<-x|<-x|<-x|<-x|
#           +---+---+---+---+---+---+---+---+
#             |   |   |   |   |   |   |   +-- RadarCfg_MaxDistance_valid
#             |   |   |   |   |   |   +-- RadarCfg_SensorID_valid
#             |   |   |   |   |   +-- RadarCfg_RadarPower_valid
#             |   |   |   |   +-- RadarCfg_OutputType_valid
#             |   |   |   +-- RadarCfg_SendQuality_valid
#             |   |   +-- RadarCfg_SendExtInfo_valid
#             |   +-- RadarCfg_SortIndex_valid
#             +-- RadarCfg_StoreInNVM_valid
#           +---+---+---+---+---+---+---+---+
#         1 |<------------------------------|
#           +---+---+---+---+---+---+---+---+
#         2 |------x|   |   |   |   |   |   |
#           +---+---+---+---+---+---+---+---+
#                 +-- RadarCfg_MaxDistance
#           +---+---+---+---+---+---+---+---+
#         3 |   |   |   |   |   |   |   |   |
#     B     +---+---+---+---+---+---+---+---+
#     y   4 |<---------x|<-----x|<---------x|
#     t     +---+---+---+---+---+---+---+---+
#     e               |       |           +-- RadarCfg_SensorID
#                     |       +-- RadarCfg_OutputType
#                     +-- RadarCfg_RadarPower
#           +---+---+---+---+---+---+---+---+
#         5 |<-x|<---------x|<-x|<-x|<-x|<-x|
#           +---+---+---+---+---+---+---+---+
#             |           |   |   |   |   +-- RadarCfg_CtrlRelay_valid
#             |           |   |   |   +-- RadarCfg_CtrlRelay
#             |           |   |   +-- RadarCfg_SendQuality
#             |           |   +-- RadarCfg_SendExtInfo
#             |           +-- RadarCfg_SortIndex
#             +-- RadarCfg_StoreInNVM
#           +---+---+---+---+---+---+---+---+
#         6 |   |   |   |   |<---------x|<-x|
#           +---+---+---+---+---+---+---+---+
#                                     |   +-- RadarCfg_RCS_Threshold_valid
#                                     +-- RadarCfg_RCS_Threshold
#           +---+---+---+---+---+---+---+---+
#         7 |   |   |   |   |   |   |   |   |
#           +---+---+---+---+---+---+---+---+
#
#  Signal tree:
#
#    -- {root}
#       +-- RadarCfg_StoreInNVM_valid
#       +-- RadarCfg_SortIndex_valid
#       +-- RadarCfg_SendExtInfo_valid
#       +-- RadarCfg_SendQuality_valid
#       +-- RadarCfg_OutputType_valid
#       +-- RadarCfg_RadarPower_valid
#       +-- RadarCfg_SensorID_valid
#       +-- RadarCfg_MaxDistance_valid
#       +-- RadarCfg_MaxDistance
#       +-- RadarCfg_RadarPower
#       +-- RadarCfg_OutputType
#       +-- RadarCfg_SensorID
#       +-- RadarCfg_StoreInNVM
#       +-- RadarCfg_SortIndex
#       +-- RadarCfg_SendExtInfo
#       +-- RadarCfg_SendQuality
#       +-- RadarCfg_CtrlRelay
#       +-- RadarCfg_CtrlRelay_valid
#       +-- RadarCfg_RCS_Threshold
#       +-- RadarCfg_RCS_Threshold_valid
#
#  ------------------------------------------------------------------------
#

std_msgs/Header header
std_msgs/UInt8 radarcfg_storeinnvm_valid
std_msgs/UInt8 radarcfg_sortindex_valid
std_msgs/UInt8 radarcfg_sendextinfo_valid
std_msgs/UInt8 radarcfg_sendquality_valid
std_msgs/UInt8 radarcfg_outputtype_valid
std_msgs/UInt8 radarcfg_radarpower_valid
std_msgs/UInt8 radarcfg_sensorid_valid
std_msgs/UInt8 radarcfg_maxdistance_valid
std_msgs/UInt8 radarcfg_maxdistance
std_msgs/UInt8 radarcfg_radarpower
std_msgs/UInt8 radarcfg_outputtype
std_msgs/UInt8 radarcfg_sensorid
std_msgs/UInt8 radarcfg_storeinnvm
std_msgs/UInt8 radarcfg_sortindex
std_msgs/UInt8 radarcfg_sendextinfo
std_msgs/UInt8 radarcfg_sendquality
std_msgs/UInt8 radarcfg_ctrlrelay
std_msgs/UInt8 radarcfg_ctrlrelay_valid
std_msgs/UInt8 radarcfg_rcs_threshold
std_msgs/UInt8 radarcfg_rcs_threshold_valid
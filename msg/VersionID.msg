#  ------------------------------------------------------------------------
#
#  Name:       VersionID
#  Id:         0x700
#  Length:     4 bytes
#  Cycle time: 0 ms
#  Senders:    ARS_ISF
#  Layout:
#
#                          Bit
#
#             7   6   5   4   3   2   1   0
#           +---+---+---+---+---+---+---+---+
#         0 |<-----------------------------x|
#           +---+---+---+---+---+---+---+---+
#                                         +-- Version_MajorRelease
#           +---+---+---+---+---+---+---+---+
#         1 |<-----------------------------x|
#     B     +---+---+---+---+---+---+---+---+
#     y                                   +-- Version_MinorRelease
#     t     +---+---+---+---+---+---+---+---+
#     e   2 |<-----------------------------x|
#           +---+---+---+---+---+---+---+---+
#                                         +-- Version_PatchLevel
#           +---+---+---+---+---+---+---+---+
#         3 |   |   |   |   |   |   |<-x|<-x|
#           +---+---+---+---+---+---+---+---+
#                                     |   +-- Version_CountryCode
#                                     +-- Version_ExtendedRange
#
#  Signal tree:
#
#    -- {root}
#       +-- Version_MajorRelease
#       +-- Version_MinorRelease
#       +-- Version_PatchLevel
#       +-- Version_ExtendedRange
#       +-- Version_CountryCode
#
#  ------------------------------------------------------------------------
std_msgs/Header header
std_msgs/Int32 version_majorrelease
std_msgs/Int32 version_minorrelease
std_msgs/Int32 version_patchlevel
std_msgs/String version_extendedrange
std_msgs/String version_countrycode
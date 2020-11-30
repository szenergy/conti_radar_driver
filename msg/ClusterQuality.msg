#  ------------------------------------------------------------------------
#
#  Name:       Cluster_2_Quality
#  Id:         0x702
#  Length:     5 bytes
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
#                                         +-- Cluster_ID
#           +---+---+---+---+---+---+---+---+
#         1 |<-----------------x|<----------|
#           +---+---+---+---+---+---+---+---+
#                             +-- Cluster_DistLong_rms
#           +---+---+---+---+---+---+---+---+
#     B   2 |------x|<-----------------x|<--|
#     y     +---+---+---+---+---+---+---+---+
#     t           |                   +-- Cluster_VrelLong_rms
#     e           +-- Cluster_DistLat_rms
#           +---+---+---+---+---+---+---+---+
#         3 |--------------x|   |<---------x|
#           +---+---+---+---+---+---+---+---+
#                         |               +-- Cluster_PdH0
#                         +-- Cluster_VrelLat_rms
#           +---+---+---+---+---+---+---+---+
#         4 |<-----------------x|<---------x|
#           +---+---+---+---+---+---+---+---+
#                             |           +-- Cluster_AmbigState
#                             +-- Cluster_InvalidState
#
#  Signal tree:
#
#    -- {root}
#       +-- Cluster_ID
#       +-- Cluster_DistLong_rms
#       +-- Cluster_DistLat_rms
#       +-- Cluster_VrelLong_rms
#       +-- Cluster_VrelLat_rms
#       +-- Cluster_PdH0
#       +-- Cluster_InvalidState
#       +-- Cluster_AmbigState
#
#  ------------------------------------------------------------------------
#
std_msgs/Float64 cluster_distlong_rms
std_msgs/Float64 cluster_distlat_rms
std_msgs/Float64 cluster_vrellong_rms
std_msgs/Float64 cluster_vrellat_rms
std_msgs/String cluster_pdh0
std_msgs/String cluster_invalidstate
std_msgs/String cluster_ambigstate
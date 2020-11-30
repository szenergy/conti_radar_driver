#  ------------------------------------------------------------------------
#
#  Name:       Cluster_0_Status
#  Id:         0x600
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
#                                         +-- Cluster_NofClustersNear
#           +---+---+---+---+---+---+---+---+
#         1 |<-----------------------------x|
#           +---+---+---+---+---+---+---+---+
#     B                                   +-- Cluster_NofClustersFar
#     y     +---+---+---+---+---+---+---+---+
#     t   2 |<------------------------------|
#     e     +---+---+---+---+---+---+---+---+
#         3 |------------------------------x|
#           +---+---+---+---+---+---+---+---+
#                                         +-- Cluster_MeasCounter
#           +---+---+---+---+---+---+---+---+
#         4 |<-------------x|   |   |   |   |
#           +---+---+---+---+---+---+---+---+
#                         +-- Cluster_InterfaceVersion
#
#  Signal tree:
#
#    -- {root}
#       +-- Cluster_NofClustersNear
#       +-- Cluster_NofClustersFar
#       +-- Cluster_MeasCounter
#       +-- Cluster_InterfaceVersion
#
#  ------------------------------------------------------------------------
#
std_msgs/Int32 cluster_nofclustersnear
std_msgs/Int32 cluster_nofclustersfar
std_msgs/Int32 cluster_meascounter
std_msgs/Int32 cluster_interfaceversion
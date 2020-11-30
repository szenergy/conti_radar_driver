std_msgs/Header header

# Number of filter status messages for configured cluster filter
#
# possible values 0-6
# 0: no cluster filter is set
# 1-6: cluster filters

std_msgs/UInt8 filterstate_nofclusterfiltercfg

# Number of filter status messages for configured object filter
#
# possible values 0-16
# 0: no object filter is set
# 1-16: object filters

std_msgs/UInt8 filterstate_nofobjectfiltercfg

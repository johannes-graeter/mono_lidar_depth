# Separate header for compatibility with current TimeSynchronizer.
std_msgs/Header header

# Data field Tracklet, with flag if is inlier or not
TrackletWithOutlierFlag[] tracks

# save timestamps corresponding to each match:
# to get stamp corresponding to match number i in tracklet take timestamp i in stamps
# stamps size is equal to the size of the longest tracklet
time[] stamps

# no header since shall not be transmitted individually
# Data field Tracklet
FeaturePoint[] feature_points

# flag if this tracklet is an outlier or not, default is false
bool is_outlier

# field for saving error, f.e. loss from optimization, the higher the value the higher the probability that it is an outlier, typically this is a relative value to the other errors
float32 error

# id given by tracker
uint64 id

# age of the tracklet
uint16 age

# label for the tracklet, LUT has to be provided by the user
int16 label

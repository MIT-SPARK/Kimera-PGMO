# This message encodes a relative SE(3) measurement

# Used for timestamp
std_msgs/Header header

# ID of first node
uint32 from

# ID of second node
uint32 to

# relative transformation
geometry_msgs/Pose transform
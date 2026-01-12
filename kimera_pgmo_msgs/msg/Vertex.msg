# vertex position
geometry_msgs/Point pos

# whether or not the vertex has color
bool has_color false
# vertex color (optional)
std_msgs/ColorRGBA color

# whether or not the vertex has a timestamp
bool has_stamp false
# vertex stamps (optional)
uint64 stamp 0

# whether or not the vertex has a label
bool has_label false
# vertex semantic label
uint32 label 0

# whether or not the vertex has a first seen timestamp
bool has_first_seen_stamp false
# timestamp the vertex was first seen (optional)
uint64 first_seen_stamp 0

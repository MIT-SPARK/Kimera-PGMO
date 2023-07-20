# This message encodes Triangle mesh message

std_msgs/Header header
geometry_msgs/Point[] vertex_updates
std_msgs/ColorRGBA[] vertex_updates_colors
uint64[] stamp_updates
uint32[] semantic_updates
TriangleIndices[] face_updates
TriangleIndices[] face_archive_updates
uint64[] deleted_indices
uint64[] prev_indices
uint64[] curr_indices
uint64 vertex_start
uint64 face_start

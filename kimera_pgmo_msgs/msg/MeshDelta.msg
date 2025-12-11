# This message encodes the information necessary to apply an update to a mesh
std_msgs/Header header

# sequence number of the delta from the compression algorithm
uint16 seq_number
# number of active vertices in the previous delta
uint64 prev_active_vertices
# number of active faces in the previous delta
uint64 prev_active_faces

# number of archived vertices
uint64 num_archived_vertices
# remapping information (previous index)
uint64[] previous_indices
# remapping information (current index)
uint64[] current_indices

# vertex positions
geometry_msgs/Point[] vertex_updates
# vertex colors (optional)
std_msgs/ColorRGBA[] color_updates
# vertex stamps (optional)
uint64[] stamp_updates
# vertex labels (optional)
uint32[] label_updates
# vertex first seen stamps (optional)
uint64[] first_seen_stamp_updates

# active face indices
kimera_pgmo_msgs/TriangleIndices[] face_updates
# archived face indices
kimera_pgmo_msgs/TriangleIndices[] face_archive_updates

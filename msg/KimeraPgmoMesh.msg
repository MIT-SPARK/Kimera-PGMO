# This message encodes Triangle mesh message

std_msgs/Header header
int32 id
TriangleIndices[] triangles # list of triangles; the index values refer to positions in vertices (and vertex_normals, if given)
geometry_msgs/Point[] vertices # the actual vertices that make up the mesh
time[] vertex_stamps # time stamps associated to each of the vertices
int32[] vertex_indices # index of mesh graph vertex associated to each of the vertices
std_msgs/ColorRGBA[] vertex_colors
std_msgs/Header header
# namespace of the mesh used to identify the corresponding robot and visual. Separating hierarchical namespaces with '/' is supported (e.g. robot0/mesh123)
string ns
# the actual vertices that make up the mesh
kimera_pgmo_msgs/Vertex[] vertices
# list of triangles; the index values refer to positions in vertices
kimera_pgmo_msgs/TriangleIndices[] triangles
# optional control point indices for every vertex
int32[] graph_indices

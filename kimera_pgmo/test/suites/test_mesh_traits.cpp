#include <gtest/gtest.h>

#include "kimera_pgmo/mesh_traits.h"
#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

TEST(MeshTraits, TraitsCorrect) {
  std::vector<traits::Vertex> points;
  EXPECT_EQ(traits::num_vertices(points), 0);
  traits::resize_vertices(points, 5);
  for (uint32_t i = 0; i < 5; ++i) {
    traits::Vertex p{traits::Pos(i, i + 1, i + 2),
                     {{true, true, true, true},
                      traits::Color{static_cast<uint8_t>(i + 3),
                                    static_cast<uint8_t>(i + 4),
                                    static_cast<uint8_t>(i + 5),
                                    static_cast<uint8_t>(i + 6)},
                      i + 7,
                      i + 8,
                      i + 9}};
    traits::set_vertex(points, i, p.pos, &p.traits);

    traits::VertexTraits vertex_traits;
    const auto p_result = traits::get_vertex(points, i, &vertex_traits);
    EXPECT_EQ(vertex_traits, p.traits);
    EXPECT_EQ(p_result, p.pos);
  }

  std::vector<traits::Face> faces;
  EXPECT_EQ(traits::num_faces(faces), 0);
  traits::resize_faces(faces, 5);
  traits::Face f{1, 2, 3};
  for (size_t i = 0; i < 5; ++i) {
    traits::set_face(faces, i, offsetFace(f, i));
  }

  EXPECT_EQ(traits::num_faces(faces), 5);
  for (size_t i = 0; i < 5; ++i) {
    const traits::Face expected{i + 1, i + 2, i + 3};
    EXPECT_EQ(traits::get_face(faces, i), expected);
  }
}

}  // namespace kimera_pgmo

/**
 * @file   DeformationSolver.h
 * @brief  Deforms mesh based on lates optimized trajectory
 * @author Yun Chang
 */

#pragma once

#include "mesher_mapper/CommonStructs.h"

namespace mesher_mapper {

  class DeformationSolver {
  public:
    DeformationSolver();
    ~DeformationSolver();

    bool solve(const DeformationGraph& graph,
               const NodePoses& fixed_nodes,
               DeformationGraph* optimized_graph);

   private:
    bool GaussNewton();
  };
}  // namespace mesher_mapper

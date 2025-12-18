// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCore>
#include <memory>

#include "odb/db.h"
#include "placerBase.h"
#include "utl/Logger.h"

namespace utl {
class Logger;
}

namespace gpl {

struct ResidualError
{
  float x;  // The relative residual error for X
  float y;  // The relative residual error for Y
};

using Eigen::BiCGSTAB;
using Eigen::IdentityPreconditioner;

using SMatrix = Eigen::SparseMatrix<float, Eigen::RowMajor>;

#ifdef ENABLE_GPU
ResidualError cudaSparseSolve(int iter,
                              SMatrix& placeInstForceMatrixX,
                              Eigen::VectorXf& fixedInstForceVecX,
                              Eigen::VectorXf& instLocVecX,
                              SMatrix& placeInstForceMatrixY,
                              Eigen::VectorXf& fixedInstForceVecY,
                              Eigen::VectorXf& instLocVecY,
                              utl::Logger* logger);
#endif

ResidualError cpuSparseSolve(int maxSolverIter,
                             int iter,
                             SMatrix& placeInstForceMatrixX,
                             Eigen::VectorXf& fixedInstForceVecX,
                             Eigen::VectorXf& instLocVecX,
                             SMatrix& placeInstForceMatrixY,
                             Eigen::VectorXf& fixedInstForceVecY,
                             Eigen::VectorXf& instLocVecY,
                             utl::Logger* logger,
                             int threads);
}  // namespace gpl

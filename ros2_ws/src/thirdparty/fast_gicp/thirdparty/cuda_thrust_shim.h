#pragma once
// Resolves thrust::pair ambiguity with std::pair under CUDA 12.x + Eigen
#define THRUST_IGNORE_CUB_VERSION_CHECK
#include <thrust/detail/config.h>
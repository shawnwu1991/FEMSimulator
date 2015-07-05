#pragma once

#include "SIMD_SVD\SIMD_SVD.h"

inline void DecomposeRotation(const double *F, double *R, double *S)
{
    computePolarDecompositionWithSVD(R, S, F);
}
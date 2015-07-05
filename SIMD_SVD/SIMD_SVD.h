#pragma once

#define rsqrt(x) 1/sqrt(x)
#define max(a, b) a>=b ? a:b

#define USE_SCALAR_IMPLEMENTATION
//#define USE_SSE_IMPLEMENTATION
//#define USE_AVX_IMPLEMENTATION

#define COMPUTE_V_AS_MATRIX
#define COMPUTE_V_AS_QUATERNION
#define COMPUTE_U_AS_MATRIX
#include "Singular_Value_Decomposition_Preamble.hpp"

void computeSVD(const double *sa,
									double *su,
									double *sv,
									double *sigma)
{
	double a11, a21, a31, a12, a22, a32, a13, a23, a33;
	double u11, u21, u31, u12, u22, u32, u13, u23, u33;
	double v11, v21, v31, v12, v22, v32, v13, v23, v33;
	double sigma1, sigma2, sigma3;

	a11 = sa[0];
	a12 = sa[1];
	a13 = sa[2];
	a21 = sa[3];
	a22 = sa[4];
	a23 = sa[5];
	a31 = sa[6];
	a32 = sa[7];
	a33 = sa[8];

	#include "Singular_Value_Decomposition_Kernel_Declarations.hpp"

    ENABLE_SCALAR_IMPLEMENTATION(Sa11.f=a11;)                                      ENABLE_SSE_IMPLEMENTATION(Va11=_mm_loadu_ps(a11);)                                  ENABLE_AVX_IMPLEMENTATION(Va11=_mm256_loadu_ps(a11);)
    ENABLE_SCALAR_IMPLEMENTATION(Sa21.f=a21;)                                      ENABLE_SSE_IMPLEMENTATION(Va21=_mm_loadu_ps(a21);)                                  ENABLE_AVX_IMPLEMENTATION(Va21=_mm256_loadu_ps(a21);)
    ENABLE_SCALAR_IMPLEMENTATION(Sa31.f=a31;)                                      ENABLE_SSE_IMPLEMENTATION(Va31=_mm_loadu_ps(a31);)                                  ENABLE_AVX_IMPLEMENTATION(Va31=_mm256_loadu_ps(a31);)
    ENABLE_SCALAR_IMPLEMENTATION(Sa12.f=a12;)                                      ENABLE_SSE_IMPLEMENTATION(Va12=_mm_loadu_ps(a12);)                                  ENABLE_AVX_IMPLEMENTATION(Va12=_mm256_loadu_ps(a12);)
    ENABLE_SCALAR_IMPLEMENTATION(Sa22.f=a22;)                                      ENABLE_SSE_IMPLEMENTATION(Va22=_mm_loadu_ps(a22);)                                  ENABLE_AVX_IMPLEMENTATION(Va22=_mm256_loadu_ps(a22);)
    ENABLE_SCALAR_IMPLEMENTATION(Sa32.f=a32;)                                      ENABLE_SSE_IMPLEMENTATION(Va32=_mm_loadu_ps(a32);)                                  ENABLE_AVX_IMPLEMENTATION(Va32=_mm256_loadu_ps(a32);)
    ENABLE_SCALAR_IMPLEMENTATION(Sa13.f=a13;)                                      ENABLE_SSE_IMPLEMENTATION(Va13=_mm_loadu_ps(a13);)                                  ENABLE_AVX_IMPLEMENTATION(Va13=_mm256_loadu_ps(a13);)
    ENABLE_SCALAR_IMPLEMENTATION(Sa23.f=a23;)                                      ENABLE_SSE_IMPLEMENTATION(Va23=_mm_loadu_ps(a23);)                                  ENABLE_AVX_IMPLEMENTATION(Va23=_mm256_loadu_ps(a23);)
    ENABLE_SCALAR_IMPLEMENTATION(Sa33.f=a33;)                                      ENABLE_SSE_IMPLEMENTATION(Va33=_mm_loadu_ps(a33);)                                  ENABLE_AVX_IMPLEMENTATION(Va33=_mm256_loadu_ps(a33);)

#include "Singular_Value_Decomposition_Main_Kernel_Body.hpp"

    ENABLE_SCALAR_IMPLEMENTATION(u11=Su11.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(u11,Vu11);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(u11,Vu11);)
    ENABLE_SCALAR_IMPLEMENTATION(u21=Su21.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(u21,Vu21);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(u21,Vu21);)
    ENABLE_SCALAR_IMPLEMENTATION(u31=Su31.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(u31,Vu31);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(u31,Vu31);)
    ENABLE_SCALAR_IMPLEMENTATION(u12=Su12.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(u12,Vu12);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(u12,Vu12);)
    ENABLE_SCALAR_IMPLEMENTATION(u22=Su22.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(u22,Vu22);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(u22,Vu22);)
    ENABLE_SCALAR_IMPLEMENTATION(u32=Su32.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(u32,Vu32);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(u32,Vu32);)
    ENABLE_SCALAR_IMPLEMENTATION(u13=Su13.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(u13,Vu13);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(u13,Vu13);)
    ENABLE_SCALAR_IMPLEMENTATION(u23=Su23.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(u23,Vu23);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(u23,Vu23);)
    ENABLE_SCALAR_IMPLEMENTATION(u33=Su33.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(u33,Vu33);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(u33,Vu33);)

    ENABLE_SCALAR_IMPLEMENTATION(v11=Sv11.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(v11,Vv11);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(v11,Vv11);)
    ENABLE_SCALAR_IMPLEMENTATION(v21=Sv21.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(v21,Vv21);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(v21,Vv21);)
    ENABLE_SCALAR_IMPLEMENTATION(v31=Sv31.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(v31,Vv31);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(v31,Vv31);)
    ENABLE_SCALAR_IMPLEMENTATION(v12=Sv12.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(v12,Vv12);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(v12,Vv12);)
    ENABLE_SCALAR_IMPLEMENTATION(v22=Sv22.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(v22,Vv22);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(v22,Vv22);)
    ENABLE_SCALAR_IMPLEMENTATION(v32=Sv32.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(v32,Vv32);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(v32,Vv32);)
    ENABLE_SCALAR_IMPLEMENTATION(v13=Sv13.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(v13,Vv13);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(v13,Vv13);)
    ENABLE_SCALAR_IMPLEMENTATION(v23=Sv23.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(v23,Vv23);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(v23,Vv23);)
    ENABLE_SCALAR_IMPLEMENTATION(v33=Sv33.f;)                                      ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(v33,Vv33);)                                 ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(v33,Vv33);)

    ENABLE_SCALAR_IMPLEMENTATION(sigma1=Sa11.f;)                                   ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(sigma1,Va11);)                              ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(sigma1,Va11);)
    ENABLE_SCALAR_IMPLEMENTATION(sigma2=Sa22.f;)                                   ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(sigma2,Va22);)                              ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(sigma2,Va22);)
    ENABLE_SCALAR_IMPLEMENTATION(sigma3=Sa33.f;)                                   ENABLE_SSE_IMPLEMENTATION(_mm_storeu_ps(sigma3,Va33);)                              ENABLE_AVX_IMPLEMENTATION(_mm256_storeu_ps(sigma3,Va33);)

	su[0] = u11;
	su[1] = u12;
	su[2] = u13;
	su[3] = u21;
	su[4] = u22;
	su[5] = u23;
	su[6] = u31;
	su[7] = u32;
	su[8] = u33;

	sv[0] = v11;
	sv[1] = v12;
	sv[2] = v13;
	sv[3] = v21;
	sv[4] = v22;
	sv[5] = v23;
	sv[6] = v31;
	sv[7] = v32;
	sv[8] = v33;

	sigma[0] = sigma1;
	sigma[1] = sigma2;
	sigma[2] = sigma3;
}

void TransposemulMatrix3X3(double *mc, const double *ma, const double *mb)
{
	mc[0] = ma[0]*mb[0] + ma[3]*mb[3] + ma[6]*mb[6];
	mc[1] = ma[0]*mb[1] + ma[3]*mb[4] + ma[6]*mb[7];
	mc[2] = ma[0]*mb[2] + ma[3]*mb[5] + ma[6]*mb[8];
	mc[3] = ma[1]*mb[0] + ma[4]*mb[3] + ma[7]*mb[6];
	mc[4] = ma[1]*mb[1] + ma[4]*mb[4] + ma[7]*mb[7];
	mc[5] = ma[1]*mb[2] + ma[4]*mb[5] + ma[7]*mb[8];
	mc[6] = ma[2]*mb[0] + ma[5]*mb[3] + ma[8]*mb[6];
	mc[7] = ma[2]*mb[1] + ma[5]*mb[4] + ma[8]*mb[7];
	mc[8] = ma[2]*mb[2] + ma[5]*mb[5] + ma[8]*mb[8];
}

// mc[3][3] = ma[3][3]^T * mb[3][3]
void computePolarDecompositionWithSVD(double *R, double *S, const double *F)
{
	double sa[9], su[9], sv[9], sigma[3];
	for(int i=0; i<9; i++)
		sa[i] = F[i];
	computeSVD(sa, su, sv, sigma);

	R[0] = su[0] * sv[0] + su[1] * sv[1] + su[2] * sv[2];
	R[1] = su[0] * sv[3] + su[1] * sv[4] + su[2] * sv[5];
	R[2] = su[0] * sv[6] + su[1] * sv[7] + su[2] * sv[8];

	R[3] = su[3] * sv[0] + su[4] * sv[1] + su[5] * sv[2];
	R[4] = su[3] * sv[3] + su[4] * sv[4] + su[5] * sv[5];
	R[5] = su[3] * sv[6] + su[4] * sv[7] + su[5] * sv[8];

	R[6] = su[6] * sv[0] + su[7] * sv[1] + su[8] * sv[2];
	R[7] = su[6] * sv[3] + su[7] * sv[4] + su[8] * sv[5];
	R[8] = su[6] * sv[6] + su[7] * sv[7] + su[8] * sv[8];

    if (NULL != S)
        TransposemulMatrix3X3(S, R, F);
}



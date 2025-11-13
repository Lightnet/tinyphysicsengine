
# Information:
  Thinking and plan to toggle int to float build test. As it need toggle between int and float.

```c

#include <math.h>  // Needed for float version

#ifndef TPE_USE_FLOAT
  #define TPE_USE_FLOAT 0  // 0 = int (default), 1 = float
#endif

#if TPE_USE_FLOAT

  typedef float TPE_Unit;
  typedef float TPE_UnitReduced;

  #define TPE_F                1.0f
  #define TPE_INFINITY         INFINITY
  #define TPE_JOINT_SIZE_MULTIPLIER 1.0f

  #define TPE_sin(x)           sinf((x) * 2.0f * (float)M_PI)
  #define TPE_cos(x)           cosf((x) * 2.0f * (float)M_PI)
  #define TPE_atan(x)          (atanf(x) / (2.0f * (float)M_PI))
  #define TPE_sqrt(x)          sqrtf(x)

  #define TPE_DISTANCE(p1,p2)  TPE_vec3Len(TPE_vec3Minus(p1,p2))
  #define TPE_LENGTH(v)        TPE_vec3Len(v)

#else

  typedef int32_t TPE_Unit;
  typedef int16_t TPE_UnitReduced;

  #define TPE_FRACTIONS_PER_UNIT 512
  #define TPE_F TPE_FRACTIONS_PER_UNIT
  #define TPE_INFINITY         2147483647
  #define TPE_JOINT_SIZE_MULTIPLIER 32

  // Keep original int functions
  TPE_Unit TPE_sin(TPE_Unit x);
  TPE_Unit TPE_cos(TPE_Unit x);
  TPE_Unit TPE_atan(TPE_Unit x);
  TPE_Unit TPE_sqrt(TPE_Unit x);

  #if !TPE_APPROXIMATE_LENGTH
    #define TPE_DISTANCE TPE_dist
    #define TPE_LENGTH TPE_vec3Len
  #else
    #define TPE_DISTANCE TPE_distApprox
    #define TPE_LENGTH TPE_vec3LenApprox
  #endif

#endif


```
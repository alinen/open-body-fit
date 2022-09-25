#ifndef ATK_GLMMATH_H_
#define ATK_GLMMATH_H_

#include <glm/vec2.hpp>               // vec2, bvec2, dvec2, ivec2 and uvec2
#include <glm/vec3.hpp>               // vec3, bvec3, dvec3, ivec3 and uvec3
#include <glm/vec4.hpp>               // vec4, bvec4, dvec4, ivec4 and uvec4
#include <glm/mat3x3.hpp>             // mat3, dmat3
#include <glm/mat4x4.hpp>             // mat4, dmat4
// all the GLSL common functions: abs, min, mix, isnan, fma, etc.
#include <glm/common.hpp>             
// all the GLSL exponential functions: pow, log, exp2, sqrt, etc.
#include <glm/exponential.hpp>        
// all the GLSL geometry functions: dot, cross, reflect, etc.
#include <glm/geometric.hpp>          
// all the GLSL matrix functions: transpose, inverse, etc.
#include <glm/matrix.hpp>             
// all the GLSL trigonometric functions: radians, cos, asin, etc.
#include <glm/trigonometric.hpp>      
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>   // euler angles
#include <glm/gtx/string_cast.hpp>   // euler angles
#include <glm/gtc/matrix_transform.hpp>
#include <glm/ext/scalar_constants.hpp>

namespace atk
{ 
  static const glm::vec3 Zero3(0.0);
  static const glm::vec3 AxisX(1.0, 0.0, 0.0);
  static const glm::vec3 AxisY(0.0, 1.0, 0.0);
  static const glm::vec3 AxisZ(0.0, 0.0, 1.0);
  static const glm::mat3 Zero3x3(0.0);
  static const glm::mat3 Identity3x3(1.0);
  static const glm::mat4 Identity4x4(1.0);
  static const glm::quat IdentityQ(1.0, 0.0, 0.0, 0.0);

  /**
   * @brief Euler angle rotation orders
   */
  enum RotOrder { XYZ, XZY, YXZ, YZX, ZXY, ZYX};

  /**
   * @brief Return a rotation matrix corresponding to the given 
   * rotation order (RO). 
   * @param roo The rotation order (e.g. XYZ, XZY, ...)
   * @param xyz The euler angles. The order is always xyz, in radians
   */
  inline glm::mat3 eulerAngleRO(RotOrder roo, const glm::vec3& xyz) 
  {
    using namespace glm;
    if (roo == XYZ) return mat3(eulerAngleXYZ(xyz[0], xyz[1], xyz[2]));
    if (roo == XZY) return mat3(eulerAngleXZY(xyz[0], xyz[2], xyz[1]));
    if (roo == YXZ) return mat3(eulerAngleYXZ(xyz[1], xyz[0], xyz[2]));
    if (roo == YZX) return mat3(eulerAngleYZX(xyz[1], xyz[2], xyz[0]));
    if (roo == ZXY) return mat3(eulerAngleZXY(xyz[2], xyz[0], xyz[1]));
    if (roo == ZYX) return mat3(eulerAngleZYX(xyz[2], xyz[1], xyz[0]));

    assert(false); // shouldn't get here: invalid roo passed to this function
    return glm::mat3(1.0);
  }

  /**
   * @brief Return the euler angles from the given matrix, with the given 
   * rotation order (RO). 
   * @param roo The rotation order (e.g. XYZ, XZY, ...)
   * @param m The matrix from which to extract the euler angles
   */
  inline glm::vec3 extractEulerAngleRO(RotOrder roo, const glm::mat3& m)
  {
    using namespace glm;
    vec3 xyz(0.0);
    if      (roo == XYZ) extractEulerAngleXYZ(mat4(m), xyz[0], xyz[1], xyz[2]);
    else if (roo == XZY) extractEulerAngleXZY(mat4(m), xyz[0], xyz[2], xyz[1]);
    else if (roo == YXZ) extractEulerAngleYXZ(mat4(m), xyz[1], xyz[0], xyz[2]);
    else if (roo == YZX) extractEulerAngleYZX(mat4(m), xyz[1], xyz[2], xyz[0]);
    else if (roo == ZXY) extractEulerAngleZXY(mat4(m), xyz[2], xyz[0], xyz[1]);
    else if (roo == ZYX) extractEulerAngleZYX(mat4(m), xyz[2], xyz[1], xyz[0]);
    else assert(false); // shouldn't get here

    return xyz;
  }

  /**
   * @brief Return the euler angles from the given quaternion, with the given 
   * rotation order (RO). 
   * @param roo The rotation order (e.g. XYZ, XZY, ...)
   * @param q The quaternion from which to extract the euler angles
   */
  inline glm::vec3 extractEulerAngleRO(RotOrder roo, const glm::quat& q)
  {
    return extractEulerAngleRO(roo, glm::mat3(q));
  }

  inline glm::mat3 angleAxisMat3(float angle, const glm::vec3& axis)
  {
    return glm::toMat3(glm::angleAxis(angle, axis));
  }

  inline void extractAngleAxisMat3(const glm::mat3& m, 
    float& angle, glm::vec3& axis)
  {
    glm::quat q(m); 
    angle = glm::angle(q);
    axis = glm::axis(q); 
  }
}
#endif

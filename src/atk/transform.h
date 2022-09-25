#ifndef ATK_TRANSFORM_H_
#define ATK_TRANSFORM_H_

#include "atk/glmmath.h" 
#include <iostream>

namespace atk {
class Transform
{
public:
   Transform();
   Transform(
      const glm::quat& rotation, 
      const glm::vec3& translation, 
      const glm::vec3& scale = glm::vec3(1.0));
   Transform(const Transform& transform);
   Transform& operator = (const Transform& source); 

   Transform inverse() const;
   glm::vec3 transformPoint(const glm::vec3& pos) const;
   glm::vec3 transformVector(const glm::vec3& dir) const;

   glm::quat r() const;	
   glm::vec3 t() const;	
   glm::vec3 s() const;	
   glm::mat4 matrix() const; //corresponds to TRS 

   void setR(const glm::quat& q);	
   void setT(const glm::vec3& t);	
   void setS(const glm::vec3& s);	

   friend Transform operator * (const Transform& a, const Transform& b); // t1 * t2
   friend std::ostream& operator << (std::ostream& s, const Transform& v); // print

   static Transform Scale(const glm::vec3& s);
   static Transform Scale(float s);
   static Transform Rot(float angle, const glm::vec3& axis);
   static Transform Rot(const glm::quat& q);
   static Transform Translate(const glm::vec3& pos);
   static const Transform Identity;

private:
   glm::quat _rotation;
   glm::vec3 _translation;
   glm::vec3 _scale;
};

typedef Transform trs;
}

#endif


#include "atk/pose.h"
using namespace glm;

namespace atk {

Pose::Pose() : rootPos(vec3(0)) {}
Pose::~Pose() {}

Pose::Pose(const Pose& p)
{
  deepCopy(p);
}

void Pose::deepCopy(const Pose& p)
{
  rootPos = p.rootPos;
  jointRots.clear();
  for (unsigned int i = 0; i < p.jointRots.size(); i++)
  {
    jointRots.push_back(p.jointRots[i]);
  }
}

Pose& Pose::operator = (const Pose& p)
{
  if (this != &p)
  {
    deepCopy(p);
  }

  return *this;
}

std::ostream& operator<<(std::ostream& s, const Pose& v)
{
  s << "pose(" << std::endl;
  s << glm::to_string(v.rootPos) << std::endl; 
  for (unsigned int i = 0; i < v.jointRots.size(); i++)
  {
    s << glm::to_string(v.jointRots[i]) << std::endl;
  }
  s << ")\n";
  return s;
}

Pose Pose::Lerp(const Pose& key1, const Pose& key2, float u)
{
  Pose pose;
  pose.rootPos = key1.rootPos * (1.0f-u) + key2.rootPos * u;
  for (int i = 0; i < (int) key1.jointRots.size(); i++)
  {
    glm::quat q1 = key1.jointRots[i];
    glm::quat q2 = key2.jointRots[i];
    glm::quat q = glm::slerp(q1, q2, u);
    pose.jointRots.push_back(q);
  }
  return pose;
}

quat qExp(const quat& q)
{
  // q = A*(x*i+y*j+z*k) where (x,y,z) is unit length
  // exp(q) = cos(A)+sin(A)*(x*i+y*j+z*k)
  double angle = sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  double sn, cs;
  sn = sin(angle);
  cs = cos(angle);

  // When A is near zero, sin(A)/A is approximately 1.  Use
  // exp(q) = cos(A)+A*(x*i+y*j+z*k)
  double coeff = ( abs(sn) < 0.000001 ? 1.0f : sn/angle );
  quat result(cs, coeff * q.x, coeff * q.y, coeff * q.z);

  return result;
}

quat qLog(const quat& q)
{
  // q = cos(A)+sin(A)*(x*i+y*j+z*k) where (x,y,z) may not be unit length
  // log(q) = A*(x*i+y*j+z*k)

  double angle = sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  double sn = sin(angle);

  // When A is near zero, A/sin(A) is approximately 1.  Use
  // log(q) = sin(A)*(x*i+y*j+z*k)
  double coeff = ( abs(sn) < 0.0000001 ? 1.0f : angle/sn );

  return quat(log(length(q)), coeff * q.x, coeff * q.y, coeff * q.z);
}


quat intermediate(const quat& q0, const quat& q1, const quat& q2)
{
  quat term1 = q2 * glm::inverse(q1);
  quat term2 = q0 * glm::inverse(q1);
  quat term3 = term1 + term2;
  quat numerator = -0.25f * qLog(term3);
  quat term4 = qExp(numerator);
  quat s1 = term4 * q1;
  return s1;
}

// TODO
Pose Pose::Squad(const Pose& key0, const Pose& key1, 
    const Pose& key2, const Pose& key3, float u)
{
  Pose pose;
  pose.rootPos = key1.rootPos * (1.0f-u) + key2.rootPos * u;
  for (int i = 0; i < (int) key1.jointRots.size(); i++)
  {
    glm::quat q0 = key0.jointRots[i];
    glm::quat q1 = key1.jointRots[i];
    glm::quat q2 = key2.jointRots[i];
    glm::quat q3 = key3.jointRots[i];
    glm::quat s1 = intermediate(q0, q1, q2); 
    glm::quat s2 = intermediate(q1, q2, q3);

    glm::quat q = glm::squad(q1, q2, s1, s2, u);
    pose.jointRots.push_back(q);
  }
  return pose;
}
}

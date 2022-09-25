#include "atk/transform.h"
#include <iostream>

using namespace glm;

namespace atk {
const Transform Transform::Identity;

Transform Transform::Scale(const vec3& s) {
   return Transform(atk::IdentityQ, vec3(0.0), s);
}

Transform Transform::Scale(float s) {
   return Transform(atk::IdentityQ, vec3(0.0), vec3(s));
}

Transform Transform::Rot(float angle, const vec3& s) {
   return Transform(angleAxis(angle, s), vec3(0.0), vec3(1.0));
}

Transform Transform::Rot(const quat& q) {
   return Transform(q, vec3(0.0), vec3(1.0));
}

Transform Transform::Translate(const vec3& pos) {
   return Transform(atk::IdentityQ, pos, vec3(1.0));
}

Transform::Transform() : 
    _rotation(glm::quat(1.0, 0.0, 0.0, 0.0)),
    _translation(glm::vec3(0.0)),
    _scale(glm::vec3(1.0)) {
}

Transform::Transform(
   const glm::quat& rot, 
   const glm::vec3& offset, 
   const glm::vec3& scale) : 
   _rotation(rot), _translation(offset), _scale(scale) {
}

Transform::Transform(const Transform& m) {
    *this = m;
}

// Assignment operators
Transform& Transform::operator = (const Transform& orig) {
    if (&orig == this) {
        return *this;
    }
    _rotation = orig._rotation;
    _translation = orig._translation;
    _scale = orig._scale;
    return *this;
}

glm::mat4 Transform::matrix() const {
   glm::mat4 mat(1.0); // initialize to identity
   mat = glm::translate(mat, _translation);
   mat = mat * glm::toMat4(_rotation);
   mat = glm::scale(mat, _scale);
   return mat;
}

glm::quat Transform::r() const {
   return _rotation;
}

void Transform::setR(const glm::quat& q) {
   _rotation = q;
}

glm::vec3 Transform::t() const {
   return _translation;
}

void Transform::setT(const glm::vec3& t) {
   _translation = t;
}

glm::vec3 Transform::s() const {
   return _scale;
}

void Transform::setS(const glm::vec3& s) {
   _scale = s;
}

std::ostream& operator << (std::ostream& s, const Transform& t) {
    s << "T: " << glm::to_string(t._translation) << "\n"
      << "R: " << glm::to_string(t._rotation) << "\n"
      << "S: " << glm::to_string(t._scale);
    return s;
}


Transform Transform::inverse() const {
   glm::quat invRot = glm::inverse(_rotation);
   glm::vec3 invS = glm::vec3(1.0/_scale[0], 1.0/_scale[1], 1.0/_scale[2]);
   glm::mat3 invSca(invS[0],0,0,  0,invS[1],0,  0,0,invS[2]);
   
   glm::vec3 offset = -(invSca * glm::mat3(invRot) * _translation);
   return Transform(invRot, offset, invS);
}

glm::vec3 Transform::transformPoint(const glm::vec3& pos) const {
   return _rotation * (_scale * pos) + _translation;
}

glm::vec3 Transform::transformVector(const glm::vec3& dir) const {
   return _rotation * (_scale * dir);
}

Transform operator * (const Transform& t1, const Transform& t2) {
   glm::quat rotation = t1._rotation * t2._rotation;
   glm::vec3 translation = t1._translation + t1._rotation * (t1._scale * t2._translation);
   glm::vec3 scale = t1._scale * t2._scale;
   return Transform(rotation, translation, scale);
}
}
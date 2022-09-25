#include "atk/skeleton.h"
#include <cstring>

namespace atk {
Skeleton::Skeleton() : mRoot(0) {
}

Skeleton::Skeleton(const Skeleton& skeleton) {
    deepCopy(skeleton); 
}

Skeleton& Skeleton::operator = (const Skeleton& orig) {
    if (&orig != this) {
        deepCopy(orig);
    }
    return *this;
}

void Skeleton::deepCopy(const Skeleton& orig) {
    mJoints.clear();
    mRoot = 0;

    // Copy joints
    for (unsigned int i = 0; i < orig.mJoints.size(); i++) {
        Joint* joint = new Joint(*(orig.mJoints[i]));
        mJoints.push_back(joint);
        //std::cout << "Copy " << joint->GetName() << std::endl;
    }

    // Copy parent/children relationships
    for (unsigned int i = 0; i < orig.mJoints.size(); i++) {
        Joint* joint = orig.mJoints[i];
        if (joint->getParent()) {
            Joint* parent = mJoints[joint->getParent()->getID()];
            mJoints[i]->setParent(parent);
        } else {
            mRoot = mJoints[i];
            mRoot->setParent(0);
        }

        for (int j = 0; j < joint->getNumChildren(); j++) {
            Joint* child = mJoints[joint->getChildAt(j)->getID()];
            mJoints[i]->appendChild(child);
        }
    }
}

Skeleton::~Skeleton() {
    clear();
}

void Skeleton::clear() {
    mRoot = NULL;
    mJoints.clear();
}

void Skeleton::fk() {
    if (!mRoot) return; // Nothing loaded
    mRoot->fk();
}


Pose Skeleton::getPose() const {
   Pose pose;
   for (int i = 0; i < getNumJoints(); i++) {
       Joint* joint = getByID(i);
       if (i==0) pose.rootPos = joint->getLocalTranslation();
       pose.jointRots.push_back(joint->getLocalRotation());
   }
   return pose;
}

void Skeleton::setPose(const Pose& pose) {
    assert (pose.jointRots.size() == getNumJoints());

    for (int i = 0; i < getNumJoints(); i++) {
        Joint* joint = getByID(i);
        if (i==0) joint->setLocalTranslation(pose.rootPos);
        joint->setLocalRotation(pose.jointRots[i]);
    }
    fk();
}

Joint* Skeleton::getByName(const std::string& name) const {
    for (unsigned int i = 0; i < mJoints.size(); i++) {
        if (name == mJoints[i]->getName()) {
            return mJoints[i];
        }
    }
    return NULL;
}

Joint* Skeleton::getByID(int id) const {
    assert(id >= 0 && id < (int) mJoints.size());
    return mJoints[id];
}

Joint* Skeleton::getRoot() const {
    return mRoot;
}

void Skeleton::addJoint(Joint* joint, Joint* parent) {
    joint->setID(mJoints.size());
    mJoints.push_back(joint);
    if (!parent) {
        mRoot = joint;
    } else {
        Joint::Attach(parent, joint);
    }
}

void Skeleton::deleteJoint(const std::string& name) {
    Joint* joint = getByName(name);
    if (!joint) return; // no work to do

    for (int i = 0; i < joint->getNumChildren(); i++) {
        Joint* child = joint->getChildAt(i);
        Joint::Detach(joint, child);
        deleteJoint(child->getName());
    }

    // Re-assign ids and delete orphans
    Joint* parent = joint->getParent();
    if (parent) {
        Joint::Detach(parent, joint);
        if (parent->getNumChildren() == 0) parent->setNumChannels(0);
    }
    for (int i = joint->getID(); i < (int) mJoints.size() - 1; i++) {
        mJoints[i] = mJoints[i + 1];
        mJoints[i]->setID(i);
    }
    mJoints.resize(mJoints.size() - 1);
    delete joint;
}

}  // end namespace atk
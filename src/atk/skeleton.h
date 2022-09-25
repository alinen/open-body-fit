#ifndef ATK_SKELETON_H_
#define ATK_SKELETON_H_

#include "atk/joint.h"
#include "atk/pose.h"
#include <vector>

namespace atk {
class Skeleton
{
public:
    // constructors
    Skeleton();
    Skeleton(const Skeleton& skeleton); // Deep copy
    virtual Skeleton& operator=(const Skeleton& orig); // Deep copy

    // destructor
    virtual ~Skeleton();

    // forward kinematics
    // calling this method updates the local2global transforms for each joint
    virtual void fk();

    // removes all joints from the skeleton
    virtual void clear();

    // returns the joint having the given name
    // returns null if no joint has the given name
    Joint* getByName(const std::string& name) const;

    // returns the joint corresponding to id
    // joint ids start at zero and are numbered in the order they are 
    //    added to the skeleton. The root joint has id = 0
    // ids must be in the range [0, numJoints-1]
    Joint* getByID(int id) const;

    // returns the root joint
    Joint* getRoot() const;

    // adds a joint to this skeleton
    // if the parent is NULL, this joint is assumed to be the root
    void addJoint(Joint* joint, Joint* parent = NULL);

    // deletes a joint from the skeleton
    // recursively deletes all children of this joint
    // does nothing if no joint corresponding to name is found
    // warning: this function will reassign joint ids
    void deleteJoint(const std::string& name);

    // returns the number of joints
    int getNumJoints() const { return (int) mJoints.size(); }

    // sets the joints of this skeleton to match pose
    // Pose stores the local rotations of each joint
    // Assumes that the joints in pose (quantity and order)
    //     matches this skeleton
    void setPose(const Pose& pose);

    // gets the pose corresponding to the skeleton's current joint values
    // Pose stores the local rotations of each joint
    Pose getPose() const;

protected:
    void deepCopy(const Skeleton& skeleton);

protected:
    std::vector<Joint*> mJoints;
    Joint* mRoot;
};
}
#endif

#ifndef ATK_JOINT_H_
#define ATK_JOINT_H_

#include "atk/transform.h"
#include <string>
#include <vector>

// Classes for organizing animatble transforms into hierachies
namespace atk {
class Joint
{
public:
    Joint();
    Joint(const std::string& name);
    Joint(const Joint& joint);
    virtual Joint& operator=(const Joint& joint);

    virtual ~Joint();

    Joint* getParent();
    void setParent(Joint* parent);

    int getNumChildren() const;
    Joint* getChildAt(int index);
    void appendChild(Joint* child);

    void fk();

    void setName(const std::string& name);
    void setID(int id);
    void setNumChannels(int count);
    void setRotationOrder(RotOrder roo);

    void setLocal2Parent(const Transform& transform);
    void setLocalTranslation(const glm::vec3& translation);
    void setLocalRotation(const glm::quat& rotation);

    int getID() const;
    const std::string& getName() const;
    int getNumChannels() const;
    RotOrder getRotationOrder() const;

    Transform getLocal2Parent() const;
    glm::vec3 getLocalTranslation() const;
    glm::quat getLocalRotation() const;

    Transform getLocal2Global() const;
    glm::vec3 getGlobalTranslation() const;
    glm::quat getGlobalRotation() const;

    static void Attach(Joint* pParent, Joint* pChild);
    static void Detach(Joint* pParent, Joint* pChild);

protected:

    int mId;
    std::string mName;
    int mChannelCount;
    RotOrder mRotOrder;
    bool mDirty;

    Joint* mParent;
    std::vector<Joint*> mChildren;

    Transform mLocal2Parent;
    Transform mLocal2Global;
};
}

#endif

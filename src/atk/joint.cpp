#include "atk/joint.h"
#include <cstring>

namespace atk {
Joint::Joint() : 
    mId(-1), 
    mName(""), 
    mChannelCount(6), 
    mRotOrder(XYZ), 
    mDirty(false),
    mParent(0),
    mChildren(),
    mLocal2Parent(),
    mLocal2Global()
{

}

Joint::Joint(const std::string& name) :
    mId(-1),
    mName(name),
    mChannelCount(6),
    mRotOrder(XYZ),
    mDirty(false),
    mParent(0),
    mChildren(),
    mLocal2Parent(),
    mLocal2Global()
{

}

Joint::Joint(const Joint& joint)
{
    *this = joint;
}

Joint& Joint::operator=(const Joint& orig)
{
    if (&orig == this)
    {
        return *this;
    }

    // copy everything except parents/children
    mParent = 0;
    mChildren.clear();
    mDirty = true;

    mId = orig.mId;
    mName = orig.mName;
    mChannelCount = orig.mChannelCount;
    mRotOrder = orig.mRotOrder;
    mLocal2Parent = orig.mLocal2Parent;
    mLocal2Global = orig.mLocal2Global;

    return *this;
}

Joint::~Joint()
{

}

Joint* Joint::getParent()
{
    return mParent;
}
void Joint::setParent(Joint* parent)
{
    mParent = parent;
}

int Joint::getNumChildren() const
{
    return mChildren.size();
}
Joint* Joint::getChildAt(int index)
{
    assert(index >= 0 && index < (int) mChildren.size());
    return mChildren[index];
}
void Joint::appendChild(Joint* child)
{
    mChildren.push_back(child);
}

void Joint::setName(const std::string& name)
{
    mName = name;
}
void Joint::setID(int id)
{
    mId = id;
    if (strncmp("Site", mName.c_str(), 4) == 0)
    {
        char dummy[32];
        sprintf(dummy, "Site%d", mId);
        mName = dummy;
    }
}
void Joint::setNumChannels(int count)
{
    mChannelCount = count;
}

void Joint::setRotationOrder(RotOrder roo) 
{
   mRotOrder = roo;
}

void Joint::setLocal2Parent(const Transform& transform)
{
    mLocal2Parent = transform;
}
void Joint::setLocalTranslation(const glm::vec3& translation)
{
    mLocal2Parent.setT(translation);
}
void Joint::setLocalRotation(const glm::quat& rotation)
{
    mLocal2Parent.setR(rotation);
}

int Joint::getID() const
{
    return mId;
}
const std::string& Joint::getName() const
{
    return mName;
}
int Joint::getNumChannels() const
{
    return mChannelCount;
}
RotOrder Joint::getRotationOrder() const
{
    return mRotOrder;
}

Transform Joint::getLocal2Parent() const
{
    return mLocal2Parent;
}
glm::vec3 Joint::getLocalTranslation() const
{
    return mLocal2Parent.t();
}
glm::quat Joint::getLocalRotation() const
{
    return mLocal2Parent.r();
}

Transform Joint::getLocal2Global() const
{
    return mLocal2Global;
}
glm::vec3 Joint::getGlobalTranslation() const
{
    return mLocal2Global.t();
}
glm::quat Joint::getGlobalRotation() const
{
    return mLocal2Global.r();
}


void Joint::fk()
{
    if (mParent)
    {
        mLocal2Global = mParent->getLocal2Global() * mLocal2Parent;
    }
    else
    {
        mLocal2Global = mLocal2Parent; // it takes a village to raise a child? the world is my parent!
    }
    std::vector<Joint*>::const_iterator iter;
    for (iter = mChildren.begin(); iter != mChildren.end(); iter++)
    {
        (*iter)->fk();
    }
}


void Joint::Attach(Joint* pParent, Joint* pChild)
{
    if (pChild)
    {
        Joint* pOldParent = pChild->mParent;
        if (pOldParent)
        {
            // erase the child from old parent's children list
            std::vector<Joint*>::iterator iter;
            for (iter = pOldParent->mChildren.begin(); iter != pOldParent->mChildren.end(); iter++)
            {
                if ((*iter) == pChild)
                {
                    iter = pOldParent->mChildren.erase(iter);
                }
            }
        }
        // Set the new parent
        pChild->mParent = pParent;
        // Add child to new parent's children list
        if (pParent)
        {
            pParent->mChildren.push_back(pChild);
        }
    }
}

void Joint::Detach(Joint* pParent, Joint* pChild)
{
    if (pChild && pChild->mParent == pParent)
    {
        if (pParent)
        {
            // erase the child from parent's children list
            for (int i = 0; i < (int) pParent->mChildren.size(); i++)
            {
                if (pParent->mChildren[i] == pChild)
                {
                    for (int j = i; j < (int) pParent->mChildren.size() - 1; j++)
                    {
                        pParent->mChildren[j] = pParent->mChildren[j + 1];
                    }
                    pParent->mChildren.resize(pParent->mChildren.size() - 1);
                    break;
                }
            }
        }
        pChild->mParent = NULL;
    }
}

}
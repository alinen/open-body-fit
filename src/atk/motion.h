#ifndef ATK_MOTION_H_
#define ATK_MOTION_H_

#include <map>
#include <string>
#include <vector>
#include "atk/skeleton.h"
#include "atk/pose.h"

// Motions are implemented as linear splines with evenly spaced keys
// The spacing of the keys depends on the framerate of motions 
// For example, 30 fps means there are 30 frames every seconds, so 
// the amount of time between each frame is 1/30
namespace atk {
class Motion
{
public:
    Motion(double fps = 120.0);
    Motion(const Motion& m); // deep copy
    Motion& operator=(const Motion& m); // deep copy

    virtual ~Motion();

    // set/get framerate
    void setFramerate(double fps);
    double getFramerate() const;

    // set/get delta time
    void setDeltaTime(double dt);
    double getDeltaTime() const;

    // updates the skeleton at the given time
    void update(Skeleton& skeleton, double time, bool loop = true) const;

    // returns the pose at the given time
    Pose getValue(double t, bool loop = true) const;

    // adds a key to the end of this motion 
    // ids are assigned in the order they are added
    void appendKey(const Pose& value);

    // changes the key corresponding to keyID
    // note: ids are assigned in the order they are added
    // keyID must be in range [0, numKeys-1]
    void editKey(int keyID, const Pose& value); 

    // deletes the given key ID
    // all subsequent keys are shifted order (and hence change id)
    // keyID must be in range [0, numKeys-1]
    void deleteKey(int keyID);

    // returns the key corresponding to keyID
    // keyID must be in range [0, numKeys-1]
    Pose getKey(int keyID) const;

    // returns the number of keys in this motion
    int getNumKeys() const;

    // removes all keys from this motion
    void clear();

    // the duration of the motion in seconds, e.g. numKeys * (1/fps)
    double getDuration() const;

    // the percentage through the motion at time time
    // for example, if the duration is 8 seconds and t is 4, returns 0.5
    double getNormalizedDuration(double t) const; 

    // returns the interval containing t
    // this interval will be between key and (key+1)
    int getKeyID(double t) const;

protected:
    void deepCopy(const Motion& m);

protected:
    double mDt;
    double mFps;
    std::vector<Pose> mKeys;
};
}
#endif

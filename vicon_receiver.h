#ifndef _VICON_RECEIVER
#define _VICON_RECEIVER

#include "ofMain.h"
#include "ofxNetwork.h"
#include <atomic>

#define maxMessageLen 1000

struct HeadPositionAndRotation {
    long time;
    float x_position;
    float y_position;
    float z_position;
    float x_rotation;
    float y_rotation;
    float z_rotation;
    float z_rot_avg;
    ofQuaternion q;
    // rotation matrix
    float rotation_matrix[9];
};

class ViconReceiver: public ofThread
{
public:
    HeadPositionAndRotation data;
    ~ViconReceiver();
    void setup(ofxUDPSettings settings);
    void start();

    static void toEulerAngle(const ofQuaternion& q, double& roll, double& pitch, double& yaw);
    void shiftFilterSamples();
    void smoothenHeadOrientation();
    void stop();
    void readEuler(string msg);
    void readRawQuaternion(string msg);
    void readRawRotationMatrix(string msg);
    void readAll(string msg);
    void convertQuaternionToHeadRotation();
    void threadedFunction();
    void updateData();
    HeadPositionAndRotation getLatestData();

protected:
    char _udpMessage[maxMessageLen];
    ofxUDPManager _udpConnection;
    HeadPositionAndRotation _buffer;
    int _avg_samples = 2;
    vector<float> _old_samples;
};

#endif // _VICON_RECEIVER

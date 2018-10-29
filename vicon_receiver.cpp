
#include "vicon_receiver.h"

ViconReceiver::~ViconReceiver(){
    stop();
    waitForThread(false);
}

void ViconReceiver::setup(ofxUDPSettings settings) {
    for (int kk=0; kk<this->_avg_samples; kk++) {
        this->_old_samples.push_back(0.0f);
    }
    _udpConnection.Setup(settings);
    start();
}

void ViconReceiver::start() {
    startThread();
}

void ViconReceiver::toEulerAngle(const ofQuaternion& q, double& roll, double& pitch, double& yaw) {
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr, cosr);
    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1) {
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
        pitch = asin(sinp);
    }
    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny, cosy);
}

void ViconReceiver::shiftFilterSamples() {
    for (int kk=1; kk<this->_avg_samples; kk++) {
        this->_old_samples[kk-1] = this->_old_samples[kk];
    }
    this->_old_samples[this->_avg_samples-1] = _buffer.z_rotation;
}

void ViconReceiver::smoothenHeadOrientation() {
    float tmp = 0;
    for (int kk=0; kk<this->_avg_samples; kk++) {
        tmp += this->_old_samples[kk];
    }
    _buffer.z_rot_avg = (tmp + _buffer.z_rotation)/(this->_avg_samples+1);
    shiftFilterSamples();
}

    /// Signal the thread to stop.  After calling this method,
    /// isThreadRunning() will return false and the while loop will stop
    /// next time it has the chance to.
    /// In order for the thread to actually go out of the while loop
    /// we need to notify the condition, otherwise the thread will
    /// sleep there forever.
    /// We also lock the mutex so the notify_all call only happens
    /// once the thread is waiting. We lock the mutex during the
    /// whole while loop but when we call condition.wait, that
    /// unlocks the mutex which ensures that we'll only call
    /// stop and notify here once the condition is waiting
void ViconReceiver::stop() {
    std::unique_lock<std::mutex> lck(mutex);
    stopThread();
}

void ViconReceiver::readEuler(string msg) {
    vector<string> msgParts = ofSplitString(msg, "/");
    _buffer.time = strtof(msgParts[0].c_str(),0);
    _buffer.x_position = strtof(msgParts[1].c_str(),0);
    _buffer.y_position = strtof(msgParts[2].c_str(),0);
    _buffer.z_position = strtof(msgParts[3].c_str(),0);
    // use eulers
    _buffer.x_rotation = strtof(msgParts[4].c_str(),0);
    _buffer.y_rotation = strtof(msgParts[5].c_str(),0);
    _buffer.z_rotation = strtof(msgParts[6].c_str(),0);
}
void ViconReceiver::readRawQuaternion(string msg) {
    vector<string> msgParts = ofSplitString(msg, "/");
    _buffer.time = strtof(msgParts[0].c_str(),0);
    _buffer.x_position = strtof(msgParts[1].c_str(),0);
    _buffer.y_position = strtof(msgParts[2].c_str(),0);
    _buffer.z_position = strtof(msgParts[3].c_str(),0);
    // use quaternion
    float a,b,g,w;
    a = strtof(msgParts[4].c_str(),0);
    b = strtof(msgParts[5].c_str(),0);
    g = strtof(msgParts[6].c_str(),0);
    w = strtof(msgParts[7].c_str(),0);
    _buffer.q = ofQuaternion(a,b,g,w);
}
void ViconReceiver::readRawRotationMatrix(string msg) {
    vector<string> msgParts = ofSplitString(msg, "/");
    _buffer.time = strtof(msgParts[0].c_str(),0);
    _buffer.x_position = strtof(msgParts[1].c_str(),0);
    _buffer.y_position = strtof(msgParts[2].c_str(),0);
    _buffer.z_position = strtof(msgParts[3].c_str(),0);
    // rotation matrix
    _buffer.rotation_matrix[0] = strtof(msgParts[4].c_str(),0);
    _buffer.rotation_matrix[1] = strtof(msgParts[5].c_str(),0);
    _buffer.rotation_matrix[2] = strtof(msgParts[6].c_str(),0);
    _buffer.rotation_matrix[3] = strtof(msgParts[7].c_str(),0);
    _buffer.rotation_matrix[4] = strtof(msgParts[8].c_str(),0);
    _buffer.rotation_matrix[5] = strtof(msgParts[9].c_str(),0);
    _buffer.rotation_matrix[6] = strtof(msgParts[10].c_str(),0);
    _buffer.rotation_matrix[7] = strtof(msgParts[11].c_str(),0);
    _buffer.rotation_matrix[8] = strtof(msgParts[12].c_str(),0);

    _buffer.z_rotation = atan2f(_buffer.rotation_matrix[5], _buffer.rotation_matrix[2]) * (180/M_PI);
}
void ViconReceiver::readAll(string msg) {
    vector<string> msgParts = ofSplitString(msg, "/");
    _buffer.time = strtof(msgParts[0].c_str(),0);
    _buffer.x_position = strtof(msgParts[1].c_str(),0);
    _buffer.y_position = strtof(msgParts[2].c_str(),0);
    _buffer.z_position = strtof(msgParts[3].c_str(),0);
    // use quaternion
    float a,b,g,w;
    a = strtof(msgParts[4].c_str(),0);
    b = strtof(msgParts[5].c_str(),0);
    g = strtof(msgParts[6].c_str(),0);
    w = strtof(msgParts[7].c_str(),0);
    _buffer.q = ofQuaternion(a,b,g,w);
    // use eulers
    _buffer.x_rotation = strtof(msgParts[8].c_str(),0);
    _buffer.y_rotation = strtof(msgParts[9].c_str(),0);
    _buffer.z_rotation = strtof(msgParts[10].c_str(),0);
    // rotation matrix
    _buffer.rotation_matrix[0] = strtof(msgParts[11].c_str(),0);
    _buffer.rotation_matrix[1] = strtof(msgParts[12].c_str(),0);
    _buffer.rotation_matrix[2] = strtof(msgParts[13].c_str(),0);
    _buffer.rotation_matrix[3] = strtof(msgParts[14].c_str(),0);
    _buffer.rotation_matrix[4] = strtof(msgParts[15].c_str(),0);
    _buffer.rotation_matrix[5] = strtof(msgParts[16].c_str(),0);
    _buffer.rotation_matrix[6] = strtof(msgParts[17].c_str(),0);
    _buffer.rotation_matrix[7] = strtof(msgParts[18].c_str(),0);
    _buffer.rotation_matrix[8] = strtof(msgParts[19].c_str(),0);
}

void ViconReceiver::convertQuaternionToHeadRotation() {
    double a,b,yaw;
    toEulerAngle(_buffer.q, a, b, yaw);
    _buffer.x_rotation = a * (180/M_PI)  + 180; // move from +-180 to 0-360
    _buffer.y_rotation = b * (180/M_PI)  + 180; // move from +-180 to 0-360
    _buffer.z_rotation = yaw *(180/M_PI) + 180; // move from +-180 to 0-360
}

/// Everything in this function will happen in a different
/// thread which leaves the main thread completelty free for
/// other tasks.
void ViconReceiver::threadedFunction() {
    while (isThreadRunning())
    {
        memset(_udpMessage, 0, maxMessageLen);
        _udpConnection.Receive(_udpMessage, maxMessageLen);
        string message = _udpMessage;
        std::unique_lock<std::mutex> lock(mutex);
        if (message != "") {
            /*readEuler(message); // read position and euler angles
            readRawQuaternion(message); // read position and raw quaternion
            convertQuaternionToHeadRotation();*/
            readRawRotationMatrix(message); // read position and raw rotation matrix
            //readAll(message);
            smoothenHeadOrientation();
        }
    }
}

void ViconReceiver::updateData() {
    // if we didn't lock here we would see
    // tearing as the thread would be updating
    // the pixels while we upload them to the texture
	std::unique_lock<std::mutex> lock(mutex);
    data = _buffer;
}

HeadPositionAndRotation ViconReceiver::getLatestData() {
    return data;
}

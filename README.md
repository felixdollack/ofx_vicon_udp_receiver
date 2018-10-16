# ofx_vicon_udp_receiver
A udp class for use in openframeworks projects to receive vicon motion tracking data

This repository contains a c++ UDP class to receive vicon motion tracking data.
The UDP portion of the code relies on openframeworks.

## Dependencies
* [openframeworks](https://openframeworks.cc) version 0.9.x or higher

## Example

```CPP
ViconReceiver udp_receiver;    // create an instance of udp trigger

ofxUDPSettings settings;       // create udp settings to setup a receiver
settings.receiveOn(port);
settings.blocking = false;
udp_receiver.setup(settings);  // sets up the connection and starts the receiver thread
udp_receiver.updateData();     // copy received data from receive to read buffer
udp_receiver.getLatestData();  // return an instance of the read buffer
udp_receiver.stop();           // stop the receiver
```

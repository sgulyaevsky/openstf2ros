# openstf2ros

This package is a bridge between [openstf/minicap](https://github.com/openstf/minicap), 
[openstf/minitouch](https://github.com/openstf/minitouch) and [ROS](https://www.ros.org/).  

The main purpose of this package is to enable usage 
of the ROS infrastructure for GUI related computer vision research.

To run this package you will need:
- adb (Ubuntu: ```sudo apt-get install adb```)
- minicap ([building minicap](https://github.com/openstf/minicap#building))
- minitouch ([building minitouch](https://github.com/openstf/minitouch#building))

See the launch file example for the parameters list.

It publishes ```/openstf_screen``` topic and subscribes to ```/openstf_control_messages``` topic for [minitouch control strings](https://github.com/openstf/minitouch#writable-to-the-socket). 

There is also an [rqt_openstf_teleop](https://github.com/sgulyaevsky/rqt_openstf_teleop) package which is a simple rqt plugin functioning as teleop for openstf2ros.  

respeaker_ros
=============

A ROS Package for Respeaker Mic Array


## Supported Devices

- [Respeaker Mic Array v2.0](http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)

    ![Respeaker Mic Array v2.0](https://github.com/SeeedDocument/ReSpeaker_Mic_Array_V2/raw/master/img/Hardware%20Overview.png)

## Preparation

1. Update firmware

    ```bash
    sudo apt-get update
    sudo pip install pyusb click
    git clone https://github.com/respeaker/usb_4_mic_array.git
    cd usb_4_mic_array
    sudo python dfu.py --download 6_channels_firmware.bin  # The 6 channels version 
    ```

2. Build this package

    Assumes ROS is installed

    ```bash
    mkdir -p ~/catkin_ws/src && ~/catkin_ws/src
    git clone https://github.com/furushchev/respeaker_ros.git
    cd ~/catkin_ws
    source /opt/ros/kinetic/setup.bash
    rosdep install --from-paths src -i -r -n -y
    catkin config --init
    catkin build
    source ~/catkin_ws/devel/setup.bash
    ```
    
3. Run executables

    ```bash
    roslaunch respeaker_ros respeaker.launch
    rostopic echo /speech_direction  # Result of DoA
    rostopic echo /is_speeching      # Result of VAD
    rostopic echo /audio             # Raw audio
    ```

## Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>

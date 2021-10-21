
Forked from luckyluckydadada. Give credit to this guy.  

# Modification
Original `CMakeLists.txt` using `link_directories`, which cause running error (cannot find xxx.so). But build correct.

Add this line to my `.bashrc`:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/MVS/lib/64
```

Changed some parameters.



---------------------------  Original README  ---------------------------


# HIKROBOT-MVS-CAMERA-ROS
The ros driver package of Hikvision Industrial Camera SDK. Support configuration parameters, the parameters have been optimized, and the photos have been transcoded to rgb format.
Please install mvs, https://blog.csdn.net/weixin_41965898/article/details/116801491

# Install
```
mkdir -p ~/ws_hikrobot_camera/src
git clone https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS.git ~/ws_hikrobot_camera/src/hikrobot_camera
cd ~/ws_hikrobot_camera
catkin_make
```
# launch run
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera.launch
```
# launch run
use rviz subscribe topic： /hikrobot_camera/rgb
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera_rviz.launch
```

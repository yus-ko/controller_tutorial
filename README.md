# controller_tutorial

# ビルド方法
 
```bash
cd ~/catkin_ws/src
git clone https://github.com/yus-ko/potbot
git clone https://github.com/yus-ko/controller_tutorial
cd potbot
ls | grep -vE 'potbot_lib|potbot_msgs' | xargs rm -r
```
```bash
cd ~/catkin_ws
catkin build controller_tutorial
```

# 起動方法

```bash
roslaunch controller_tutorial controller_tutorial.launch
...

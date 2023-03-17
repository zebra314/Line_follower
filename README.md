# ROS Line Follower

## Brief
The robot runs ROS on Raspberry Pi, using usb camera and opencv library to detect the line
and an Arduino to control the wheels by PID.  

![Body](/asset/images/robot.jpg)  
## Architecture
![Architecture](/asset/images/Architecture_4.png)
  
![Architecture](/asset/images/rosgraph_2.png)

``` c
@startuml
!pragma teoz true

camera -> Line_detector : img
Line_detector -> Coachman : img, contours, points
Coachman -> toarduino : pwm_msg
camera -> recorder : img
Coachman -> recorder : img
toarduino -> motor_controll : pwm_msg
motor_controll -> wheels : pwm

box "ROS NODES"
  participant camera
  participant recorder
    box "detector"
      participant Line_detector
      participant Coachman
    endbox
  participant toarduino
endbox

box "ARDUINO"
  participant motor_controll
  participant wheels
@enduml
```

## Commands

1. To launch 
```bash
roslaunch detector detector.launch 
```
  
2. To view the image
```bash
rqt_image_view
```

3. To view the node graph
```bash
rosrun rqt_graph rqt_graph
```

4. To upload to ardunio
```bash
cd ~/Line_follower/onArdu/Line_follower
make upload clean
```

## Devlog

> 3/17  
> Fix uart delay problem  

> 3/13   
> Test on Pi  
> Couldn't reduce the period from arduino echo to the new msg arrived    
> It might caused by the while in toarduino.py for waiting the arduino echo  

> 3/8  
> Optimize motor controll  
> Add recorder node 

> 3/5  
> Optimize line detect

> 3/3  
> Fuck up raspberry

> 3/2  
> PID controll

> 2/20  
> Update line detect

> 2/19  
> Test Rasberry pi 

> 2/18  
> Toarduino pkg

> 2/17  
> Line detect and camera pkg  

## ToDo
> Tweak motor speed  
> Test on Pi  
> Adjust PID parameters  
> Check if the Arduino receieve the msg  
> Optimize the line detector 

## Reference

[Canny](https://blog.csdn.net/sunny2038/article/details/9202641)  
[HoughLines](https://blog.csdn.net/dcrmg/article/details/78880046)  
[HoughLines vs HoughLinesP](https://blog.csdn.net/ftimes/article/details/106816736)  
[Line detect algorithm](https://hackmd.io/@0xff07/cv-tracking)  
[Upload to arduino using terminal](https://www.youtube.com/watch?v=qAM2S27FWAI)
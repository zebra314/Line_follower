# ROS Line Follower

## Brief
The robot runs ROS on Raspberry Pi, using usb camera and opencv library to detect the line
and an Arduino to control the wheels by PID.

## Architecture
![Architecture](/asset/images/Architecture_3.png)

``` c
@startuml
!pragma teoz true

camera -> Line_detector : img
Line_detector -> Coachman : img, contours, points
Coachman -> Recorder : img
Coachman -> toarduino : pwm_msg
toarduino -> motor_controll : pwm_msg
motor_controll -> wheels : pwm

box "ROS NODES"
  participant camera
    box "detector"
      participant Line_detector
      participant Coachman
      participant Recorder
    endbox
  participant toarduino
endbox

box "ARDUINO"
  participant motor_controll
  participant wheels
@enduml
```

## Devlog

> 3/8  
> Optimize motor controll  

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
> Test on Pi  
> Adjust PID parameters  
> Check if the Arduino receieve the msg  
> Optimize the line detector 

## Reference

[Canny](https://blog.csdn.net/sunny2038/article/details/9202641)  
[HoughLines](https://blog.csdn.net/dcrmg/article/details/78880046)  
[HoughLines vs HoughLinesP](https://blog.csdn.net/ftimes/article/details/106816736)  
[Line detect algorithm](https://hackmd.io/@0xff07/cv-tracking)
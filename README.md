# ROS Line Follower

## Brief
The robot runs ROS on Raspberry Pi, using usb camera and opencv library to detect the line
and an Arduino to control the wheels by PID.

## Architecture
![Architecture](/asset/images/Architecture.png)

``` c
@startuml
!pragma teoz true

camera -> main : image
Object_detector -> main : import
Line_detector -> main : import
Recorder -> main : import
main -> toarduino : pwm_msg
toarduino -> motor_controll : pwm_msg
motor_controll -> wheels : pwm

box "ROS NODES"
  participant camera
    box "detector"
      participant Object_detector
      participant Line_detector
      participant Recorder
      participant main
    endbox
  participant toarduino
endbox

box "ARDUINO"
  participant motor_controll
  participant wheels
@enduml
```

## Devlog

> 2/20  
> Update line detect

> 2/19  
> Test Rasberry pi 

> 2/18  
> Toarduino pkg

> 2/17  
> Line detect and camera pkg  

## ToDo
> Check if the Arduino receieve the msg  
> Optimize the line detector 

##　Reference

[Canny](https://blog.csdn.net/sunny2038/article/details/9202641)
[HoughLines](https://blog.csdn.net/dcrmg/article/details/78880046)
[HoughLines vs HoughLinesP](https://blog.csdn.net/ftimes/article/details/106816736)
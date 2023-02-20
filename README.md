# ROS Line Follower

## Brief
The robot runs ROS on Raspberry Pi, using usb camera and opencv library to detect the line
and an Arduino to control the wheels by PID.

## Architecture
![Architecture](/asset/images/Line_follower%20sys%20structure.png)

``` c
@startuml
!pragma teoz true
Master -> Line_detector : launch
Master -> Terminator : launch
Master -> Img_capture : launch
Master -> Toarduino : launch
Terminator -> Line_detector : shutdown
Terminator -> Img_capture : shutdown
Terminator -> Toarduino : shutdown

box "ROS NODES"
  participant Master
  participant Img_capture
  participant Toarduino
  participant Terminator
  participant Line_detector
    box "detector"
      participant Terminator
      participant Line_detector
    endbox
endbox
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

## wt901B_ros

リファクタリング頑張って

[TODO]
- ros package nameを小文字にする
- portとbaurateをParamにする
- sendデバッグする
- 高さしかだしません

10axes-imu's ros package
see
https://github.com/WITMOTION/WT901B

I use Sample code -> raspberry -> SampleLinux 

## Set Driver
See docbase...

## Pub/Sub

#### Pub
`/imu_height`: Int32, 大体の高さをだす。時間経過で上がっていく

#### Sub
`/imu_reset_height`: Bool, pubすると現在の高さを0にする
高さを0にするには`{0xFF, 0xAA, 0x01, 0x03, 0x00}`をシリアル通信でsendしてもできる

Publish例
```
rostopic pub --once /imu_reset_height std_msgs/Bool True
```

## Quick usage

```
cd ~/ros/src/
git clone <URL>
cd wt901B_ros
catkin bt
roslaunch wt901B_ros wt901B_ros.launch 
```

## Include Launch file

```
<include file="$(find wt901B_ros)/launch/wt901B_ros.launch"/>
```
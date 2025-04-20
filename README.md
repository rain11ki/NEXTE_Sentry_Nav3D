# 关于NEXTE_Sentry_Nav的3D版本

## Introduction

本仓库整合了NEXTE_Sentry_Nav的一套流程后，加以升级，引入了3D运动规划器，

环境配置：参考[NEXTE_Sentry_Nav/README_CN.md at main · 66Lau/NEXTE_Sentry_Nav](https://github.com/66Lau/NEXTE_Sentry_Nav/blob/main/README_CN.md)

三维规划器：[MoveBase3D 仓库](https://github.com/WilsonGuo/MoveBase3D)
![MoveBase3D 演示截图](https://github.com/user-attachments/assets/cf40b041-b371-43cf-8372-e8a0ce4ff554)


## 使用说明：

该运动规划器对电脑性能要求较高，请酌情选择，并且该方案只适合UGV，雷达为Mid360(正装)，后续预计会补充将雷达倒装或者倾斜着装，因为该规划器对地面点云有要求。

使用该规划器需要构建先验地图，将PCD命名并且保存至src/FAST_LIO/PCD下，并且需要稠密的地面点云，具体要求请查看[MoveBase3D 的使用说明](https://github.com/WilsonGuo/MoveBase3D/blob/main/README.md)
，个人使用过程中会增加雷达扫描时间，以便构建可供投射的位姿。

如果你已经跑通了来自 [66Lau](https://github.com/66Lau)的 [NEXTE_Sentry_Nav](https://github.com/66Lau/NEXTE_Sentry_Nav)的优秀工作，那么直接克隆本仓库，即可使用，一些修改存在于FAST_LIO_LOCALIZATION中，添加了一个base_link，并且用三维的地图给予初始位姿，若电脑带不动，可以选择加载二维的地图给与初始位姿，该工作在NEXTE_Sentry_Nav中完成。

## 运行命令：

```
roslaunch livox_ros_driver2 msg_MID360.launch

roslaunch fast_lio_localization localization_avia.launch

roslaunch rover_launch move_base_3d.launch
```
运行roslaunch fast_lio_localization localization_avia.launch后，在rviz中给予初始位姿，成功后在运行下一句命令

运行roslaunch rover_launch move_base_3d.launch后，在该rviz中，左侧勾选Allmap，即可看到当前地图，选用上方3D goal给予目标点，右侧会出先目标点消息，点击开始导航即可。
## 致谢
- [NEXTE_Sentry_Nav](https://github.com/66Lau/NEXTE_Sentry_Nav)
- [MoveBase3D](https://github.com/WilsonGuo/MoveBase3D)

## F&Q
1.关于实时避障：由于MoveBase3D中关于避障的部分计算量大，会导致规划也出现问题，所以我注释了该部分的功能，具体内容可以在src/MoveBase3D/global_planner_3d/src/global_planning_obs.cpp中找到，我注释的部分在214行，pt_sub = nh.subscribe("/cloud_registered_body", 1, rcvLidarCallBack)。有兴趣的可以去debug一下。

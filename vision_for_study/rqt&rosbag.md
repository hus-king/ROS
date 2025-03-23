##### 1.rqt
- 先使用`roscore`启动`rosmaster`
- 在可视化界面中点击菜单栏`Plugins`
- 在其中的`Visualization`中可以查看图像等话题信息
- 在`Topic`中可以查看特定的话题信息

##### 2.rosbag
主要分为两个命令

- rosbag record topic(例如:rosbag record camera/image_raw)
- rosbag play *.bag(例如:rosbag play 2025_3_9_######.bag)

*第一条命令可以记录当前ros中发布的对应`话题`,并将其保存在后缀为`.bag`的文件中.*
*第二条命令可以在ros中发布`.bag`文件中的话题信息*
*可以提前录制视频并在仿真环境中测试*

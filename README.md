# Occupancy Grid Mapping
## 用于物体主动slam的gazebo仿真

## 各Demo的作用

1. run_gazebo.launch  运行gazebo仿真环境

2. move_base_config.launch   运行movebase

3. navigation.launch  综合型launch文件
   1. 构建栅格地图
   2. 发布map和odom的tf
   3. 发布机器人和odom的tf
   4. 运行move_base_config.launch
   
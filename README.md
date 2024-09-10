# Occupancy Grid Mapping
## 用于物体主动slam的gazebo仿真

## 各Demo的作用

### run_gazebo.launch  运行gazebo仿真环境
roslaunch  gazebo_object_env_2024 run_gazebo.launch 

### 直接控制相机的位姿
   1. 环形
    rosrun gazebo_object_env_2024 circle
   2. 移动物体，来改变相机
    rosrun gazebo_object_env_2024 object_direction
    
## Active QSP SLAM 流程
   1. QSP SLAM：基于二次曲面和隐式形状表达的物体级 SLAM
      在空间中建立二次曲面和点云之间的关联关系，为隐式形状编码提供更准确的初始位姿和更完整的点云信息。	
   2. 基于


<!-- 2. move_base_config.launch   运行movebase -->

<!-- 3. navigation.launch  综合型launch文件
   1. 构建栅格地图
   2. 发布map和odom的tf
   3. 发布机器人和odom的tf
   4. 运行move_base_config.launch
    -->




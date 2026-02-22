#总体环境
本项目基于 Ubuntu 20.04 和 ROS Noetic 进行开发和测试.

# swarm-lio2

## 安装swarm-lio2
https://github.com/hku-mars/Swarm-LIO2
git链接可自行安装.

这里有他用的GTSAM版本,下载地址:还有他的数据集. https://drive.google.com/drive/folders/17e-Fe5h3LApskJFN_Z0GEGu4ZoavBZsr

### 环境配置
```bash
sudo apt-get install libboost-all-dev
sudo apt-get install cmake
sudo apt-get install libtbb-dev
```

### 安装GTSAM优化库 
```bash
mkdir build
cd build
cmake ..
make check (optional, runs unit tests)
make install
sudo cp /usr/local/lib/libgtsam.so.4 /usr/lib
sudo cp /usr/local/lib/libmetis-gtsam.so /usr/lib
```
### bag运行swarm-lio2
```bash
cd ~/swarm_ws/src
git clone git@github.com:hku-mars/Swarm-LIO2.git
cd ..
catkin_make -j
source devel/setup.bash
```

### 仿真中运行swarm-lio2
修改single_drone_sim.xml
```xml
<launch>
	<arg name="drone_id"/>
    <arg name="output_mode"/>
    <arg name="init_x" default="0.0"/>
    <arg name="init_y" default="0.0"/>
    <arg name="init_z" default="1.0"/>
    <arg name="init_yaw" default="0.0"/>		# 确定无人机在世界系下的初始位置默认值
    <node pkg="swarm_lio" type="swarm_lio" name="laserMapping_quad$(arg drone_id)" output="$(arg output_mode)">
        <rosparam command="load" file="$(find swarm_lio)/config/simulation.yaml" />
        <param name="common/drone_id" type="int" value="$(arg drone_id)"/>
        <param name="common/lid_topic" type="string" value="/quad$(arg drone_id)_pcl_render_node/sensor_cloud"/>
        <param name="common/imu_topic" type="string" value="/quad_$(arg drone_id)/imu"/>
        <param name="common/world_frame_id" value="quad$(arg drone_id)/world"/> 
        <param name="common/odom_frame_id" value="quad$(arg drone_id)/odom"/>
		<param name="topic_name_prefix" value="quad$(arg drone_id)/" />
    </node>
	<node pkg="tf" type="static_transform_publisher" name="link_quad$(arg drone_id)_world" 
			args="$(arg init_x) $(arg init_y) $(arg init_z) $(arg init_yaw) 0 0 world quad$(arg drone_id)/world 100" />	
</launch>
```

#修改simulation.launch
```xml
	<include file="$(find swarm_lio)/launch/single_drone_sim.xml">
		<arg name="drone_id" value="0"/>		## 无人机编号 sim里是几架这里就要有几架
		<arg name="output_mode" value="screen"/>
		<arg name="init_x" value="0.0"/>
		<arg name="init_y" value="0.0"/>		// 载入你想要的无人机的初始位置
	</include>
```

#运行marsim

source devel/setup.bash
roslaunch test_interface triple_drone_mid360.launch

#运行swarm-lio2仿真

source devel/setup.bash
roslaunch swarm_lio simulation.launch

这里以三个无人机为例. 分别编写控制单无人机的python脚本  ,先让0号机启动 ,一段时间后可以看到0号无人机被队友识别到即rviz中出现绿框， 
然后1号机2号机启动 可以看到所有无人机都被识别到了

###同FAST-LIO2-Implementation-Notes一样，更改triple_drone_mid360.launch里的pcd文件，改为sparse_forest.pcd


#编写运动控制脚本
八字轨迹：伯努利双纽线（Lemniscate of Bernoulli）
Lemniscate.py
Lemniscate1.py
Lemniscate2.py

#记录数据，创建python脚本，编写⼀个轻量级的订阅节点，在程序运⾏时直接将数据写⼊⽂本⽂件
swarm_lio_recorder_estimated.py
swarm_lio_recorder_groundtruth.py

#先记录再启动脚本
# 启动MARSIM多机仿真环境（如三机）
roslaunch test_interface triple_drone_mid360.launch

# 启动Swarm-LIO2
source devel/setup.bash
roslaunch swarm_lio simulation.launch


python3 scripts/swarm_lio_recorder_groundtruth.py 
python3 scripts/swarm_lio_recorder_estimated.py 

python3 scripts/Lemniscate.py
python3 scripts/Lemniscate1.py
python3 scripts/Lemniscate2.py


#分别对比，以无人机0为例
#全局⼀致性评估 (APE - Absolute Pose Error)
评估整条轨迹与真值的重合程度，检查 SLAM 系统是否存在全局地图变形。

evo_ape tum groundtruth_quad0_tum.txt estimated_quad0_tum.txt  -va --plot  --save_plot ape_plot.pdf

evo_ape tum groundtruth_quad0_tum.txt estimated_quad0_tum.txt -va --save_results ape_corridor_01.zip  #(保存为zip）

evo_ape tum groundtruth_quad0_tum.txt estimated_quad0_tum.txt -va 2>&1 | tee ape_results_01.txt


#⾥程计漂移评估 (RPE - Relative Pose Error)
#评估系统在局部运动中的精度，即 “ 每⾛⼀⽶或每过⼀秒产⽣的误差 ”
evo_rpe tum groundtruth_quad0_tum.txt estimated_quad0_tum.txt --plot 
evo_rpe tum groundtruth_quad0_tum.txt estimated_quad0_tum.txt --plot        --save_results rpe_translational.zip 
evo_rpe tum groundtruth_quad0_tum.txt estimated_quad0_tum.txt --plot        --save_plot rpe_translational_plot.pdf
        
# 将详细结果输出到文本文件
evo_rpe tum groundtruth_quad0_tum.txt estimated_quad0_tum.txt -va 2>&1 | tee rpe_translational_results.txt



# 评估旋转误差（角度制）并保存完整结果
evo_rpe tum groundtruth_quad0_tum.txt estimated_quad0_tum.txt -va --plot --pose_relation angle_deg \
        --save_results rpe_rotational.zip \
        --save_plot rpe_rotational_plot.pdf

# 将旋转误差详细结果输出到文本文件
evo_rpe tum groundtruth_quad0_tum.txt estimated_quad0_tum.txt -va --pose_relation angle_deg 2>&1 | tee    rpe_rotational_results.txt


无人机1与无人机2的操作类似


#得到结果，生成csv

使用evo_res命令生成表格
对比ape
evo_res single_ape.zip multi0_ape.zip multi1_ape.zip multi2_ape.zip   --save_table benchmark_results.csv


对比rpe平移
evo_res single_rpe_translation.zip multi0_rpe_translational.zip multi1_rpe_translational.zip multi2_rpe_translational.zip   --save_table rpe_translation.csv


对比rpe旋转

evo_res single_rpe_rotational.zip multi0_rpe_rotational.zip multi1_rpe_rotational.zip multi2_rpe_rotational.zip   --save_table rpe_rotational.csv

#rosbag的操作不多赘述
 最后，得到的数据，文件，图片等均在data文件夹，分析与结论results


#######################
iiwa + 平动二指夹爪仿真平台
修改时间： 2020 / 06 /08

#######################

启动gazebo：roslaunch iiwa_gazebo iiwa_gazebo.launch

#######################

启动move group：roslaunch config move_group.launch

#######################

运行moveit顺向运动学控制：rosrun iiwa_control moveit_fk_demo.py

#######################

运行moveit逆向运动学控制：rosrun iiwa_control moveit_ik_demo.py

#######################

运行逆向运动学+夹爪控制示例：rosrun iiwa_control grasp_demo.py
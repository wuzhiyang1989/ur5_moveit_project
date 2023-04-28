#jump to t_ws
cd
cd t_ws

# rebuild ocrtoc ros
source /home/wy/t_ws/devel/setup.bash
catkin_make -j12 -DCATKIN_WHITELIST_PACKAGES="yolo_detection" -DPYTHON_EXECUTABLE=/home/wy/anaconda3/envs/YOLO/bin/python3
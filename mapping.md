建图流程
1：搜索 map1 ,修改相关参数
ros2 launch robot_bring_up sentry.launch.py
2:保存了多个pcd时:
(1)pcd全放入/home/tc/Desktop/hero_assistant_test/src/hero_assistant/Point-LIO/PCD/merge
(2)编译后运行：ros2 launch merge_pcd merge_pcd.launch.py
3:将得到的总pcd重命名，搜索 map2 ,修改相关参数
ros2 launch robot_bring_up pcd2pgm.launch.py
4:搜索 map3 ,修改相关参数
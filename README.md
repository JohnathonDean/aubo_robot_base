# aubo_robot_base

Launch files:
call_rscamera_apriltag.launch
call_rscamera_artrack.launch
call_rscamera_aruco.launch
    使用realsense相机的color_cam定位AR标签，打开Node：image_draw显示图像并标记

apriltag_fetch.launch
ar_tracker_fetch.launch
aruco_fetch.launch
    基于realsense相机定位AR标签的位姿信息，控制AUBO机械臂末端TCP对齐到AR标签

demo11_test.launch
    基于realsense相机的color_cam定位AR标签物体，使用DH夹爪进行抓取操作测试

dependency package：
    realsense-ros
    apriltag_ros
    aruco_ros
    ar_track_alvar-kinetic-devel
    dh_ag95l    （DH夹爪）

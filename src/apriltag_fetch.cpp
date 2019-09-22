#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "aubo_robot_base/robot_control_base.h"
#include "aubo_robot_base/fetch_marker_id.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

using namespace std;

struct Position
{
    int id;
    Eigen::Vector3d point;
    Eigen::Quaterniond orientation;
};

std::vector<Position> poses;
Position target;
bool goFetch = false;
bool isWorking = false;

Eigen::Isometry3d GetRigidTrans3D(Eigen::Quaterniond q, Eigen::Vector3d d)
{
    Eigen::Isometry3d T(q);
    T.pretranslate(d);
    return T;
}

void poseCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{   
    // ROS_INFO("Receice one message!");
    //初始化poses
    poses.resize(msg->detections.size());
    Position p0;
    p0.id = 0;
    p0.point = Eigen::Vector3d(0, 0, 0);
    p0.orientation = Eigen::Quaterniond(1, 0, 0, 0).normalized();
    poses.assign(msg->detections.size(), p0);

    for (size_t i = 0; i < msg->detections.size(); i++)
    {
        poses[i].id = msg->detections[i].id[0];
        poses[i].point = Eigen::Vector3d(msg->detections[i].pose.pose.pose.position.x,
                                         msg->detections[i].pose.pose.pose.position.y,
                                         msg->detections[i].pose.pose.pose.position.z);
        poses[i].orientation =
            Eigen::Quaterniond(msg->detections[i].pose.pose.pose.orientation.w,
                               msg->detections[i].pose.pose.pose.orientation.x,
                               msg->detections[i].pose.pose.pose.orientation.y,
                               msg->detections[i].pose.pose.pose.orientation.z)
                .normalized();
        // Eigen::Isometry3d T1 = GetRigidTrans3D(poses[i].orientation, poses[i].point);
        // cout << "ID " << poses[i].id << endl;
        // cout << "Isometry3d " << T1.matrix() << endl << endl;
    }
}

bool fetch_apriltagMarkers_Callback(aubo_robot_base::fetch_marker_id::Request &req,
                                    aubo_robot_base::fetch_marker_id::Response &res)
{
    ROS_INFO("request: marker_id=%ld", (long int)req.id);
    if (req.id > 0)
    {
        target.id = req.id;
        goFetch = true;
        res.res = 1;
    }
    else
    {
        res.res = 0;
    }

    ROS_INFO("sending back response: [%ld]", (long int)res.res);
    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "apriltag_fetch");
    ros::NodeHandle n;

    AuboRobotControl Example0;
    Example0.RobotLogin();
    Example0.RobotStartup();

    // Example0.JointMoveToZero();      //机械臂回零点
    double jointAngle1[aubo_robot_namespace::ARM_DOF] = {0};
    AuboRobotControl::initJointAngleArray(
        jointAngle1, 0.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI,
        -90.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI, -90.0 / 180.0 * M_PI,
        0.0 / 180.0 * M_PI); 
    Example0.JointMove(jointAngle1);    //运动到初始位置
    Example0.MoveLtoPosition(0.0, 0.0, 0.10);  

    //获取并打印当前位姿
    aubo_robot_namespace::wayPoint_S currentWaypoint;
    Example0.GetCurrentPosition(currentWaypoint);
    cout << "初始位置路点信息" << endl;
    AuboRobotControl::printWaypoint(currentWaypoint);

    ros::Subscriber sub = n.subscribe("/tag_detections", 10, &poseCallback);
    ros::ServiceServer fetch_service = n.advertiseService("/fetch_apriltagMarkers", fetch_apriltagMarkers_Callback);

    //定义物体基于AR标记坐标系变换矩阵
    Eigen::Vector3d rpy0(0.0 / 180.0 * M_PI, 180.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI); //yaw,pitch,roll
    Eigen::Quaterniond tag0_orientation;
    tag0_orientation = Eigen::AngleAxisd(rpy0[0], Eigen::Vector3d::UnitZ()) * 
                  Eigen::AngleAxisd(rpy0[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(rpy0[2], Eigen::Vector3d::UnitX());
    Eigen::Vector3d tag0_point = Eigen::Vector3d(0.0, 0.0, 0.018);
    Eigen::Isometry3d tag0_isome = GetRigidTrans3D(tag0_orientation, tag0_point);

    //定义物体基于AR标记坐标系变换矩阵 ***带有抓取姿态偏移
    Eigen::Quaterniond tag_orientation = tag0_orientation;
    Eigen::Vector3d tag_point = Eigen::Vector3d(0.0, 0.0, 0.15);
    Eigen::Isometry3d tag_isome = GetRigidTrans3D(tag_orientation, tag_point);

    //定义相机基于工具坐标系变换矩阵
    Eigen::Vector3d rpy1(180.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI); //yaw,pitch,roll
    Eigen::Quaterniond cam_orientation;
    cam_orientation = Eigen::AngleAxisd(rpy1[0], Eigen::Vector3d::UnitZ()) * 
                  Eigen::AngleAxisd(rpy1[1], Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(rpy1[2], Eigen::Vector3d::UnitX());
    // Eigen::Quaterniond cam_orientation = Eigen::Quaterniond(0.5, -0.5, -0.5, -0.5).normalized();
    Eigen::Vector3d cam_point = Eigen::Vector3d(0.033, 0.07, 0.0);
    Eigen::Isometry3d cam_isome = GetRigidTrans3D(cam_orientation, cam_point);

    ros::Rate rate(10.0);

    while (ros::ok()) {
        if(goFetch){
            for(size_t i = 0; i < poses.size(); i++){
                if(poses[i].id == target.id){
                    target.point = poses[i].point;
                    target.orientation = poses[i].orientation;
                    // cout << target.orientation.coeffs() << endl;
                    // cout << target.point.transpose() << endl;
                    Eigen::Isometry3d target_isome = GetRigidTrans3D(target.orientation, target.point);
                    // cout << "target ID " << target.id << endl;
                    // cout << "Isometry3d " << T1.matrix() << endl;

                    Position tcp;
                    Example0.GetCurrentPosition(currentWaypoint);
                    tcp.id = 0;
                    tcp.point = Eigen::Vector3d(currentWaypoint.cartPos.position.x,
                                                currentWaypoint.cartPos.position.y,
                                                currentWaypoint.cartPos.position.z);
                    tcp.orientation = Eigen::Quaterniond(currentWaypoint.orientation.w,
                                                        currentWaypoint.orientation.x,
                                                        currentWaypoint.orientation.y,
                                                        currentWaypoint.orientation.z)
                                        .normalized();
                    Eigen::Isometry3d tcp_isome = GetRigidTrans3D(tcp.orientation, tcp.point);

                    Eigen::Isometry3d target_base = tcp_isome * cam_isome * target_isome * tag_isome;

                    Eigen::Quaterniond q = Eigen::Quaterniond (target_base.rotation());
                    Eigen::Vector3d p = target_base.translation();
                    // cout << q.coeffs() << endl;
                    // cout << p.transpose() << endl;
                    cout << "x y z :" << p(0) << " " << p(1) << " " << p(2) << endl;
                    cout << "w x y z :" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;
                    // cout << target_base.matrix() << endl;
                    Eigen::Vector3d eulerAngle4 = q.matrix().eulerAngles(2,1,0);
                    cout << "yaw(z) pitch(y) roll(x) = " << eulerAngle4.transpose() << endl;

                    aubo_robot_namespace::wayPoint_S targetWaypoint;
                    targetWaypoint = currentWaypoint;
                    targetWaypoint.cartPos.position.x = p(0);
                    targetWaypoint.cartPos.position.y = p(1);
                    targetWaypoint.cartPos.position.z = p(2);
                    targetWaypoint.orientation.x = q.x();
                    targetWaypoint.orientation.y = q.y();
                    targetWaypoint.orientation.z = q.z();
                    targetWaypoint.orientation.w = q.w();
                    Example0.JointMove(targetWaypoint);
                    sleep(2);

                    Eigen::Isometry3d target0_base = tcp_isome * cam_isome * target_isome * tag0_isome;
                    Eigen::Quaterniond q0 = Eigen::Quaterniond (target0_base.rotation());
                    Eigen::Vector3d p0 = target0_base.translation();

                    aubo_robot_namespace::wayPoint_S target0Waypoint;
                    target0Waypoint = currentWaypoint;
                    target0Waypoint.cartPos.position.x = p0(0);
                    target0Waypoint.cartPos.position.y = p0(1);
                    target0Waypoint.cartPos.position.z = p0(2);
                    target0Waypoint.orientation.x = q0.x();
                    target0Waypoint.orientation.y = q0.y();
                    target0Waypoint.orientation.z = q0.z();
                    target0Waypoint.orientation.w = q0.w();
                    Example0.LineMove(target0Waypoint);

                    sleep(8);
                    Example0.JointMove(jointAngle1);    //回到初始位置
                    Example0.MoveLtoPosition(0.0, 0.0, 0.10);  
                    goFetch = false;
                }
            }

        }
        ros::spinOnce(); // allow data update from callback;
        rate.sleep();
    }

    Example0.RobotShutdown();
    return 0;

}


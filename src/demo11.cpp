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

#include "aubo_robot_base/robot_control_base.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "aubo_robot_base/fetch_box_num.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "dh_ag95l/dhGripper.h"
#include "ros/ros.h"

/*方德档案袋抓取demo*/


using namespace std;

class Position {
public:
  int id;
  Eigen::Vector3d point;
  Eigen::Quaterniond orientation;
};

std::vector<Position> poses;
Position target; 
bool spinFlag = true;
bool goFetch = false;
bool isWorking = true;

Eigen::Isometry3d matrix44(Eigen::Quaterniond q, Eigen::Vector3d d)
{
    //    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    //    T.rotate(q); // q.toRotationMatrix()
    Eigen::Isometry3d T(q);
    T.pretranslate(d);
    //    cout << "T:" << endl << T.matrix() << endl;
    return T;
}

void alvarMarkers_Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg) {

  //初始化poses
  poses.resize(msg->markers.size());
  Position p0;
  p0.point = Eigen::Vector3d(0,0,0);
  p0.orientation = Eigen::Quaterniond(1, 0, 0, 0).normalized();
  poses.assign(msg->markers.size(), p0); 

  for (size_t i = 0; i < msg->markers.size(); i++) {
    poses[i].id = msg->markers[i].id;
    poses[i].point = Eigen::Vector3d(msg->markers[i].pose.pose.position.x,
                                     msg->markers[i].pose.pose.position.y,
                                     msg->markers[i].pose.pose.position.z);
    poses[i].orientation =
        Eigen::Quaterniond(msg->markers[i].pose.pose.orientation.w,
                           msg->markers[i].pose.pose.orientation.x,
                           msg->markers[i].pose.pose.orientation.y,
                           msg->markers[i].pose.pose.orientation.z)
            .normalized();
    // Eigen::Isometry3d T1 = matrix44(poses[i].orientation, poses[i].point);
    // cout << "ID " << poses[i].id << endl;
    // cout << "Isometry3d " << T1.matrix() << endl;
  }

}


bool fetch_box(aubo_robot_base::fetch_box_num::Request  &req,
         aubo_robot_base::fetch_box_num::Response &res)
{
    ROS_INFO("request: floor=%ld, box_num=%ld", (long int)req.floor, (long int)req.box_num);

    target.id = req.box_num;

    goFetch = true;
    isWorking = false;

    res.res = 1;
    ROS_INFO("sending back response: [%ld]", (long int)res.res);
    return true;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "demo11");
    ros::NodeHandle n;


    dh_ag95l::dhGripper srv;
    ros::ServiceClient client =
        n.serviceClient<dh_ag95l::dhGripper>("dhGripperServer");
    std_msgs::Bool relay_msg;
    ros::Publisher relaypub = n.advertise<std_msgs::Bool>("relay_fetch", 1);
    
    AuboRobotControl Example0;
    Example0.RobotLogin();
    Example0.RobotStartup();

    Example0.JointMoveToZero();
    double jointAngle1[aubo_robot_namespace::ARM_DOF] = {0};
    AuboRobotControl::initJointAngleArray(
        jointAngle1, -90.0 / 180.0 * M_PI, -30.0 / 180.0 * M_PI,
        -120.0 / 180.0 * M_PI, -90.0 / 180.0 * M_PI, -90.0 / 180.0 * M_PI,
        0.0 / 180.0 * M_PI); //初始位置
    Example0.JointMove(jointAngle1);

    //初始化夹爪
    srv.request.mode = 0;
    srv.request.param = 0;
    if (client.call(srv)) {
    ROS_INFO("Res: %ld", (long int)srv.response.res);
    } else {
    ROS_ERROR("Failed to call service dhgripper! 夹爪初始化失败");
    }

    // 夹爪打开
    srv.request.mode = 2;
    srv.request.param = 55;
    if (client.call(srv)) {
    ROS_INFO("Res: %ld", (long int)srv.response.res);
    } else {
    ROS_ERROR("Failed to call service dhgripper! 夹爪位置控制失败");
    }

    //获取并打印当前位姿
    aubo_robot_namespace::wayPoint_S currentWaypoint;
    Example0.GetCurrentPosition(currentWaypoint);
    cout << "初始位置路点信息" << endl;
    AuboRobotControl::printWaypoint(currentWaypoint);

   
    ros::Subscriber alvarMarkers_sub =
        n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 1, alvarMarkers_Callback);
    ros::ServiceServer service = n.advertiseService("fetchBoxServer", fetch_box);
    ROS_INFO("open fetch service!");

    // target.id = 16;   //设置目标标记的ID

    //定义物体基于AR标记坐标系变换矩阵
    Eigen::Quaterniond tag0_orientation = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).normalized();
    Eigen::Vector3d tag0_point = Eigen::Vector3d(0.0, 0.0, 0.22);    
    Eigen::Isometry3d tag0_isome = matrix44(tag0_orientation, tag0_point);
    //定义物体基于AR标记坐标系变换矩阵 ***带有抓取姿态偏移
    Eigen::Quaterniond tag_orientation = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).normalized();
    Eigen::Vector3d tag_point = Eigen::Vector3d(0.0, 0.0, 0.38);   
    Eigen::Isometry3d tag_isome = matrix44(tag_orientation, tag_point);

    //定义相机基于工具坐标系变换矩阵
    Eigen::Quaterniond cam_orientation = Eigen::Quaterniond(0.5, -0.5, -0.5, -0.5).normalized();
    Eigen::Vector3d cam_point = Eigen::Vector3d(0.037, 0.19, 0.0);
    Eigen::Isometry3d cam_isome = matrix44(cam_orientation, cam_point);

    ros::Rate rate(20.0);
   
    while (ros::ok() && spinFlag) {
        if(goFetch){
            ROS_INFO("Start fetch box!");
            isWorking =false;
            relay_msg.data = isWorking;
            relaypub.publish(relay_msg);
            for(size_t i = 0; i < poses.size(); i++){
                if(poses[i].id == target.id){
                    target.point = poses[i].point;
                    target.orientation = poses[i].orientation;
                    cout << target.orientation.coeffs() << endl;
                    cout << target.point.transpose() << endl;
                    Eigen::Isometry3d target_isome = matrix44(target.orientation, target.point);
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
                    Eigen::Isometry3d tcp_isome = matrix44(tcp.orientation, tcp.point);

                    Eigen::Isometry3d target_base = tcp_isome * cam_isome * target_isome * tag_isome;

                    Eigen::Quaterniond q = Eigen::Quaterniond (target_base.rotation());
                    Eigen::Vector3d p = target_base.translation();
                    // cout << q.coeffs() << endl;
                    // cout << p.transpose() << endl;
                    cout << p(0) << " " << p(1) << " " << p(2) << endl;
                    cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
                    cout << target_base.matrix() << endl;

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

                    // 夹爪夹紧
                    srv.request.mode = 2;
                    srv.request.param = 33;
                    if (client.call(srv)) {
                    ROS_INFO("Res: %ld", (long int)srv.response.res);
                    } else {
                    ROS_ERROR("Failed to call service dhgripper! 夹爪位置控制失败");
                    }

                    sleep(2);
                    Example0.MoveLtoPosition(0.0, 0.0, 0.01);
                    Example0.MoveLtoPosition(0.0, 0.0, -0.01);

                    //夹爪打开
                    srv.request.mode = 2;
                    srv.request.param = 55;
                    if (client.call(srv)) {
                    ROS_INFO("Res: %ld", (long int)srv.response.res);
                    } else {
                    ROS_ERROR("Failed to call service dhgripper! 夹爪位置控制失败");
                    }


                    Example0.LineMove(targetWaypoint);
                    Example0.MoveLtoPosition(0.0, -0.15, 0.0);


                    sleep(2);
                    Example0.JointMove(jointAngle1);
                    // Example0.MoveLtoPosition(0.0, -0.17, 0.0);  
                    sleep(8);
                    isWorking =true;
                    relay_msg.data = isWorking;
                    relaypub.publish(relay_msg);

                    goFetch = false;
                    // spinFlag = false; //关闭线程循环回调
                }
            }

        }
        relay_msg.data = isWorking;
        relaypub.publish(relay_msg);
        ros::spinOnce(); // allow data update from callback;
        rate.sleep();
    }

    Example0.RobotShutdown();
    return 0;

}

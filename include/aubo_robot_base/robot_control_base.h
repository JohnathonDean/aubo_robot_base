#ifndef ROBOT_CONTROL_BASE_H
#define ROBOT_CONTROL_BASE_H

#include "aubo_robot_base/AuboRobotMetaType.h"
#include "aubo_robot_base/serviceinterface.h"

class AuboRobotControl
{
public:
    AuboRobotControl();
    // ~AuboRobotControl();
    ServiceInterface robotService; //实例化一个AUBO服务接口的对象

public:
    /** 打印路点信息 **/
    static void printWaypoint(aubo_robot_namespace::wayPoint_S &wayPoint);
    /** 打印关节状态信息 **/
    static void printJointStatus(const aubo_robot_namespace::JointStatus *jointStatus, int len);
    /** 打印事件信息 **/
    static void printEventInfo(const aubo_robot_namespace::RobotEventInfo &eventInfo);
    /** 打印诊断信息 **/
    static void printRobotDiagnosis(const aubo_robot_namespace::RobotDiagnosis &robotDiagnosis);
    /** 用于获取实时路点回调函数 **/
    static void RealTimeWaypointCallback (const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg); 
    /** 获取实时末端速度回调函数 **/
    static void RealTimeEndSpeedCallback (double speed, void *arg);  
    /** 获取实时机械臂事件回调函数 **/
    static void RealTimeEventInfoCallback(const aubo_robot_namespace::RobotEventInfo *pEventInfo, void *arg); 

private:
    /** 实时关节状态回调函数　**/
    static void RealTimeJointStatusCallback (const aubo_robot_namespace::JointStatus *jointStatus, int size, void *arg);

public:
    /** 关节角度向量赋值函数　**/
    static void initJointAngleArray(double *array, double joint0,double joint1,double joint2,double joint3,double joint4,double joint5);
    /** 四元素转欧拉角**/
    static void GetQuaternionToRPY(aubo_robot_namespace::Ori &ori, aubo_robot_namespace::Rpy &rpy);
    /** 欧拉角转四元素**/
    static void GetRPYToQuaternion(aubo_robot_namespace::Rpy &rpy, aubo_robot_namespace::Ori &ori);


public:
    /** 机械臂控制函数　**/
    void RobotLogin();
    void RobotStartup();
    void RobotShutdown();
    void GetRobotStatus();
    void JointMove(double *jointAngle);
    void JointMove(aubo_robot_namespace::wayPoint_S &wayPoint);
    void JointMoveToZero();
    void LineMove(double *jointAngle);
    void LineMove(aubo_robot_namespace::wayPoint_S &wayPoint);
    void MoveLtoPosition(double offsetX, double offsetY, double offsetZ);

    /** 获取当前机械臂位姿函数 **/
    void GetCurrentPosition(aubo_robot_namespace::wayPoint_S &currentWaypoint);


};

#endif // ROBOT_CONTROL_BASE_H

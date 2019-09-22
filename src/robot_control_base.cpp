#include "aubo_robot_base/robot_control_base.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>


#define SERVER_HOST "192.168.0.123"
#define SERVER_PORT 8899


AuboRobotControl::AuboRobotControl()
{
}

/** 实时路点回调函数　**/
void AuboRobotControl::RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    (void)arg;
    aubo_robot_namespace::wayPoint_S waypoint = *wayPointPtr;
    printWaypoint(waypoint);
}

/** 实时关节状态回调函数　**/
void AuboRobotControl::RealTimeJointStatusCallback(const aubo_robot_namespace::JointStatus *jointStatus, int size, void *arg)
{
    (void)arg;
    printJointStatus(jointStatus, size);
}

/** 实时末端速度回调函数　**/
void AuboRobotControl::RealTimeEndSpeedCallback(double speed, void *arg)
{
    (void)arg;
    std::cout<<"实时末端速度:"<<speed<<std::endl;
}

/** 实时机械臂事件回调函数　**/
void AuboRobotControl::RealTimeEventInfoCallback(const aubo_robot_namespace::RobotEventInfo *pEventInfo, void *arg)
{
    (void)arg;
    printEventInfo(*pEventInfo);
}

//打印路点信息
void AuboRobotControl::printWaypoint(aubo_robot_namespace::wayPoint_S &wayPoint)
{
    std::cout<<std::endl<<"start-------------路点信息---------------"<<std::endl;
    //位置信息
    std::cout<<"x:"<<wayPoint.cartPos.position.x<<"  ";
    std::cout<<"y:"<<wayPoint.cartPos.position.y<<"  ";
    std::cout<<"z:"<<wayPoint.cartPos.position.z<<std::endl;

    //姿态信息
    std::cout<<"w:"<<wayPoint.orientation.w<<"  ";
    std::cout<<"x:"<<wayPoint.orientation.x<<"  ";
    std::cout<<"y:"<<wayPoint.orientation.y<<"  ";
    std::cout<<"z:"<<wayPoint.orientation.z<<std::endl;

    //    aubo_robot_namespace::Rpy tempRpy;
    //    robotService.quaternionToRPY(wayPoint.orientation,tempRpy);
    //    std::cout<<"RX:"<<tempRpy.rx<<"  RY:"<<tempRpy.ry<<"   RZ:"<<tempRpy.rz<<std::endl;

    //关节信息
    for(int i=0;i<aubo_robot_namespace::ARM_DOF;i++)
    {
        std::cout<<"joint"<<i+1<<":"<<wayPoint.jointpos[i]<<" | "<<wayPoint.jointpos[i]*180.0/M_PI<<std::endl;
    }
}


//打印关节状态信息
void AuboRobotControl::printJointStatus(const aubo_robot_namespace::JointStatus *jointStatus, int len)
{
    std::cout<<std::endl<<"start----------关节状态信息-------" << std::endl;

    for(int i=0; i<len; i++)
    {
        std::cout<<"关节ID:"   <<i<<"  " ;
        std::cout<<"电流:"     <<jointStatus[i].jointCurrentI<<" ";
        std::cout<<"速度:"     <<jointStatus[i].jointSpeedMoto<<" ";
        std::cout<<"关节角:"   <<jointStatus[i].jointPosJ<<" ";
        std::cout<<"电压   :"  <<jointStatus[i].jointCurVol<<" ";
        std::cout<<"温度   :"  <<jointStatus[i].jointCurTemp<<" ";
        std::cout<<"目标电流:"  <<jointStatus[i].jointTagCurrentI<<" ";
        std::cout<<"目标电机速度:" <<jointStatus[i].jointTagSpeedMoto<<" ";
        std::cout<<"目标关节角 :"  <<jointStatus[i].jointTagPosJ<<" ";
        std::cout<<"关节错误   :"  <<jointStatus[i].jointErrorNum <<std::endl;
    }
    std::cout<<std::endl;
}


//打印事件信息
void AuboRobotControl::printEventInfo(const aubo_robot_namespace::RobotEventInfo &eventInfo)
{
    std::cout<<"事件类型:"<<eventInfo.eventType <<"  code:"<<eventInfo.eventCode<<"  内容:"<<eventInfo.eventContent<<std::endl;
}


void AuboRobotControl::printRobotDiagnosis(const aubo_robot_namespace::RobotDiagnosis &robotDiagnosis)
{
    std::cout<<std::endl<<"start----------机械臂统计信息-------" << std::endl;

    std::cout<<std::endl<<"   "<<"CAN通信状态:"<<(int)robotDiagnosis.armCanbusStatus;
    std::cout<<std::endl<<"   "<<"电源当前电流:"<<robotDiagnosis.armPowerCurrent;
    std::cout<<std::endl<<"   "<<"电源当前电压:"<<robotDiagnosis.armPowerVoltage;

    (robotDiagnosis.armPowerStatus)? std::cout<<std::endl<<"   "<<"48V电源状态:开":std::cout<<std::endl<<"   "<<"48V电源状态:关";

    std::cout<<std::endl<<"   "<<"控制箱温度:"<<(int)robotDiagnosis.contorllerTemp;
    std::cout<<std::endl<<"   "<<"控制箱湿度:"<<(int)robotDiagnosis.contorllerHumidity;
    std::cout<<std::endl<<"   "<<"远程关机信号:"<<robotDiagnosis.remoteHalt;
    std::cout<<std::endl<<"   "<<"机械臂软急停:"<<robotDiagnosis.softEmergency;
    std::cout<<std::endl<<"   "<<"远程急停信号:"<<robotDiagnosis.remoteEmergency;
    std::cout<<std::endl<<"   "<<"碰撞检测位:"<<robotDiagnosis.robotCollision;
    std::cout<<std::endl<<"   "<<"进入力控模式标志位:"<<robotDiagnosis.forceControlMode;
    std::cout<<std::endl<<"   "<<"刹车状态:"<<robotDiagnosis.brakeStuats;
    std::cout<<std::endl<<"   "<<"末端速度:"<<robotDiagnosis.robotEndSpeed;
    std::cout<<std::endl<<"   "<<"最大加速度:"<<robotDiagnosis.robotMaxAcc;
    std::cout<<std::endl<<"   "<<"上位机软件状态位:"<<robotDiagnosis.orpeStatus;
    std::cout<<std::endl<<"   "<<"位姿读取使能位:"<<robotDiagnosis.enableReadPose;
    std::cout<<std::endl<<"   "<<"安装位置状态:"<<robotDiagnosis.robotMountingPoseChanged;
    std::cout<<std::endl<<"   "<<"磁编码器错误状态:"<<robotDiagnosis.encoderErrorStatus;
    std::cout<<std::endl<<"   "<<"静止碰撞检测开关:"<<robotDiagnosis.staticCollisionDetect;
    std::cout<<std::endl<<"   "<<"关节碰撞检测:"<<robotDiagnosis.jointCollisionDetect;
    std::cout<<std::endl<<"   "<<"光电编码器不一致错误:"<<robotDiagnosis.encoderLinesError;
    std::cout<<std::endl<<"   "<<"关节错误状态:"<<robotDiagnosis.jointErrorStatus;
    std::cout<<std::endl<<"   "<<"奇异点过速警告:"<<robotDiagnosis.singularityOverSpeedAlarm;
    std::cout<<std::endl<<"   "<<"电流错误警告:"<<robotDiagnosis.robotCurrentAlarm;
    std::cout<<std::endl<<"   "<<"tool error:"<<(int)robotDiagnosis.toolIoError;
    std::cout<<std::endl<<"   "<<"安装位置错位:"<<robotDiagnosis.robotMountingPoseWarning;
    std::cout<<std::endl<<"   "<<"mac缓冲器长度:"<<robotDiagnosis.macTargetPosBufferSize;
    std::cout<<std::endl<<"   "<<"mac缓冲器有效数据长度:"<<robotDiagnosis.macTargetPosDataSize;
    std::cout<<std::endl<<"   "<<"mac数据中断:"<<robotDiagnosis.macDataInterruptWarning;

    std::cout<<std::endl<<"----------------------------------end."<<std::endl;
}

void AuboRobotControl::initJointAngleArray(double *array, double joint0, double joint1, double joint2, double joint3, double joint4, double joint5)
{
    array[0] = joint0;
    array[1] = joint1;
    array[2] = joint2;
    array[3] = joint3;
    array[4] = joint4;
    array[5] = joint5;
}

void AuboRobotControl::GetQuaternionToRPY(aubo_robot_namespace::Ori &ori, aubo_robot_namespace::Rpy &rpy)
{
    ServiceInterface tran;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用 ***/
    ret = tran.quaternionToRPY(ori, rpy);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<" RX "<< rpy.rx <<" RY "<< rpy.ry <<" RZ "<< rpy.rz <<std::endl;
    }
    else
    {
        std::cerr<<"转换失败."<<std::endl;
    }
}

void AuboRobotControl::GetRPYToQuaternion(aubo_robot_namespace::Rpy &rpy, aubo_robot_namespace::Ori &ori)
{
    ServiceInterface tran;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用 ***/
    ret = tran.RPYToQuaternion(rpy, ori);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<" W "<< ori.w <<" X "<< ori.x <<" Y "<< ori.y <<" Z "<< ori.z <<std::endl;
    }
    else
    {
        std::cerr<<"转换失败."<<std::endl;
    }
}


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


void AuboRobotControl::RobotLogin()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录失败."<<std::endl;
    }

}

void AuboRobotControl::RobotStartup()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"机械臂初始化成功."<<std::endl;
    }
    else
    {
        std::cerr<<"机械臂初始化失败."<<std::endl;
    }

}

void AuboRobotControl::RobotShutdown()
{

    /** 机械臂Shutdown断电 **/
    robotService.rootServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}

void AuboRobotControl::GetRobotStatus()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 获取真实臂是否存在 **/
    bool IsRealRobotExist = false;
    ret = robotService.robotServiceGetIsRealRobotExist(IsRealRobotExist);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        (IsRealRobotExist)? std::cout<<"真实臂存在."<<std::endl:std::cout<<"真实臂不存在."<<std::endl;
        std::cout<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:获取机械臂真实臂是否存在失败."<<std::endl;
    }

    /** 接口调用: 主动获取关节状态 **/
    aubo_robot_namespace::JointStatus jointStatus[6];
    ret = robotService.robotServiceGetRobotJointStatus(jointStatus, 6);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        printJointStatus(jointStatus, 6);
    }
    else
    {
        std::cerr<<"ERROR:获取关节状态失败."<<"ret:"<<ret<<std::endl;
    }

    /** 接口调用: 机械臂诊断信息 **/
    aubo_robot_namespace::RobotDiagnosis robotDiagnosisInfo;
    ret = robotService.robotServiceGetRobotDiagnosisInfo(robotDiagnosisInfo);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        printRobotDiagnosis(robotDiagnosisInfo);
    }
    else
    {
        std::cerr<<"ERROR:获取机械臂诊断信息失败."<<std::endl;
    }
    sleep(120);

}

void AuboRobotControl::JointMove(double *jointAngle)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** 接口调用: 设置关节型运动的最大速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    /** 机械臂关节运动到输入关节位置 **/
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动至输入关节位置失败.　ret:"<<ret<<std::endl;
    }
    std::cerr<<"开始运动到关节位置"<<std::endl;
}

void AuboRobotControl::JointMove(aubo_robot_namespace::wayPoint_S &wayPoint)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** 接口调用: 设置关节型运动的最大速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    aubo_robot_namespace::wayPoint_S targetWayPoint;
    aubo_robot_namespace::Pos targetPos;
    aubo_robot_namespace::Ori targetOri;
    targetPos.x = wayPoint.cartPos.position.x;
    targetPos.y = wayPoint.cartPos.position.y;
    targetPos.z = wayPoint.cartPos.position.z;
    targetOri.w = wayPoint.orientation.w;
    targetOri.x = wayPoint.orientation.x;
    targetOri.y = wayPoint.orientation.y;
    targetOri.z = wayPoint.orientation.z;

    /** 获取当前位姿信息 **/
    aubo_robot_namespace::wayPoint_S currentWaypoint;
    ret = robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"获取当前位姿信息失败.　ret:"<<ret<<std::endl;
    }

    /** 获取当前关节角 **/
    double startPointJointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    startPointJointAngle[0] = currentWaypoint.jointpos[0];
    startPointJointAngle[1] = currentWaypoint.jointpos[1];
    startPointJointAngle[2] = currentWaypoint.jointpos[2];
    startPointJointAngle[3] = currentWaypoint.jointpos[3];
    startPointJointAngle[4] = currentWaypoint.jointpos[4];
    startPointJointAngle[5] = currentWaypoint.jointpos[5];

    /** 调用逆解函数 **/
    ret = robotService.robotServiceRobotIk(startPointJointAngle, targetPos, targetOri, targetWayPoint);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"调用逆解函数失败"<<std::endl;
    }

    /** 机械臂关节运动到输入位置 **/
    ret = robotService.robotServiceJointMove(targetWayPoint, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动至输入关节位置失败.　ret:"<<ret<<std::endl;
    }
    std::cerr<<"开始运动到关节位置"<<std::endl;
}

void AuboRobotControl::JointMoveToZero()
{
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    initJointAngleArray(jointAngle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    JointMove(jointAngle);
    std::cerr<<"关节开始运动到零位置"<<std::endl;

}

void AuboRobotControl::LineMove(double *jointAngle)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 1;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 1;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
    robotService.robotServiceGetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);

    /** 机械臂关节运动到输入关节位置 **/
    ret = robotService.robotServiceLineMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动至输入关节位置失败.　ret:"<<ret<<std::endl;
    }
    std::cerr<<"开始运动到关节位置"<<std::endl;
}

void AuboRobotControl::LineMove(aubo_robot_namespace::wayPoint_S &wayPoint)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 1;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 1;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
    robotService.robotServiceGetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);

    aubo_robot_namespace::wayPoint_S targetWayPoint;
    aubo_robot_namespace::Pos targetPos;
    aubo_robot_namespace::Ori targetOri;
    targetPos.x = wayPoint.cartPos.position.x;
    targetPos.y = wayPoint.cartPos.position.y;
    targetPos.z = wayPoint.cartPos.position.z;
    targetOri.w = wayPoint.orientation.w;
    targetOri.x = wayPoint.orientation.x;
    targetOri.y = wayPoint.orientation.y;
    targetOri.z = wayPoint.orientation.z;

    /** 获取当前位姿信息 **/
    aubo_robot_namespace::wayPoint_S currentWaypoint;
    ret = robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"获取当前位姿信息失败.　ret:"<<ret<<std::endl;
    }

    /** 获取当前关节角 **/
    double startPointJointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    startPointJointAngle[0] = currentWaypoint.jointpos[0];
    startPointJointAngle[1] = currentWaypoint.jointpos[1];
    startPointJointAngle[2] = currentWaypoint.jointpos[2];
    startPointJointAngle[3] = currentWaypoint.jointpos[3];
    startPointJointAngle[4] = currentWaypoint.jointpos[4];
    startPointJointAngle[5] = currentWaypoint.jointpos[5];

    /** 调用逆解函数 **/
    ret = robotService.robotServiceRobotIk(startPointJointAngle, targetPos, targetOri, targetWayPoint);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"调用逆解函数失败"<<std::endl;
    }

    /** 机械臂关节运动到输入位置 **/
    ret = robotService.robotServiceLineMove(targetWayPoint, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动至输入关节位置失败.　ret:"<<ret<<std::endl;
    }
    std::cerr<<"开始运动到关节位置"<<std::endl;
}

void AuboRobotControl::MoveLtoPosition(double offsetX, double offsetY, double offsetZ)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 1;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);

    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动 ***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 1;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
    /** 获取当前路点信息 **/
    aubo_robot_namespace::wayPoint_S currentWaypoint;
    ret = robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"获取当前路点信息失败.　ret:"<<ret<<std::endl;
    }
    /** 设置坐标系参数 **/
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    /** 设置工具端参数 **/
    aubo_robot_namespace::ToolInEndDesc toolDesc;
    toolDesc.toolInEndPosition.x = 0;
    toolDesc.toolInEndPosition.y = 0;
    toolDesc.toolInEndPosition.z = 0;

    toolDesc.toolInEndOrientation.w=1;
    toolDesc.toolInEndOrientation.x=0;
    toolDesc.toolInEndOrientation.y=0;
    toolDesc.toolInEndOrientation.z=0;

    aubo_robot_namespace::Pos position;
    position.x = currentWaypoint.cartPos.position.x+offsetX;
    position.y = currentWaypoint.cartPos.position.y+offsetY;
    position.z = currentWaypoint.cartPos.position.z+offsetZ;

    //保持当前位姿通过直线运动的方式运动到目标位置
    ret = robotService.robotMoveLineToTargetPosition(userCoord, position, toolDesc, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"robotMoveLineToTargetPosition.　ret:"<<ret<<std::endl;
    }

}

void AuboRobotControl::GetCurrentPosition(aubo_robot_namespace::wayPoint_S &currentWaypoint)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;
    /** 获取当前位姿信息 **/
    ret = robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"获取当前位姿信息失败.　ret:"<<ret<<std::endl;
    }

}




/**
 * @file robot_interface.h
 * @brief
 * @mainpage Innfos Gluon API Instructions
 * @author innfos
 * @email www.innfos.com
 * @version 1.0.0
 * @date 2019-12-02
 * @update 2019-12-03
 * @license 
 * @note
 * @history:
 */

#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "mechnical_unit.h"

/**
 * @brief 以关节插补的形式移动机械臂到目标点
 * @detail 运动快慢可通过SetMaxJointVelocity、SetMaxJointAcceleration和SetMaxJointAcceleration函数来调节
 * 
 * @param target_joint 目标点的关节角度，单位为弧度，六轴机械臂维数为6，亦可通过GetAxisNum获取
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int MoveToTargetJoint(ActuatorGroup *robot, const double target_joint[]);

/**
 * @brief 以关节插补的形式增量移动机械臂
 * @detail 运动快慢可通过SetMaxJointVelocity、SetMaxJointAcceleration和SetMaxJointAcceleration函数来调节
 * 
 * @param inc_joint 增量关节角度，单位为弧度，六轴机械臂维数为6，亦可通过GetAxisNum获取
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int MoveJointIncremental(ActuatorGroup *robot, const double inc_joint[]);

/**
 * @brief 以直线插补的形式移动机械臂到目标点
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 * 
 * @param target_pose 目标点的位姿，前三维为位置单位毫米，后四维为姿态,表述方式为单位四元数
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int MoveLineToTargetPose(ActuatorGroup *robot,const double target_pose[7]);

/**
 * @brief 以圆弧插补的形式移动机械臂到目标点
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 *
 * @param auxiliary_pose 圆弧辅助点的位姿，前三维为位置单位毫米，后四维为姿态,表述方式为单位四元数
 * @param target_pose	 圆弧目标点的位姿，前三维为位置单位毫米，后四维为姿态,表述方式为单位四元数
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int MoveArcToTargetPose(ActuatorGroup *robot,const double auxiliary_pose[7],const double target_pose[7]);


/**
 * @brief  沿着x轴平移
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 * 
 * @param  x为移动的距离，单位为mm
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int TranslateXaxis(ActuatorGroup * robot,double x);

/**
 * @brief  沿着y轴平移
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 * 
 * @param  y为移动的距离，单位为mm
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int TranslateYaxis(ActuatorGroup * robot,double y);

/**
 * @brief  沿着z轴平移
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 * 
 * @param  z为移动的距离，单位为mm
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int TranslateZaxis(ActuatorGroup * robot,double z);

/**
 * @brief  绕x轴旋转
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 * 
 * @param  rx为旋转角度，单位为弧度
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int RotateXAxis(ActuatorGroup * robot,double rx);

/**
 * @brief  绕y轴旋转
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 * 
 * @param  ry为旋转角度，单位为弧度
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int RotateYAxis(ActuatorGroup * robot,double ry);

/**
 * @brief  绕z轴旋转
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 * 
 * @param  rz为旋转角度，单位为弧度
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int RotateZAxis(ActuatorGroup * robot,double rz);


#endif

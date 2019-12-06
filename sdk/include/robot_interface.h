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
 * @warning
 * @todo
 * @history:
 */

#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "mechnical_unit.h"

/**
 * @brief 以关节插补的形式移动机械臂到目标点
 * @detail 运动快慢可通过SetMaxJointVelocity、SetMaxJointAcceleration和SetMaxJointAcceleration函数来调节
 * 
 * @param target_joint 目标点的关节角度，单位为弧度
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int MoveToTargetJoint(ActuatorGroup *robot, const double target_joint[]);

/**
 * @brief 以关节插补的形式移动机械臂到目标点
 * @detail 运动快慢可通过SetMaxJointVelocity、SetMaxJointAcceleration和SetMaxJointAcceleration函数来调节
 * 
 * @param target_joint 目标点的关节角度，单位为弧度
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int MoveJointIncremental(ActuatorGroup *robot, const double inc_joint[]);


/**
 * @brief 以直线插补的形式从当前点增量移动
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 * 
 * @param target_joint 目标点的关节角度，单位为弧度
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int MoveLineIncremental(ActuatorGroup *robot,const double inc_pose[]);

/**
 * @brief 以直线插补的形式移动机械臂到目标点
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 * 
 * @param target_joint 目标点的关节角度，单位为弧度
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int MoveLineToTargetPose(ActuatorGroup *robot,const double target_pose[]);


/**
 * @brief 以圆弧插补的形式移动机械臂到目标点
 * @detail 位置运动快慢可通过SetMaxLineJerk、SetMaxLineAcceleration和SetMaxLineJerk来调节
 * @detail 姿态运动快慢可通过SetMaxAngularVelocity、SetMaxAngularAcceleration和SetMaxAngularJerk来调节
 *
 * @param auxiliary_joint 圆弧辅助点的关节角度，单位为弧度
 * @param target_joint	  圆弧目标点的关节角度，单位为弧度
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int MoveArcToTargetPose(ActuatorGroup *robot,const double auxiliary_pose[],const double target_pose[]);



#endif

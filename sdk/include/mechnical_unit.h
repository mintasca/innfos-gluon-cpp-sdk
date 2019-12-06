#ifndef MECHNICAL_UNIT_H
#define MECHNICAL_UNIT_H

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


#include "base.h"
#include <vector>
#include "actuatorcontroller.h"

/**
 * @brief 和执行器建立通讯
 * @detail 最优先函数，即要对机械臂进行任何操作需要先执行此函数
 * 
 * @return 返回说明
 *	 @retval 0   执行成功
 *	 @retval 非0 执行失败
 */
int EstablishConnection();

#if 0
namespace InnfosRobotAPI
{
	enum CoordinateSystem 
	{
		BASE_COORD,
		TOOL_COORD,
		USER_COORD,
		WORLD_COORD,
		JOINT_COORD, 
	};
};
#endif

typedef struct AccDecDate_
{
	double vel;
	double acc;
	double jerk;
}AccDecDate;

enum RobotType
{
	GL_4L3,
	GL_6L3,
	GL_2P8_P6_3P3,
	GL_6L6,
	GL_2L6_4L3,
	LINK_MECHANISM,
	FREE_COMBINATION,
};

enum ActuatorType
{
	QDD_LITE_PR60_36,
	QDD_LITE_NE30_36,
	QDD_LITE_EL20_36,
	QDD_PR60_36,
	QDD_NE30_36,
	QDD_EL20_36,
	QDD_PRO_NU80_100_110,
	QDD_PRO_PR60_80_90,
	QDD_PRO_NE30_50_70,
	ACTUATOR_FREE,
};

typedef struct LinkUnit_
{
	uint8_t id_; //sca id
	ActuatorType type_;//sca type
	/*get from sca type*/
	double reduction_rate_;
	double kt_;//torque constant
	int direct_; //move direct 1:-1 default:1 
	/*sca parameters,get from sca*/
	double max_motor_speed_;//transfer to axis
	int joint_type_;//0:revolute, 1:prismatic defult:0
	double home_position_;
	/*dynamics parameters*/
	double m_; //mass
	double p_[3];// position of center-of-mass(in the link coordinantes)	
	double Ixx_;//inertia Ixx
	double Ixy_;//inertia Ixy
	double Ixz_;//inertia Ixz
	double Iyy_;//inertia Iyy
	double Iyz_;//inertia Iyz
	double Izz_;//inertia Izz
	double fv_;// viscous friction
	double fc_; //coulomb friction
	double Jm_;//motor inertia 
	AccDecDate joint_speed_;
	double min_soft_limit_;
	double max_soft_limit_;
	/*kinematics MDH parameters*/
	double theta_;
	double alpha_;
	double a_;
	double d_;
}LinkUnit;

class ActuatorGroup
{
private:
	int axis_num_;
	RobotType type_;
	AccDecDate rot_speed_;
	AccDecDate line_speed_;
	vector <uint8_t> idArray;
	ActuatorController *pointer_;
	LinkUnit link_data_[MAX_AXIS_NUM];
	double realtime_pos_[MAX_AXIS_NUM];
	vector< ActuatorController::UnifiedID > unified_id_array_;
	int GetParametersFromActuator();
	int GetParametersFromDatabase();	

public:

	ActuatorGroup();

	/**
	 * @brief 启动机械臂，运行其它函数接口的前提均为机械臂处于开机状态
	 * @detail 使能执行器，并初始化机械臂参数
	 * 
	 * @return 返回说明
	 *	 @retval 0	 执行成功
	 *	 @retval 非0 执行失败
	 */
	int Initialization(const RobotType type=GL_6L3);

	/**
	 * @brief 关闭机械臂，在断电之前请务必执行此函数
	 * @detail 失能执行器
	 * 
	 */

	void Shutdown();

	/**
	 * @brief 设置机械臂最大关节速度
	 * 
	 * @param vel 需要设置的值,单位rad/s
	 */
	void SetMaxJointVelocity(const        double vel[]);


	/**
	 * @brief 设置机械臂最大关节加速度
	 * 
	 * @param acc 需要设置的值,单位rad/s^2
	 */
	void SetMaxJointAcceleration(const double acc[]);

	/**
	 * @brief 设置机械臂最大关节加加速度
	 * 
	 * @param jerk 需要设置的值,单位rad/s^3
	 */
	void SetMaxJointJerk(const double jerk[]);

	/**
	 * @brief 设置机械臂最大线速度
	 * 
	 * @param vel 需要设置的值,单位mm/s
	 */
	void SetMaxLineVelocity(double vel);

	/**
	 * @brief 设置机械臂最大线加速度
	 * 
	 * @param acc 需要设置的值,单位mm/s^2
	 */
	void SetMaxLineAcceleration(double acc);

	/**
	 * @brief 设置机械臂最大线加加速度
	 * 
	 * @param jerk 需要设置的值,单位mm/s^3
	 */
	void SetMaxLineJerk(double jerk);

	/**
	 * @brief 设置机械臂最大姿态角速度
	 * 
	 * @param vel 需要设置的值,单位rad/s
	 */
	void SetMaxAngularVelocity(double vel);

	/**
	 * @brief 设置机械臂最大姿态角加速度
	 * 
	 * @param acc 需要设置的值,单位rad/s^2
	 */
	void SetMaxAngularAcceleration(double acc);

	/**
	 * @brief 设置机械臂最大姿态角加加速度
	 * 
	 * @param jerk 需要设置的值,单位rad/s^3
	 */
	void SetMaxAngularJerk(double jerk);

	/**
	 * @brief 获取机械臂最大关节速度
	 * 
	 * @param vel 需要获取的值,单位rad/s
	 */
	void GetMaxJointVelocity(double vel[]);

	/**
	 * @brief 获取机械臂最大关节加速度
	 * 
	 * @param acc 需要获取的值,单位rad/s^2
	 */
	void GetMaxJointAcceleration(double acc[]);

	/**
	 * @brief 获取机械臂最大关节加加速度
	 * 
	 * @param jerk 需要获取的值,单位rad/s^3
	 */
	void GetMaxJointJerk(double jerk[]);

	/**
	 * @brief 获取机械臂最大线速度
	 * 
	 * @param vel 需要获取的值,单位mm/s
	 */
	void GetMaxLineVelocity(double &vel);

	/**
	 * @brief 获取机械臂最大线加速度
	 * 
	 * @param acc 需要获取的值,单位mm/s^2
	 */
	void GetMaxLineAcceleration(double &acc);

	/**
	 * @brief 获取机械臂最大线加加速度
	 * 
	 * @param jerk 需要获取的值,单位mm/s^3
	 */
	void GetMaxLineJerk(double &jerk);

	/**
	 * @brief 获取机械臂姿态角速度
	 * 
	 * @param vel 需要获取的值,单位rad/s
	 */
	void GetMaxAngularVelocity(double &vel);

	/**
	 * @brief 获取机械臂最大姿态角加速度
	 * 
	 * @param acc 需要获取的值,单位rad/s^2
	 */
	void GetMaxAngularAcceleration(double &acc);

	/**
	 * @brief 获取机械臂最大姿态角加加速度
	 * 
	 * @param jerk 需要获取的值,单位rad/s^3
	 */
	void GetMaxAngularJerk(double &jerk);

	/**
	 * @brief 获取机械臂当前的关节角度
	 * 
	 * @param angle 需要获取的值,单位rad
	 */
	void GetCurrentJointAngle(double angle[]);

	/**
	 * @brief 获取机械臂当前的位姿
	 * 
	 * @param pose 需要获取的值,默认为RPY型式
	 */
	void GetCurrentPoseRPY(double pose[6]);

	/**
	 * @brief 获取机械臂的轴数
	 * 
	 * @return 返回说明
	 *	 @retval 机械臂的轴数
	 */
	int GetAxisNum();

	int GetRobotType();
	void GetCurrentCVPFast(double current[],double vel[],double pos[]);
	void GetCurrentTVPFast(double tau[],double vel[],double pos[]);
	void GetCurrentJointAngleFast(double angle[]);
	void GetJointSpeedAccDecParameters(AccDecDate *joint_speed);
	void GetLineSpeedAccDecParameters(AccDecDate *line_speed);
	void GetRotateSpeedAccDecParameters(AccDecDate *rot_speed);
	void GetMinLimitData(double min_limit[]);
	void GetMaxLimitData(double max_limit[]);
	int SaveParameters();
	void ActivateHomingMode();
	void ActivateCurrentMode();
	void ActivateVelocityMode();
	void ActivatePositionMode();
	void RequestCVPValue();
	int SetPosition(double joint[],double *last_joint=NULL,bool check=false);
	int SetTorque(double tau[]);
	int SetHomingPosition();
	int SetLimitData(double min_pos[],double max_pos[]);	
	int SetLockEnergy(double energy[]);
	int SetPositionPidData(double kp[],double ki[],double kd[]);
	int SetVelocityPiData(double kp[],double ki[]);
	int ForwardKinematics(double q[],double pose[]);
	int InverseKinematics(double q[],const double pose[]);
	int ForwardDynamics();
	int InverseDynamics(double q[],double qd[],double qdd [],double tau[]);
	int GravityCompensation(double q[],double qd[]);
	void SetRealtimePos(const double value[]);
	void GetRealtimePos(double value[]);

	~ActuatorGroup();

};

void DisableAllActuatorGroup();


#endif

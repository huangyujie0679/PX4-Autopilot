#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_target_los.h>
#include <uORB/topics/vehicle_guidance_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <matrix/matrix/math.hpp>

#include "acceleration_to_attitude.hpp"


using namespace matrix;


class PnGuidance :
	public ModuleBase<PnGuidance>,
	public px4::ScheduledWorkItem
{

public:

	PnGuidance() :
		ScheduledWorkItem(
			MODULE_NAME,
			px4::wq_configurations::nav_and_controllers)
	{
	}


	~PnGuidance() override = default;


	/*
	 * ============================================================
	 * 创建模块实例
	 * ============================================================
	 */

	static int task_spawn(
		int argc,
		char *argv[])
	{
		PnGuidance *instance = new PnGuidance();

		if (instance == nullptr) {
			return -1;
		}

		_object.store(instance);

		/*
		 * 50 Hz
		 *
		 * 20000 us = 20 ms
		 */
		instance->ScheduleOnInterval(20000);

		return 0;
	}


	/*
	 * ============================================================
	 * 实例化
	 * ============================================================
	 */

	static PnGuidance *instantiate(
		int argc,
		char *argv[])
	{
		return new PnGuidance();
	}


	/*
	 * ============================================================
	 * 使用说明
	 * ============================================================
	 */

	static int print_usage(
		const char *reason = nullptr)
	{
		PX4_INFO("pn_guidance");

		return 0;
	}


	static int custom_command(
		int argc,
		char *argv[])
	{
		return print_usage();
	}


	/*
	 * ============================================================
	 * ScheduledWorkItem入口
	 * ============================================================
	 */

	void Run() override;


private:

	/*
	 * ============================================================
	 * 清除调度
	 * ============================================================
	 */

	void Stop()
	{
		ScheduleClear();
	}


private:

	/*
	 * ============================================================
	 * 视觉LOS输入
	 * ============================================================
	 */

	uORB::Subscription _los_sub{
		ORB_ID(vehicle_target_los)
	};


	/*
	 * ============================================================
	 * 飞机当前姿态
	 * ============================================================
	 */

	uORB::Subscription _attitude_sub{
		ORB_ID(vehicle_attitude)
	};


	/*
	 * ============================================================
	 * PN加速度输出
	 * ============================================================
	 */

	uORB::Publication<vehicle_guidance_acceleration_s> _accel_pub{
		ORB_ID(vehicle_guidance_acceleration)
	};


	/*
	 * ============================================================
	 * 姿态期望输出
	 * ============================================================
	 */

	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{
		ORB_ID(vehicle_attitude_setpoint)
	};


	/*
	 * ============================================================
	 * 初始航向锁定
	 * ============================================================
	 */

	float _yaw_lock{0.0f};

	bool _yaw_initialized{false};
};


/*
 * ================================================================
 * Run()
 * ================================================================
 */

void PnGuidance::Run()
{
	/*
	 * ============================================================
	 * 1. 获取视觉LOS数据
	 * ============================================================
	 */

	vehicle_target_los_s los{};

	if (!_los_sub.update(&los)) {
		return;
	}


	/*
	 * ============================================================
	 * 2. 判断目标是否有效
	 * ============================================================
	 */

	if (!los.target_valid) {
		return;
	}


	/*
	 * ============================================================
	 * 3. 获取当前飞机姿态
	 *
	 * vehicle_attitude.q：
	 *
	 * Body -> NED
	 * ============================================================
	 */

	vehicle_attitude_s att{};

	if (!_attitude_sub.update(&att)) {
		return;
	}


	/*
	 * 当前姿态四元数
	 */

	const Quatf q_current(att.q);


	/*
	 * 当前欧拉角
	 */

	const Eulerf euler(q_current);


	/*
	 * ============================================================
	 * 4. 锁定初始Yaw
	 *
	 * 只在第一次收到有效姿态时锁定。
	 * ============================================================
	 */

	if (!_yaw_initialized) {

		_yaw_lock = euler.psi();

		_yaw_initialized = true;
	}


	/*
	 * ============================================================
	 * 5. 获取LOS角速度
	 *
	 * 视觉板输出：
	 *
	 * los_az_rate : rad/s
	 * los_el_rate : rad/s
	 * ============================================================
	 */

	const float lambda_az =
		los.los_az_rate;


	const float lambda_el =
		los.los_el_rate;


	/*
	 * ============================================================
	 * 6. Camera -> Body FRD
	 *
	 * 当前已经确定：
	 *
	 * Camera X -> Body Y
	 * Camera Y -> -Body Z
	 *
	 * 因此：
	 *
	 * lambda_body_X = 0
	 * lambda_body_Y = lambda_az
	 * lambda_body_Z = -lambda_el
	 * ============================================================
	 */

	Vector3f lambda_body;

	lambda_body(0) = 0.0f;

	lambda_body(1) = lambda_az;

	lambda_body(2) = -lambda_el;


	/*
	 * ============================================================
	 * 7. PN比例导引
	 *
	 * 当前阶段：
	 *
	 * 暂时不使用PN计算结果，
	 * 固定给一个测试加速度。
	 *
	 * 目的：
	 * 首先验证
	 *
	 * accel_body
	 *      ↓
	 * acceleration_to_attitude
	 *      ↓
	 * q_d
	 *      ↓
	 * vehicle_attitude_setpoint
	 *
	 * 这一整条链路。
	 *
	 * ============================================================
	 */

	Vector3f accel_body;


	/*
	 * 测试加速度：
	 *
	 * Xbody = +5 m/s²
	 * Ybody =  0 m/s²
	 * Zbody = -20 m/s²
	 *
	 * 对应：
	 *
	 * 前向加速度 +5
	 * 横向加速度 0
	 * 向上加速度 20
	 */

	accel_body(0) = 0.0f;

	accel_body(1) = 50.0f;

	accel_body(2) = -20.0f;


	/*
	 * ============================================================
	 * 8. 加速度限制
	 * ============================================================
	 */

	const float ACC_MAX = 30.0f;


	for (int i = 0; i < 3; i++) {

		if (accel_body(i) > ACC_MAX) {
			accel_body(i) = ACC_MAX;
		}

		if (accel_body(i) < -ACC_MAX) {
			accel_body(i) = -ACC_MAX;
		}
	}


	/*
	 * ============================================================
	 * 9. 发布vehicle_guidance_acceleration
	 * ============================================================
	 */

	vehicle_guidance_acceleration_s accel{};

	accel.timestamp =
		hrt_absolute_time();


	accel.acceleration[0] =
		accel_body(0);


	accel.acceleration[1] =
		accel_body(1);


	accel.acceleration[2] =
		accel_body(2);


	accel.valid = true;


	_accel_pub.publish(accel);


	/*
	 * ============================================================
	 * 10. 加速度 -> 期望姿态
	 *
	 * 注意：
	 *
	 * 现在使用三个输入：
	 *
	 * accel_body
	 * q_current
	 * _yaw_lock
	 *
	 * ============================================================
	 */

	const Quatf q_d =
		AccelerationToAttitude::calculate(
			accel_body,
			q_current,
			_yaw_lock
		);


	/*
	 * ============================================================
	 * 11. 发布vehicle_attitude_setpoint
	 * ============================================================
	 */

	vehicle_attitude_setpoint_s att_sp{};

	att_sp.timestamp =
		hrt_absolute_time();


	att_sp.q_d[0] = q_d(0);

	att_sp.q_d[1] = q_d(1);

	att_sp.q_d[2] = q_d(2);

	att_sp.q_d[3] = q_d(3);


	/*
	 * 当前PX4版本中不要随便添加：
	 *
	 * att_sp.q_d_valid = true;
	 *
	 * 因为你之前已经验证过该字段不存在。
	 */
	// 每次 Run() 都发布
	 _att_sp_pub.publish(att_sp);

	 // 仅用于调试，每 5 秒打印一次
	static hrt_abstime last_print_time = 0;

	const hrt_abstime now = hrt_absolute_time();
	if (now - last_print_time >= 5000000)
	{
		last_print_time = now;

	 PX4_INFO("PN q_d: %.3f %.3f %.3f %.3f",
		static_cast<double>(q_d(0)),
		static_cast<double>(q_d(1)),
		static_cast<double>(q_d(2)),
		static_cast<double>(q_d(3)));
	}

}


/*
 * ================================================================
 * PX4 module entry
 * ================================================================
 */

extern "C" __EXPORT int pn_guidance_main(
	int argc,
	char *argv[])
{
	return PnGuidance::main(argc, argv);
}

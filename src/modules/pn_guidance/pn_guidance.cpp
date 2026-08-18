/****************************************************************************
 *
 * PN guidance module for PX4 v1.17-dev SITL
 *
 * 当前状态机：
 *
 * WAIT_LOS
 *    ↓
 * LAUNCH
 *    ↓
 * BPN READY
 *    ↓ ARM
 * WAIT_RELEASE
 *    ↓ RELEASE SIGNAL
 * LAUNCH_CLEARANCE
 *    ↓
 * TRANSITION
 *    ↓
 * BPN
 *
 * 主要特点：
 *
 * 1. RELEASE同步使用 vehicle_command.command == 31010。
 *
 * 2. PN使用固定闭合速度 PN_VC_FIXED。
 *    vehicle_local_position速度只监视，不参与PN控制。
 *
 * 3. 发射前：
 *    WAIT_LOS / LAUNCH / BPN_READY / WAIT_RELEASE
 *    均不主动改变飞机姿态。
 *
 * 4. 删除原BOOST固定姿态猛推逻辑。
 *
 * 5. RELEASE后进入LAUNCH_CLEARANCE：
 *
 *    第一段：
 *    从“当前姿态 + CLEARANCE_START_THRUST”
 *    对应的净惯性加速度平滑过渡到：
 *
 *        水平方向朝目标 CLEARANCE_FORWARD_ACCEL
 *        NED Z方向向上 CLEARANCE_UP_ACCEL
 *
 *    第二段：
 *    保持该净加速度一段时间，获得离架高度和初始速度。
 *
 * 6. CLEARANCE结束后继续使用净加速度smoothstep，
 *    平滑进入：
 *
 *        LOS轴向加速度 + BPN法向加速度
 *
 * 7. 增加vehicle_attitude.quat_reset_counter监控。
 *
 *    如果EKF姿态参考发生reset：
 *
 *        yaw_lock += delta_yaw
 *        hold_q    = delta_q_reset * hold_q
 *
 *    如果正在CLEARANCE/TRANSITION，
 *    则从当前已经修正后的姿态/推力重新开始平滑过渡，
 *    防止EKF reset产生瞬时大姿态误差。
 *
 ****************************************************************************/

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#include <uORB/topics/vehicle_target_los.h>
#include <uORB/topics/vehicle_guidance_acceleration.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

#include <cmath>
#include <cfloat>
#include <cstdint>


using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;


class PnGuidance :
	public ModuleBase<PnGuidance>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:

	PnGuidance() :
		ModuleParams(nullptr),
		ScheduledWorkItem(
			MODULE_NAME,
			px4::wq_configurations::nav_and_controllers
		)
	{
	}

	~PnGuidance() override
	{
		ScheduleClear();
	}


	static int task_spawn(
		int argc,
		char *argv[]
	)
	{
		PnGuidance *instance =
			new PnGuidance();

		if (instance != nullptr) {

			_object.store(
				instance
			);

			_task_id =
				task_id_is_work_queue;

			if (instance->init()) {
				return PX4_OK;
			}

		} else {

			PX4_ERR(
				"alloc failed"
			);
		}


		delete instance;

		_object.store(
			nullptr
		);

		_task_id = -1;

		return PX4_ERROR;
	}


	static int custom_command(
		int argc,
		char *argv[]
	)
	{
		return print_usage(
			"unknown command"
		);
	}


	static int print_usage(
		const char *reason = nullptr
	)
	{
		if (reason) {

			PX4_WARN(
				"%s",
				reason
			);
		}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
### Description
LOS-rate proportional navigation guidance module for SITL.

The module publishes vehicle_attitude_setpoint directly.
)DESCR_STR"
		);

		PRINT_MODULE_USAGE_NAME(
			"pn_guidance",
			"controller"
		);

		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

		return 0;
	}


	bool init()
	{
		ScheduleOnInterval(
			LOOP_INTERVAL_US
		);

		return true;
	}


	int print_status() override
	{
		PX4_INFO(
			"state=%u yaw_lock=%.2f deg",
			static_cast<unsigned>(
				_state
			),
			(double)(
				_yaw_lock
				*
				180.f
				/
				M_PI_F
			)
		);

		PX4_INFO(
			"AXIAL_ACCEL=%.2f m/s^2",
			(double)AXIAL_ACCEL
		);

		PX4_INFO(
			"CLEARANCE start_thr=%.2f ramp=%.2f s hold=%.2f s",
			(double)CLEARANCE_START_THRUST,
			(double)(
				CLEARANCE_RAMP_DURATION_US
				*
				1e-6f
			),
			(double)(
				CLEARANCE_HOLD_DURATION_US
				*
				1e-6f
			)
		);

		PX4_INFO(
			"CLEARANCE forward=%.2f up=%.2f m/s^2",
			(double)CLEARANCE_FORWARD_ACCEL,
			(double)CLEARANCE_UP_ACCEL
		);

		PX4_INFO(
			"TRANSITION=%.2f s",
			(double)(
				TRANSITION_DURATION_US
				*
				1e-6f
			)
		);

		PX4_INFO(
			"PN N=%.2f Vc_fixed=%.2f m/s",
			(double)PN_N,
			(double)PN_VC_FIXED
		);

		PX4_INFO(
			"RELEASE_SYNC_COMMAND=%u",
			(unsigned)
			RELEASE_SYNC_COMMAND
		);

		PX4_INFO(
			"EKF velocity is MONITOR ONLY"
		);

		return 0;
	}


private:

	// ============================================================
	// 状态机
	//
	// 为了兼容之前日志：
	//
	// BPN = 2
	// 发射动力阶段仍保持编号4，
	// 只是BOOST改成LAUNCH_CLEARANCE。
	// ============================================================

	enum class GuidanceState : uint8_t
	{
		WAIT_LOS         = 0,
		LAUNCH           = 1,
		BPN              = 2,
		STOP             = 3,
		LAUNCH_CLEARANCE = 4,
		TRANSITION       = 5,
		WAIT_RELEASE     = 6
	};


	// ============================================================
	// 基本周期
	// ============================================================

	static constexpr uint32_t
	LOOP_INTERVAL_US =
		20000;


	static constexpr uint64_t
	LOS_TIMEOUT_US =
		500000;


	static constexpr uint64_t
	LAUNCH_DURATION_US =
		100000;


	// ============================================================
	// Rotor-only launch clearance
	// ============================================================

	// RELEASE之后不要立即跳姿态。
	//
	// 先在0.8秒内：
	//
	// 当前姿态
	//     ↓ smoothstep
	// 向上+向目标水平方向
	//
	// 约70deg初始姿态变化时，
	// 0.8s比原来的瞬时变化温和很多。
	static constexpr uint64_t
	CLEARANCE_RAMP_DURATION_US =
		800000;


	// 完成姿态转换后继续保持0.6s，
	// 建立向上速度和一定水平初速度。
	static constexpr uint64_t
	CLEARANCE_HOLD_DURATION_US =
		600000;


	// clearance刚开始时，
	// 用这个推力和当前真实姿态构造起始净加速度。
	//
	// 这样phase=0时：
	//
	// q_sp ≈ 当前姿态
	//
	// 而不是release瞬间直接产生几十度姿态阶跃。
	static constexpr float
	CLEARANCE_START_THRUST =
		0.80f;


	// clearance阶段水平方向朝目标的净加速度。
	static constexpr float
	CLEARANCE_FORWARD_ACCEL =
		4.0f;


	// NED坐标：
	//
	// Z正 = Down
	//
	// 因此向上5m/s²写成：
	//
	// accel_ned.z = -5
	static constexpr float
	CLEARANCE_UP_ACCEL =
		5.0f;


	// ============================================================
	// CLEARANCE -> BPN transition
	// ============================================================

	static constexpr uint64_t
	TRANSITION_DURATION_US =
		2000000;


	static constexpr float
	WAIT_RELEASE_THRUST =
		0.0f;


	// ============================================================
	// Guidance参数
	// ============================================================

	static constexpr float
	GRAVITY =
		9.80665f;


	static constexpr float
	HOVER_THRUST =
		0.50f;


	// 期望净惯性轴向加速度。
	static constexpr float
	AXIAL_ACCEL =
		10.0f;


	static constexpr float
	PN_N =
		3.0f;


	// 当前固定闭合速度。
	static constexpr float
	PN_VC_FIXED =
		35.0f;


	static constexpr float
	BPN_ACCEL_MAX =
		12.0f;


	static constexpr float
	MIN_THRUST =
		0.10f;


	static constexpr float
	MAX_THRUST =
		0.95f;


	// MAV_CMD_USER_1
	static constexpr uint32_t
	RELEASE_SYNC_COMMAND =
		31010;


	// ============================================================
	// uORB
	// ============================================================

	uORB::Subscription
	_los_sub{
		ORB_ID(
			vehicle_target_los
		)
	};


	uORB::Subscription
	_attitude_sub{
		ORB_ID(
			vehicle_attitude
		)
	};


	uORB::Subscription
	_local_position_sub{
		ORB_ID(
			vehicle_local_position
		)
	};


	uORB::Subscription
	_vehicle_status_sub{
		ORB_ID(
			vehicle_status
		)
	};


	uORB::Subscription
	_vehicle_command_sub{
		ORB_ID(
			vehicle_command
		)
	};


	uORB::Publication<
		vehicle_attitude_setpoint_s
	>
	_attitude_setpoint_pub{
		ORB_ID(
			vehicle_attitude_setpoint
		)
	};


	uORB::Publication<
		vehicle_guidance_acceleration_s
	>
	_guidance_accel_pub{
		ORB_ID(
			vehicle_guidance_acceleration
		)
	};


	// ============================================================
	// 消息缓存
	// ============================================================

	vehicle_target_los_s
	_los{};


	vehicle_attitude_s
	_attitude{};


	vehicle_local_position_s
	_local_position{};


	vehicle_status_s
	_vehicle_status{};


	vehicle_command_s
	_vehicle_command{};


	// ============================================================
	// 状态
	// ============================================================

	GuidanceState
	_state{
		GuidanceState::WAIT_LOS
	};


	uint64_t
	_state_enter_time{0};


	uint64_t
	_last_los_rx_time{0};


	uint64_t
	_last_print_time{0};


	bool
	_yaw_locked{false};


	float
	_yaw_lock{0.f};


	bool
	_was_armed{false};


	bool
	_launch_sequence_complete{false};


	// ============================================================
	// 发射姿态 / clearance / transition
	// ============================================================

	Quatf
	_hold_q{
		1.f,
		0.f,
		0.f,
		0.f
	};


	// RELEASE刚发生时：
	//
	// 根据当前姿态+CLEARANCE_START_THRUST
	// 反算得到的起始净加速度。
	Vector3f
	_clearance_start_accel_ned{
		0.f,
		0.f,
		0.f
	};


	// CLEARANCE结束后：
	//
	// transition从这个净加速度继续进入BPN。
	Vector3f
	_transition_start_accel_ned{
		0.f,
		0.f,
		0.f
	};


	float
	_transition_phase{1.f};


	// ============================================================
	// 最后一次输出，用于debug/reset恢复
	// ============================================================

	Quatf
	_last_q_sp{
		1.f,
		0.f,
		0.f,
		0.f
	};


	float
	_last_thrust_sp{0.f};


	Vector3f
	_last_bpn_body{
		0.f,
		0.f,
		0.f
	};


	// ============================================================
	// EKF reset监控
	// ============================================================

	bool
	_ekf_reset_counter_initialized{false};


	uint8_t
	_last_xy_reset_counter{0};


	uint8_t
	_last_vxy_reset_counter{0};


	bool
	_attitude_reset_counter_initialized{false};


	uint8_t
	_last_quat_reset_counter{0};


	// ============================================================
	// 工具函数
	// ============================================================

	float wrap_pi_local(
		float angle
	) const
	{
		while (
			angle
			>
			M_PI_F
		) {

			angle -=
				2.f
				*
				M_PI_F;
		}


		while (
			angle
			<
			-M_PI_F
		) {

			angle +=
				2.f
				*
				M_PI_F;
		}


		return angle;
	}


	float smoothstep01(
		const float input
	) const
	{
		const float p =
			math::constrain(
				input,
				0.f,
				1.f
			);


		return
			p
			*
			p
			*
			(
				3.f
				-
				2.f
				*
				p
			);
	}


	// ============================================================
	// 基本判断
	// ============================================================

	bool attitude_valid() const
	{
		float norm_sq =
			0.f;


		for (
			int i = 0;
			i < 4;
			++i
		) {

			if (
				!PX4_ISFINITE(
					_attitude.q[i]
				)
			) {

				return false;
			}


			norm_sq +=
				_attitude.q[i]
				*
				_attitude.q[i];
		}


		return
			norm_sq
			>
			0.25f;
	}


	bool los_fresh(
		const uint64_t now
	) const
	{
		return
			(
				_last_los_rx_time
				>
				0
			)

			&&

			(
				now
				-
				_last_los_rx_time
				<
				LOS_TIMEOUT_US
			);
	}


	bool armed() const
	{
		return
			_vehicle_status.arming_state
			==
			vehicle_status_s::
			ARMING_STATE_ARMED;
	}


	Quatf current_attitude_q() const
	{
		Quatf q{
			_attitude.q
		};


		const float norm =
			q.norm();


		if (
			PX4_ISFINITE(
				norm
			)
			&&
			norm
			>
			FLT_EPSILON
		) {

			q.normalize();

		} else {

			q =
				Quatf{
					1.f,
					0.f,
					0.f,
					0.f
				};
		}


		return q;
	}


	float get_ekf_speed_monitor() const
	{
		if (
			!_local_position.v_xy_valid
			||
			!_local_position.v_z_valid
		) {

			return NAN;
		}


		if (
			!PX4_ISFINITE(
				_local_position.vx
			)
			||
			!PX4_ISFINITE(
				_local_position.vy
			)
			||
			!PX4_ISFINITE(
				_local_position.vz
			)
		) {

			return NAN;
		}


		return sqrtf(
			_local_position.vx
			*
			_local_position.vx

			+

			_local_position.vy
			*
			_local_position.vy

			+

			_local_position.vz
			*
			_local_position.vz
		);
	}


	// ============================================================
	// LOS几何
	// ============================================================

	Vector3f calculate_los_body(
		const float alpha,
		const float beta
	) const
	{
		Vector3f los_body{

			-sinf(
				beta
			),

			sinf(
				alpha
			)
			*
			cosf(
				beta
			),

			-cosf(
				alpha
			)
			*
			cosf(
				beta
			)
		};


		const float norm =
			los_body.norm();


		if (
			PX4_ISFINITE(
				norm
			)
			&&
			norm
			>
			FLT_EPSILON
		) {

			los_body /=
				norm;
		}


		return los_body;
	}


	Vector3f calculate_los_dot_body(
		const float alpha,
		const float beta,
		const float alpha_dot,
		const float beta_dot
	) const
	{
		const float ca =
			cosf(
				alpha
			);

		const float sa =
			sinf(
				alpha
			);

		const float cb =
			cosf(
				beta
			);

		const float sb =
			sinf(
				beta
			);


		const Vector3f e_az{

			0.f,

			ca,

			sa
		};


		const Vector3f e_el{

			-cb,

			-sa
			*
			sb,

			ca
			*
			sb
		};


		return
			e_az
			*
			(
				alpha_dot
				*
				cb
			)

			+

			e_el
			*
			beta_dot;
	}


	Vector3f calculate_bpn_body(
		const Vector3f &los_body,
		const Vector3f &los_dot_body
	) const
	{
		Vector3f accel_body =
			los_dot_body
			*
			(
				PN_N
				*
				PN_VC_FIXED
			);


		// 去掉LOS方向分量，
		// BPN只保留法向加速度。
		accel_body -=
			los_body
			*
			accel_body.dot(
				los_body
			);


		const float accel_norm =
			accel_body.norm();


		if (
			PX4_ISFINITE(
				accel_norm
			)
			&&
			accel_norm
			>
			BPN_ACCEL_MAX
		) {

			accel_body *=
				BPN_ACCEL_MAX
				/
				accel_norm;
		}


		return accel_body;
	}


	bool calculate_los_ned(
		Vector3f &los_ned
	) const
	{
		if (
			!attitude_valid()
			||
			!_los.target_valid
		) {

			return false;
		}


		const float alpha =
			_los.los_azimuth;


		const float beta =
			_los.los_elevation;


		if (
			!PX4_ISFINITE(
				alpha
			)
			||
			!PX4_ISFINITE(
				beta
			)
		) {

			return false;
		}


		const Vector3f los_body =
			calculate_los_body(
				alpha,
				beta
			);


		const Dcmf R_nb{
			current_attitude_q()
		};


		los_ned =
			R_nb
			*
			los_body;


		const float norm =
			los_ned.norm();


		if (
			!PX4_ISFINITE(
				norm
			)
			||
			norm
			<
			FLT_EPSILON
		) {

			return false;
		}


		los_ned /=
			norm;


		return true;
	}


	// ============================================================
	// attitude + thrust -> 净惯性加速度
	//
	// 用于构造CLEARANCE的连续起点。
	//
	// 旋翼推力方向：
	//
	// PX4 FRD body -Z
	//
	// a_net =
	// g +
	// R_nb * [0,0,-1] * a_thrust
	// ============================================================

	Vector3f attitude_thrust_to_net_accel(
		const Quatf &q_nb,
		const float thrust_norm
	) const
	{
		const Vector3f gravity_ned{

			0.f,

			0.f,

			GRAVITY
		};


		const Vector3f thrust_axis_body{

			0.f,

			0.f,

			-1.f
		};


		const Dcmf R_nb{
			q_nb
		};


		const Vector3f thrust_axis_ned =
			R_nb
			*
			thrust_axis_body;


		const float thrust_accel =
			GRAVITY
			*
			thrust_norm
			/
			HOVER_THRUST;


		return
			gravity_ned
			+
			thrust_axis_ned
			*
			thrust_accel;
	}


	// ============================================================
	// acceleration -> attitude
	//
	// accel_sp_ned：
	// 期望净惯性加速度。
	//
	// gravity只在这里处理一次。
	// ============================================================

	void acceleration_to_attitude(
		const Vector3f &accel_sp_ned,
		const float yaw_sp,
		Quatf &q_sp,
		float &thrust_norm
	) const
	{
		const Vector3f gravity_ned{

			0.f,

			0.f,

			GRAVITY
		};


		const Vector3f thrust_accel_ned =
			accel_sp_ned
			-
			gravity_ned;


		const float thrust_accel_mag =
			thrust_accel_ned.norm();


		Vector3f body_z =
			-thrust_accel_ned;


		if (
			body_z.norm_squared()
			<
			FLT_EPSILON
		) {

			body_z =
				Vector3f{
					0.f,
					0.f,
					1.f
				};
		}


		body_z.normalize();


		const Vector3f y_C{

			-sinf(
				yaw_sp
			),

			cosf(
				yaw_sp
			),

			0.f
		};


		Vector3f body_x =
			y_C
			%
			body_z;


		if (
			body_z(
				2
			)
			<
			0.f
		) {

			body_x =
				-body_x;
		}


		if (
			fabsf(
				body_z(
					2
				)
			)
			<
			0.000001f
		) {

			body_x.zero();

			body_x(
				2
			) =
				1.f;
		}


		if (
			body_x.norm_squared()
			<
			FLT_EPSILON
		) {

			body_x =
				Vector3f{

					cosf(
						yaw_sp
					),

					sinf(
						yaw_sp
					),

					0.f
				};
		}


		body_x.normalize();


		const Vector3f body_y =
			body_z
			%
			body_x;


		Dcmf R_sp;


		for (
			int i = 0;
			i < 3;
			++i
		) {

			R_sp(
				i,
				0
			) =
				body_x(
					i
				);


			R_sp(
				i,
				1
			) =
				body_y(
					i
				);


			R_sp(
				i,
				2
			) =
				body_z(
					i
				);
		}


		q_sp =
			Quatf{
				R_sp
			};


		q_sp.normalize();


		thrust_norm =
			HOVER_THRUST
			*
			thrust_accel_mag
			/
			GRAVITY;


		thrust_norm =
			math::constrain(
				thrust_norm,
				MIN_THRUST,
				MAX_THRUST
			);
	}


	// ============================================================
	// 发布
	// ============================================================

	void publish_direct_attitude(
		const Quatf &q_d,
		const float thrust_norm,
		const uint64_t now
	)
	{
		vehicle_attitude_setpoint_s sp{};


		sp.timestamp =
			now;


		q_d.copyTo(
			sp.q_d
		);


		sp.yaw_sp_move_rate =
			0.f;


		sp.thrust_body[0] =
			0.f;

		sp.thrust_body[1] =
			0.f;

		sp.thrust_body[2] =
			-math::constrain(
				thrust_norm,
				0.f,
				MAX_THRUST
			);


		_attitude_setpoint_pub.publish(
			sp
		);


		_last_q_sp =
			q_d;


		_last_thrust_sp =
			math::constrain(
				thrust_norm,
				0.f,
				MAX_THRUST
			);
	}


	void publish_accel_attitude_setpoint(
		const Vector3f &accel_sp_ned,
		const uint64_t now
	)
	{
		Quatf q_sp;


		float thrust_norm =
			0.f;


		acceleration_to_attitude(

			accel_sp_ned,

			_yaw_lock,

			q_sp,

			thrust_norm
		);


		publish_direct_attitude(

			q_sp,

			thrust_norm,

			now
		);
	}


	void publish_guidance_acceleration(
		const Vector3f &bpn_accel_body,
		const bool valid,
		const uint64_t now
	)
	{
		vehicle_guidance_acceleration_s msg{};


		msg.timestamp =
			now;


		msg.acceleration[0] =
			bpn_accel_body(
				0
			);

		msg.acceleration[1] =
			bpn_accel_body(
				1
			);

		msg.acceleration[2] =
			bpn_accel_body(
				2
			);


		msg.valid =
			valid;


		_guidance_accel_pub.publish(
			msg
		);
	}


	// ============================================================
	// BPN目标净加速度
	// ============================================================

	bool calculate_bpn_target(
		Vector3f &accel_sp_ned,
		Vector3f &bpn_accel_body
	)
	{
		if (
			!attitude_valid()
			||
			!_los.target_valid
		) {

			return false;
		}


		const float alpha =
			_los.los_azimuth;


		const float beta =
			_los.los_elevation;


		const float alpha_dot =
			_los.los_az_rate;


		const float beta_dot =
			_los.los_el_rate;


		if (
			!PX4_ISFINITE(
				alpha
			)
			||
			!PX4_ISFINITE(
				beta
			)
			||
			!PX4_ISFINITE(
				alpha_dot
			)
			||
			!PX4_ISFINITE(
				beta_dot
			)
		) {

			return false;
		}


		const Vector3f los_body =
			calculate_los_body(

				alpha,

				beta
			);


		const Vector3f los_dot_body =
			calculate_los_dot_body(

				alpha,

				beta,

				alpha_dot,

				beta_dot
			);


		bpn_accel_body =
			calculate_bpn_body(

				los_body,

				los_dot_body
			);


		const Dcmf R_nb{
			current_attitude_q()
		};


		Vector3f los_ned =
			R_nb
			*
			los_body;


		const float los_ned_norm =
			los_ned.norm();


		if (
			!PX4_ISFINITE(
				los_ned_norm
			)
			||
			los_ned_norm
			<
			FLT_EPSILON
		) {

			return false;
		}


		los_ned /=
			los_ned_norm;


		const Vector3f axial_accel_ned =
			los_ned
			*
			AXIAL_ACCEL;


		const Vector3f bpn_accel_ned =
			R_nb
			*
			bpn_accel_body;


		accel_sp_ned =
			axial_accel_ned
			+
			bpn_accel_ned;


		return true;
	}


	// ============================================================
	// LAUNCH CLEARANCE目标
	//
	// 这里只取LOS的水平投影用于向前。
	//
	// 垂直方向单独明确指定：
	//
	// accel_z = -CLEARANCE_UP_ACCEL
	//
	// 这样无论目标仰角是多少，
	// clearance阶段都有明确向上净加速度。
	// ============================================================

	bool calculate_clearance_target(
		Vector3f &accel_sp_ned
	)
	{
		Vector3f los_ned;


		if (
			!calculate_los_ned(
				los_ned
			)
		) {

			return false;
		}


		Vector3f horizontal_dir{

			los_ned(
				0
			),

			los_ned(
				1
			),

			0.f
		};


		const float horizontal_norm =
			horizontal_dir.norm();


		if (
			PX4_ISFINITE(
				horizontal_norm
			)
			&&
			horizontal_norm
			>
			FLT_EPSILON
		) {

			horizontal_dir /=
				horizontal_norm;

		} else {

			horizontal_dir.zero();
		}


		accel_sp_ned =
			horizontal_dir
			*
			CLEARANCE_FORWARD_ACCEL;


		// PX4 NED：
		//
		// z负 = 向上
		accel_sp_ned(
			2
		) =
			-CLEARANCE_UP_ACCEL;


		return true;
	}


	// ============================================================
	// 发射阶段
	// ============================================================

	void enter_wait_release(
		const uint64_t now
	)
	{
		_hold_q =
			current_attitude_q();


		_state =
			GuidanceState::
			WAIT_RELEASE;


		_state_enter_time =
			now;


		_transition_phase =
			0.f;


		PX4_INFO(
			"ARM -> WAIT_RELEASE"
		);
	}


	void enter_launch_clearance(
		const uint64_t now
	)
	{
		// RELEASE瞬间的真实姿态。
		_hold_q =
			current_attitude_q();


		// 用“当前姿态 + 0.80推力”
		// 构造clearance phase=0的净加速度。
		//
		// 再送回acceleration_to_attitude()时，
		// 初始q_sp会接近当前真实姿态。
		_clearance_start_accel_ned =
			attitude_thrust_to_net_accel(

				_hold_q,

				CLEARANCE_START_THRUST
			);


		_state =
			GuidanceState::
			LAUNCH_CLEARANCE;


		_state_enter_time =
			now;


		_transition_phase =
			0.f;


		PX4_INFO(
			"RELEASE SIGNAL -> LAUNCH_CLEARANCE"
		);


		PX4_INFO(
			"CLEARANCE start_thr %.2f ramp %.2f s hold %.2f s",
			(double)CLEARANCE_START_THRUST,
			(double)(
				CLEARANCE_RAMP_DURATION_US
				*
				1e-6f
			),
			(double)(
				CLEARANCE_HOLD_DURATION_US
				*
				1e-6f
			)
		);


		PX4_INFO(
			"CLEARANCE forward %.2f up %.2f m/s^2",
			(double)CLEARANCE_FORWARD_ACCEL,
			(double)CLEARANCE_UP_ACCEL
		);
	}


	void enter_transition(
		const uint64_t now,
		const Vector3f &start_accel_ned
	)
	{
		_transition_start_accel_ned =
			start_accel_ned;


		_state =
			GuidanceState::
			TRANSITION;


		_state_enter_time =
			now;


		_transition_phase =
			0.f;


		PX4_INFO(
			"LAUNCH_CLEARANCE -> TRANSITION"
		);


		PX4_INFO(
			"TRANSITION duration %.2f s",
			(double)(
				TRANSITION_DURATION_US
				*
				1e-6f
			)
		);
	}


	void run_launch_clearance(
		const uint64_t now
	)
	{
		Vector3f clearance_target_ned;


		if (
			!calculate_clearance_target(
				clearance_target_ned
			)
		) {

			enter_stop(
				now,
				"invalid clearance target"
			);

			return;
		}


		const uint64_t elapsed =
			now
			-
			_state_enter_time;


		Vector3f accel_sp_ned;


		if (
			elapsed
			<
			CLEARANCE_RAMP_DURATION_US
		) {

			const float raw_phase =
				math::constrain(

					(float)elapsed
					/
					(float)
					CLEARANCE_RAMP_DURATION_US,

					0.f,

					1.f
				);


			const float phase =
				smoothstep01(
					raw_phase
				);


			_transition_phase =
				phase;


			accel_sp_ned =

				_clearance_start_accel_ned
				*
				(
					1.f
					-
					phase
				)

				+

				clearance_target_ned
				*
				phase;

		} else {

			// ramp完成后保持clearance目标。
			_transition_phase =
				1.f;


			accel_sp_ned =
				clearance_target_ned;
		}


		_last_bpn_body.zero();


		publish_accel_attitude_setpoint(

			accel_sp_ned,

			now
		);


		// clearance期间尚未真正启用BPN。
		publish_guidance_acceleration(

			_last_bpn_body,

			false,

			now
		);


		const uint64_t total_clearance_time =
			CLEARANCE_RAMP_DURATION_US
			+
			CLEARANCE_HOLD_DURATION_US;


		if (
			elapsed
			>=
			total_clearance_time
		) {

			// transition直接从当前clearance净加速度继续，
			// 因此这里不会发生加速度setpoint跳变。
			enter_transition(

				now,

				accel_sp_ned
			);
		}
	}


	void run_transition(
		const uint64_t now
	)
	{
		Vector3f target_accel_ned;


		Vector3f bpn_accel_body;


		if (
			!calculate_bpn_target(

				target_accel_ned,

				bpn_accel_body
			)
		) {

			enter_stop(
				now,
				"invalid BPN target in transition"
			);

			return;
		}


		const float raw_phase =
			math::constrain(

				(float)(
					now
					-
					_state_enter_time
				)
				/
				(float)
				TRANSITION_DURATION_US,

				0.f,

				1.f
			);


		const float phase =
			smoothstep01(
				raw_phase
			);


		_transition_phase =
			phase;


		const Vector3f accel_sp_ned =

			_transition_start_accel_ned
			*
			(
				1.f
				-
				phase
			)

			+

			target_accel_ned
			*
			phase;


		_last_bpn_body =
			bpn_accel_body;


		publish_accel_attitude_setpoint(

			accel_sp_ned,

			now
		);


		publish_guidance_acceleration(

			bpn_accel_body,

			true,

			now
		);


		if (
			raw_phase
			>=
			1.f
		) {

			_state =
				GuidanceState::
				BPN;


			_state_enter_time =
				now;


			_launch_sequence_complete =
				true;


			_transition_phase =
				1.f;


			PX4_INFO(
				"TRANSITION -> BPN"
			);
		}
	}


	// ============================================================
	// 正常BPN
	// ============================================================

	void run_bpn(
		const uint64_t now
	)
	{
		Vector3f accel_sp_ned;


		Vector3f bpn_accel_body;


		if (
			!calculate_bpn_target(

				accel_sp_ned,

				bpn_accel_body
			)
		) {

			enter_stop(
				now,
				"invalid BPN target"
			);

			return;
		}


		_last_bpn_body =
			bpn_accel_body;


		publish_accel_attitude_setpoint(

			accel_sp_ned,

			now
		);


		publish_guidance_acceleration(

			bpn_accel_body,

			true,

			now
		);
	}


	// ============================================================
	// STOP
	// ============================================================

	void enter_stop(
		const uint64_t now,
		const char *reason
	)
	{
		if (
			_state
			!=
			GuidanceState::
			STOP
		) {

			PX4_WARN(
				"-> STOP: %s",
				reason
			);
		}


		_state =
			GuidanceState::
			STOP;


		_state_enter_time =
			now;


		_transition_phase =
			1.f;
	}


	void run_stop(
		const uint64_t now
	)
	{
		const Vector3f zero_net_accel{

			0.f,

			0.f,

			0.f
		};


		_last_bpn_body.zero();


		publish_accel_attitude_setpoint(

			zero_net_accel,

			now
		);


		publish_guidance_acceleration(

			_last_bpn_body,

			false,

			now
		);
	}


	// ============================================================
	// EKF local position reset监控
	// ============================================================

	void monitor_ekf_resets()
	{
		if (
			!_ekf_reset_counter_initialized
		) {

			_last_xy_reset_counter =
				_local_position.
				xy_reset_counter;


			_last_vxy_reset_counter =
				_local_position.
				vxy_reset_counter;


			_ekf_reset_counter_initialized =
				true;


			return;
		}


		if (
			_local_position.
			xy_reset_counter
			!=
			_last_xy_reset_counter
		) {

			PX4_WARN(
				"EKF XY RESET %u -> %u",
				(unsigned)
				_last_xy_reset_counter,
				(unsigned)
				_local_position.
				xy_reset_counter
			);


			_last_xy_reset_counter =
				_local_position.
				xy_reset_counter;
		}


		if (
			_local_position.
			vxy_reset_counter
			!=
			_last_vxy_reset_counter
		) {

			PX4_WARN(
				"EKF VXY RESET %u -> %u",
				(unsigned)
				_last_vxy_reset_counter,
				(unsigned)
				_local_position.
				vxy_reset_counter
			);


			_last_vxy_reset_counter =
				_local_position.
				vxy_reset_counter;
		}
	}


	// ============================================================
	// EKF attitude quaternion reset处理
	//
	// PX4自身的思路：
	//
	// q_setpoint_new =
	// delta_q_reset * q_setpoint_old
	//
	// yaw_lock_new =
	// yaw_lock_old + delta_yaw
	//
	// 这里必须在pn_guidance内部也处理。
	//
	// 原因：
	// mc_att_control虽然也会修正它内部已经收到的setpoint，
	// 但pn_guidance下一周期仍会继续发布自己的yaw_lock。
	//
	// 如果pn_guidance不修，
	// 下一帧就又会把旧参考系的yaw送回去。
	// ============================================================

	void monitor_attitude_reset(
		const uint64_t now
	)
	{
		if (
			!_attitude_reset_counter_initialized
		) {

			_last_quat_reset_counter =
				_attitude.
				quat_reset_counter;


			_attitude_reset_counter_initialized =
				true;


			return;
		}


		if (
			_attitude.
			quat_reset_counter
			==
			_last_quat_reset_counter
		) {

			return;
		}


		const uint8_t old_counter =
			_last_quat_reset_counter;


		const uint8_t new_counter =
			_attitude.
			quat_reset_counter;


		_last_quat_reset_counter =
			new_counter;


		Quatf delta_q{
			_attitude.
			delta_q_reset
		};


		const float delta_norm =
			delta_q.norm();


		if (
			!PX4_ISFINITE(
				delta_norm
			)
			||
			delta_norm
			<
			FLT_EPSILON
		) {

			PX4_WARN(
				"EKF ATT RESET %u -> %u invalid delta_q",
				(unsigned)old_counter,
				(unsigned)new_counter
			);

			return;
		}


		delta_q.normalize();


		const Eulerf delta_euler{
			delta_q
		};


		const float delta_yaw =
			delta_euler.psi();


		PX4_WARN(
			"EKF ATT RESET %u -> %u dYaw=%.1f deg",
			(unsigned)old_counter,
			(unsigned)new_counter,
			(double)(
				delta_yaw
				*
				180.f
				/
				M_PI_F
			)
		);


		// --------------------------------------------------------
		// 修正锁定yaw
		// --------------------------------------------------------

		if (
			_yaw_locked
		) {

			const float old_yaw =
				_yaw_lock;


			_yaw_lock =
				wrap_pi_local(
					_yaw_lock
					+
					delta_yaw
				);


			PX4_WARN(
				"YAW LOCK RESET %.1f -> %.1f deg",
				(double)(
					old_yaw
					*
					180.f
					/
					M_PI_F
				),
				(double)(
					_yaw_lock
					*
					180.f
					/
					M_PI_F
				)
			);
		}


		// --------------------------------------------------------
		// 修正固定姿态
		// --------------------------------------------------------

		_hold_q =
			delta_q
			*
			_hold_q;


		_hold_q.normalize();


		// --------------------------------------------------------
		// 修正最后一次导航setpoint
		// --------------------------------------------------------

		_last_q_sp =
			delta_q
			*
			_last_q_sp;


		_last_q_sp.normalize();


		// --------------------------------------------------------
		// 如果reset发生在CLEARANCE，
		//
		// 从“已经修正后的当前指令姿态+推力”
		// 重新开始clearance ramp。
		//
		// 这样不会因为reset导致存储的旧参考系加速度
		// 和新的姿态参考系不一致。
		// --------------------------------------------------------

		if (
			_state
			==
			GuidanceState::
			LAUNCH_CLEARANCE
		) {

			_clearance_start_accel_ned =
				attitude_thrust_to_net_accel(

					_last_q_sp,

					_last_thrust_sp
				);


			_state_enter_time =
				now;


			_transition_phase =
				0.f;


			PX4_WARN(
				"restart LAUNCH_CLEARANCE after ATT reset"
			);
		}


		// --------------------------------------------------------
		// TRANSITION期间同理。
		//
		// 从当前已经适配reset的姿态+推力重新起步。
		// --------------------------------------------------------

		if (
			_state
			==
			GuidanceState::
			TRANSITION
		) {

			_transition_start_accel_ned =
				attitude_thrust_to_net_accel(

					_last_q_sp,

					_last_thrust_sp
				);


			_state_enter_time =
				now;


			_transition_phase =
				0.f;


			PX4_WARN(
				"restart TRANSITION after ATT reset"
			);
		}
	}


	// ============================================================
	// Debug
	// ============================================================

	float attitude_error_angle_deg() const
	{
		const Quatf q_actual =
			current_attitude_q();


		float q_dot =

			_last_q_sp(
				0
			)
			*
			q_actual(
				0
			)

			+

			_last_q_sp(
				1
			)
			*
			q_actual(
				1
			)

			+

			_last_q_sp(
				2
			)
			*
			q_actual(
				2
			)

			+

			_last_q_sp(
				3
			)
			*
			q_actual(
				3
			);


		q_dot =
			fabsf(
				q_dot
			);


		q_dot =
			math::constrain(
				q_dot,
				0.f,
				1.f
			);


		const float angle_rad =
			2.f
			*
			acosf(
				q_dot
			);


		return
			angle_rad
			*
			180.f
			/
			M_PI_F;
	}


	void print_debug(
		const uint64_t now
	)
	{
		if (
			now
			-
			_last_print_time
			<
			500000
		) {

			return;
		}


		_last_print_time =
			now;


		const float ekf_speed =
			get_ekf_speed_monitor();


		const Eulerf att_euler{
			current_attitude_q()
		};


		const Eulerf sp_euler{
			_last_q_sp
		};


		PX4_INFO(
			"STATE=%u EKF_SPEED_MON=%.2f Vc_fixed=%.2f",
			(unsigned)
			_state,
			(double)ekf_speed,
			(double)PN_VC_FIXED
		);


		PX4_INFO(
			"GIMBAL AZ=%.2f PIT=%.2f",
			(double)(
				_los.los_azimuth
				*
				180.f
				/
				M_PI_F
			),
			(double)(
				_los.los_elevation
				*
				180.f
				/
				M_PI_F
			)
		);


		PX4_INFO(
			"LOS RATE AZ=%.4f EL=%.4f",
			(double)
			_los.los_az_rate,
			(double)
			_los.los_el_rate
		);


		PX4_INFO(
			"BPN body %.2f %.2f %.2f",
			(double)
			_last_bpn_body(
				0
			),
			(double)
			_last_bpn_body(
				1
			),
			(double)
			_last_bpn_body(
				2
			)
		);


		PX4_INFO(
			"ATT RPY %.1f %.1f %.1f",
			(double)(
				att_euler.phi()
				*
				180.f
				/
				M_PI_F
			),
			(double)(
				att_euler.theta()
				*
				180.f
				/
				M_PI_F
			),
			(double)(
				att_euler.psi()
				*
				180.f
				/
				M_PI_F
			)
		);


		PX4_INFO(
			"QSP RPY %.1f %.1f %.1f",
			(double)(
				sp_euler.phi()
				*
				180.f
				/
				M_PI_F
			),
			(double)(
				sp_euler.theta()
				*
				180.f
				/
				M_PI_F
			),
			(double)(
				sp_euler.psi()
				*
				180.f
				/
				M_PI_F
			)
		);


		PX4_INFO(
			"ATT_ERR=%.1f deg THR=%.3f phase=%.2f yaw_lock=%.1f",
			(double)
			attitude_error_angle_deg(),
			(double)
			_last_thrust_sp,
			(double)
			_transition_phase,
			(double)(
				_yaw_lock
				*
				180.f
				/
				M_PI_F
			)
		);


		PX4_INFO(
			"EKF reset quat=%u xy=%u vxy=%u dead_reckoning=%d",
			(unsigned)
			_attitude.
			quat_reset_counter,
			(unsigned)
			_local_position.
			xy_reset_counter,
			(unsigned)
			_local_position.
			vxy_reset_counter,
			(int)
			_local_position.
			dead_reckoning
		);
	}


	// ============================================================
	// Run
	// ============================================================

	void Run() override
	{
		if (
			should_exit()
		) {

			ScheduleClear();

			exit_and_cleanup();

			return;
		}


		const uint64_t now =
			hrt_absolute_time();


		// --------------------------------------------------------
		// LOS
		// --------------------------------------------------------

		if (
			_los_sub.update(
				&_los
			)
		) {

			_last_los_rx_time =
				now;
		}


		// --------------------------------------------------------
		// attitude
		//
		// 每次vehicle_attitude更新以后立即检查quat reset。
		// --------------------------------------------------------

		if (
			_attitude_sub.update(
				&_attitude
			)
		) {

			monitor_attitude_reset(
				now
			);
		}


		// --------------------------------------------------------
		// local position
		// --------------------------------------------------------

		if (
			_local_position_sub.update(
				&_local_position
			)
		) {

			monitor_ekf_resets();
		}


		_vehicle_status_sub.update(
			&_vehicle_status
		);


		// --------------------------------------------------------
		// 第一次有效LOS
		//
		// WAIT_LOS -> LAUNCH
		// --------------------------------------------------------

		if (
			_state
			==
			GuidanceState::
			WAIT_LOS

			&&

			los_fresh(
				now
			)

			&&

			_los.target_valid

			&&

			attitude_valid()
		) {

			if (
				!_yaw_locked
			) {

				const Eulerf euler{
					current_attitude_q()
				};


				_yaw_lock =
					euler.psi();


				_yaw_locked =
					true;


				PX4_INFO(
					"WAIT_LOS -> LAUNCH"
				);


				PX4_INFO(
					"LOCK YAW %.2f deg",
					(double)(
						_yaw_lock
						*
						180.f
						/
						M_PI_F
					)
				);
			}


			_hold_q =
				current_attitude_q();


			_state =
				GuidanceState::
				LAUNCH;


			_state_enter_time =
				now;


			_transition_phase =
				1.f;
		}


		// --------------------------------------------------------
		// ARM边沿
		// --------------------------------------------------------

		const bool is_armed =
			armed();


		if (
			is_armed
			&&
			!_was_armed
		) {

			if (
				_state
				==
				GuidanceState::
				BPN

				&&

				!_launch_sequence_complete
			) {

				enter_wait_release(
					now
				);
			}
		}


		if (
			!is_armed
			&&
			_was_armed
		) {

			_launch_sequence_complete =
				false;


			_transition_phase =
				1.f;


			if (
				los_fresh(
					now
				)
				&&
				_los.target_valid
				&&
				attitude_valid()
			) {

				_hold_q =
					current_attitude_q();


				_state =
					GuidanceState::
					BPN;

			} else {

				_state =
					GuidanceState::
					WAIT_LOS;
			}
		}


		_was_armed =
			is_armed;


		// --------------------------------------------------------
		// RELEASE SYNC
		// --------------------------------------------------------

		bool release_sync_received =
			false;


		if (
			_vehicle_command_sub.update(
				&_vehicle_command
			)
		) {

			if (
				_vehicle_command.command
				==
				RELEASE_SYNC_COMMAND
			) {

				release_sync_received =
					true;
			}
		}


		if (
			release_sync_received

			&&

			_state
			==
			GuidanceState::
			WAIT_RELEASE

			&&

			is_armed
		) {

			enter_launch_clearance(
				now
			);
		}


		// --------------------------------------------------------
		// LOS失效保护
		//
		// WAIT_RELEASE以前不严格触发STOP。
		//
		// 一旦物理release以后：
		//
		// CLEARANCE / TRANSITION / BPN
		//
		// 都要求LOS持续有效。
		// --------------------------------------------------------

		if (
			(
				_state
				==
				GuidanceState::
				LAUNCH_CLEARANCE

				||

				_state
				==
				GuidanceState::
				TRANSITION

				||

				(
					_state
					==
					GuidanceState::
					BPN

					&&

					_launch_sequence_complete
				)
			)

			&&

			(
				!los_fresh(
					now
				)

				||

				!_los.target_valid
			)
		) {

			enter_stop(
				now,
				"LOS invalid/stale"
			);
		}


		// --------------------------------------------------------
		// 状态机
		// --------------------------------------------------------

		switch (
			_state
		) {

		case GuidanceState::WAIT_LOS:
		{
			_last_bpn_body.zero();


			publish_guidance_acceleration(

				_last_bpn_body,

				false,

				now
			);


			break;
		}


		case GuidanceState::LAUNCH:
		{
			// 发射准备阶段保持当前真实姿态，
			// 推力保持0。
			if (
				attitude_valid()
			) {

				_hold_q =
					current_attitude_q();


				publish_direct_attitude(

					_hold_q,

					WAIT_RELEASE_THRUST,

					now
				);
			}


			_last_bpn_body.zero();


			publish_guidance_acceleration(

				_last_bpn_body,

				false,

				now
			);


			if (
				now
				-
				_state_enter_time
				>=
				LAUNCH_DURATION_US
			) {

				_state =
					GuidanceState::
					BPN;


				_state_enter_time =
					now;


				_launch_sequence_complete =
					false;


				PX4_INFO(
					"LAUNCH -> BPN READY"
				);
			}


			break;
		}


		case GuidanceState::BPN:
		{
			if (
				_launch_sequence_complete
			) {

				run_bpn(
					now
				);

			} else {

				// BPN READY：
				//
				// 尚未ARM/RELEASE。
				if (
					attitude_valid()
				) {

					_hold_q =
						current_attitude_q();


					publish_direct_attitude(

						_hold_q,

						WAIT_RELEASE_THRUST,

						now
					);
				}


				_last_bpn_body.zero();


				publish_guidance_acceleration(

					_last_bpn_body,

					false,

					now
				);
			}


			break;
		}


		case GuidanceState::WAIT_RELEASE:
		{
			// ARM之后、Gazebo真正release以前：
			//
			// 固定ARM时真实姿态。
			publish_direct_attitude(

				_hold_q,

				WAIT_RELEASE_THRUST,

				now
			);


			_last_bpn_body.zero();


			publish_guidance_acceleration(

				_last_bpn_body,

				false,

				now
			);


			break;
		}


		case GuidanceState::LAUNCH_CLEARANCE:
		{
			run_launch_clearance(
				now
			);


			break;
		}


		case GuidanceState::TRANSITION:
		{
			run_transition(
				now
			);


			break;
		}


		case GuidanceState::STOP:
		default:
		{
			run_stop(
				now
			);


			break;
		}
		}


		print_debug(
			now
		);
	}
};


extern "C"
__EXPORT
int pn_guidance_main(
	int argc,
	char *argv[]
)
{
	return PnGuidance::main(
		argc,
		argv
	);
}

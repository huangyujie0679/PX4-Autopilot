/****************************************************************************
 *
 * PN guidance module for PX4 v1.17-dev
 *
 * 状态机：
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
 *    ↓ 目标已越过 / LOS丢失
 * SAFE_HOVER
 *
 * 本版本相对上一版的主要修改：
 *
 * 1. 制导循环固定 50 Hz：LOOP_INTERVAL_US = 20000。
 *
 * 2. 正常速度规划改为：
 *      LOS rate <= 0.05 rad/s -> 26 m/s
 *      0.05 ~ 0.15          -> 26 -> 24 m/s
 *      0.15 ~ 0.30          -> 24 -> 21 m/s
 *      >= 0.30               -> 18 m/s
 *    28 m/s作为高速保护阈值，不再出现原代码26 -> 40 m/s的反向插值。
 *
 * 3. 增强超速制动：
 *    - 超过动态目标速度后主动制动；
 *    - 27 m/s开始进入硬速度保护；
 *    - 硬速度保护按三维速度方向制动。
 *
 * 4. LAUNCH_CLEARANCE使用独立推力预算0.95，
 *    不再被正常BPN的0.75推力预算裁掉。
 *
 * 5. 删除CLEARANCE阶段持续固定“向上5m/s^2”的控制。
 *    CLEARANCE本身只给小幅前向净加速度，垂向默认0；
 *    只有检测到相对RELEASE高度掉高/即将掉高时，
 *    RELEASE高度保护才按PD + 制动距离施加向上加速度。
 *    目的是避免发射后形成十几m/s的巨大向上速度。
 *
 * 6. 保留：
 *      PN_N = 3.0
 *      BPN_ACCEL_MAX = 12 m/s^2
 *      attitude slew = 60 deg/s
 *      attitude lead = 30 deg
 *      TRANSITION = 2.0 s
 *    先隔离速度和发射阶段问题，不同时大改导引增益。
 *
 * 7. WAIT_RELEASE保持0 guidance thrust。
 *    真机若需要“解锁后电机怠速”，应由PX4输出/ESC armed-idle机制负责，
 *    不建议由制导模块在发射架上主动施加额外推力。
 *
 * 8. 增加LOS角速度实时调试输出（默认5 Hz）：
 *      LOS_RT AZ_RATE=... EL_RATE=... MAG=...
 *    真机正式使用时可将LOS_RATE_REALTIME_PRINT改为false，降低控制台负担。
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
	PnGuidance() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) {}

	~PnGuidance() override
	{
		ScheduleClear();
	}

	static int task_spawn(int argc, char *argv[])
	{
		PnGuidance *instance = new PnGuidance();

		if (instance != nullptr) {
			_object.store(instance);
			_task_id = task_id_is_work_queue;

			if (instance->init()) {
				return PX4_OK;
			}

		} else {
			PX4_ERR("alloc failed");
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
### Description
LOS-rate proportional-navigation guidance module.

The module publishes vehicle_attitude_setpoint directly.
)DESCR_STR"
		);

		PRINT_MODULE_USAGE_NAME("pn_guidance", "controller");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(LOOP_INTERVAL_US);
		return true;
	}

	int print_status() override
	{
		PX4_INFO("state=%u yaw_lock=%.2f deg", static_cast<unsigned>(_state),
			 (double)(_yaw_lock * 180.f / M_PI_F));

		PX4_INFO("LOOP=%.1f Hz", (double)(1e6f / (float)LOOP_INTERVAL_US));
		PX4_INFO("AXIAL_ACCEL_MAX=%.2f m/s^2", (double)AXIAL_ACCEL_MAX);

		PX4_INFO("CLEARANCE start_thr=%.2f thrust_limit=%.2f ramp=%.2f s hold=%.2f s",
			 (double)CLEARANCE_START_THRUST,
			 (double)CLEARANCE_THRUST_LIMIT,
			 (double)(CLEARANCE_RAMP_DURATION_US * 1e-6f),
			 (double)(CLEARANCE_HOLD_DURATION_US * 1e-6f));

		PX4_INFO("CLEARANCE forward=%.2f m/s^2, vertical controlled only by ALT floor",
			 (double)CLEARANCE_FORWARD_ACCEL);

		PX4_INFO("TRANSITION=%.2f s", (double)(TRANSITION_DURATION_US * 1e-6f));

		PX4_INFO("ATT slew max=%.1f deg/s", (double)GUIDANCE_ATTITUDE_SLEW_DEG_S);
		PX4_INFO("ATT lead max=%.1f deg", (double)GUIDANCE_ATTITUDE_LEAD_MAX_DEG);

		PX4_INFO("PN N=%.2f Vc EKF LOS projection", (double)PN_N);

		PX4_INFO("Vc fallback=%.1f clamp=0.0..%.1f LPF rise=%.2f fall=%.2f s",
			 (double)VC_FALLBACK,
			 (double)VC_MAX,
			 (double)VC_LPF_TAU_RISE,
			 (double)VC_LPF_TAU_FALL);

		PX4_INFO("BPN_ACCEL_MAX=%.2f m/s^2", (double)BPN_ACCEL_MAX);

		PX4_INFO("GUIDANCE SPEED cruise=%.1f hard=%.1f m/s",
			 (double)GUIDANCE_SPEED_CRUISE,
			 (double)GUIDANCE_SPEED_HARD_LIMIT);

		PX4_INFO("TERMINAL SPEED los_rate %.2f/%.2f/%.2f rad/s -> %.1f/%.1f/%.1f/%.1f m/s",
			 (double)LOS_RATE_SPEED_POINT_1,
			 (double)LOS_RATE_SPEED_POINT_2,
			 (double)LOS_RATE_SPEED_POINT_3,
			 (double)GUIDANCE_SPEED_CRUISE,
			 (double)GUIDANCE_SPEED_MID_1,
			 (double)GUIDANCE_SPEED_MID_2,
			 (double)GUIDANCE_SPEED_MIN);

		PX4_INFO("SPEED brake kp=%.1f max=%.1f, hard_start=%.1f hard_kp=%.1f hard_max=%.1f",
			 (double)SPEED_BRAKE_KP,
			 (double)SPEED_BRAKE_ACCEL_MAX,
			 (double)HARD_SPEED_BRAKE_START,
			 (double)HARD_SPEED_BRAKE_KP,
			 (double)HARD_SPEED_BRAKE_ACCEL_MAX);

		PX4_INFO("GUIDANCE thrust budget=%.2f clearance=%.2f physical_max=%.2f",
			 (double)GUIDANCE_THRUST_LIMIT,
			 (double)CLEARANCE_THRUST_LIMIT,
			 (double)MAX_THRUST);

		PX4_INFO("ALT floor soft=%.2f hard=%.2f m up_max=%.1f m/s^2",
			 (double)ALT_FLOOR_SOFT_DROP,
			 (double)ALT_FLOOR_HARD_DROP,
			 (double)ALT_FLOOR_UP_ACCEL_MAX);

		PX4_INFO("MISS trigger: arm Vc>%.1f, then raw Vc<%.1f for %.2f s",
			 (double)MISS_ARM_VC,
			 (double)MISS_TRIGGER_VC,
			 (double)(MISS_TRIGGER_HOLD_US * 1e-6f));

		PX4_INFO("SAFE_HOVER brake_xy=%.1f brake_z=%.1f m/s^2",
			 (double)SAFE_BRAKE_ACCEL_XY_MAX,
			 (double)SAFE_BRAKE_ACCEL_Z_MAX);

		PX4_INFO("LOS realtime print=%d rate=%.1f Hz",
			 (int)LOS_RATE_REALTIME_PRINT,
			 (double)(1e6f / (float)LOS_RATE_PRINT_INTERVAL_US));

		PX4_INFO("RELEASE_SYNC_COMMAND=%u", (unsigned)RELEASE_SYNC_COMMAND);
		return 0;
	}

private:
	// ============================================================
	// 状态机
	// ============================================================

	enum class GuidanceState : uint8_t
	{
		WAIT_LOS         = 0,
		LAUNCH           = 1,
		BPN              = 2,
		SAFE_HOVER       = 3,
		LAUNCH_CLEARANCE = 4,
		TRANSITION       = 5,
		WAIT_RELEASE     = 6
	};

	// ============================================================
	// 基本周期
	// ============================================================

	static constexpr uint32_t LOOP_INTERVAL_US = 20000; // 50 Hz
	static constexpr uint64_t LOS_TIMEOUT_US = 500000;
	static constexpr uint64_t LAUNCH_DURATION_US = 100000;

	// LOS实时打印默认5 Hz。
	// 真机正式飞行若不需要终端打印，建议设为false。
	static constexpr bool LOS_RATE_REALTIME_PRINT = true;
	static constexpr uint64_t LOS_RATE_PRINT_INTERVAL_US = 200000;

	// ============================================================
	// Rotor-only launch clearance
	// ============================================================

	static constexpr uint64_t CLEARANCE_RAMP_DURATION_US = 300000;
	static constexpr uint64_t CLEARANCE_HOLD_DURATION_US = 600000;

	static constexpr float CLEARANCE_START_THRUST = 0.80f;
	static constexpr float CLEARANCE_THRUST_LIMIT = 0.95f;
	static constexpr float CLEARANCE_FORWARD_ACCEL = 1.5f;

	// ============================================================
	// CLEARANCE -> BPN transition
	// ============================================================

	static constexpr uint64_t TRANSITION_DURATION_US = 1000000;

	// WAIT_RELEASE阶段guidance本身不给主动推力。
	// 真机armed-idle由PX4输出/ESC机制负责。
	static constexpr float WAIT_RELEASE_THRUST = 0.0f;

	// ============================================================
	// Guidance基本参数
	// ============================================================

	static constexpr float GRAVITY = 9.80665f;
	static constexpr float HOVER_THRUST = 0.50f;

	static constexpr float AXIAL_ACCEL_MAX = 6.0f;

	static constexpr float PN_N = 3.0f;
	static constexpr float BPN_ACCEL_MAX = 12.0f;

	// ============================================================
	// EKF闭合速度
	// ============================================================

	static constexpr float VC_FALLBACK = 20.0f;
	static constexpr float VC_MAX = 35.0f;
	static constexpr float VC_LPF_TAU_RISE = 0.12f;
	static constexpr float VC_LPF_TAU_FALL = 0.03f;

	// ============================================================
	// 制导速度管理
	//
	// <=0.05 rad/s -> 26m/s
	//  0.05~0.15  -> 26 -> 24m/s
	//  0.15~0.30  -> 24 -> 21m/s
	// >=0.30      -> 18m/s
	//
	// 28m/s作为硬速度保护阈值。
	// ============================================================

	static constexpr float GUIDANCE_SPEED_CRUISE = 26.0f;
	static constexpr float GUIDANCE_SPEED_MID_1 = 24.0f;
	static constexpr float GUIDANCE_SPEED_MID_2 = 21.0f;
	static constexpr float GUIDANCE_SPEED_MIN = 18.0f;
	static constexpr float GUIDANCE_SPEED_HARD_LIMIT = 28.0f;

	static constexpr float LOS_RATE_SPEED_POINT_1 = 0.05f;
	static constexpr float LOS_RATE_SPEED_POINT_2 = 0.15f;
	static constexpr float LOS_RATE_SPEED_POINT_3 = 0.30f;

	// 在目标速度前4m/s开始逐渐收掉轴向加速。
	static constexpr float SPEED_ACCEL_TAPER_BAND = 4.0f;

	// 一旦超过动态目标速度就开始主动制动。
	static constexpr float SPEED_BRAKE_DEADBAND = 0.0f;
	static constexpr float SPEED_BRAKE_KP = 3.0f;
	static constexpr float SPEED_BRAKE_ACCEL_MAX = 8.0f;

	// 在28m/s之前提前1m/s进入硬保护，避免等超过28再反应。
	static constexpr float HARD_SPEED_BRAKE_START = 27.0f;
	static constexpr float HARD_SPEED_BRAKE_KP = 5.0f;
	static constexpr float HARD_SPEED_BRAKE_ACCEL_MAX = 10.0f;

	// ============================================================
	// 推力限制
	// ============================================================

	static constexpr float GUIDANCE_THRUST_LIMIT = 0.75f;

	static constexpr float MIN_THRUST = 0.10f;
	static constexpr float MAX_THRUST = 0.95f;

	// 发射初段允许姿态目标快速变化
	static constexpr float CLEARANCE_ATTITUDE_SLEW_DEG_S = 100.0f;
	static constexpr float CLEARANCE_ATTITUDE_SLEW_RAD_S = CLEARANCE_ATTITUDE_SLEW_DEG_S * M_PI_F / 180.f;

	// Guidance姿态setpoint限制,正常BPN继续保持比较平滑
	static constexpr float GUIDANCE_ATTITUDE_SLEW_DEG_S = 60.0f;
	static constexpr float GUIDANCE_ATTITUDE_SLEW_RAD_S = GUIDANCE_ATTITUDE_SLEW_DEG_S * M_PI_F / 180.f;

	static constexpr float GUIDANCE_ATTITUDE_LEAD_MAX_DEG = 60.0f;
	static constexpr float GUIDANCE_ATTITUDE_LEAD_MAX_RAD = GUIDANCE_ATTITUDE_LEAD_MAX_DEG * M_PI_F / 180.f;

	// ============================================================
	// RELEASE高度底线保护
	// PX4 NED：z增加 = 向下
	// CLEARANCE不再固定施加向上5m/s^2。只有掉高或预测会继续掉高时，这个保护才介入。
	// ============================================================

	// ============================================================
	// RELEASE高度单边保护
	// ============================================================
	static constexpr float ALT_FLOOR_KP = 6.0f;
	static constexpr float ALT_FLOOR_KD = 2.5f;

	// 实际掉高15cm以内基本认为正常
	static constexpr float ALT_FLOOR_SOFT_DROP = 0.15f;

	// 根据当前下降速度预测最终会掉超过30cm时提前介入
	static constexpr float ALT_FLOOR_PREDICT_TRIGGER = 0.30f;

	// 希望尽量在掉高50cm附近把下降速度刹住
	static constexpr float ALT_FLOOR_HARD_DROP = 0.50f;

	// 不再使用原来的“恢复release高度”位置P,改成对“预测越界量”的P
	static constexpr float ALT_FLOOR_PRED_KP = 6.0f;

	// 对向下速度的阻尼
	static constexpr float ALT_FLOOR_VZ_KD = 2.5f;

	// 下降速度已经接近0时立即退出高度保护
	static constexpr float ALT_FLOOR_RELEASE_VZ = 0.05f;

	// 原来4.0，先提高到6.0
	static constexpr float ALT_FLOOR_UP_ACCEL_MAX = 6.0f;

	// 理论制动加速度上的余量
	static constexpr float ALT_FLOOR_ACCEL_MARGIN = 0.8f;
	// ============================================================
	// 越靶判定
	// ============================================================

	static constexpr float MISS_ARM_VC = 8.0f;
	static constexpr float MISS_TRIGGER_VC = -3.0f;
	static constexpr float MISS_MIN_SPEED = 5.0f;
	static constexpr uint64_t MISS_TRIGGER_HOLD_US = 120000;

	// ============================================================
	// SAFE_HOVER
	// ============================================================

	static constexpr float SAFE_BRAKE_KV_XY = 1.00f;
	static constexpr float SAFE_BRAKE_KV_Z = 1.20f;

	static constexpr float SAFE_BRAKE_ACCEL_XY_MAX = 8.0f;
	static constexpr float SAFE_BRAKE_ACCEL_Z_MAX = 4.5f;

	static constexpr float SAFE_LATCH_SPEED_XY = 1.0f;
	static constexpr float SAFE_LATCH_SPEED_Z = 0.5f;
	static constexpr uint64_t SAFE_LATCH_HOLD_US = 500000;

	static constexpr float SAFE_HOLD_KP_XY = 0.8f;
	static constexpr float SAFE_HOLD_KD_XY = 1.4f;
	static constexpr float SAFE_HOLD_KP_Z = 1.0f;
	static constexpr float SAFE_HOLD_KD_Z = 1.5f;

	static constexpr float SAFE_HOLD_ACCEL_XY_MAX = 4.0f;
	static constexpr float SAFE_HOLD_ACCEL_Z_MAX = 3.5f;

	// MAV_CMD_USER_1，继续兼容当前SITL Python脚本。
	static constexpr uint32_t RELEASE_SYNC_COMMAND = 31010;

	// ============================================================
	// uORB
	// ============================================================

	uORB::Subscription _los_sub{ORB_ID(vehicle_target_los)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_guidance_acceleration_s> _guidance_accel_pub{ORB_ID(vehicle_guidance_acceleration)};

	// ============================================================
	// 消息缓存
	// ============================================================

	vehicle_target_los_s _los{};
	vehicle_attitude_s _attitude{};
	vehicle_local_position_s _local_position{};
	vehicle_status_s _vehicle_status{};
	vehicle_command_s _vehicle_command{};

	// ============================================================
	// 状态
	// ============================================================

	GuidanceState _state{GuidanceState::WAIT_LOS};

	uint64_t _state_enter_time{0};
	uint64_t _last_los_rx_time{0};
	uint64_t _last_print_time{0};
	uint64_t _last_los_rate_print_time{0};
	uint64_t _last_setpoint_pub_time{0};

	bool _yaw_locked{false};
	float _yaw_lock{0.f};

	bool _was_armed{false};
	bool _launch_sequence_complete{false};

	// ============================================================
	// 发射姿态 / clearance / transition
	// ============================================================

	Quatf _hold_q{1.f, 0.f, 0.f, 0.f};

	Vector3f _clearance_start_accel_ned{0.f, 0.f, 0.f};
	Vector3f _transition_start_accel_ned{0.f, 0.f, 0.f};

	float _transition_phase{1.f};

	// ============================================================
	// Vc状态
	// ============================================================

	bool _vc_filter_initialized{false};
	bool _vc_valid{false};

	float _vc_raw{0.f};
	float _vc_filtered{0.f};
	float _vc_used{VC_FALLBACK};

	uint64_t _last_vc_update_time{0};

	bool _had_positive_closing{false};
	uint64_t _negative_closing_start_time{0};

	// ============================================================
	// 速度控制debug
	// ============================================================

	float _last_los_rate_mag{0.f};
	float _last_speed_target{GUIDANCE_SPEED_CRUISE};
	float _last_speed_axial_scale{1.f};
	float _last_speed_brake{0.f};
	float _last_axial_limit_cmd{AXIAL_ACCEL_MAX};

	// ============================================================
	// 高度底线保护
	// ============================================================

	bool _release_alt_ref_valid{false};
	float _release_alt_ref_z_ned{0.f};

	bool _alt_floor_active{false};
	float _last_alt_floor_drop{0.f};
	float _last_alt_floor_predicted_drop{0.f};
	float _last_alt_floor_up_accel{0.f};

	// ============================================================
	// 最后一次guidance结果/debug
	// ============================================================

	Quatf _last_raw_q_sp{1.f, 0.f, 0.f, 0.f};
	Quatf _last_q_sp{1.f, 0.f, 0.f, 0.f};

	float _last_thrust_sp{0.f};
	float _last_attitude_slew_deg_s{GUIDANCE_ATTITUDE_SLEW_DEG_S};
	bool _slew_active{false};
	float _last_raw_q_step_deg{0.f};

	bool _lead_active{false};
	float _last_attitude_lead_deg{0.f};

	Vector3f _last_bpn_body{0.f, 0.f, 0.f};

	float _last_axial_used{0.f};
	float _last_bpn_norm_raw{0.f};
	float _last_bpn_norm_used{0.f};
	float _last_predicted_thrust{0.f};
	float _last_bpn_scale{1.f};

	// ============================================================
	// SAFE_HOVER状态
	// ============================================================

	bool _safe_hold_latched{false};
	uint64_t _safe_low_speed_start_time{0};
	Vector3f _safe_hold_position_ned{0.f, 0.f, 0.f};

	// ============================================================
	// EKF reset监控
	// ============================================================

	bool _ekf_reset_counter_initialized{false};
	uint8_t _last_xy_reset_counter{0};
	uint8_t _last_z_reset_counter{0};
	uint8_t _last_vxy_reset_counter{0};
	uint8_t _last_vz_reset_counter{0};

	bool _attitude_reset_counter_initialized{false};
	uint8_t _last_quat_reset_counter{0};

	// ============================================================
	// 工具函数
	// ============================================================

	float wrap_pi_local(float angle) const
	{
		while (angle > M_PI_F) { angle -= 2.f * M_PI_F; }
		while (angle < -M_PI_F) { angle += 2.f * M_PI_F; }
		return angle;
	}

	float smoothstep01(const float input) const
	{
		const float p = math::constrain(input, 0.f, 1.f);
		return p * p * (3.f - 2.f * p);
	}

	float lerp(const float a, const float b, const float t) const
	{
		return a + (b - a) * math::constrain(t, 0.f, 1.f);
	}

	float quat_angle_rad(const Quatf &qa, const Quatf &qb) const
	{
		float dot = qa(0) * qb(0) + qa(1) * qb(1) + qa(2) * qb(2) + qa(3) * qb(3);
		dot = fabsf(dot);
		dot = math::constrain(dot, 0.f, 1.f);
		return 2.f * acosf(dot);
	}

	Quatf slerp_limited(const Quatf &q_from_in, const Quatf &q_to_in, const float max_angle_rad,
			   bool &limited, float &raw_angle_rad) const
	{
		Quatf q_from = q_from_in;
		Quatf q_to = q_to_in;

		q_from.normalize();
		q_to.normalize();

		float dot = q_from(0) * q_to(0) + q_from(1) * q_to(1) + q_from(2) * q_to(2) + q_from(3) * q_to(3);

		if (dot < 0.f) {
			dot = -dot;
			q_to = Quatf{-q_to(0), -q_to(1), -q_to(2), -q_to(3)};
		}

		dot = math::constrain(dot, 0.f, 1.f);
		raw_angle_rad = 2.f * acosf(dot);

		if (!PX4_ISFINITE(raw_angle_rad) || raw_angle_rad <= max_angle_rad || raw_angle_rad < 1e-6f) {
			limited = false;
			return q_to;
		}

		limited = true;

		const float fraction = math::constrain(max_angle_rad / raw_angle_rad, 0.f, 1.f);
		const float theta = acosf(dot);
		const float sin_theta = sinf(theta);

		Quatf q_out;

		if (fabsf(sin_theta) < 1e-6f) {
			q_out = Quatf{
				(1.f - fraction) * q_from(0) + fraction * q_to(0),
				(1.f - fraction) * q_from(1) + fraction * q_to(1),
				(1.f - fraction) * q_from(2) + fraction * q_to(2),
				(1.f - fraction) * q_from(3) + fraction * q_to(3)
			};

		} else {
			const float w0 = sinf((1.f - fraction) * theta) / sin_theta;
			const float w1 = sinf(fraction * theta) / sin_theta;

			q_out = Quatf{
				w0 * q_from(0) + w1 * q_to(0),
				w0 * q_from(1) + w1 * q_to(1),
				w0 * q_from(2) + w1 * q_to(2),
				w0 * q_from(3) + w1 * q_to(3)
			};
		}

		q_out.normalize();
		return q_out;
	}

	void constrain_xy(Vector3f &accel, const float max_xy) const
	{
		const float norm_xy = sqrtf(accel(0) * accel(0) + accel(1) * accel(1));

		if (PX4_ISFINITE(norm_xy) && norm_xy > max_xy && norm_xy > FLT_EPSILON) {
			const float scale = max_xy / norm_xy;
			accel(0) *= scale;
			accel(1) *= scale;
		}
	}

	// ============================================================
	// 基本判断
	// ============================================================

	bool attitude_valid() const
	{
		float norm_sq = 0.f;

		for (int i = 0; i < 4; ++i) {
			if (!PX4_ISFINITE(_attitude.q[i])) { return false; }
			norm_sq += _attitude.q[i] * _attitude.q[i];
		}

		return norm_sq > 0.25f;
	}

	bool los_fresh(const uint64_t now) const
	{
		return (_last_los_rx_time > 0) && (now - _last_los_rx_time < LOS_TIMEOUT_US);
	}

	bool armed() const
	{
		return _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	bool velocity_valid() const
	{
		return _local_position.v_xy_valid && _local_position.v_z_valid
		       && PX4_ISFINITE(_local_position.vx)
		       && PX4_ISFINITE(_local_position.vy)
		       && PX4_ISFINITE(_local_position.vz);
	}

	bool position_valid() const
	{
		return _local_position.xy_valid && _local_position.z_valid
		       && PX4_ISFINITE(_local_position.x)
		       && PX4_ISFINITE(_local_position.y)
		       && PX4_ISFINITE(_local_position.z);
	}

	Quatf current_attitude_q() const
	{
		Quatf q{_attitude.q};
		const float norm = q.norm();

		if (PX4_ISFINITE(norm) && norm > FLT_EPSILON) {
			q.normalize();

		} else {
			q = Quatf{1.f, 0.f, 0.f, 0.f};
		}

		return q;
	}

	float get_ekf_speed() const
	{
		if (!velocity_valid()) { return NAN; }

		return sqrtf(
			       _local_position.vx * _local_position.vx
			       + _local_position.vy * _local_position.vy
			       + _local_position.vz * _local_position.vz
		       );
	}

	float get_ekf_speed_xy() const
	{
		if (!velocity_valid()) { return NAN; }

		return sqrtf(
			       _local_position.vx * _local_position.vx
			       + _local_position.vy * _local_position.vy
		       );
	}

	// ============================================================
	// LOS几何
	// ============================================================

	Vector3f calculate_los_body(const float alpha, const float beta) const
	{
		Vector3f los_body{-sinf(beta), sinf(alpha) * cosf(beta), -cosf(alpha) * cosf(beta)};
		const float norm = los_body.norm();

		if (PX4_ISFINITE(norm) && norm > FLT_EPSILON) {
			los_body /= norm;
		}

		return los_body;
	}

	Vector3f calculate_los_dot_body(const float alpha, const float beta,
					const float alpha_dot, const float beta_dot) const
	{
		const float ca = cosf(alpha);
		const float sa = sinf(alpha);
		const float cb = cosf(beta);
		const float sb = sinf(beta);

		const Vector3f e_az{0.f, ca, sa};
		const Vector3f e_el{-cb, -sa * sb, ca * sb};

		return e_az * (alpha_dot * cb) + e_el * beta_dot;
	}

	Vector3f calculate_bpn_body(const Vector3f &los_body, const Vector3f &los_dot_body,
				    const float vc_used) const
	{
		Vector3f accel_body = los_dot_body * (PN_N * vc_used);

		// 去掉LOS轴向分量，仅保留法向加速度。
		accel_body -= los_body * accel_body.dot(los_body);

		const float accel_norm = accel_body.norm();

		if (PX4_ISFINITE(accel_norm) && accel_norm > BPN_ACCEL_MAX) {
			accel_body *= BPN_ACCEL_MAX / accel_norm;
		}

		return accel_body;
	}

	bool calculate_los_ned(Vector3f &los_ned) const
	{
		if (!attitude_valid() || !_los.target_valid) { return false; }

		const float alpha = _los.los_azimuth;
		const float beta = _los.los_elevation;

		if (!PX4_ISFINITE(alpha) || !PX4_ISFINITE(beta)) { return false; }

		const Vector3f los_body = calculate_los_body(alpha, beta);
		const Dcmf R_nb{current_attitude_q()};

		los_ned = R_nb * los_body;

		const float norm = los_ned.norm();

		if (!PX4_ISFINITE(norm) || norm < FLT_EPSILON) { return false; }

		los_ned /= norm;
		return true;
	}

	// ============================================================
	// 动态制导目标速度
	// ============================================================

	float calculate_guidance_speed_target(const float los_rate_mag) const
	{
		if (!PX4_ISFINITE(los_rate_mag)) { return GUIDANCE_SPEED_CRUISE; }

		if (los_rate_mag <= LOS_RATE_SPEED_POINT_1) {
			return GUIDANCE_SPEED_CRUISE;
		}

		if (los_rate_mag <= LOS_RATE_SPEED_POINT_2) {
			const float t = (los_rate_mag - LOS_RATE_SPEED_POINT_1)
					/ (LOS_RATE_SPEED_POINT_2 - LOS_RATE_SPEED_POINT_1);

			return lerp(GUIDANCE_SPEED_CRUISE, GUIDANCE_SPEED_MID_1, t);
		}

		if (los_rate_mag <= LOS_RATE_SPEED_POINT_3) {
			const float t = (los_rate_mag - LOS_RATE_SPEED_POINT_2)
					/ (LOS_RATE_SPEED_POINT_3 - LOS_RATE_SPEED_POINT_2);

			return lerp(GUIDANCE_SPEED_MID_1, GUIDANCE_SPEED_MID_2, t);
		}

		return GUIDANCE_SPEED_MIN;
	}

	float calculate_axial_speed_scale(const float speed, const float target_speed) const
	{
		if (!PX4_ISFINITE(speed)) { return 1.f; }

		const float taper_start = fmaxf(target_speed - SPEED_ACCEL_TAPER_BAND, 0.f);

		if (speed <= taper_start) { return 1.f; }
		if (speed >= target_speed) { return 0.f; }

		return math::constrain(
			       (target_speed - speed) / fmaxf(target_speed - taper_start, 0.1f),
			       0.f,
			       1.f
		       );
	}

	Vector3f calculate_speed_brake_accel_ned(const float target_speed, float &brake_mag) const
	{
		Vector3f brake{0.f, 0.f, 0.f};
		brake_mag = 0.f;

		if (!velocity_valid()) { return brake; }

		const Vector3f velocity_ned{
			_local_position.vx,
			_local_position.vy,
			_local_position.vz
		};

		const float speed = velocity_ned.norm();
		const float speed_xy = sqrtf(
				       _local_position.vx * _local_position.vx
				       + _local_position.vy * _local_position.vy
			       );

		if (!PX4_ISFINITE(speed) || speed < 0.1f) { return brake; }

		float normal_brake = 0.f;
		const float normal_brake_start = target_speed + SPEED_BRAKE_DEADBAND;

		if (speed > normal_brake_start) {
			normal_brake = math::constrain(
					       SPEED_BRAKE_KP * (speed - normal_brake_start),
					       0.f,
					       SPEED_BRAKE_ACCEL_MAX
				       );
		}

		float hard_brake = 0.f;

		if (speed > HARD_SPEED_BRAKE_START) {
			hard_brake = math::constrain(
					     HARD_SPEED_BRAKE_KP * (speed - HARD_SPEED_BRAKE_START),
					     0.f,
					     HARD_SPEED_BRAKE_ACCEL_MAX
				     );
		}

		// 硬速度保护优先：直接沿三维速度反方向减速。
		if (hard_brake > 0.f && hard_brake >= normal_brake) {
			brake_mag = hard_brake;
			brake = -velocity_ned * (hard_brake / speed);
			return brake;
		}

		// 正常动态目标速度制动仍尽量只使用XY，
		// 把垂向自由度留给BPN和高度保护。
		if (normal_brake > 0.f && PX4_ISFINITE(speed_xy) && speed_xy > 0.1f) {
			brake_mag = normal_brake;
			brake(0) = -normal_brake * _local_position.vx / speed_xy;
			brake(1) = -normal_brake * _local_position.vy / speed_xy;
			brake(2) = 0.f;
		}

		return brake;
	}

	// ============================================================
	// EKF闭合速度
	//
	// 静止目标 / 未知目标速度：
	// Vc = v_aircraft_NED · LOS_NED
	// 正：接近目标
	// 负：远离目标
	// ============================================================

	void update_closing_speed(const uint64_t now)
	{
		Vector3f los_ned;

		const bool valid = velocity_valid() && calculate_los_ned(los_ned);

		if (!valid) {
			_vc_valid = false;
			_vc_used = VC_FALLBACK;
			_last_vc_update_time = now;
			return;
		}

		const Vector3f velocity_ned{
			_local_position.vx,
			_local_position.vy,
			_local_position.vz
		};

		_vc_raw = velocity_ned.dot(los_ned);

		if (!PX4_ISFINITE(_vc_raw)) {
			_vc_valid = false;
			_vc_used = VC_FALLBACK;
			_last_vc_update_time = now;
			return;
		}

		float dt = LOOP_INTERVAL_US * 1e-6f;

		if (_last_vc_update_time > 0 && now > _last_vc_update_time) {
			dt = (float)(now - _last_vc_update_time) * 1e-6f;
			dt = math::constrain(dt, 0.001f, 0.10f);
		}

		_last_vc_update_time = now;

		if (!_vc_filter_initialized) {
			_vc_filtered = _vc_raw;
			_vc_filter_initialized = true;

		} else {
			const float tau = (_vc_raw < _vc_filtered) ? VC_LPF_TAU_FALL : VC_LPF_TAU_RISE;
			const float alpha = dt / (tau + dt);
			_vc_filtered += alpha * (_vc_raw - _vc_filtered);
		}

		_vc_valid = true;
		_vc_used = math::constrain(fmaxf(_vc_filtered, 0.f), 0.f, VC_MAX);

		if (_vc_raw > MISS_ARM_VC) {
			_had_positive_closing = true;
		}
	}

	// ============================================================
	// attitude + thrust -> 净惯性加速度
	// ============================================================

	Vector3f attitude_thrust_to_net_accel(const Quatf &q_nb, const float thrust_norm) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};
		const Vector3f thrust_axis_body{0.f, 0.f, -1.f};

		const Dcmf R_nb{q_nb};
		const Vector3f thrust_axis_ned = R_nb * thrust_axis_body;

		const float thrust_accel = GRAVITY * thrust_norm / HOVER_THRUST;

		return gravity_ned + thrust_axis_ned * thrust_accel;
	}

	// ============================================================
	// 推力预算
	// ============================================================

	float thrust_accel_budget(const float thrust_limit) const
	{
		return GRAVITY * thrust_limit / HOVER_THRUST;
	}

	float predicted_thrust_norm_from_accel(const Vector3f &accel_sp_ned) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};
		const Vector3f thrust_accel = accel_sp_ned - gravity_ned;

		return HOVER_THRUST * thrust_accel.norm() / GRAVITY;
	}

	// ============================================================
	// 垂向优先推力预算
	//
	// thrust_limit由调用者决定：
	// CLEARANCE = 0.95
	// BPN/TRANSITION = 0.75
	// ============================================================

	void constrain_net_accel_vertical_priority(Vector3f &accel_sp_ned, const float thrust_limit) const
	{
		const float budget = thrust_accel_budget(thrust_limit);

		float thrust_z = accel_sp_ned(2) - GRAVITY;
		float thrust_xy = sqrtf(
				  accel_sp_ned(0) * accel_sp_ned(0)
				  + accel_sp_ned(1) * accel_sp_ned(1)
			  );

		if (!PX4_ISFINITE(thrust_z) || !PX4_ISFINITE(thrust_xy)) { return; }

		if (fabsf(thrust_z) >= budget) {
			thrust_z = math::constrain(thrust_z, -budget, budget);

			accel_sp_ned(0) = 0.f;
			accel_sp_ned(1) = 0.f;
			accel_sp_ned(2) = thrust_z + GRAVITY;
			return;
		}

		const float xy_available = sqrtf(fmaxf(budget * budget - thrust_z * thrust_z, 0.f));

		if (thrust_xy > xy_available && thrust_xy > FLT_EPSILON) {
			const float scale = xy_available / thrust_xy;
			accel_sp_ned(0) *= scale;
			accel_sp_ned(1) *= scale;
		}
	}

	float scale_bpn_to_budget(const Vector3f &bpn_accel_ned, const float budget_accel) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};

		if ((bpn_accel_ned - gravity_ned).norm() <= budget_accel) {
			return 1.f;
		}

		float low = 0.f;
		float high = 1.f;

		for (int i = 0; i < 12; ++i) {
			const float mid = 0.5f * (low + high);
			const float required = (bpn_accel_ned * mid - gravity_ned).norm();

			if (required <= budget_accel) {
				low = mid;

			} else {
				high = mid;
			}
		}

		return math::constrain(low, 0.f, 1.f);
	}

	void allocate_guidance_acceleration(const Vector3f &los_ned,
					    const Vector3f &bpn_accel_ned_raw,
					    const float axial_accel_limit,
					    Vector3f &accel_sp_ned,
					    float &axial_used,
					    float &bpn_scale,
					    float &predicted_thrust) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};
		const float budget = thrust_accel_budget(GUIDANCE_THRUST_LIMIT);

		// 第一级：BPN优先。
		bpn_scale = scale_bpn_to_budget(bpn_accel_ned_raw, budget);

		const Vector3f bpn_accel_ned = bpn_accel_ned_raw * bpn_scale;

		// 第二级：剩余推力分配给LOS轴向加速度。
		const Vector3f c = bpn_accel_ned - gravity_ned;

		const float d = c.dot(los_ned);
		const float c2 = c.norm_squared();

		const float discriminant = d * d - (c2 - budget * budget);

		axial_used = 0.f;

		if (discriminant >= 0.f && PX4_ISFINITE(discriminant)) {
			const float root = -d + sqrtf(discriminant);

			if (PX4_ISFINITE(root) && root > 0.f) {
				axial_used = math::constrain(root, 0.f, axial_accel_limit);
			}
		}

		accel_sp_ned = bpn_accel_ned + los_ned * axial_used;
		predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);
	}

	// ============================================================
	// RELEASE高度底线
	// ============================================================

	void apply_release_altitude_floor_guard(Vector3f &accel_sp_ned)
	{
	    _alt_floor_active = false;
	    _last_alt_floor_up_accel = 0.f;

	    if (!_release_alt_ref_valid
		|| !_local_position.z_valid
		|| !_local_position.v_z_valid
		|| !PX4_ISFINITE(_local_position.z)
		|| !PX4_ISFINITE(_local_position.vz)) {
		return;
	    }

	    // PX4 NED：
	    // z增大 = 向下
	    // vz > 0 = 正在下降
	    const float drop =
		_local_position.z - _release_alt_ref_z_ned;

	    const float vz_down =
		fmaxf(_local_position.vz, 0.f);

	    // 已经停止下降或开始上升：
	    // ALT_FLOOR立即退出，不负责把飞机拉回release高度。
	    if (_local_position.vz <= ALT_FLOOR_RELEASE_VZ) {
		_last_alt_floor_drop = drop;
		_last_alt_floor_predicted_drop = drop;
		return;
	    }

	    // 按最大允许向上加速度估算：
	    // 以当前下降速度刹到0，还需要多少距离。
	    const float stop_distance =
		(vz_down * vz_down)
		/ fmaxf(
		    2.f * ALT_FLOOR_UP_ACCEL_MAX,
		    0.1f
		);

	    const float predicted_drop =
		drop + stop_distance;

	    _last_alt_floor_drop = drop;
	    _last_alt_floor_predicted_drop = predicted_drop;

	    // 当前掉高不大，同时预测也不会越过警戒线：
	    // 不需要高度保护。
	    if (drop <= ALT_FLOOR_SOFT_DROP
		&& predicted_drop <= ALT_FLOOR_PREDICT_TRIGGER) {
		return;
	    }

	    _alt_floor_active = true;

	    // 距离HARD_DROP还剩多少制动空间。
	    const float remaining =
		fmaxf(
		    ALT_FLOOR_HARD_DROP - drop,
		    0.05f
		);

	    // 基于物理停车距离计算最小制动加速度。
	    const float accel_stop =
		(vz_down * vz_down)
		/ (2.f * remaining)
		+ ALT_FLOOR_ACCEL_MARGIN;

	    // 预测越过SOFT线多少。
	    const float predicted_violation =
		fmaxf(
		    predicted_drop - ALT_FLOOR_SOFT_DROP,
		    0.f
		);

	    // 单边barrier控制：
	    // P项看“预测越界量”
	    // D项看“当前下降速度”
	    const float accel_barrier =
		ALT_FLOOR_PRED_KP * predicted_violation
		+ ALT_FLOOR_VZ_KD * vz_down;

	    // ★关键就在这里： accel_barrier必须实际参与最终计算。
	    const float accel_up =
	    math::constrain(
		fmaxf(accel_stop, accel_barrier),
		0.f,
		ALT_FLOOR_UP_ACCEL_MAX
	    );

	    _last_alt_floor_up_accel = accel_up;

	    // NED中负Z方向是向上。
	    // 只允许ALT_FLOOR增加向上控制，
	    // 不削弱原本已经更强的向上控制。
	    accel_sp_ned(2) =
		fminf(
		    accel_sp_ned(2),
		    -accel_up
		);
	}
	// ============================================================
	// acceleration -> attitude
	// ============================================================

	void acceleration_to_attitude(const Vector3f &accel_sp_ned, const float yaw_sp,
				      Quatf &q_sp, float &thrust_norm) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};

		const Vector3f thrust_accel_ned = accel_sp_ned - gravity_ned;
		const float thrust_accel_mag = thrust_accel_ned.norm();

		Vector3f body_z = -thrust_accel_ned;

		if (body_z.norm_squared() < FLT_EPSILON) {
			body_z = Vector3f{0.f, 0.f, 1.f};
		}

		body_z.normalize();

		const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

		Vector3f body_x = y_C % body_z;

		if (body_z(2) < 0.f) {
			body_x = -body_x;
		}

		if (fabsf(body_z(2)) < 0.000001f) {
			body_x.zero();
			body_x(2) = 1.f;
		}

		if (body_x.norm_squared() < FLT_EPSILON) {
			body_x = Vector3f{cosf(yaw_sp), sinf(yaw_sp), 0.f};
		}

		body_x.normalize();

		const Vector3f body_y = body_z % body_x;

		Dcmf R_sp;

		for (int i = 0; i < 3; ++i) {
			R_sp(i, 0) = body_x(i);
			R_sp(i, 1) = body_y(i);
			R_sp(i, 2) = body_z(i);
		}

		q_sp = Quatf{R_sp};
		q_sp.normalize();

		thrust_norm = HOVER_THRUST * thrust_accel_mag / GRAVITY;
		thrust_norm = math::constrain(thrust_norm, MIN_THRUST, MAX_THRUST);
	}

	// ============================================================
	// 发布
	// ============================================================

	void publish_attitude_setpoint_internal(const Quatf &q_d, const float thrust_norm, const uint64_t now)
	{
		vehicle_attitude_setpoint_s sp{};

		sp.timestamp = now;
		q_d.copyTo(sp.q_d);
		sp.yaw_sp_move_rate = 0.f;

		sp.thrust_body[0] = 0.f;
		sp.thrust_body[1] = 0.f;
		sp.thrust_body[2] = -math::constrain(thrust_norm, 0.f, MAX_THRUST);

		_attitude_setpoint_pub.publish(sp);

		_last_q_sp = q_d;
		_last_thrust_sp = math::constrain(thrust_norm, 0.f, MAX_THRUST);
		_last_setpoint_pub_time = now;
	}

	void publish_direct_attitude(const Quatf &q_d, const float thrust_norm, const uint64_t now)
	{
		_last_raw_q_sp = q_d;
		_slew_active = false;
		_last_raw_q_step_deg = 0.f;
		_lead_active = false;
		_last_attitude_lead_deg = 0.f;

		publish_attitude_setpoint_internal(q_d, thrust_norm, now);
	}

	void publish_accel_attitude_setpoint(const Vector3f &accel_sp_ned, const uint64_t now)
	{
		Quatf q_raw;
		float thrust_norm = 0.f;

		acceleration_to_attitude(accel_sp_ned, _yaw_lock, q_raw, thrust_norm);

		_last_raw_q_sp = q_raw;

		float dt = LOOP_INTERVAL_US * 1e-6f;

		if (_last_setpoint_pub_time > 0 && now > _last_setpoint_pub_time) {
			dt = (float)(now - _last_setpoint_pub_time) * 1e-6f;
			dt = math::constrain(dt, 0.001f, 0.10f);
		}

		// const float max_step_rad = GUIDANCE_ATTITUDE_SLEW_RAD_S * dt;

		// ============================================================
		// 根据当前制导状态选择姿态setpoint最大变化速度
		// LAUNCH_CLEARANCE：100 deg/s
		// TRANSITION：100 -> 60 deg/s
		// BPN/其他状态：60 deg/s
		// ============================================================

		float attitude_slew_deg_s = GUIDANCE_ATTITUDE_SLEW_DEG_S;
		if (_state == GuidanceState::LAUNCH_CLEARANCE) {
		    // 发射初段允许姿态目标更快变化，尽快把推力方向从大俯仰姿态转向有效方向。
		    attitude_slew_deg_s =
			CLEARANCE_ATTITUDE_SLEW_DEG_S;
		} else if (_state == GuidanceState::TRANSITION) {
		    // _transition_phase:
		    // 0 -> 刚进入TRANSITION
		    // 1 -> TRANSITION结束
		    // 因此：
		    // phase=0 -> 100 deg/s
		    // phase=0.5 -> 80 deg/s
		    // phase=1 -> 60 deg/s
		    attitude_slew_deg_s =
			CLEARANCE_ATTITUDE_SLEW_DEG_S
			+ _transition_phase
			* (GUIDANCE_ATTITUDE_SLEW_DEG_S
			   - CLEARANCE_ATTITUDE_SLEW_DEG_S);
		}

		const float attitude_slew_rad_s =
		    attitude_slew_deg_s * M_PI_F / 180.f;

		const float max_step_rad =
		    attitude_slew_rad_s * dt;

		_last_attitude_slew_deg_s = attitude_slew_deg_s;

		float raw_step_rad = 0.f;

		// 第一级：限制setpoint本身角速度。
		const Quatf q_slew = slerp_limited(
					     _last_q_sp,
					     q_raw,
					     max_step_rad,
					     _slew_active,
					     raw_step_rad
				     );

		_last_raw_q_step_deg = raw_step_rad * 180.f / M_PI_F;

		// 第二级：限制setpoint相对于飞机真实姿态最多领先30deg。
		float lead_angle_rad = 0.f;

		const Quatf q_cmd = slerp_limited(
					    current_attitude_q(),
					    q_slew,
					    GUIDANCE_ATTITUDE_LEAD_MAX_RAD,
					    _lead_active,
					    lead_angle_rad
				    );

		_last_attitude_lead_deg = lead_angle_rad * 180.f / M_PI_F;

		publish_attitude_setpoint_internal(q_cmd, thrust_norm, now);
	}

	void publish_guidance_acceleration(const Vector3f &bpn_accel_body, const bool valid, const uint64_t now)
	{
		vehicle_guidance_acceleration_s msg{};

		msg.timestamp = now;

		msg.acceleration[0] = bpn_accel_body(0);
		msg.acceleration[1] = bpn_accel_body(1);
		msg.acceleration[2] = bpn_accel_body(2);

		msg.valid = valid;

		_guidance_accel_pub.publish(msg);
	}

	// ============================================================
	// BPN目标净加速度
	// ============================================================

	bool calculate_bpn_target(Vector3f &accel_sp_ned, Vector3f &bpn_accel_body)
	{
		if (!attitude_valid() || !_los.target_valid) { return false; }

		const float alpha = _los.los_azimuth;
		const float beta = _los.los_elevation;
		const float alpha_dot = _los.los_az_rate;
		const float beta_dot = _los.los_el_rate;

		if (!PX4_ISFINITE(alpha)
		    || !PX4_ISFINITE(beta)
		    || !PX4_ISFINITE(alpha_dot)
		    || !PX4_ISFINITE(beta_dot)) {
			return false;
		}

		const Vector3f los_body = calculate_los_body(alpha, beta);
		const Vector3f los_dot_body =
			calculate_los_dot_body(alpha, beta, alpha_dot, beta_dot);

		// ========================================================
		// 动态速度规划
		// ========================================================

		_last_los_rate_mag = los_dot_body.norm();
		_last_speed_target = calculate_guidance_speed_target(_last_los_rate_mag);

		const float ekf_speed = get_ekf_speed();

		_last_speed_axial_scale =
			PX4_ISFINITE(ekf_speed)
			? calculate_axial_speed_scale(ekf_speed, _last_speed_target)
			: 1.f;

		_last_axial_limit_cmd = AXIAL_ACCEL_MAX * _last_speed_axial_scale;

		// ========================================================
		// BPN
		// ========================================================

		bpn_accel_body = calculate_bpn_body(los_body, los_dot_body, _vc_used);

		_last_bpn_norm_raw = bpn_accel_body.norm();

		const Dcmf R_nb{current_attitude_q()};

		Vector3f los_ned = R_nb * los_body;

		const float los_ned_norm = los_ned.norm();

		if (!PX4_ISFINITE(los_ned_norm) || los_ned_norm < FLT_EPSILON) {
			return false;
		}

		los_ned /= los_ned_norm;

		const Vector3f bpn_accel_ned_raw = R_nb * bpn_accel_body;

		float axial_used = 0.f;
		float bpn_scale = 1.f;
		float predicted_thrust = 0.f;

		allocate_guidance_acceleration(
			los_ned,
			bpn_accel_ned_raw,
			_last_axial_limit_cmd,
			accel_sp_ned,
			axial_used,
			bpn_scale,
			predicted_thrust
		);

		// ========================================================
		// 超速 / 末段动态减速
		// ========================================================

		float speed_brake_mag = 0.f;

		const Vector3f speed_brake_ned =
			calculate_speed_brake_accel_ned(
				_last_speed_target,
				speed_brake_mag
			);

		accel_sp_ned += speed_brake_ned;

		_last_speed_brake = speed_brake_mag;

		// 发布的BPN与第一阶段分配后的BPN一致。
		bpn_accel_body *= bpn_scale;

		_last_axial_used = axial_used;
		_last_bpn_scale = bpn_scale;
		_last_bpn_norm_used = bpn_accel_body.norm();
		_last_predicted_thrust = predicted_thrust;

		return true;
	}

	// ============================================================
	// LAUNCH_CLEARANCE目标
	//
	// 只给小幅水平前向净加速度。
	// z=0表示默认不主动制造爬升/下降净加速度。
	// 垂向掉高由release altitude floor guard单独处理。
	// ============================================================

	bool calculate_clearance_target(Vector3f &accel_sp_ned)
	{
		Vector3f los_ned;

		if (!calculate_los_ned(los_ned)) { return false; }

		Vector3f horizontal_dir{los_ned(0), los_ned(1), 0.f};

		const float horizontal_norm = horizontal_dir.norm();

		if (PX4_ISFINITE(horizontal_norm) && horizontal_norm > FLT_EPSILON) {
			horizontal_dir /= horizontal_norm;

		} else {
			horizontal_dir.zero();
		}

		accel_sp_ned = horizontal_dir * CLEARANCE_FORWARD_ACCEL;
		accel_sp_ned(2) = 0.f;

		return true;
	}

	// ============================================================
	// 发射阶段
	// ============================================================

	void enter_wait_release(const uint64_t now)
	{
		_hold_q = current_attitude_q();

		_state = GuidanceState::WAIT_RELEASE;
		_state_enter_time = now;
		_transition_phase = 0.f;

		PX4_INFO("ARM -> WAIT_RELEASE");
	}

	void enter_launch_clearance(const uint64_t now)
	{
		_hold_q = current_attitude_q();

		_clearance_start_accel_ned =
			attitude_thrust_to_net_accel(
				_hold_q,
				CLEARANCE_START_THRUST
			);

		// RELEASE高度作为掉高保护参考。
		if (_local_position.z_valid && PX4_ISFINITE(_local_position.z)) {
			_release_alt_ref_z_ned = _local_position.z;
			_release_alt_ref_valid = true;

		} else {
			_release_alt_ref_valid = false;
		}

		_alt_floor_active = false;
		_last_alt_floor_drop = 0.f;
		_last_alt_floor_predicted_drop = 0.f;
		_last_alt_floor_up_accel = 0.f;

		_state = GuidanceState::LAUNCH_CLEARANCE;
		_state_enter_time = now;
		_transition_phase = 0.f;

		_had_positive_closing = false;
		_negative_closing_start_time = 0;

		PX4_INFO("RELEASE SIGNAL -> LAUNCH_CLEARANCE");

		PX4_INFO("CLEARANCE start_thr %.2f limit %.2f ramp %.2f s hold %.2f s",
			 (double)CLEARANCE_START_THRUST,
			 (double)CLEARANCE_THRUST_LIMIT,
			 (double)(CLEARANCE_RAMP_DURATION_US * 1e-6f),
			 (double)(CLEARANCE_HOLD_DURATION_US * 1e-6f));

		PX4_INFO("CLEARANCE forward %.2f m/s^2, no fixed climb accel",
			 (double)CLEARANCE_FORWARD_ACCEL);

		PX4_INFO("GUIDANCE slew %.1f deg/s lead %.1f deg",
			 (double)GUIDANCE_ATTITUDE_SLEW_DEG_S,
			 (double)GUIDANCE_ATTITUDE_LEAD_MAX_DEG);

		if (_release_alt_ref_valid) {
			PX4_INFO("ALT floor reference z=%.2f", (double)_release_alt_ref_z_ned);
		}
	}

	void enter_transition(const uint64_t now, const Vector3f &start_accel_ned)
	{
		_transition_start_accel_ned = start_accel_ned;

		_state = GuidanceState::TRANSITION;
		_state_enter_time = now;
		_transition_phase = 0.f;

		PX4_INFO("LAUNCH_CLEARANCE -> TRANSITION");
		PX4_INFO("TRANSITION duration %.2f s", (double)(TRANSITION_DURATION_US * 1e-6f));
	}

	void run_launch_clearance(const uint64_t now)
	{
		Vector3f clearance_target_ned;

		if (!calculate_clearance_target(clearance_target_ned)) {
			enter_safe_hover(now, "invalid clearance target");
			return;
		}

		const uint64_t elapsed = now - _state_enter_time;

		Vector3f accel_sp_ned;

		if (elapsed < CLEARANCE_RAMP_DURATION_US) {
			const float raw_phase = math::constrain(
							(float)elapsed
							/ (float)CLEARANCE_RAMP_DURATION_US,
							0.f,
							1.f
						);

			const float phase = smoothstep01(raw_phase);

			_transition_phase = phase;

			accel_sp_ned =
				_clearance_start_accel_ned * (1.f - phase)
				+ clearance_target_ned * phase;

		} else {
			_transition_phase = 1.f;
			accel_sp_ned = clearance_target_ned;
		}

		// 只有发生/预测发生掉高时才向上干预。
		apply_release_altitude_floor_guard(accel_sp_ned);

		// CLEARANCE使用独立0.95推力预算，而不是BPN的0.75。
		constrain_net_accel_vertical_priority(accel_sp_ned, CLEARANCE_THRUST_LIMIT);

		_last_bpn_body.zero();
		_last_axial_used = 0.f;
		_last_bpn_norm_raw = 0.f;
		_last_bpn_norm_used = 0.f;
		_last_bpn_scale = 1.f;
		_last_speed_brake = 0.f;

		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);

		publish_accel_attitude_setpoint(accel_sp_ned, now);
		publish_guidance_acceleration(_last_bpn_body, false, now);

		const uint64_t total_clearance_time =
			CLEARANCE_RAMP_DURATION_US
			+ CLEARANCE_HOLD_DURATION_US;

		if (elapsed >= total_clearance_time) {
			enter_transition(now, accel_sp_ned);
		}
	}

	void run_transition(const uint64_t now)
	{
		Vector3f target_accel_ned;
		Vector3f bpn_accel_body;

		if (!calculate_bpn_target(target_accel_ned, bpn_accel_body)) {
			enter_safe_hover(now, "invalid BPN target in transition");
			return;
		}

		const float raw_phase = math::constrain(
						(float)(now - _state_enter_time)
						/ (float)TRANSITION_DURATION_US,
						0.f,
						1.f
					);

		const float phase = smoothstep01(raw_phase);

		_transition_phase = phase;

		Vector3f accel_sp_ned =
			_transition_start_accel_ned * (1.f - phase)
			+ target_accel_ned * phase;

		apply_release_altitude_floor_guard(accel_sp_ned);

		// 正常TRANSITION使用0.75；如果高度保护正在工作，则临时开放到CLEARANCE的0.95。
		// 注意：这是“最大允许推力”，不是强制输出0.95。
		const float active_thrust_limit =
		    _alt_floor_active
		    ? CLEARANCE_THRUST_LIMIT
		    : GUIDANCE_THRUST_LIMIT;

		constrain_net_accel_vertical_priority(
		    accel_sp_ned,
		    active_thrust_limit
		);

		// TRANSITION回到正常BPN推力预算0.75。
		// constrain_net_accel_vertical_priority(accel_sp_ned, GUIDANCE_THRUST_LIMIT);

		_last_bpn_body = bpn_accel_body;
		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);

		publish_accel_attitude_setpoint(accel_sp_ned, now);
		publish_guidance_acceleration(bpn_accel_body, true, now);

		if (raw_phase >= 1.f) {
			_state = GuidanceState::BPN;
			_state_enter_time = now;

			_launch_sequence_complete = true;
			_transition_phase = 1.f;

			PX4_INFO("TRANSITION -> BPN");
		}
	}

	// ============================================================
	// 正常BPN
	// ============================================================

	void run_bpn(const uint64_t now)
	{
		Vector3f accel_sp_ned;
		Vector3f bpn_accel_body;

		if (!calculate_bpn_target(accel_sp_ned, bpn_accel_body)) {
			enter_safe_hover(now, "invalid BPN target");
			return;
		}

		apply_release_altitude_floor_guard(accel_sp_ned);

		// 正常BPN最大推力0.75。如果仍然发生危险下降，高度保护可以临时使用0.95。
		const float active_thrust_limit =
		    _alt_floor_active
		    ? CLEARANCE_THRUST_LIMIT
		    : GUIDANCE_THRUST_LIMIT;

		constrain_net_accel_vertical_priority(
		    accel_sp_ned,
		    active_thrust_limit
		);

		// 正常BPN推力预算0.75，并优先保留Z方向能力。
		// constrain_net_accel_vertical_priority(accel_sp_ned, GUIDANCE_THRUST_LIMIT);

		_last_bpn_body = bpn_accel_body;
		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);

		publish_accel_attitude_setpoint(accel_sp_ned, now);
		publish_guidance_acceleration(bpn_accel_body, true, now);
	}

	// ============================================================
	// 越靶检测
	// ============================================================

	void monitor_target_pass(const uint64_t now)
	{
		if (_state != GuidanceState::TRANSITION
		    && !(_state == GuidanceState::BPN && _launch_sequence_complete)) {

			_negative_closing_start_time = 0;
			return;
		}

		if (!_vc_valid || !_had_positive_closing) {
			_negative_closing_start_time = 0;
			return;
		}

		const float speed = get_ekf_speed();

		if (!PX4_ISFINITE(speed) || speed < MISS_MIN_SPEED) {
			_negative_closing_start_time = 0;
			return;
		}

		if (_vc_raw < MISS_TRIGGER_VC) {
			if (_negative_closing_start_time == 0) {
				_negative_closing_start_time = now;
			}

			if (now - _negative_closing_start_time >= MISS_TRIGGER_HOLD_US) {
				enter_safe_hover(now, "target passed: Vc became negative");
			}

		} else {
			_negative_closing_start_time = 0;
		}
	}

	// ============================================================
	// SAFE_HOVER
	// ============================================================

	void enter_safe_hover(const uint64_t now, const char *reason)
	{
		if (_state == GuidanceState::SAFE_HOVER) { return; }

		PX4_WARN("-> SAFE_HOVER: %s", reason);

		_state = GuidanceState::SAFE_HOVER;
		_state_enter_time = now;
		_transition_phase = 1.f;

		_safe_hold_latched = false;
		_safe_low_speed_start_time = 0;

		_last_bpn_body.zero();
		_last_axial_used = 0.f;
		_last_bpn_norm_raw = 0.f;
		_last_bpn_norm_used = 0.f;
		_last_bpn_scale = 1.f;

		_last_speed_brake = 0.f;
		_alt_floor_active = false;

		if (attitude_valid()) {
			_last_q_sp = current_attitude_q();
			_last_raw_q_sp = _last_q_sp;
		}

		PX4_WARN("SAFE_HOVER braking with EKF velocity");
	}

	Vector3f calculate_safe_brake_accel() const
	{
		Vector3f accel_sp{0.f, 0.f, 0.f};

		if (!velocity_valid()) { return accel_sp; }

		accel_sp(0) = -SAFE_BRAKE_KV_XY * _local_position.vx;
		accel_sp(1) = -SAFE_BRAKE_KV_XY * _local_position.vy;
		accel_sp(2) = -SAFE_BRAKE_KV_Z * _local_position.vz;

		constrain_xy(accel_sp, SAFE_BRAKE_ACCEL_XY_MAX);

		accel_sp(2) = math::constrain(
				      accel_sp(2),
				      -SAFE_BRAKE_ACCEL_Z_MAX,
				      SAFE_BRAKE_ACCEL_Z_MAX
			      );

		return accel_sp;
	}

	Vector3f calculate_safe_hold_accel() const
	{
		Vector3f accel_sp{0.f, 0.f, 0.f};

		if (!position_valid() || !velocity_valid()) {
			return calculate_safe_brake_accel();
		}

		const float ex = _local_position.x - _safe_hold_position_ned(0);
		const float ey = _local_position.y - _safe_hold_position_ned(1);
		const float ez = _local_position.z - _safe_hold_position_ned(2);

		accel_sp(0) =
			-SAFE_HOLD_KP_XY * ex
			-SAFE_HOLD_KD_XY * _local_position.vx;

		accel_sp(1) =
			-SAFE_HOLD_KP_XY * ey
			-SAFE_HOLD_KD_XY * _local_position.vy;

		accel_sp(2) =
			-SAFE_HOLD_KP_Z * ez
			-SAFE_HOLD_KD_Z * _local_position.vz;

		constrain_xy(accel_sp, SAFE_HOLD_ACCEL_XY_MAX);

		accel_sp(2) = math::constrain(
				      accel_sp(2),
				      -SAFE_HOLD_ACCEL_Z_MAX,
				      SAFE_HOLD_ACCEL_Z_MAX
			      );

		return accel_sp;
	}

	void run_safe_hover(const uint64_t now)
	{
		const float speed_xy = get_ekf_speed_xy();

		const float vz =
			velocity_valid()
			? fabsf(_local_position.vz)
			: NAN;

		if (!_safe_hold_latched) {
			if (PX4_ISFINITE(speed_xy)
			    && PX4_ISFINITE(vz)
			    && speed_xy < SAFE_LATCH_SPEED_XY
			    && vz < SAFE_LATCH_SPEED_Z
			    && position_valid()) {

				if (_safe_low_speed_start_time == 0) {
					_safe_low_speed_start_time = now;
				}

				if (now - _safe_low_speed_start_time >= SAFE_LATCH_HOLD_US) {
					_safe_hold_position_ned = Vector3f{
						_local_position.x,
						_local_position.y,
						_local_position.z
					};

					_safe_hold_latched = true;

					PX4_WARN(
						"SAFE_HOVER hold latched x=%.1f y=%.1f z=%.1f",
						(double)_safe_hold_position_ned(0),
						(double)_safe_hold_position_ned(1),
						(double)_safe_hold_position_ned(2)
					);
				}

			} else {
				_safe_low_speed_start_time = 0;
			}
		}

		const Vector3f accel_sp_ned =
			_safe_hold_latched
			? calculate_safe_hold_accel()
			: calculate_safe_brake_accel();

		_last_bpn_body.zero();

		_last_axial_used = 0.f;
		_last_bpn_norm_raw = 0.f;
		_last_bpn_norm_used = 0.f;
		_last_bpn_scale = 1.f;

		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);

		publish_accel_attitude_setpoint(accel_sp_ned, now);
		publish_guidance_acceleration(_last_bpn_body, false, now);
	}

	// ============================================================
	// EKF local position reset监控
	// ============================================================

	void monitor_ekf_resets()
	{
		if (!_ekf_reset_counter_initialized) {
			_last_xy_reset_counter = _local_position.xy_reset_counter;
			_last_z_reset_counter = _local_position.z_reset_counter;
			_last_vxy_reset_counter = _local_position.vxy_reset_counter;
			_last_vz_reset_counter = _local_position.vz_reset_counter;

			_ekf_reset_counter_initialized = true;
			return;
		}

		if (_local_position.xy_reset_counter != _last_xy_reset_counter) {
			PX4_WARN(
				"EKF XY RESET %u -> %u",
				(unsigned)_last_xy_reset_counter,
				(unsigned)_local_position.xy_reset_counter
			);

			if (_safe_hold_latched) {
				_safe_hold_position_ned(0) += _local_position.delta_xy[0];
				_safe_hold_position_ned(1) += _local_position.delta_xy[1];
			}

			_last_xy_reset_counter = _local_position.xy_reset_counter;
		}

		if (_local_position.z_reset_counter != _last_z_reset_counter) {
			PX4_WARN(
				"EKF Z RESET %u -> %u",
				(unsigned)_last_z_reset_counter,
				(unsigned)_local_position.z_reset_counter
			);

			if (_safe_hold_latched) {
				_safe_hold_position_ned(2) += _local_position.delta_z;
			}

			if (_release_alt_ref_valid && PX4_ISFINITE(_local_position.delta_z)) {
				_release_alt_ref_z_ned += _local_position.delta_z;
			}

			_last_z_reset_counter = _local_position.z_reset_counter;
		}

		if (_local_position.vxy_reset_counter != _last_vxy_reset_counter) {
			PX4_WARN(
				"EKF VXY RESET %u -> %u",
				(unsigned)_last_vxy_reset_counter,
				(unsigned)_local_position.vxy_reset_counter
			);

			_last_vxy_reset_counter = _local_position.vxy_reset_counter;
		}

		if (_local_position.vz_reset_counter != _last_vz_reset_counter) {
			PX4_WARN(
				"EKF VZ RESET %u -> %u",
				(unsigned)_last_vz_reset_counter,
				(unsigned)_local_position.vz_reset_counter
			);

			_last_vz_reset_counter = _local_position.vz_reset_counter;
		}
	}

	// ============================================================
	// EKF attitude quaternion reset
	// ============================================================

	void monitor_attitude_reset(const uint64_t now)
	{
		if (!_attitude_reset_counter_initialized) {
			_last_quat_reset_counter = _attitude.quat_reset_counter;
			_attitude_reset_counter_initialized = true;
			return;
		}

		if (_attitude.quat_reset_counter == _last_quat_reset_counter) {
			return;
		}

		const uint8_t old_counter = _last_quat_reset_counter;
		const uint8_t new_counter = _attitude.quat_reset_counter;

		_last_quat_reset_counter = new_counter;

		Quatf delta_q{_attitude.delta_q_reset};

		const float delta_norm = delta_q.norm();

		if (!PX4_ISFINITE(delta_norm) || delta_norm < FLT_EPSILON) {
			PX4_WARN(
				"EKF ATT RESET %u -> %u invalid delta_q",
				(unsigned)old_counter,
				(unsigned)new_counter
			);

			return;
		}

		delta_q.normalize();

		const Eulerf delta_euler{delta_q};
		const float delta_yaw = delta_euler.psi();

		PX4_WARN(
			"EKF ATT RESET %u -> %u dYaw=%.1f deg",
			(unsigned)old_counter,
			(unsigned)new_counter,
			(double)(delta_yaw * 180.f / M_PI_F)
		);

		if (_yaw_locked) {
			const float old_yaw = _yaw_lock;

			_yaw_lock = wrap_pi_local(_yaw_lock + delta_yaw);

			PX4_WARN(
				"YAW LOCK RESET %.1f -> %.1f deg",
				(double)(old_yaw * 180.f / M_PI_F),
				(double)(_yaw_lock * 180.f / M_PI_F)
			);
		}

		_hold_q = delta_q * _hold_q;
		_hold_q.normalize();

		_last_q_sp = delta_q * _last_q_sp;
		_last_q_sp.normalize();

		_last_raw_q_sp = delta_q * _last_raw_q_sp;
		_last_raw_q_sp.normalize();

		if (_state == GuidanceState::LAUNCH_CLEARANCE) {
			_clearance_start_accel_ned =
				attitude_thrust_to_net_accel(
					_last_q_sp,
					_last_thrust_sp
				);

			_state_enter_time = now;
			_transition_phase = 0.f;

			PX4_WARN("restart LAUNCH_CLEARANCE after ATT reset");
		}

		if (_state == GuidanceState::TRANSITION) {
			_transition_start_accel_ned =
				attitude_thrust_to_net_accel(
					_last_q_sp,
					_last_thrust_sp
				);

			_state_enter_time = now;
			_transition_phase = 0.f;

			PX4_WARN("restart TRANSITION after ATT reset");
		}
	}

	// ============================================================
	// LOS角速度实时打印
	// ============================================================

	void print_los_rate_realtime(const uint64_t now)
	{
		if (!LOS_RATE_REALTIME_PRINT) { return; }

		if (now - _last_los_rate_print_time < LOS_RATE_PRINT_INTERVAL_US) {
			return;
		}

		_last_los_rate_print_time = now;

		if (!_los.target_valid
		    || !PX4_ISFINITE(_los.los_azimuth)
		    || !PX4_ISFINITE(_los.los_elevation)
		    || !PX4_ISFINITE(_los.los_az_rate)
		    || !PX4_ISFINITE(_los.los_el_rate)) {
			return;
		}

		const Vector3f los_dot_body = calculate_los_dot_body(
						      _los.los_azimuth,
						      _los.los_elevation,
						      _los.los_az_rate,
						      _los.los_el_rate
						      );

		const float mag = los_dot_body.norm();

		PX4_INFO(
			"LOS_RT AZ_RATE=%+.4f EL_RATE=%+.4f MAG=%.4f rad/s",
			(double)_los.los_az_rate,
			(double)_los.los_el_rate,
			(double)mag
		);
	}

	// ============================================================
	// Debug
	// ============================================================

	float attitude_error_angle_deg() const
	{
		return quat_angle_rad(_last_q_sp, current_attitude_q())
		       * 180.f
		       / M_PI_F;
	}

	void print_debug(const uint64_t now)
	{
		if (now - _last_print_time < 500000) { return; }

		_last_print_time = now;

		const float ekf_speed = get_ekf_speed();

		const Eulerf att_euler{current_attitude_q()};
		const Eulerf raw_sp_euler{_last_raw_q_sp};
		const Eulerf sp_euler{_last_q_sp};

		PX4_INFO(
			"STATE=%u EKF_SPEED=%.2f",
			(unsigned)_state,
			(double)ekf_speed
		);

		PX4_INFO(
			"VC raw=%.2f filt=%.2f used=%.2f valid=%d",
			(double)_vc_raw,
			(double)_vc_filtered,
			(double)_vc_used,
			(int)_vc_valid
		);

		PX4_INFO(
			"GIMBAL AZ=%.2f PIT=%.2f",
			(double)(_los.los_azimuth * 180.f / M_PI_F),
			(double)(_los.los_elevation * 180.f / M_PI_F)
		);

		PX4_INFO(
			"LOS RATE AZ=%.4f EL=%.4f mag=%.4f",
			(double)_los.los_az_rate,
			(double)_los.los_el_rate,
			(double)_last_los_rate_mag
		);

		PX4_INFO(
			"SPEED target=%.1f scale=%.2f brake=%.2f hard=%.1f",
			(double)_last_speed_target,
			(double)_last_speed_axial_scale,
			(double)_last_speed_brake,
			(double)GUIDANCE_SPEED_HARD_LIMIT
		);

		PX4_INFO(
			"BPN body %.2f %.2f %.2f",
			(double)_last_bpn_body(0),
			(double)_last_bpn_body(1),
			(double)_last_bpn_body(2)
		);

		PX4_INFO(
			"ACC axial=%.2f/%.2f cmd_max=%.2f bpn=%.2f->%.2f scale=%.2f",
			(double)_last_axial_used,
			(double)AXIAL_ACCEL_MAX,
			(double)_last_axial_limit_cmd,
			(double)_last_bpn_norm_raw,
			(double)_last_bpn_norm_used,
			(double)_last_bpn_scale
		);

		const float active_budget =
			(_state == GuidanceState::LAUNCH_CLEARANCE)
			? CLEARANCE_THRUST_LIMIT
			: GUIDANCE_THRUST_LIMIT;

		PX4_INFO(
			"THR pred=%.3f budget=%.3f actual=%.3f",
			(double)_last_predicted_thrust,
			(double)active_budget,
			(double)_last_thrust_sp
		);

		PX4_INFO(
			"ALT_FLOOR active=%d drop=%.2f pred=%.2f up=%.2f",
			(int)_alt_floor_active,
			(double)_last_alt_floor_drop,
			(double)_last_alt_floor_predicted_drop,
			(double)_last_alt_floor_up_accel
		);

		PX4_INFO(
			"ATT RPY %.1f %.1f %.1f",
			(double)(att_euler.phi() * 180.f / M_PI_F),
			(double)(att_euler.theta() * 180.f / M_PI_F),
			(double)(att_euler.psi() * 180.f / M_PI_F)
		);

		PX4_INFO(
			"RAW_QSP RPY %.1f %.1f %.1f",
			(double)(raw_sp_euler.phi() * 180.f / M_PI_F),
			(double)(raw_sp_euler.theta() * 180.f / M_PI_F),
			(double)(raw_sp_euler.psi() * 180.f / M_PI_F)
		);

		PX4_INFO(
			"QSP RPY %.1f %.1f %.1f",
			(double)(sp_euler.phi() * 180.f / M_PI_F),
			(double)(sp_euler.theta() * 180.f / M_PI_F),
			(double)(sp_euler.psi() * 180.f / M_PI_F)
		);

		PX4_INFO(
			"SLEW active=%d raw_step=%.1f deg max=%.1f deg/s",
			(int)_slew_active,
			(double)_last_raw_q_step_deg,
			(double)_last_attitude_slew_deg_s
		);

		PX4_INFO(
			"LEAD active=%d angle=%.1f deg max=%.1f deg",
			(int)_lead_active,
			(double)_last_attitude_lead_deg,
			(double)GUIDANCE_ATTITUDE_LEAD_MAX_DEG
		);

		PX4_INFO(
			"ATT_ERR=%.1f deg phase=%.2f yaw_lock=%.1f",
			(double)attitude_error_angle_deg(),
			(double)_transition_phase,
			(double)(_yaw_lock * 180.f / M_PI_F)
		);

		if (_state == GuidanceState::SAFE_HOVER) {
			PX4_INFO(
				"SAFE_HOVER hold=%d speed_xy=%.2f vz=%.2f",
				(int)_safe_hold_latched,
				(double)get_ekf_speed_xy(),
				(double)(velocity_valid() ? _local_position.vz : NAN)
			);
		}

		PX4_INFO(
			"EKF reset quat=%u xy=%u z=%u vxy=%u vz=%u dead_reckoning=%d",
			(unsigned)_attitude.quat_reset_counter,
			(unsigned)_local_position.xy_reset_counter,
			(unsigned)_local_position.z_reset_counter,
			(unsigned)_local_position.vxy_reset_counter,
			(unsigned)_local_position.vz_reset_counter,
			(int)_local_position.dead_reckoning
		);
	}

	// ============================================================
	// Run
	// ============================================================

	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		const uint64_t now = hrt_absolute_time();

		// --------------------------------------------------------
		// LOS
		// --------------------------------------------------------

		if (_los_sub.update(&_los)) {
			_last_los_rx_time = now;
		}

		// --------------------------------------------------------
		// attitude
		// --------------------------------------------------------

		if (_attitude_sub.update(&_attitude)) {
			monitor_attitude_reset(now);
		}

		// --------------------------------------------------------
		// local position
		// --------------------------------------------------------

		if (_local_position_sub.update(&_local_position)) {
			monitor_ekf_resets();
		}

		_vehicle_status_sub.update(&_vehicle_status);

		// 每周期根据LOS + EKF更新闭合速度。
		update_closing_speed(now);

		// 实时观察视觉输入的LOS角速度。
		print_los_rate_realtime(now);

		// --------------------------------------------------------
		// 第一次有效LOS
		// WAIT_LOS -> LAUNCH
		// --------------------------------------------------------

		if (_state == GuidanceState::WAIT_LOS
		    && los_fresh(now)
		    && _los.target_valid
		    && attitude_valid()) {

			if (!_yaw_locked) {
				const Eulerf euler{current_attitude_q()};

				_yaw_lock = euler.psi();
				_yaw_locked = true;

				PX4_INFO("WAIT_LOS -> LAUNCH");
				PX4_INFO("LOCK YAW %.2f deg", (double)(_yaw_lock * 180.f / M_PI_F));
			}

			_hold_q = current_attitude_q();

			_state = GuidanceState::LAUNCH;
			_state_enter_time = now;
			_transition_phase = 1.f;
		}

		// --------------------------------------------------------
		// ARM边沿
		// --------------------------------------------------------

		const bool is_armed = armed();

		if (is_armed && !_was_armed) {
			if (_state == GuidanceState::BPN && !_launch_sequence_complete) {
				enter_wait_release(now);
			}
		}

		if (!is_armed && _was_armed) {
			_launch_sequence_complete = false;
			_transition_phase = 1.f;

			_had_positive_closing = false;
			_negative_closing_start_time = 0;

			_safe_hold_latched = false;
			_safe_low_speed_start_time = 0;

			_release_alt_ref_valid = false;
			_alt_floor_active = false;

			if (los_fresh(now)
			    && _los.target_valid
			    && attitude_valid()) {

				_hold_q = current_attitude_q();
				_state = GuidanceState::BPN;

			} else {
				_state = GuidanceState::WAIT_LOS;
			}
		}

		_was_armed = is_armed;

		// --------------------------------------------------------
		// RELEASE SYNC
		// --------------------------------------------------------

		bool release_sync_received = false;

		if (_vehicle_command_sub.update(&_vehicle_command)) {
			if (_vehicle_command.command == RELEASE_SYNC_COMMAND) {
				release_sync_received = true;
			}
		}

		if (release_sync_received
		    && _state == GuidanceState::WAIT_RELEASE
		    && is_armed) {

			enter_launch_clearance(now);
		}

		// --------------------------------------------------------
		// 越靶检测
		// --------------------------------------------------------

		monitor_target_pass(now);

		// --------------------------------------------------------
		// LOS失效保护
		// --------------------------------------------------------

		if ((_state == GuidanceState::LAUNCH_CLEARANCE
		     || _state == GuidanceState::TRANSITION
		     || (_state == GuidanceState::BPN && _launch_sequence_complete))
		    && (!los_fresh(now) || !_los.target_valid)) {

			enter_safe_hover(now, "LOS invalid/stale");
		}

		// --------------------------------------------------------
		// 状态机
		// --------------------------------------------------------

		switch (_state) {

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
			if (attitude_valid()) {
				_hold_q = current_attitude_q();

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

			if (now - _state_enter_time >= LAUNCH_DURATION_US) {
				_state = GuidanceState::BPN;
				_state_enter_time = now;

				_launch_sequence_complete = false;

				PX4_INFO("LAUNCH -> BPN READY");
			}

			break;
		}

		case GuidanceState::BPN:
		{
			if (_launch_sequence_complete) {
				run_bpn(now);

			} else {
				if (attitude_valid()) {
					_hold_q = current_attitude_q();

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
			run_launch_clearance(now);
			break;
		}

		case GuidanceState::TRANSITION:
		{
			run_transition(now);
			break;
		}

		case GuidanceState::SAFE_HOVER:
		default:
		{
			run_safe_hover(now);
			break;
		}
		}

		print_debug(now);
	}
};

extern "C"
__EXPORT
int pn_guidance_main(int argc, char *argv[])
{
	return PnGuidance::main(argc, argv);
}

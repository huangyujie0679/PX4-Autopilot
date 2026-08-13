#include "acceleration_to_attitude.hpp"

#include <cmath>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

using namespace matrix;


Quatf AccelerationToAttitude::calculate(
	const Vector3f &accel_body,
	const Quatf &q_current,
	float yaw_sp)
{
	/*
	 * ============================================================
	 * 1. 当前姿态：Body -> NED
	 * ============================================================
	 */

	Dcmf R_nb(q_current);


	/*
	 * ============================================================
	 * 2. 将机体系期望加速度转换到 NED
	 *
	 * Body FRD:
	 * X Forward
	 * Y Right
	 * Z Down
	 *
	 * NED:
	 * X North
	 * Y East
	 * Z Down
	 * ============================================================
	 */

	Vector3f accel_ned = R_nb * accel_body;


	/*
	 * ============================================================
	 * 3. 加入重力
	 *
	 * NED中重力方向：
	 *
	 * [0, 0, +g]
	 *
	 * 要产生期望加速度，需要的合力：
	 *
	 * F = a - g
	 *
	 * ============================================================
	 */

	const float g = 9.81f;

	Vector3f force_ned;

	force_ned(0) = accel_ned(0);
	force_ned(1) = accel_ned(1);
	force_ned(2) = accel_ned(2) - g;


	/*
	 * ============================================================
	 * 4. 判断合力是否有效
	 * ============================================================
	 */

	if (force_ned.norm() < 0.01f)
	{
		return Quatf(
			Eulerf(
				0.0f,
				0.0f,
				yaw_sp
			)
		);
	}


	/*
	 * ============================================================
	 * 5. 计算期望机体Z轴
	 *
	 * PX4机体系：
	 *
	 * Zb = Down
	 *
	 * 推力方向与合力方向相反。
	 * ============================================================
	 */

	Vector3f zb = -force_ned.normalized();


	/*
	 * ============================================================
	 * 6. 根据固定yaw构造水平X轴参考方向
	 * ============================================================
	 */

	Vector3f xc(
		cosf(yaw_sp),
		sinf(yaw_sp),
		0.0f
	);


	/*
	 * ============================================================
	 * 7. 计算机体Y轴
	 * ============================================================
	 */

	Vector3f yb = zb % xc;


	if (yb.norm() < 0.01f)
	{
		/*
		 * 当zb与xc接近平行时，使用yaw方向的水平Y轴
		 */

		yb = Vector3f(
			-sinf(yaw_sp),
			cosf(yaw_sp),
			0.0f
		);
	}
	else
	{
		yb.normalize();
	}


	/*
	 * ============================================================
	 * 8. 重新计算机体X轴
	 * ============================================================
	 */

	Vector3f xb = yb % zb;

	xb.normalize();


	/*
	 * ============================================================
	 * 9. 构造期望姿态
	 *
	 * R_des：
	 *
	 * Body -> NED
	 *
	 * 每一列分别是机体X/Y/Z轴在NED中的表示。
	 * ============================================================
	 */

	Dcmf R_des;

	R_des(0, 0) = xb(0);
	R_des(1, 0) = xb(1);
	R_des(2, 0) = xb(2);

	R_des(0, 1) = yb(0);
	R_des(1, 1) = yb(1);
	R_des(2, 1) = yb(2);

	R_des(0, 2) = zb(0);
	R_des(1, 2) = zb(1);
	R_des(2, 2) = zb(2);


	/*
	 * ============================================================
	 * 10. DCM -> Quaternion
	 * ============================================================
	 */

	Quatf q_out(R_des);


	/*
	 * ============================================================
	 * 11. 调试输出
	 *
	 * 限制为1秒打印一次。
	 *
	 * 注意这里使用hrt_absolute_time()，
	 * 不要直接写一个不存在的now变量。
	 * ============================================================
	 */

	static hrt_abstime last_print_time = 0;
	const hrt_abstime now = hrt_absolute_time();
	if (now - last_print_time >= 5000000)
	{
		last_print_time = now;

		Eulerf euler(q_out);

		const double roll_deg =
			static_cast<double>(euler.phi()) * 180.0 / M_PI;

		const double pitch_deg =
			static_cast<double>(euler.theta()) * 180.0 / M_PI;

		const double yaw_deg =
			static_cast<double>(euler.psi()) * 180.0 / M_PI;

		PX4_INFO(
			"ATT: %.1f %.1f %.1f",
			roll_deg,
			pitch_deg,
			yaw_deg
		);
	}


	/*
	 * ============================================================
	 * 12. 返回期望姿态
	 * ============================================================
	 */

	return q_out;
}

#include "acceleration_to_attitude.hpp"

#include <cmath>

using namespace matrix;


namespace
{

constexpr float GRAVITY =
	9.80665f;


float constrainFloat(
	float value,
	float minimum,
	float maximum)
{
	if (value < minimum) {
		return minimum;
	}

	if (value > maximum) {
		return maximum;
	}

	return value;
}


float norm3(
	const Vector3f &v)
{
	return sqrtf(
		v(0) * v(0)
		+
		v(1) * v(1)
		+
		v(2) * v(2)
	);
}

}


AccelerationToAttitude::Result
AccelerationToAttitude::calculate(
	const Vector3f &accel_sp_ned,
	float yaw_sp,
	float hover_thrust)
{
	Result result{};


	/*
	 * ============================================================
	 * 车辆动力学：
	 *
	 * a_des =
	 *
	 * gravity
	 * +
	 * thrust_specific_force
	 *
	 *
	 * NED中：
	 *
	 * gravity =
	 *
	 * [0, 0, +g]
	 *
	 *
	 * 因此：
	 *
	 * thrust_accel =
	 *
	 * a_des - gravity
	 *
	 * 多旋翼实际推力方向为：
	 *
	 * Body -Z
	 * ============================================================
	 */

	const Vector3f gravity_ned{
		0.0f,
		0.0f,
		GRAVITY
	};


	Vector3f thrust_accel_ned =
		accel_sp_ned
		-
		gravity_ned;


	float thrust_accel_norm =
		norm3(
			thrust_accel_ned
		);


	if (thrust_accel_norm < 0.01f) {

		thrust_accel_ned =
			Vector3f{
				0.0f,
				0.0f,
				-GRAVITY
			};

		thrust_accel_norm =
			GRAVITY;
	}


	/*
	 * ============================================================
	 * Body +Z 与真实推力方向相反。
	 *
	 * 所以：
	 *
	 * body_z_des =
	 *
	 * -thrust_accel / |thrust_accel|
	 * ============================================================
	 */

	Vector3f body_z_des =
		-thrust_accel_ned
		/
		thrust_accel_norm;


	/*
	 * ============================================================
	 * 固定yaw。
	 *
	 * 构造世界水平面上的yaw参考Y轴。
	 * ============================================================
	 */

	const Vector3f y_c{

		-sinf(
			yaw_sp),

		cosf(
			yaw_sp),

		0.0f
	};


	/*
	 * ============================================================
	 * body_x =
	 *
	 * y_c × body_z
	 * ============================================================
	 */

	Vector3f body_x_des =
		y_c.cross(
			body_z_des
		);


	float body_x_norm =
		norm3(
			body_x_des
		);


	/*
	 * 接近奇异状态保护。
	 */
	if (body_x_norm < 0.001f) {

		body_x_des =
			Vector3f{

				cosf(
					yaw_sp),

				sinf(
					yaw_sp),

				0.0f
			};

		body_x_norm =
			1.0f;
	}


	body_x_des /=
		body_x_norm;


	/*
	 * ============================================================
	 * body_y =
	 *
	 * body_z × body_x
	 * ============================================================
	 */

	Vector3f body_y_des =
		body_z_des.cross(
			body_x_des
		);


	const float body_y_norm =
		norm3(
			body_y_des
		);


	if (body_y_norm > 0.001f) {

		body_y_des /=
			body_y_norm;
	}


	/*
	 * ============================================================
	 * Body FRD -> NED
	 *
	 * DCM三列分别是：
	 *
	 * body X轴在NED中的方向
	 * body Y轴在NED中的方向
	 * body Z轴在NED中的方向
	 * ============================================================
	 */

	Dcmf R_sp;

	R_sp.setCol(
		0,
		body_x_des
	);

	R_sp.setCol(
		1,
		body_y_des
	);

	R_sp.setCol(
		2,
		body_z_des
	);


	result.q_d =
		Quatf(
			R_sp
		);

	result.q_d.normalize();


	/*
	 * ============================================================
	 * thrust归一化。
	 *
	 * hover_thrust对应：
	 *
	 * |a_thrust| = g
	 *
	 * 所以：
	 *
	 * thrust =
	 *
	 * hover_thrust
	 * *
	 * |a_thrust| / g
	 * ============================================================
	 */

	float thrust_norm =

		hover_thrust

		*

		thrust_accel_norm

		/

		GRAVITY;


	thrust_norm =
		constrainFloat(

			thrust_norm,

			0.10f,

			0.95f
		);


	result.thrust_norm =
		thrust_norm;


	/*
	 * PX4 multicopter：
	 *
	 * thrust_body Z为负值。
	 */

	result.thrust_body =
		Vector3f{

			0.0f,

			0.0f,

			-thrust_norm
		};


	return result;
}

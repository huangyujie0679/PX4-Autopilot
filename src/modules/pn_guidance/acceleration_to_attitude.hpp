#pragma once

#include <matrix/matrix/math.hpp>


class AccelerationToAttitude
{
public:

	struct Result
	{
		matrix::Quatf q_d;

		matrix::Vector3f thrust_body;

		float thrust_norm;
	};


	/*
	 * 输入：
	 *
	 * accel_sp_ned
	 *     期望惯性加速度
	 *     NED坐标系
	 *     m/s^2
	 *
	 * yaw_sp
	 *     固定期望yaw
	 *
	 * hover_thrust
	 *     PX4 hover thrust
	 *
	 * 输出：
	 *
	 * q_d
	 * thrust_body
	 */
	static Result calculate(
		const matrix::Vector3f &accel_sp_ned,
		float yaw_sp,
		float hover_thrust
	);
};

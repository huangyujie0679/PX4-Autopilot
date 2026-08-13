#pragma once

#include <matrix/matrix/math.hpp>

class AccelerationToAttitude
{
public:

	/**
	 * @brief
	 * 根据机体系期望加速度、当前姿态和固定偏航角
	 * 计算期望姿态。
	 *
	 * @param accel_body
	 *        机体系期望加速度，FRD，单位 m/s^2
	 *
	 * @param q_current
	 *        当前姿态，Body -> NED
	 *
	 * @param yaw_sp
	 *        期望偏航角，rad
	 */

	static matrix::Quatf calculate(
		const matrix::Vector3f &accel_body,
		const matrix::Quatf &q_current,
		float yaw_sp
	);
};

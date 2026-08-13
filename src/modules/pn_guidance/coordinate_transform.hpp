#pragma once

#include <matrix/matrix/math.hpp>


class CoordinateTransform
{
public:

	/*
	 * 云台角度 + 相机LOS角速度
	 *
	 * 转换到飞机FRD坐标系
	 */
	static matrix::Vector3f cameraLosToBody(
		float los_az_rate,
		float los_el_rate,
		float gimbal_roll,
		float gimbal_pitch
	);

};

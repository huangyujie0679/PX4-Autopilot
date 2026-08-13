#include "coordinate_transform.hpp"

using namespace matrix;


/*
 * 相机LOS角速度转换到机体FRD
 *
 * 云台:
 *
 * X轴: pitch
 * Y轴: roll
 *
 * 正方向:
 *
 * pitch向上为+
 * roll向右为+
 *
 */
Vector3f CoordinateTransform::cameraLosToBody(
	float az_rate,
	float el_rate,
	float roll,
	float pitch
)
{

	/*
	 * 相机坐标:
	 *
	 * Xcam:
	 *   右
	 *
	 * Ycam:
	 *   上
	 *
	 * Zcam:
	 *   前
	 *
	 */


	Vector3f los_cam;

	los_cam(0)=az_rate;
	los_cam(1)=el_rate;
	los_cam(2)=0.0f;


	/*
	 * 云台旋转矩阵
	 *
	 * 旋转顺序:
	 *
	 * roll -> pitch
	 *
	 */


	Dcmf R_roll(
		Eulerf(
			roll,
			0,
			0
		)
	);


	Dcmf R_pitch(
		Eulerf(
			0,
			pitch,
			0
		)
	);


	/*
	 * camera到body
	 *
	 * body_R_cam
	 *
	 */

	Dcmf R =
		R_roll *
		R_pitch;


	return R * los_cam;

}

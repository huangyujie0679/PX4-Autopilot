#include <gz/sim/System.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>

#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>


namespace prelaunch_tracker
{


class PrelaunchTracker :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{

public:

	PrelaunchTracker() = default;

	~PrelaunchTracker() override = default;


	/*
	 * ============================================================
	 * Configure
	 *
	 * 插件加载时执行一次。
	 * ============================================================
	 */
	void Configure(
		const gz::sim::Entity &_entity,
		const std::shared_ptr<const sdf::Element> &_sdf,
		gz::sim::EntityComponentManager &_ecm,
		gz::sim::EventManager & /* _eventMgr */) override
	{
		/*
		 * 当前插件挂载在哪个模型下面，
		 * _entity就是哪个模型。
		 *
		 * 当前即：
		 *
		 * fpv_15_0
		 */
		_model =
			gz::sim::Model(
				_entity
			);


		if (!_model.Valid(_ecm)) {

			std::cerr
				<< "[PrelaunchTracker] invalid model entity"
				<< std::endl;

			return;
		}


		/*
		 * ========================================================
		 * 读取SDF参数
		 * ========================================================
		 */

		if (_sdf) {

			/*
			 * 目标模型名称。
			 *
			 * 当前：
			 *
			 * target_drone
			 */
			if (
				_sdf->HasElement(
					"target_model"
				)
			) {

				_target_model_name =
					_sdf->Get<std::string>(
						"target_model"
					);
			}


			/*
			 * 发射前飞机固定的世界坐标。
			 *
			 * 当前例如：
			 *
			 * -50 -50 1
			 */
			if (
				_sdf->HasElement(
					"launch_position"
				)
			) {

				_launch_position =
					_sdf->Get<
						gz::math::Vector3d>(
							"launch_position"
						);
			}


			/*
			 * 释放topic。
			 */
			if (
				_sdf->HasElement(
					"release_topic"
				)
			) {

				_release_topic =
					_sdf->Get<std::string>(
						"release_topic"
					);
			}
		}


		/*
		 * ========================================================
		 * 订阅释放命令
		 *
		 * 收到：
		 *
		 * /fpv_15/release
		 *
		 * 后停止强制位置/姿态控制。
		 * ========================================================
		 */

		const bool subscribed =
			_node.Subscribe(
				_release_topic,
				&PrelaunchTracker::ReleaseCallback,
				this
			);


		if (!subscribed) {

			std::cerr
				<< "[PrelaunchTracker] failed to subscribe: "
				<< _release_topic
				<< std::endl;
		}


		/*
		 * ========================================================
		 * 发射前：
		 *
		 * 关闭重力
		 * 关闭碰撞
		 *
		 * 注意：
		 * 不把整个模型设置成static，
		 * 否则可能影响PX4传感器仿真。
		 * ========================================================
		 */

		_model.SetGravityEnabled(
			_ecm,
			false
		);

		_model.SetCollisionEnabled(
			_ecm,
			false
		);


		/*
		 * ========================================================
		 * 获取飞机base_link
		 *
		 * 优先找：
		 *
		 * base_link
		 *
		 * 如果没有，
		 * 使用canonical link。
		 * ========================================================
		 */

		gz::sim::Entity base_link_entity =
			_model.LinkByName(
				_ecm,
				"base_link"
			);


		if (
			base_link_entity
			==
			gz::sim::kNullEntity
		) {

			base_link_entity =
				_model.CanonicalLink(
					_ecm
				);
		}


		_base_link =
			gz::sim::Link(
				base_link_entity
			);


		if (!_base_link.Valid(_ecm)) {

			std::cerr
				<< "[PrelaunchTracker] base_link not found"
				<< std::endl;

			return;
		}


		std::cout
			<< "[PrelaunchTracker] base_link ready"
			<< std::endl;


		_configured = true;


		std::cout
			<< "[PrelaunchTracker] configured"
			<< std::endl;


		std::cout
			<< "[PrelaunchTracker] target = "
			<< _target_model_name
			<< std::endl;


		std::cout
			<< "[PrelaunchTracker] launch_position = ("
			<< _launch_position.X()
			<< ", "
			<< _launch_position.Y()
			<< ", "
			<< _launch_position.Z()
			<< ")"
			<< std::endl;


		std::cout
			<< "[PrelaunchTracker] release topic = "
			<< _release_topic
			<< std::endl;
	}


	/*
	 * ============================================================
	 * PreUpdate
	 *
	 * Gazebo每一个仿真更新周期都会进入这里。
	 *
	 * 注意：
	 *
	 * 下面虽然日志每2秒打印一次，
	 * 但是：
	 *
	 * 目标位置获取
	 * 姿态计算
	 * SetWorldPoseCmd
	 * 速度清零
	 *
	 * 仍然在每一个PreUpdate周期执行。
	 * ============================================================
	 */
	void PreUpdate(
		const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm) override
	{
		if (!_configured) {
			return;
		}


		/*
		 * ========================================================
		 * RELEASE
		 * ========================================================
		 */

		 if (
			_release_requested.load()
			&&
			!_released
		    ) {

			std::cout
			    << "[PrelaunchTracker] RELEASE"
			    << std::endl;


			/*
			 * ========================================================
			 * 非常重要：
			 *
			 * 发射前调用过：
			 *
			 * SetWorldPoseCmd()
			 * SetLinearVelocity()
			 * SetAngularVelocity()
			 *
			 * 它们会在Gazebo ECS中创建command component。
			 *
			 * RELEASE以后必须显式删除，
			 * 否则速度command有可能继续把飞机速度压成0。
			 * ========================================================
			 */

			const bool pose_cmd_removed =
			    _ecm.RemoveComponent<
				gz::sim::components::WorldPoseCmd>(
				    _model.Entity()
				);


			const bool linear_cmd_removed =
			    _ecm.RemoveComponent<
				gz::sim::components::LinearVelocityCmd>(
				    _base_link.Entity()
				);


			const bool angular_cmd_removed =
			    _ecm.RemoveComponent<
				gz::sim::components::AngularVelocityCmd>(
				    _base_link.Entity()
				);


			std::cout
			    << "[PrelaunchTracker] command cleanup:"
			    << " pose=" << pose_cmd_removed
			    << " linear=" << linear_cmd_removed
			    << " angular=" << angular_cmd_removed
			    << std::endl;


			/*
			 * 恢复正常物理环境。
			 */
			_model.SetCollisionEnabled(
			    _ecm,
			    true
			);


			_model.SetGravityEnabled(
			    _ecm,
			    true
			);


			_released = true;


			std::cout
			    << "[PrelaunchTracker] PHYSICS FREE"
			    << std::endl;


			return;
		    }
		/*
		 * 已释放后，
		 * PrelaunchTracker彻底停止干预飞机。
		 */
		if (_released) {
			return;
		}


		/*
		 * ========================================================
		 * 获取当前飞机所属World
		 * ========================================================
		 */

		const gz::sim::Entity world_entity =
			gz::sim::worldEntity(
				_model.Entity(),
				_ecm
			);


		if (
			world_entity
			==
			gz::sim::kNullEntity
		) {

			PrintErrorEveryTwoSeconds(
				_info,
				"[PrelaunchTracker] WORLD NOT FOUND"
			);

			return;
		}


		const gz::sim::World world(
			world_entity
		);


		/*
		 * ========================================================
		 * 在World中查找目标模型
		 *
		 * 当前：
		 *
		 * target_drone
		 * ========================================================
		 */

		const gz::sim::Entity target_entity =
			world.ModelByName(
				_ecm,
				_target_model_name
			);


		if (
			target_entity
			==
			gz::sim::kNullEntity
		) {

			/*
			 * 找不到目标时也限制为2秒打印一次，
			 * 防止错误日志刷屏。
			 */
			if (
				!_error_printed_once
				||
				(
					_info.simTime
					-
					_last_error_print_time
					>=
					std::chrono::seconds(2)
				)
			) {

				_last_error_print_time =
					_info.simTime;

				_error_printed_once =
					true;


				std::cerr
					<< "[PrelaunchTracker] "
					<< "TARGET MODEL NOT FOUND: "
					<< _target_model_name
					<< std::endl;
			}


			return;
		}


		/*
		 * ========================================================
		 * 获取目标世界坐标
		 * ========================================================
		 */

		const gz::math::Pose3d target_pose =
			gz::sim::worldPose(
				target_entity,
				_ecm
			);


		const gz::math::Vector3d
			target_position =
				target_pose.Pos();


		/*
		 * ========================================================
		 * 飞机 -> 目标
		 *
		 * 世界坐标系方向向量
		 * ========================================================
		 */

		const gz::math::Vector3d direction =
			target_position
			-
			_launch_position;


		const double dx =
			direction.X();

		const double dy =
			direction.Y();

		const double dz =
			direction.Z();


		const double horizontal =
			std::sqrt(
				dx * dx
				+
				dy * dy
			);


		const double distance =
			std::sqrt(
				dx * dx
				+
				dy * dy
				+
				dz * dz
			);


		if (distance < 0.001) {
			return;
		}


		/*
		 * ========================================================
		 * 计算Gazebo模型姿态
		 *
		 * 目标：
		 *
		 * 让Gazebo飞机模型局部 +Z
		 *
		 * 始终指向target_drone。
		 *
		 *
		 * 当前采用：
		 *
		 * Roll固定为0
		 *
		 * Pitch和Yaw完成瞄准。
		 *
		 *
		 * yaw：
		 *
		 * 世界XY平面上的目标方向。
		 *
		 *
		 * pitch：
		 *
		 * 从世界+Z方向倾斜到LOS所需角度。
		 *
		 * 注意：
		 *
		 * 这里是Gazebo FLU Euler pitch。
		 *
		 * 它和PX4 FRD中的pitch符号含义不同。
		 * ========================================================
		 */

		const double yaw =
			std::atan2(
				dy,
				dx
			);


		const double pitch =
			std::atan2(
				horizontal,
				dz
			);


		const double roll =
			0.0;


		/*
		 * Gazebo quaternion
		 *
		 * 输入：
		 *
		 * roll
		 * pitch
		 * yaw
		 */
		const gz::math::Quaterniond
			orientation(
				roll,
				pitch,
				yaw
			);


		/*
		 * ========================================================
		 * 发射前期望Pose
		 *
		 * Position：
		 *
		 * 固定在launch_position。
		 *
		 * Orientation：
		 *
		 * 实时跟踪目标。
		 * ========================================================
		 */

		const gz::math::Pose3d desired_pose(
			_launch_position,
			orientation
		);


		/*
		 * ========================================================
		 * 强制飞机保持：
		 *
		 * 位置固定
		 * 姿态指向目标
		 *
		 * Gazebo官方Model API提供SetWorldPoseCmd，
		 * 用于向模型设置世界坐标系Pose命令。
		 * ========================================================
		 */

		_model.SetWorldPoseCmd(
			_ecm,
			desired_pose
		);


		/*
		 * ========================================================
		 * 发射前把飞机速度保持为0。
		 *
		 * 防止：
		 *
		 * 重力残余
		 * PX4电机输出
		 * 数值误差
		 *
		 * 导致模型缓慢漂移。
		 * ========================================================
		 */

		_base_link.SetLinearVelocity(
			_ecm,
			gz::math::Vector3d::Zero
		);


		_base_link.SetAngularVelocity(
			_ecm,
			gz::math::Vector3d::Zero
		);


		/*
		 * ========================================================
		 * 调试日志：
		 *
		 * 每2秒仿真时间打印一次。
		 *
		 * ★ 这里只降低std::cout频率。
		 *
		 * 上面的：
		 *
		 * target读取
		 * direction计算
		 * RPY计算
		 * SetWorldPoseCmd
		 *
		 * 都还是按照Gazebo PreUpdate原始频率执行。
		 * ========================================================
		 */

		if (
			!_pose_printed_once
			||
			(
				_info.simTime
				-
				_last_pose_print_time
				>=
				std::chrono::seconds(2)
			)
		) {

			_last_pose_print_time =
				_info.simTime;


			_pose_printed_once =
				true;


			constexpr double RAD_TO_DEG =
				57.29577951308232;


			std::cout
				<< "[PrelaunchTracker] "
				<< "target=("

				<< target_position.X()
				<< ", "

				<< target_position.Y()
				<< ", "

				<< target_position.Z()

				<< ") "

				<< "launch=("

				<< _launch_position.X()
				<< ", "

				<< _launch_position.Y()
				<< ", "

				<< _launch_position.Z()

				<< ") "

				/*
				 * 特意改成GZ_RPY，
				 * 避免以后误认为PX4 FRD姿态。
				 */
				<< "GZ_RPY=("

				<< roll
					*
					RAD_TO_DEG
				<< ", "

				<< pitch
					*
					RAD_TO_DEG
				<< ", "

				<< yaw
					*
					RAD_TO_DEG

				<< ")"

				<< std::endl;
		}
	}


private:

	/*
	 * ============================================================
	 * Release callback
	 * ============================================================
	 */

	void ReleaseCallback(
		const gz::msgs::Empty & /* _msg */)
	{
		_release_requested.store(
			true
		);
	}


	/*
	 * ============================================================
	 * 通用错误日志：
	 *
	 * 每2秒最多打印一次。
	 * ============================================================
	 */

	void PrintErrorEveryTwoSeconds(
		const gz::sim::UpdateInfo &_info,
		const std::string &_message)
	{
		if (
			!_error_printed_once
			||
			(
				_info.simTime
				-
				_last_error_print_time
				>=
				std::chrono::seconds(2)
			)
		) {

			_last_error_print_time =
				_info.simTime;


			_error_printed_once =
				true;


			std::cerr
				<< _message
				<< std::endl;
		}
	}


	/*
	 * ============================================================
	 * Gazebo模型
	 * ============================================================
	 */

	gz::sim::Model _model{
		gz::sim::kNullEntity
	};


	gz::sim::Link _base_link{
		gz::sim::kNullEntity
	};


	/*
	 * ============================================================
	 * Transport
	 * ============================================================
	 */

	gz::transport::Node _node;


	/*
	 * ============================================================
	 * SDF参数
	 * ============================================================
	 */

	std::string _target_model_name{
		"target_drone"
	};


	std::string _release_topic{
		"/fpv_15/release"
	};


	gz::math::Vector3d _launch_position{
		0.0,
		0.0,
		1.0
	};


	/*
	 * ============================================================
	 * Release状态
	 * ============================================================
	 */

	std::atomic_bool _release_requested{
		false
	};


	bool _released{
		false
	};


	bool _configured{
		false
	};


	/*
	 * ============================================================
	 * 正常Pose日志控制
	 *
	 * 使用Gazebo仿真时间。
	 * ============================================================
	 */

	std::chrono::steady_clock::duration
		_last_pose_print_time{
			std::chrono::steady_clock::duration::zero()
		};


	bool _pose_printed_once{
		false
	};


	/*
	 * ============================================================
	 * 错误日志控制
	 * ============================================================
	 */

	std::chrono::steady_clock::duration
		_last_error_print_time{
			std::chrono::steady_clock::duration::zero()
		};


	bool _error_printed_once{
		false
	};

};


} // namespace prelaunch_tracker


/*
 * ================================================================
 * Gazebo plugin注册
 * ================================================================
 */

GZ_ADD_PLUGIN(
	prelaunch_tracker::PrelaunchTracker,
	gz::sim::System,
	prelaunch_tracker::PrelaunchTracker::ISystemConfigure,
	prelaunch_tracker::PrelaunchTracker::ISystemPreUpdate
)


GZ_ADD_PLUGIN_ALIAS(
	prelaunch_tracker::PrelaunchTracker,
	"prelaunch_tracker::PrelaunchTracker"
)

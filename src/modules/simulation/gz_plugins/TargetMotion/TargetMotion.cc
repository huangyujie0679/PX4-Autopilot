#include "TargetMotion.hh"
#include <gz/sim/components/PoseCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <gz/sim/Model.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/Pose.hh>

namespace target_motion
{
void TargetMotion::Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &,
        gz::sim::EventManager &)
    {
        target_entity = _entity;
        std::cout
        << "[TargetMotion] entity id = "
        << target_entity
        << std::endl;


        std::cout
        << "[TargetMotion] Loaded"
        << std::endl;

        /*读取fpv_attack.sdf中的参数*/
        if(_sdf)
        {

            if(_sdf->HasElement("motion_mode"))
            {
                motion_mode =
                _sdf->Get<int>("motion_mode");
            }


            if(_sdf->HasElement("distance"))
            {
                distance =
                _sdf->Get<double>("distance");
            }


            if(_sdf->HasElement("altitude"))
            {
                altitude =
                _sdf->Get<double>("altitude");
            }


            if(_sdf->HasElement("speed"))
            {
                speed =
                _sdf->Get<double>("speed");
            }


            if(_sdf->HasElement("amplitude"))
            {
                amplitude =
                _sdf->Get<double>("amplitude");
            }


            if(_sdf->HasElement("radius"))
            {
                radius =
                _sdf->Get<double>("radius");
            }


            if(_sdf->HasElement("omega"))
            {
                omega =
                _sdf->Get<double>("omega");
            }

        }



    // 输出最终生效参数
        std::cout
        << "========== Target Motion Parameter =========="
        << std::endl;


        std::cout
        << "motion_mode = "
        << motion_mode
        << std::endl;


        std::cout
        << "distance = "
        << distance
        << " m"
        << std::endl;


        std::cout
        << "altitude = "
        << altitude
        << " m"
        << std::endl;


        std::cout
        << "speed = "
        << speed
        << " m/s"
        << std::endl;


        std::cout
        << "amplitude = "
        << amplitude
        << " m"
        << std::endl;


        std::cout
        << "radius = "
        << radius
        << " m"
        << std::endl;


        std::cout
        << "omega = "
        << omega
        << " rad/s"
        << std::endl;


        std::cout
        << "=============================================="
        << std::endl;

    }




void TargetMotion::PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm)
{

    if(target_entity ==
       gz::sim::kNullEntity)
        return;

    /*
     * 获取仿真时间
     */
    time_s =
    std::chrono::duration<double>(
        _info.simTime).count();

    double x;
    double y;
    double z;

    /*
     * 根据模式生成目标轨迹
     */

    switch(motion_mode)
    {

    /*
     * 模式0
     *
     * 静止目标
     */
    case 0:
        x = distance;
        y = 0;
        z = altitude;
        break;

    /*
     * 模式1
     *
     * 匀速直线飞行
     */
    case 1:
        x =
        distance
        +
        speed*time_s;

        y = 0;
        z = altitude;

        break;

    /*
     * 模式2
     *
     * 正弦横向机动
     *
     * x方向前进
     * y方向摆动
     */
    case 2:
        x =
        distance
        +
        speed*time_s;


        y =
        amplitude*
        sin(0.5*time_s);

        z =
        altitude;

        break;

    /*
     * 模式3
     *
     * 圆周运动
     */
    case 3:
        x =
        distance
        +
        radius*cos(omega*time_s);

        y =
        radius*sin(omega*time_s);

        z =
        altitude;

        break;

    /*
     * 模式4
     *
     * 水平往返
     */
    case 4:
        x =
        distance
        +
        amplitude*
        sin(speed*time_s);

        y=500;

        z=altitude;

        break;

    /*
     * 模式5
     *
     * 随机机动
     *
     * 用于测试鲁棒性
     */
    case 5:
    {
        double dx =
        random_offset*
        sin(0.7*time_s);

        double dy =
        random_offset*
        cos(0.4*time_s);

        x =
        distance
        +
        speed*time_s
        +
        dx;

        y=dy;

        z=altitude;

        break;

    }

    default:

        x=distance;

        y=0;

        z=altitude;

        break;

    }

    /*
     * 设置目标姿态位置
     */
    gz::math::Pose3d pose(

        x,
        y,
        z,

        0,
        0,
        0

    );



    _ecm.SetComponentData
    <
    gz::sim::components::WorldPoseCmd
    >
    (
        target_entity,
        pose
    );


}


}



GZ_ADD_PLUGIN(
    target_motion::TargetMotion,
    gz::sim::System,
    target_motion::TargetMotion::ISystemConfigure,
    target_motion::TargetMotion::ISystemPreUpdate
)



GZ_ADD_PLUGIN_ALIAS(
    target_motion::TargetMotion,
    "target_motion::TargetMotion"
)

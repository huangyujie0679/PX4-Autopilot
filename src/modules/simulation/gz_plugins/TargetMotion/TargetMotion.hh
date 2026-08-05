#pragma once
/*
 * TargetMotion.hh
 *
 * Gazebo目标运动插件
 *
 * 用于PX4无人机视觉/制导算法仿真
 *
 * 支持：
 *
 * 0 静止目标
 * 1 匀速直线目标
 * 2 横向正弦机动
 * 3 圆周运动
 * 4 往返运动
 * 5 随机机动
 *
 */


#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>

namespace target_motion
{

class TargetMotion :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
{

public:


    /*
     * Gazebo加载插件时调用
     *
     * 读取SDF参数
     */
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr
    ) override;

    /*
     * 每个仿真周期执行
     *
     * 默认Gazebo Harmonic 250Hz
     */
    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm
    ) override;

private:

    //目标模型实体ID
    gz::sim::Entity target_entity{
        gz::sim::kNullEntity
    };

    //仿真时间
    double time_s{0.0};

    /*
     * 目标运动模式
     *
     * 0 静止
     *
     * 1 匀速直线
     *
     * 2 正弦横向机动
     *
     * 3 圆周运动
     *
     * 4 水平往返
     *
     * 5 随机机动
     *
     */
    int motion_mode{4};
    //目标初始距离
    double distance{0.0};
    //目标高度
    double altitude{100.0};
    //目标速度
    double speed{10.0};
    //正弦运动幅值
    double amplitude{10.0};
    //圆周半径
    double radius{6.0};
    //圆周角速度
    double omega{10.0};
    //随机机动最大偏移
    double random_offset{10};

};


}

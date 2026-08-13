#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_target_los.h>
#include <px4_platform_common/time.h>
#include <mavlink/mavlink_log.h>
#include <mavlink/mavlink_messages.h>
#include <mavlink/mavlink_main.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_target_los.h>

class TargetLOSReceiver :
    public ModuleBase<TargetLOSReceiver>
{
public:

    TargetLOSReceiver() = default;

    ~TargetLOSReceiver() override = default;


    static int task_spawn(
        int argc,
        char *argv[])
    {
        TargetLOSReceiver *instance =
            new TargetLOSReceiver();

        if (instance == nullptr)
        {
            return PX4_ERROR;
        }


        _object.store(instance);

        instance->ScheduleNow();

        return PX4_OK;
    }



    static TargetLOSReceiver *instantiate(
        int argc,
        char *argv[])
    {
        return new TargetLOSReceiver();
    }



    static int custom_command(
        int argc,
        char *argv[])
    {
        return print_usage(
            "unknown command"
        );
    }



    static int print_usage(
        const char *reason = nullptr)
    {

        if(reason)
        {
            PX4_WARN("%s", reason);
        }


        PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Target LOS receiver

Publishes vehicle_target_los
        )DESCR_STR");


        return 0;
    }



    void Run() override
    {

        vehicle_target_los_s msg{};


        msg.timestamp =
            hrt_absolute_time();


        /*
         * 临时测试数据
         * 后续替换为MAVLink接收数据
         */


        msg.los_azimuth =
            0.46f;


        msg.los_elevation =
            0.39f;


        msg.los_az_rate =
            0.01f;


        msg.los_el_rate =
            0.02f;


        msg.target_valid =
            true;



        _los_pub.publish(msg);



        ScheduleDelayed(
            20000
        );
    }



private:

    uORB::Publication<
        vehicle_target_los_s
    > _los_pub{
        ORB_ID(vehicle_target_los)
    };

};



extern "C"
__EXPORT
int target_los_receiver_main(
    int argc,
    char *argv[])
{

    return TargetLOSReceiver::main(
        argc,
        argv
    );

}

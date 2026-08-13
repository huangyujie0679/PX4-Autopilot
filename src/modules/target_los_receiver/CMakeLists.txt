px4_add_module(
    MODULE modules__target_los_receiver
    MAIN target_los_receiver

    SRCS
        target_los_receiver.cpp

    DEPENDS
        px4_work_queue
)

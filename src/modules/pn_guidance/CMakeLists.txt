px4_add_module(
 MODULE modules__pn_guidance

 MAIN pn_guidance

 STACK_MAIN 2000


 SRCS
     pn_guidance.cpp
     coordinate_transform.cpp
     acceleration_to_attitude.cpp

 DEPENDS
)

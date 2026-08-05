project(TargetMotionPlugin)


add_library(${PROJECT_NAME}
    SHARED
    TargetMotion.cc
)



set_target_properties(${PROJECT_NAME}
PROPERTIES
LIBRARY_OUTPUT_DIRECTORY
${PX4_GZ_PLUGIN_OUTPUT_DIR}
)



target_link_libraries(${PROJECT_NAME}

PUBLIC
${GZ_SIM_TARGET}

PUBLIC
${GZ_PLUGIN_TARGET}

PUBLIC
${GZ_TRANSPORT_TARGET}

)

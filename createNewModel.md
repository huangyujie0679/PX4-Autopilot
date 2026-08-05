参考fpv_15

/home/huang/PX4/PX4-Autopilot-v1.17-dev/Tools/simulation/gz/models/fpv_15
model.config里  <name>fpv_15</name>
model.sdf里  <model name='fpv_15'>


/home/huang/PX4/PX4-Autopilot-v1.17-dev/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
里新增4501_gz_fpv_15

/home/huang/PX4/PX4-Autopilot-v1.17-dev/ROMFS/px4fmu_common/init.d-posix/airframes/4501_gz_fpv_15
里PX4_SIM_MODEL=${PX4_SIM_MODEL:=fpv_15}

/home/huang/PX4/PX4-Autopilot-v1.17-dev/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/4501_gz_fpv_15
里PX4_SIM_MODEL=${PX4_SIM_MODEL:=fpv_15}


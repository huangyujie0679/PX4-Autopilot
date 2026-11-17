/****************************************************************************
 *
 * PN guidance module for PX4 v1.17-dev
 *
 * 统一vehicle_target_los语义：gimbal_roll是历史字段名，实际承载云台方位角(azimuth)，gimbal_pitch为俯仰角；los_azimuth/elevation为光轴脱靶角；los_az_rate/el_rate为RK3576由相邻图像帧目标像素差换算得到的相机相对LOS角速度。
 * 同一PN代码同时用于SITL与真机：SITL旧发送器若gimbal_*保持0，新的完整LOS公式会严格退化为旧版仅使用los_azimuth/elevation的几何。
 *
 * 状态机（V21.3航拍跟随直接BPN）：
 *
 * WAIT_LOS
 *    ↓
 * LAUNCH
 *    ↓
 * BPN READY
 *    ↓ ARM
 * WAIT_RELEASE
 *    ↓ RC AUX1 LOW->HIGH (100ms)
 * LAUNCH_CLEARANCE
 *    ↓
 * TRANSITION
 *    ↓
 * BPN ACTIVE
 *    ↓ 目标已越过 / LOS丢失
 * SAFE_HOVER
 *
 * LAUNCH_CLEARANCE / TRANSITION 的枚举、参数和函数继续保留用于回退/对照，
 * V21.5将 DIRECT_BPN_AFTER_RELEASE=false，正式飞行路径重新进入这两个阶段。
 *
 * 本版本相对上一版的主要修改：
 *
 * 0. 修正Guidance/光轴坐标与PX4标准Body FRD混用：真实安装为Camera +X->Body +Y、Camera +Y->Body -X、Camera +Z(光轴)->Body -Z；结合Guidance定义Gx=-Cy、Gy=Cx、Gz=Cz，得到Bx=Gx、By=Gy、Bz=-Gz，再使用vehicle_attitude旋转到NED；LOS/BPN几何公式本身不改。
 *
 * 1. 制导循环固定 50 Hz：LOOP_INTERVAL_US = 20000。
 *
 * 2. 当前速度规划（V21.3连续化）：
 *      LOS rate <= 0.05 rad/s -> 25 m/s
 *      0.05 ~ 0.15          -> 25 -> 24 m/s
 *      0.15 ~ 0.35          -> 24 -> 22 m/s
 *      0.35 ~ 0.60          -> 22 -> 20 m/s
 *      >= 0.60               -> 20 m/s
 *    26 m/s开始硬速度制动，28 m/s为当前名义硬速度保护上限。
 *
 * 3. 增强超速制动：
 *    - 超过动态目标速度后主动制动；
 *    - 27 m/s开始进入硬速度保护；
 *    - 硬速度保护按三维速度方向制动。
 *
 * 4. LAUNCH_CLEARANCE使用独立推力预算0.95，
 *    不再被正常BPN的0.75推力预算裁掉。
 *
 * 5. 删除CLEARANCE阶段持续固定“向上5m/s^2”的控制。
 *    CLEARANCE本身只给小幅前向净加速度，垂向默认0；
 *    只有检测到相对RELEASE高度掉高/即将掉高时，
 *    RELEASE高度保护才按PD + 制动距离施加向上加速度。
 *    目的是避免发射后形成十几m/s的巨大向上速度。
 *
 * 6. 保留：
 *      PN_N = 3.0
 *      BPN_ACCEL_MAX = 12 m/s^2
 *      attitude slew = 60 deg/s
 *      attitude lead = 60 deg
 *      TRANSITION = 1.0 s
 *    先隔离速度和发射阶段问题，不同时大改导引增益。
 *
 * 7. WAIT_RELEASE保持0 guidance thrust。
 *    真机若需要“解锁后电机怠速”，应由PX4输出/ESC armed-idle机制负责，
 *    不建议由制导模块在发射架上主动施加额外推力。
 *
 * 8. 增加可命令控制的BENCH_RT台架诊断输出（5 Hz，默认关闭）：
 *      pn_guidance bench on|off|once；连续输出LOS/BPN/q_preview完整链路
 *    正式运行默认不打印；仅在台架验证时执行pn_guidance bench on；开关使用PX4 atomic在NSH线程与WorkQueue之间同步。
 *
 * 9. V10增加一键台架命令pn_guidance test on|off|status：test on自动关闭BENCH_RT/SHADOW、锁定当前姿态为15deg cage锚点，
 *    以正常BPN 0.75推力预算计算候选RAW/CMD，并以50Hz发布vehicle_attitude_setpoint；真实发布推力始终强制为0且只允许DISARMED。
 * 10. TEST模式每5Hz自动打印BPN_RAW/BPN_USED、AOPT、RAW、CMD、候选推力和cage角度，移动目标时无需反复执行sp once；
 *     原bench/sp/shadow命令继续保留作为底层诊断接口，不影响正常飞行状态机。
 *
 * 11. V11把正式RELEASE触发源改为真实遥控器AUX1：
 *     - ARM只进入WAIT_RELEASE，pn_guidance主动推力保持0，电机怠速由PX4/ESC armed-idle负责；
 *     - ARM之后必须先检测到AUX1低位，再检测LOW->HIGH，并连续保持100ms才接受RELEASE；
 *     - 如果ARM时AUX1已经在高位，绝不会直接发射，必须先拨回LOW再重新拨到HIGH；
 *     - 只接受manual_control_setpoint中data_source=SOURCE_RC的真实RC输入；
 *     - RC输入失效/超时后释放互锁复位，恢复后必须重新LOW->HIGH；
 *     - RELEASE真正触发前再次检查LOS、姿态以及本地Z/VZ有效，保证高度保护参考可建立；V21.2默认随后直接进入BPN ACTIVE。
 *
 * 12. 历史版本曾调整CLEARANCE起始推力；V21.7当前真机实飞基线以实际常量为准：start=0.25、limit=0.30。
 *     高度保护仍从LAUNCH_CLEARANCE开始并持续覆盖TRANSITION/BPN。
 *
 * 13. V12在pn_guidance内部完成惯性LOS方向导数重构：
 *     - RK3576的los_az_rate/el_rate只按“图像相对角速度”解释，不再假设上游已惯性化；
 *     - 图像相对LOS方向导数按当前云台姿态旋转到PX4 Body FRD；
 *     - 用相邻两帧gimbal_roll/gimbal_pitch差分得到云台相对机体运动对LOS的贡献；
 *     - 订阅vehicle_angular_velocity，用机体系角速度omega_B补偿机体自身旋转；
 *     - 最终得到 inertial_LOS_dot_B = image_LOS_dot_B + gimbal_LOS_dot_B + omega_B × LOS_B；
 *     - BPN、BENCH、TEST/SP预览统一使用该惯性LOS方向导数，避免真机稳定云台大幅运动时把图像相对运动误当成惯性LOS运动。
 *
 * 14. V13增加仿真/真机RELEASE模式开关：
 *     - SIMULATION_MODE=true：不要求真实RC输入；PX4仍需正常ARM，进入WAIT_RELEASE后程序内部模拟AUX1先LOW；
 *       LOS/姿态/Z-VZ/惯性LOS重构等条件连续稳定后，自动模拟AUX1拨到HIGH，并沿用100ms确认后触发RELEASE；
 *     - SIMULATION_MODE=false：完全保持V12真实遥控器逻辑，只接受SOURCE_RC，ARM后必须真实AUX1 LOW->HIGH并保持100ms；
 *     - 仿真模式只替代“RELEASE拨杆”，不自动执行ARM，避免pn_guidance越权管理PX4解锁状态。
 *
 * 15. V14强化RK3576同步测量语义：
 *     - los_azimuth/elevation、los_az_rate/el_rate、gimbal_roll/pitch必须来自同一视觉/云台测量包；
 *     - 惯性LOS重构的gimbal差分dt优先使用vehicle_target_los.timestamp，而不是WorkQueue执行时刻，降低调度抖动；
 *     - 若timestamp不可用/异常则安全回退hrt_absolute_time()；
 *     - BENCH_RT连续输出LDimgB/LDgimB/LDbodB/LDinB和实际重构dt，便于直接和Gazebo真值做A/B比较；
 *     - V14不在飞控端模拟任何“10ms云台延迟”；该延迟只存在于SITL传感器模型，真机使用RK3576实际测量。
 *
 * 16. V15改为离散惯性LOS向量观测器：
 *     - 真机仅需要RK3576的raw los_az_rate/el_rate + gimbal_roll/pitch；
 *     - 飞控内部积分raw rate估计小范围相机残余角，并结合当前gimbal姿态得到完整Body LOS；
 *     - 再用vehicle_attitude把LOS直接旋到NED，使用相邻完整惯性LOS单位向量差分得到惯性LOS方向导数；
 *     - 不再把image/gimbal/body三个大运动项显式相加，避免大项相消、采样相位差和瞬时gyro不同步造成的数值放大；
 *     - EKF attitude reset时强制重置LOS观测器，禁止跨姿态参考跳变做差分；
 *     - 旧IMG/GIM/BODY三项仅保留BENCH诊断，不再参与BPN控制输入。
 *
 * 17. V16增加EKF姿态reset后的LOS重捕获与时间同步：
 *     - EKF attitude reset时只重播种NED LOS差分基准，保留raw-rate积分得到的相机残余miss角；
 *     - reset后允许最多120ms的observer重捕获宽限，期间LOS/姿态仍健康时保持上一帧有效制导指令，禁止一帧invalid直接SAFE_HOVER；
 *     - 超过宽限仍未恢复、或LOS stale/target_invalid/attitude_invalid时仍进入SAFE_HOVER，不削弱原安全保护；
 *     - 使用vehicle_attitude.timestamp_sample与vehicle_target_los.timestamp估计测量时差，并用新鲜body rate把当前姿态小时间外推/回推到LOS采样时刻；
 *     - 诊断增加reacquire状态与attitude-sync dt，便于检查真机RK3576串口包与EKF姿态的时序。
 *
 * 18. V19改为Body-relative离散惯性LOS observer，并加入实机observer-only模式：
 *     - 不再把相邻LOS旋到绝对NED后差分；上一帧LOS_B先用陀螺平均角速度积分旋转到当前Body，再与当前LOS_B比较；
 *     - 对同一惯性向量有 dL_B/dt = -omega_B x L_B，因此使用旋转向量 -omega_avg*dt 传播上一帧Body LOS；
 *     - EKF quaternion/yaw reset不再重播种LOS observer，只继续修正yaw_lock/hold_q等依赖NED参考的控制状态；
 *     - SITL自动使用TRUTH闭环，V19只做shadow；同时累计TRUE-RECON的RMS、MAX、reset后100ms MAX和方向误差；
 *     - 非SITL固件自动进入HARDWARE_OBSERVER_ONLY：pn_guidance只订阅/计算/记录，不发布vehicle_attitude_setpoint或推力，
 *       因而不会抢占PX4正常遥控器控制与Commander解锁；实机第一阶段只用于RK3576/云台/时间同步/observer日志验证；
 *     - SITL阶段直接使用Python发送的真实camera miss angle；当前实机协议若仍只有raw rate+gimbal角，则只在observer-only中积分miss，绝不直接闭环。
 *
 * 19. V19.1仅增加诊断输出，不改变observer/PN/控制逻辑：
 *     - 保留惯性LOS方向导数在PX4 Body FRD下的内部量 _los_dot_inertial_body；
 *     - 仅为终端显示，用当前vehicle_attitude将该向量旋转到NED，打印[N,E,D]；
 *     - 同一组输出同步打印云台AZ/PITCH，便于台架对照；
 *     - NED显示量不反馈observer、不参与PN计算，不会改变V19结果。
 *
 * 20. V20针对移动目标试验暴露出的时间一致性和A/B统计问题改进：
 *     - 不改变PrelaunchTracker、PN/BPN增益、CLEARANCE/TRANSITION/发射状态机；
 *     - vehicle_target_los.timestamp被明确解释为“PX4 HRT时钟域下的视觉测量/图像采样时刻”；
 *       RK3576自己的板载时间戳必须先在串口驱动中同步/映射到PX4 HRT，禁止直接把板载boot-us与hrt直接相减；
 *     - 增加vehicle_angular_velocity历史缓存，并按LOS sample_time插值对应时刻omega_B，
 *       不再拿WorkQueue当前时刻的最新gyro与10~几十ms之前的视觉帧硬配；
 *     - attitude->LOS小时间传播同样优先使用sample_time附近的gyro；
 *     - RELEASE进入LAUNCH_CLEARANCE时清空A/B统计，排除发射前PrelaunchTracker整体转动造成的统计污染；
 *     - A/B终端TRUE/V20直接打印实时向量norm，不再依赖calculate_bpn_target()内部_last_*缓存；
 *     - A/B统计只在RELEASE后有效，并屏蔽truth LOS rate>0.5rad/s的极近距离/越靶几何奇异样本；
 *     - 缩短NED/时间诊断打印，避免PX4终端长行互相粘连；
 *     - 真机默认仍强制HARDWARE_OBSERVER_ONLY；仅显式pn_guidance rig stage1/2/3时进入固定机械台架专用路径。
 *
 * 21. V20.3增加固定机械台架三阶段闭环测试：
 *     - stage1：DISARMED + zero thrust，只验证LOS/BPN/q_d/vehicle_attitude_setpoint数学与接口；
 *     - stage2：ARM后AUX1 LOW->HIGH，固定低集体推力0.12，无桨/测功台验证姿态环、角速度环和电机差动符号；
 *     - stage3：固定台架“真实BPN控制链”验证：复用正式BPN的LOS->BPN->轴向分配->加速度姿态->60deg/s姿态slew->60deg lead，仅在最终发布前把真实推力硬限制<=0.25；仍保留台架姿态笼、实际姿态/角速度/motor安全退出；
 *     - RIG路径使用固定RIG_VC_TEST=20m/s，避免固定台架EKF速度为0导致BPN失去可观测性；
 *     - RIG路径不进入速度规划、高度保护、CLEARANCE、TRANSITION、SAFE_HOVER等自由运动逻辑；
 *     - 增加vehicle_rates_setpoint和actuator_motors订阅，5Hz同步打印BPN/q_d/attitude/rate/motor链路；
 *     - stage2/3 RC失效、AUX1回LOW、姿态/角速度/motor越界立即撤到zero-thrust姿态保持；硬件急停/总断能仍必须独立保留。
 *
 * 22. V20.4真机LOS输入改为CGTD070硬件反馈直连：
 *     - SITL继续订阅vehicle_target_los，保持原Gazebo/Python测试链不变；
 *     - 非SITL真机改为订阅cgtd070_status，不再依赖仿真/外部vehicle_target_los发布器；
 *     - Cgtd070Status的deg/deg/s在pn_guidance入口统一转换为rad/rad/s；
 *     - 协议raw LOS rate保持“右/上为正”；协议miss angle为“左/下为正”，在入口统一反号为pn_guidance的“右/上为正”；
 *     - gimbal_roll历史字段继续承载云台方位角，gimbal_pitch承载云台俯仰角，均按deg->rad直接转换；
 *     - target_valid直接使用cgtd070_status.target_valid；真机LOS observer直接使用硬件miss angle，不再积分raw rate估计miss；
 *     - RIG stage1/2/3逻辑、AUX1 RELEASE互锁、AUX2由cgtd070模块控制跟踪/取消跟踪的职责均保持不变。
 *
 * 23. V20.5改为“板卡raw LOS rate + 云台运动 + 机体角速度”融合重构惯性LOS角速度：
 *     - miss angle只负责和gimbal angle一起构造当前完整LOS方向，不再对完整LOS做20ms单帧差分作为主角速度来源；
 *     - RK3576的los_az_rate/el_rate作为高频图像相对LOS角速度，直接变换到PX4 Body FRD；
 *     - gimbal angle相邻帧差分得到云台角速度，并用一阶低通抑制0.01deg级角度量化导致的微分噪声；
 *     - 用解析雅可比把云台方位/俯仰角速度转换为LOS方向导数，再与image LOS rate及omega_B x LOS相加；
 *     - 融合后的LOS方向导数再次投影到LOS切平面，并做轻度低通后作为BPN唯一LOS-rate输入；
 *     - 保留原有时间戳/gyro历史对齐、RIG/TEST/SHADOW/正常状态机及所有推力/姿态限制不变。
 *
 * 24. V20.7仅增强诊断，不改变observer/BPN/状态机/控制输出：
 *     - BENCH、RIG、硬件observer-only和正式控制debug统一按5Hz打印，减少QGC控制台滚屏覆盖；
 *     - BENCH 5Hz增加LOS健康、Vc raw/filt/used、IMG/GIM/BODY/FRAW/IN、固定Vc=20的BPN预览与姿态预览；
 *     - RIG Stage3 5Hz增加Vc、speed target/brake、BPN raw/used/scale、axial used/limit、ACMD_N、推力预算、姿态/rate/torque/motor；
 *     - 正式控制debug压缩为4行5Hz：健康+Vc、LOS+BPN、加速度/推力/限幅、姿态/rate/torque/motor；
 *     - 所有新增内容仅PX4_INFO诊断，不参与任何控制计算。
 *
 * 25. V20.9针对真机已确认“图像capture到PX4完整收包约100ms、但同包gimbal角是发包时最新值”的时间错位做第一阶段修正：
 *     - 真机把内部LOS sample_time固定解释为cgtd070 RX-HRT减100ms，不改变SITL时间语义；
 *     - 当前包gimbal角先按RX-HRT写入飞控历史缓存，再插值得到capture时刻的gimbal角供LOS几何使用；
 *     - 云台角速度正式输入改为capture时刻前后各40ms的历史中心差分，利用100ms已知延迟获得未来侧样本，避免原50ms因果LPF造成额外相位滞后；
 *     - 相邻capture帧gimbal差分继续保留到ULog的gimbal_rate_raw中作为诊断；中心差分结果继续写gimbal_rate_filt字段，字段名不改但语义变为“正式使用的中心差分rate”；
 *     - IMG、GIM、BODY统一按同一capture时刻构造；最终25ms IN低通、50Hz调度、姿态slew/PID/Stage参数本版暂不修改，便于单独验证时间对齐效果。
 *
 * 25. V21将正式IMG角速度来源由RK3576黑盒los_az_rate/el_rate改为PX4根据direct miss_angle历史自行重构：
 *     - 保留V20.9固定100ms capture补偿、云台历史回查以及gimbal ±40ms中心差分；
 *     - 真机每帧direct miss angle按capture_time=RX-100ms写入历史；
 *     - 使用±20ms中心差分（总40ms窗口）计算miss_rate，正式observer时刻取capture_time-20ms；
 *     - miss/gimbal/body gyro全部插值到同一observer_time后再计算IMG/GIM/BODY并融合；
 *     - 板卡原los_az_rate/el_rate降级为ULog诊断量，不再参与真机observer/BPN；
 *     - SITL输入链保持原vehicle_target_los语义不变；
 *     - 最终25ms IN低通、50Hz WorkQueue、PN/PID/RIG/Stage3参数本版均不修改。
 *
 * 26. 【V21.1修改】仅对固定机械台架Stage3增加BPN缩放，不改变正式飞行、Stage1或Stage2：
 *     - RIG_STAGE3_BPN_SCALE固定为0.20，只在rig stage3路径生效；
 *     - 先完整运行原calculate_bpn_target()，保留正式BPN、速度管理、轴向加速度和0.75推力预算；
 *     - 随后只把法向BPN分量缩放为20%，并在NED加速度命令中用缩放后的BPN替换原BPN分量；
 *     - 轴向加速度、speed brake、重力补偿、60deg/s姿态slew、60deg lead、8deg cage以及所有abort阈值均保持不变；
 *     - RIG日志中的Braw继续表示未缩放BPN，Buse与_last_bpn_scale反映Stage3最终实际使用的缩放结果。
 *
 * 27. 【V21.2修改】航拍跟随正式飞行改为RELEASE后直接进入BPN ACTIVE：
 *     - V21.2曾启用DIRECT_BPN_AFTER_RELEASE=true；V21.5现已回退为false，恢复LAUNCH_CLEARANCE/TRANSITION正式路径；
 *     - 新增enter_bpn_active()，在直接接管前重新以当前真实姿态初始化slew基准，并保留RELEASE高度底线保护基准；
 *     - 不增加BPN gain ramp：AUX1 RELEASE通过后，同一控制周期直接运行完整BPN（方案A）；
 *     - 因不再有CLEARANCE/TRANSITION等待窗口，真机RELEASE额外要求LOS-rate observer有效且body-rate新鲜；
 *     - 二代机HOVER_THRUST改为0.09；MIN_THRUST由0.10降为0.04，避免悬停推力被最小推力夹高；
 *     - PN_N、BPN_ACCEL_MAX、AXIAL_ACCEL_MAX、FLIGHT_COMMISSIONING_GAIN、速度规划/制动参数均保持原值；
 *     - 将法向BPN、轴向加速度、Vc、速度规划、推力预算等调参量集中标注，并在status中显式打印，便于航拍跟随调参。
 *
 * 28. 【V21.3修改】保持直接BPN和现有BPN增益不变，仅修正二代机参数自洽性与速度规划连续性：
 *     - HOVER_THRUST确认采用实测0.15；不新增velocity_valid() RELEASE硬门槛；
 *     - ALT_FLOOR_THRUST_LIMIT改为与GUIDANCE_THRUST_LIMIT相同，避免高度保护介入时最大推力预算反而下降；
 *     - 新增LOS_RATE_SPEED_POINT_4=0.60 rad/s，在0.35~0.60 rad/s区间把目标速度由13 m/s连续插值到6 m/s；
 *     - 0.60 rad/s以上保持GUIDANCE_SPEED_MIN=6 m/s，消除原0.35 rad/s处13->6 m/s阶跃；
 *     - PN_N、BPN_ACCEL_MAX、AXIAL_ACCEL_MAX、FLIGHT_COMMISSIONING_GAIN、speed brake、姿态限制和高度保护算法其余参数均不改。
 *
 * 29. 【V21.4修改】增加RC绝对优先级与多publisher防竞争仲裁：
 *     - 识别PX4 Manual/Stabilized姿态模式：manual=true且altitude/velocity/position=false；
 *       该模式下mc_att_control会由遥控器直接生成vehicle_attitude_setpoint，因此pn_guidance硬禁止发布同一topic；
 *     - RELEASE进入正式制导时锁存当前vehicle_status.nav_state；制导激活后只要飞行模式发生变化，
 *       或进入Manual/Stabilized姿态模式，立即锁存RC_OVERRIDE并停止pn_guidance所有姿态setpoint发布；
 *     - RC_OVERRIDE在本次ARM周期内保持锁存，绝不因模式抖动/短时切回而自动重新夺权；只有DISARM后清除；
 *     - RELEASE若当前已经处在Manual/Stabilized姿态模式则拒绝，必须先切回制导所用模式并重新AUX1 LOW->HIGH；
 *     - publish_attitude_setpoint_internal()增加最终防线，即使其它状态路径遗漏仲裁，也不能在RC拥有姿态控制权时发布；
 *     - RIG固定台架路径和DISARMED接口测试不受该正式飞行RC仲裁影响。
 * 30. 【V21.5修改】恢复正式发射状态机：WAIT_RELEASE -> LAUNCH_CLEARANCE -> TRANSITION -> BPN。
 *     仅把DIRECT_BPN_AFTER_RELEASE切回false；保留V21.4 RC绝对优先级/多publisher防竞争仲裁不变。
 *     用于A/B验证“RELEASE后直接BPN”是否与偶发调度拥塞存在相关性。
 * 31. 【V21.6修改】仅缩短发射缓冲时间：CLEARANCE=0.20s（0.10s ramp+0.10s hold），TRANSITION=0.20s；
 *     加上AUX1 HIGH的100ms RELEASE确认，从物理拨杆HIGH到正式BPN约0.50s；其余推力、BPN、姿态限制参数保持不变。
 * 32. 【V21.7修改】针对实飞云台瞬时掉锁与过渡期机体角速度过大：
 *     - 真机CLEARANCE改为100ms保持RELEASE瞬间姿态，不再在CLEARANCE内主动把姿态拉向近水平；
 *     - TRANSITION改为300ms，因此无掉锁时仍保持AUX1 HIGH确认100ms + CLEARANCE100ms + TRANSITION300ms ≈ 0.50s进入正式BPN；
 *     - CLEARANCE/TRANSITION专用姿态lead限制分别为8deg与8->10deg，正式BPN仍恢复45deg；
 *     - control LOS/target_valid短暂无效增加250ms宽限，期间保持最后一帧有效制导指令；CLEARANCE/TRANSITION计时暂停，重捕后从原进度继续；
 *     - 连续无效超过250ms或LOS数据真正stale时仍进入SAFE_HOVER，不自动从SAFE_HOVER重入BPN。
 * 33. 【V21.8修改】将正式BPN速度规划提升到20~25m/s，并同步提高高速段法向机动能力：
 *     - speed target改为25/24/22/20m/s，名义hard limit改为28m/s，26m/s开始硬制动；
 *     - VC_MAX由15提高到25m/s，使有效EKF闭合速度不再被15m/s截断；VC_FALLBACK保持20m/s；
 *     - BPN_ACCEL_MAX由10提高到12m/s^2，PN_N=3.0与FLIGHT_COMMISSIONING_GAIN=1.0保持不变；
 *     - AXIAL_ACCEL_MAX保持5m/s^2；正常speed brake最大6m/s^2，hard brake最大8m/s^2；
 *     - V21.7的0.5s过渡、250ms掉锁宽限与过渡期lead限制全部保持不变。
 * 34. 【V21.9修改】补齐20~25m/s高速版的速度失效降级与RC Acro接管：
 *     - VC_FALLBACK改为15m/s，VC_MAX仍为25m/s；有效EKF速度参与正常Vc闭环，无效时BPN按15m/s保守fallback；
 *     - SPEED_ACCEL_TAPER_BAND由2m/s扩大到4m/s，目标速度前4m/s开始逐渐收掉轴向加速度；
 *     - EKF三维速度无效时，正式制导AXIAL上限自动从5降到3m/s^2；速度恢复有效后自动回到5m/s^2并继续按速度目标taper；
 *     - RC所有权识别扩展到Manual/Stabilized/Acro：manual+rates且无alt/vel/pos控制时均视为飞手接管；
 *     - RELEASE后RC override监控持续覆盖SAFE_HOVER，不再只在CLEARANCE/TRANSITION/BPN生效；一旦切人工模式或nav_state变化即锁存到DISARM；
 *     - V21.8的25/24/22/20m/s速度目标、BPN_MAX=12、28m/s硬上限及V21.7过渡/掉锁保护全部保持。
 * 35. 【V21.9.1修改】仅做两项小改动，保持v21.9其余正式飞行逻辑/参数完全不变：
 *     - AUX1 HIGH通过100ms确认且RELEASE全部guard通过后，在真正进入CLEARANCE/BPN前，用当前vehicle_attitude重新锁定_yaw_lock；
 *       避免WAIT_LOS早期锁定的旧航向与实际RELEASE航向相差较大，引发无意义的大幅偏航并增加云台补偿压力。
 *     - TARGET_LOSS_GRACE_US由150ms增加到250ms；短时target/control LOS失效期间仍保持最后一帧有效制导指令，
 *       其余LOS_REACQUIRE_GRACE、CLEARANCE/TRANSITION、PN/BPN/AXIAL、速度规划、姿态限制和高度保护全部不变。
 * 36. 【V21.9.2修改】基于2026-10-31实飞日志的小步响应调整：
 *     - VC_FALLBACK由15m/s降为10m/s，仅在EKF三维速度无效时降低BPN闭合速度估计；VC_MAX仍为25m/s；
 *     - CLEARANCE_ATTITUDE_SLEW_DEG_S由75提高到85deg/s；
 *     - GUIDANCE_ATTITUDE_SLEW_DEG_S由75提高到85deg/s；
 *     - GUIDANCE_ATTITUDE_LEAD_MAX_DEG保持65deg不变；PN_N、BPN_ACCEL_MAX、AXIAL、速度规划、状态机和推力预算均不改。
 * 37. 【V21.10修改】针对真机约125ms LOS相位滞后增加当前时刻预测：
 *     - 真机direct miss angle仍严格按capture_time=RX-100ms解释，不伪造视觉时间戳；
 *     - IMG miss-rate由原capture-20ms中心差分改为capture时刻三点后向二阶差分，去掉人为增加的20ms observer滞后；
 *     - observer_time改为capture_time，GIM仍利用已存在的RX历史在capture±40ms做中心差分，BODY gyro与IMG/GIM保持同一capture时刻；
 *     - 利用body-rate历史把capture时刻的LOS与惯性LOS角速度传播到当前Body坐标，再按恒定惯性LOS角速度模型外推到now；
 *     - 正式BPN、Vc投影和CLEARANCE使用同一份current-time预测LOS，避免BPN用预测LOS但Vc仍用历史LOS；
 *     - 预测测量年龄上限提高到170ms，body-rate历史保持64帧；历史不足/预测失败时正式真机控制返回invalid，由现有BPN invalid/重捕获/target-loss安全路径处理，不回退混用旧时基。
 *     - V21.13保持VC_FALLBACK=10、lead=65deg、PN_N=3、BPN_MAX=12和状态机；slew提高为95deg/s，LOS-rate LPF为10ms。
 * 39. 【V21.12修改】增加完整LOS离散shadow observer（仅诊断，不参与正式控制）：
 *     - 正式IMG miss-rate由三点后向差分升级为6点因果二次最小二乘端点导数；只使用当前/历史帧，不等待未来帧；
 *     - 重捕获初期历史不足6帧时自动回退原三点后向二阶差分，避免额外增加约100ms重新建立时间；
 *     - IMG+GIM+BODY融合后的最终LOS-rate LPF由25ms减到10ms；该组合已用两份实飞日志做capture->now端到端回放验证；
 *     - 完整LOS离散shadow observer直接比较连续两帧完整LOS单位向量：先用gyro历史把上一帧LOS传播到当前Body时刻，再用球面最短旋转求惯性LOS-rate；
 *     - shadow observer不参与正式控制，只做12ms低通并用相同capture->now预测模型生成shadow predicted LOS；
 *     - 新增pn_guidance_observer_diag，50Hz记录formal/shadow LOS-rate、两者夹角、两套predicted LOS、正式/旧三点miss-rate以及估计器模式；
 *     - 本版不改变PN_N、BPN_MAX、速度规划、slew、lead、IMU速度回退、target-loss grace或状态机。
 *
 * 38. 【V21.11修改】增加EKF速度失效时的短时IMU惯导速度回退：
 *     - 订阅vehicle_acceleration（FRD Body，比力/加速度，含重力项），用姿态旋到NED后加g补偿得到惯性加速度；
 *     - EKF三维速度有效时每周期把IMU积分速度重新锚定到EKF速度，防止长期漂移；
 *     - 若起飞前EKF速度已无效，则在未RELEASE/WAIT_RELEASE静止阶段持续以0m/s锚定，RELEASE后再开始积分；
 *     - V22.01固定翼分支将EKF无效后的纯IMU积分上限设为6s；超过时限后Vc与轴向能力平滑退化，避免硬切换；
 *     - 正式Vc投影、动态速度目标taper、正常speed brake和越靶速度门槛优先使用EKF，否则使用短时IMU速度；
 *     - SAFE_HOVER只允许IMU速度参与短时制动，不用IMU积分位置做位置保持；其余状态机、BPN、LOS预测、姿态限制不变。
 *
 ****************************************************************************/

// V22.04: fixed-wing tracking adaptive-BPN-jerk / slew110 derived from V22.03.
// Keeps V22.02 LOS/BPN/thrust shaping, but releases acceleration and attitude authority progressively after RELEASE.
/*
 * V22.03 (2026-11-05) -- fixed-wing tracking / gimbal-protection launch scheduling:
 *  - Keep V22.02: Vt_NED=[0,0,0], 4Hz LOS-rate notch, NED BPN jerk limit=60m/s^3,
 *    final-q vertical-support-priority thrust sync, 6s IMU fallback and smooth velocity-source degradation.
 *  - Add a 1.5s gimbal-protection window measured from accepted RELEASE (not from BPN entry).
 *  - During that window, axial acceleration cap is smoothstep-ramped from 3.5 to the normal 6.5m/s^2.
 *  - Lateral BPN authority is smoothstep-ramped from 0.75 to 1.00 before the existing NED jerk limiter.
 *  - Attitude-setpoint slew cap is smoothstep-ramped from 80 to the normal 105deg/s.
 *    This indirectly limits initial body-rate demand while the stabilized gimbal is acquiring the post-launch motion.
 *  - The existing CLEARANCE/TRANSITION state machine is retained; the protection window spans those states and
 *    naturally continues into early BPN, reaching full authority 1.5s after RELEASE.
 *  - Keep PN_N=3, BPN_ACCEL_MAX=12m/s^2, lead=65deg, max tilt=75deg and thrust budgets unchanged.

 *
 * V22.04 (2026-11-05) -- adaptive BPN jerk + 110deg/s full slew:
 *  - Replace fixed 60m/s^3 BPN jerk limit with LOS-rate adaptive jerk:
 *      <=0.12rad/s: 90m/s^3;
 *      0.12~0.30rad/s: 90->160m/s^3;
 *      0.30~0.50rad/s: 160->240m/s^3;
 *      >=0.50rad/s: 240m/s^3.
 *  - If raw-vs-shaped BPN direction error exceeds 35deg, add up to 25% catch-up boost
 *    by 75deg error; absolute cap 300m/s^3. At 50Hz this allows up to 6.0m/s^2
 *    BPN-vector change per cycle only in severe terminal mismatch.
 *  - Full guidance/clearance slew increases 105->110deg/s. V22.03 1.5s gimbal-protect
 *    ramp remains 80deg/s -> 110deg/s.
 *  - Keep 4Hz LOS-rate notch, 6.5m/s^2 axial cap, vertical-priority thrust sync,
 *    Vt=[0,0,0], EKF/IMU fallback logic, and all safety/state-machine logic unchanged.
*/
#define PN_GUIDANCE_BUILD_TAG "v22.06-gimbal-load-supervisor-yawreset-guard-20261106"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/uORB.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

#include <uORB/topics/vehicle_target_los.h>
#include <uORB/topics/cgtd070_status.h>
#include <uORB/topics/vehicle_guidance_acceleration.h>
#include <uORB/topics/pn_guidance_los_diag.h>
#include <uORB/topics/pn_guidance_guidance_diag.h>
#include <uORB/topics/pn_guidance_control_diag.h>
#include <uORB/topics/pn_guidance_observer_diag.h>
#include <uORB/topics/debug_vect.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

#include <cmath>
// #include <cfloat>
#include <float.h>
#include <cstdint>
#include <cstring>

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

class PnGuidance :
	public ModuleBase<PnGuidance>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	PnGuidance() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) {}

	~PnGuidance() override
	{
		ScheduleClear();
	}

	static int task_spawn(int argc, char *argv[])
	{
		PnGuidance *instance = new PnGuidance();

		if (instance != nullptr) {
			_object.store(instance);
			_task_id = task_id_is_work_queue;

			if (instance->init()) {
				return PX4_OK;
			}

		} else {
			PX4_ERR("alloc failed");
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	static int custom_command(int argc, char *argv[])
	{
		// 一键台架入口：test on=开始50Hz零推力姿态接口测试；test off=退出；test status=查看发布状态。
		if (argc >= 1 && std::strcmp(argv[0], "test") == 0) {
			PnGuidance *instance = _object.load();
			if (instance == nullptr) { PX4_ERR("not running; run 'pn_guidance start' first"); return PX4_ERROR; }
			if (argc < 2) { return print_usage("test usage: pn_guidance test on|off|status"); }
			if (std::strcmp(argv[1], "on") == 0) { return instance->enable_test_mode(); }
			if (std::strcmp(argv[1], "off") == 0) { return instance->disable_test_mode(); }
			if (std::strcmp(argv[1], "status") == 0) { return instance->print_test_mode_status(); }
			return print_usage("test usage: pn_guidance test on|off|status");
		}

		// 固定机械台架三阶段闭环验证：完全独立于LAUNCH/CLEARANCE/自由飞行状态机。
		if (argc >= 1 && std::strcmp(argv[0], "rig") == 0) {
			PnGuidance *instance = _object.load();
			if (instance == nullptr) { PX4_ERR("not running; run 'pn_guidance start' first"); return PX4_ERROR; }
			if (argc < 2) { return print_usage("rig usage: pn_guidance rig stage1|stage2|stage3|off|status"); }
			if (std::strcmp(argv[1], "stage1") == 0) { return instance->set_rig_stage(1); }
			if (std::strcmp(argv[1], "stage2") == 0) { return instance->set_rig_stage(2); }
			if (std::strcmp(argv[1], "stage3") == 0) { return instance->set_rig_stage(3); }
			if (std::strcmp(argv[1], "off") == 0) { return instance->set_rig_stage(0); }
			if (std::strcmp(argv[1], "status") == 0) { return instance->print_rig_status(); }
			return print_usage("rig usage: pn_guidance rig stage1|stage2|stage3|off|status");
		}

		// 以下bench/sp/shadow保留为底层诊断兼容命令；日常台架测试优先使用pn_guidance test on|off|status。
		if (argc >= 1 && std::strcmp(argv[0], "bench") == 0) {
			PnGuidance *instance = _object.load();
			if (instance == nullptr) { PX4_ERR("not running; run 'pn_guidance start' first"); return PX4_ERROR; }
			if (argc == 1 || (argc >= 2 && std::strcmp(argv[1], "once") == 0)) { instance->print_bench_snapshot(hrt_absolute_time()); return PX4_OK; }
			if (std::strcmp(argv[1], "on") == 0) {
				// bench命令在NSH线程执行，而Run()在WorkQueue中运行；使用PX4原子变量跨线程传递开关，避免普通bool数据竞争导致“status看到1但Run()一直看到0”。
				instance->_bench_realtime_enabled.store(true);
				PX4_INFO("BENCH_RT enabled at %.1f Hz; diagnostic only, nothing is published to control outputs", (double)(1e6f / (float)BENCH_PRINT_INTERVAL_US));
				instance->print_bench_snapshot(hrt_absolute_time()); // 立即打印一帧确认输入有效；之后5Hz连续打印只由WorkQueue执行。
				return PX4_OK;
			}
			if (std::strcmp(argv[1], "off") == 0) {
				instance->_bench_realtime_enabled.store(false);
				PX4_INFO("BENCH_RT disabled; WorkQueue printed %u frames", (unsigned)instance->_bench_rt_count.load());
				return PX4_OK;
			}
			return print_usage("bench usage: pn_guidance bench on|off|once");
		}
		if (argc >= 1 && std::strcmp(argv[0], "sp") == 0) {
			PnGuidance *instance = _object.load();
			if (instance == nullptr) { PX4_ERR("not running; run 'pn_guidance start' first"); return PX4_ERROR; }
			if (argc == 1 || (argc >= 2 && std::strcmp(argv[1], "once") == 0)) {
				instance->print_setpoint_snapshot(hrt_absolute_time());
				return PX4_OK;
			}
			if (argc >= 2 && std::strcmp(argv[1], "iface") == 0) {
				if (argc < 3) { return print_usage("sp iface usage: pn_guidance sp iface on|off|status"); }
				// 旧SP_IFACE命令作为TEST模式别名保留，避免已有测试脚本失效。
				if (std::strcmp(argv[2], "on") == 0) { return instance->enable_test_mode(); }
				if (std::strcmp(argv[2], "off") == 0) { return instance->disable_test_mode(); }
				if (std::strcmp(argv[2], "status") == 0) { return instance->print_test_mode_status(); }
				return print_usage("sp iface usage: pn_guidance sp iface on|off|status");
			}
			return print_usage("sp usage: pn_guidance sp once | pn_guidance sp iface on|off|status");
		}
		if (argc >= 1 && std::strcmp(argv[0], "shadow") == 0) {
			PnGuidance *instance = _object.load();
			if (instance == nullptr) { PX4_ERR("not running; run 'pn_guidance start' first"); return PX4_ERROR; }
			if (argc < 2) { return print_usage("shadow usage: pn_guidance shadow on|off|status"); }
			if (std::strcmp(argv[1], "on") == 0) {
				// SHADOW允许人工控制/已解锁场景使用，但绝不发布vehicle_attitude_setpoint或推力，只计算并打印候选结果。
				instance->_sp_interface_test_enabled.store(false);
				instance->_sp_test_anchor_q = instance->current_attitude_q();
				instance->_sp_test_anchor_q.normalize();
				instance->_sp_test_anchor_valid = true;
				instance->_last_q_sp = instance->_sp_test_anchor_q;
				instance->_last_raw_q_sp = instance->_sp_test_anchor_q;
				const Eulerf shadow_euler{instance->_sp_test_anchor_q};
				instance->_yaw_lock = shadow_euler.psi();
				instance->_yaw_locked = true;
				instance->_last_shadow_print_time = 0;
				instance->_shadow_mode_enabled.store(true);
				PX4_INFO("SHADOW enabled: preview budget=%.2f cage=%.1fdeg; NO attitude/thrust publication", (double)BPN_TEST_THRUST_LIMIT, (double)BPN_TEST_ATTITUDE_MAX_DEG);
				return PX4_OK;
			}
			if (std::strcmp(argv[1], "off") == 0) {
				instance->_shadow_mode_enabled.store(false);
				instance->_sp_test_anchor_valid = false;
				PX4_INFO("SHADOW disabled");
				return PX4_OK;
			}
			if (std::strcmp(argv[1], "status") == 0) {
				PX4_INFO("SHADOW enabled=%d armed=%d anchor_valid=%d test_thr=%.2f cage=%.1fdeg publishes_control=0",
					 (int)instance->_shadow_mode_enabled.load(), (int)instance->armed(), (int)instance->_sp_test_anchor_valid,
					 (double)BPN_TEST_THRUST_LIMIT, (double)BPN_TEST_ATTITUDE_MAX_DEG);
				return PX4_OK;
			}
			return print_usage("shadow usage: pn_guidance shadow on|off|status");
		}
		// RELEASE只允许由真实RC AUX1触发；这里仅提供状态查询，故意不提供shell人工发射命令。
		if (argc >= 1 && std::strcmp(argv[0], "release") == 0) {
			PnGuidance *instance = _object.load();
			if (instance == nullptr) { PX4_ERR("not running; run 'pn_guidance start' first"); return PX4_ERROR; }
			if (argc >= 2 && std::strcmp(argv[1], "status") == 0) { return instance->print_release_status(); }
			return print_usage("release usage: pn_guidance release status");
		}

		if (argc >= 1 && std::strcmp(argv[0], "version") == 0) { PX4_INFO("BUILD_TAG=%s", PN_GUIDANCE_BUILD_TAG); return PX4_OK; }
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
### Description
LOS-rate proportional-navigation guidance module.

The module publishes vehicle_attitude_setpoint directly.
)DESCR_STR"
		);

		PRINT_MODULE_USAGE_NAME("pn_guidance", "controller");
		PRINT_MODULE_USAGE_COMMAND_DESCR("test", "One-command bench mode: pn_guidance test on|off|status (disarmed + zero thrust)");
		PRINT_MODULE_USAGE_COMMAND_DESCR("rig", "Fixed-rig 3-stage test: stage1 zero-thrust, stage2 armed low-output, stage3 limited-thrust closed-loop");
		PRINT_MODULE_USAGE_COMMAND_DESCR("bench", "Low-level diagnostics: pn_guidance bench on|off|once");
		PRINT_MODULE_USAGE_COMMAND_DESCR("sp", "Low-level setpoint diagnostics; sp iface is an alias of test mode");
		PRINT_MODULE_USAGE_COMMAND_DESCR("shadow", "Observer mode: preview candidate only, never publishes control outputs");
		PRINT_MODULE_USAGE_COMMAND_DESCR("release", "Show RC AUX1 release interlock status (read-only)");
		PRINT_MODULE_USAGE_COMMAND_DESCR("version", "Print pn_guidance build tag");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(LOOP_INTERVAL_US);
		return true;
	}

	int print_status() override
	{
		PX4_INFO("BUILD_TAG=%s", PN_GUIDANCE_BUILD_TAG);
		const char *stage = "UNKNOWN";
		switch (_state) {
		case GuidanceState::WAIT_LOS: stage = "WAIT_LOS"; break;
		case GuidanceState::LAUNCH: stage = "LAUNCH"; break;
		case GuidanceState::BPN: stage = _launch_sequence_complete ? "BPN_ACTIVE" : "BPN_READY"; break;
		case GuidanceState::SAFE_HOVER: stage = "SAFE_HOVER"; break;
		case GuidanceState::LAUNCH_CLEARANCE: stage = "LAUNCH_CLEARANCE"; break;
		case GuidanceState::TRANSITION: stage = "TRANSITION"; break;
		case GuidanceState::WAIT_RELEASE: stage = "WAIT_RELEASE"; break;
		}
		PX4_INFO("state=%u(%s) launch_complete=%d yaw_lock=%.2f deg", static_cast<unsigned>(_state), stage,
			 (int)_launch_sequence_complete, (double)(_yaw_lock * 180.f / M_PI_F));

		PX4_INFO("LOOP=%.1f Hz", (double)(1e6f / (float)LOOP_INTERVAL_US));
		PX4_INFO("FLIGHT PATH direct_bpn=%d: WAIT_RELEASE -> %s; legacy CLEARANCE/TRANSITION retained but %s",
			 (int)DIRECT_BPN_AFTER_RELEASE,
			 DIRECT_BPN_AFTER_RELEASE ? "BPN_ACTIVE" : "LAUNCH_CLEARANCE",
			 DIRECT_BPN_AFTER_RELEASE ? "BYPASSED" : "ENABLED");
		const uint64_t status_now_rc = hrt_absolute_time();
		PX4_INFO("RC_AUTH override=%d rc_manual_owner=%d nav_base=%s%u nav_now=%u; override clears only on DISARM",
			 (int)_rc_override_latched,
			 (int)rc_manual_attitude_owner_active(status_now_rc),
			 _guidance_release_nav_state_valid ? "" : "invalid/",
			 (unsigned)_guidance_release_nav_state,
			 (unsigned)_vehicle_status.nav_state);
		PX4_INFO("BPN TUNE lateral: PN_N=%.2f BPN_MAX=%.2f m/s^2 flight_gain=%.2f",
			 (double)PN_N, (double)BPN_ACCEL_MAX, (double)FLIGHT_COMMISSIONING_GAIN);
		PX4_INFO("BPN TUNE axial: AXIAL_MAX=%.2f degraded_no_vel=%.2f m/s^2 taper=%.2f m/s speed=%.1f/%.1f/%.1f/%.1f hard=%.1f",
			 (double)AXIAL_ACCEL_MAX, (double)AXIAL_ACCEL_DEGRADED_NO_VEL, (double)SPEED_ACCEL_TAPER_BAND,
			 (double)GUIDANCE_SPEED_CRUISE, (double)GUIDANCE_SPEED_MID_1,
			 (double)GUIDANCE_SPEED_MID_2, (double)GUIDANCE_SPEED_MIN,
			 (double)GUIDANCE_SPEED_HARD_LIMIT);
		PX4_INFO("BPN TUNE thrust: hover=%.3f min=%.3f guidance_limit=%.2f alt_floor_limit=%.2f max=%.2f",
			 (double)HOVER_THRUST, (double)MIN_THRUST,
			 (double)GUIDANCE_THRUST_LIMIT, (double)ALT_FLOOR_THRUST_LIMIT, (double)MAX_THRUST);
		PX4_INFO("V21.7 CLEARANCE hw_hold=%.2fs start_thr=%.2f limit=%.2f; TRANSITION=%.2fs",
			 (double)(HW_CLEARANCE_HOLD_DURATION_US * 1e-6f),
			 (double)CLEARANCE_START_THRUST, (double)CLEARANCE_THRUST_LIMIT,
			 (double)(TRANSITION_DURATION_US * 1e-6f));
		PX4_INFO("ATT slew=%.1fdeg/s lead clearance=%.1f transition=%.1f->%.1f bpn=%.1fdeg max_tilt=%.1fdeg target_grace=%ums",
			 (double)GUIDANCE_ATTITUDE_SLEW_DEG_S,
			 (double)CLEARANCE_ATTITUDE_LEAD_MAX_DEG,
			 (double)TRANSITION_ATTITUDE_LEAD_START_DEG,
			 (double)TRANSITION_ATTITUDE_LEAD_END_DEG,
			 (double)GUIDANCE_ATTITUDE_LEAD_MAX_DEG,
			 (double)GUIDANCE_THRUST_VECTOR_MAX_TILT_DEG,
			 (unsigned)(TARGET_LOSS_GRACE_US / 1000ULL));

		PX4_INFO("PN N=%.2f fixed-wing UNKNOWN Vt: control Vc_est=Va.LOS with EKF/IMU aircraft velocity", (double)PN_N);
		PX4_INFO("FLIGHT commissioning gain=%.2f (TRANSITION/BPN normal BPN only)", (double)FLIGHT_COMMISSIONING_GAIN);
		PX4_INFO("V20 MODE sitl=%d truth_control=%d hw_observer_only=%d control_publish=%d",
			 (int)SIMULATION_MODE, (int)SIM_USE_TRUTH_LOS_CONTROL,
			 (int)HARDWARE_OBSERVER_ONLY, (int)!HARDWARE_OBSERVER_ONLY);
		PX4_INFO("V20 INPUT source=%s miss_angle_direct=%d truth_timeout=%ums",
			 SIMULATION_MODE ? "vehicle_target_los(SITL)" : "cgtd070_status(HW)",
			 (int)LOS_INPUT_HAS_MISS_ANGLE, (unsigned)(TRUTH_LOS_TIMEOUT_US / 1000ULL));
		// 【V21修改】状态页显示100ms capture补偿、miss中心差分额外20ms时基以及gyro/gimbal历史对齐状态。
		PX4_INFO("V21.10 TIME LOSage=%.1fms capture=%.0fms miss_step=%.0fms gyro_err=%+.1fms ginterp=%d gim_err=%+.1fms gim_interp=%d",
			 (double)(_last_los_sample_age_us * 1e-3f), (double)(VISION_FIXED_DELAY_US * 1e-3f),
			 (double)(MISS_RATE_BACKWARD_STEP_US * 1e-3f),
			 (double)(_last_body_rate_align_error_us * 1e-3f), (int)_last_body_rate_interpolated,
			 (double)(_last_gimbal_align_error_us * 1e-3f), (int)_last_gimbal_interpolated);

		PX4_INFO("LOS GEOM Camera->Body: Cx->+By Cy->-Bx Cz->-Bz; Guidance: Gx=-Cy Gy=Cx Gz=Cz; G->B: Gx->Bx Gy->By Gz->-Bz");
		// 【V21修改】真机正式IMG rate来源改为PX4 direct-miss中心差分；RK3576 black-box rate仅ULog诊断。
		PX4_INFO("LOS RATE HW=PX4-miss-poly6/back3 valid=%d mode=%u board_rate=diag_only body_rate_fresh=%d",
			 (int)_diag_miss_rate_valid, (unsigned)_diag_miss_rate_mode, (int)body_rate_valid(hrt_absolute_time()));
		// V21.12: IMG formal rate为poly6因果端点拟合（bootstrap时back3）；GIM保持capture±40ms中心差分；最终rate LPF=10ms。
		PX4_INFO("V22.02 RATE fusion=IMGmissPoly6+GIMcenter+omegaXLOS gim_half=%.0fms tau=%.0fms + notch4Hz",
			 (double)(GIMBAL_RATE_CENTER_HALF_WINDOW_US * 1e-3f),
			 (double)(INERTIAL_LOS_RATE_LPF_TAU * 1000.f));
		PX4_INFO("V21.13 PRED valid=%d age=%.1fms extra_lead=%ums horizon=%.1fms angle=%.2fdeg age_max=%ums body_hist=%u",
			 (int)_los_prediction_valid, (double)(_los_prediction_age_us * 1e-3f),
			 (unsigned)(_last_control_extra_lead_us / 1000ULL),
			 (double)((_los_prediction_age_us + _last_control_extra_lead_us) * 1e-3f),
			 (double)_los_prediction_angle_deg, (unsigned)(LOS_PREDICT_MAX_US / 1000ULL),
			 (unsigned)BODY_RATE_HISTORY_LEN);
		const uint64_t status_now = hrt_absolute_time();
		const float last_cmd_age_ms = (_last_valid_guidance_cmd_time > 0 && status_now >= _last_valid_guidance_cmd_time)
			? (status_now - _last_valid_guidance_cmd_time) * 1e-3f : 0.f;
		PX4_INFO("LOS REACQUIRE active=%d grace=%ums att_sync_dt=%.2fms last_cmd=%d age=%.1fms",
			 (int)_los_reacquire_active, (unsigned)(LOS_REACQUIRE_GRACE_US / 1000),
			 (double)(_last_los_attitude_sync_dt_s * 1000.f), (int)_last_valid_guidance_cmd_valid,
			 (double)last_cmd_age_ms);
		PX4_INFO("V20 OBS dt=%.1fms bodyRot=%.2fdeg metrics_n=%u RMS=%.5f MAX=%.5f RESET100_MAX=%.5f dir_err=%.1fdeg",
			 (double)(_last_los_recon_dt_s * 1000.f), (double)_last_v19_body_rotation_deg,
			 (unsigned)_v19_metric_count, (double)_v19_error_rms, (double)_v19_error_max,
			 (double)_v19_reset100_error_max, (double)_v19_direction_error_deg);
		if (HARDWARE_OBSERVER_ONLY) {
			PX4_INFO("HW SAFE MODE: observer-only; pn_guidance never arms vehicle and never publishes attitude/thrust control");
			PX4_INFO("HW RC FLOW preview=%d: Commander ARM -> WAIT_RELEASE; AUX1 LOW->HIGH -> RELEASE PREVIEW only",
				 (int)HARDWARE_RELEASE_PREVIEW_ONLY);
		}

		PX4_INFO("Vc fallback=%.1f clamp=0.0..%.1f LPF rise=%.2f fall=%.2f s",
			 (double)VC_FALLBACK,
			 (double)VC_MAX,
			 (double)VC_LPF_TAU_RISE,
			 (double)VC_LPF_TAU_FALL);
		PX4_INFO("V22.02 FIXED-WING target_vel=UNKNOWN comp=%d Vt_NED=[0 0 0]; Vc_est=Va.LOS",
			 (int)TARGET_VELOCITY_COMPENSATION_ENABLED);
		PX4_INFO("V22.04 shape: LOS_notch=%d f0=%.1fHz Q=%.1f adaptive_jerk=%.0f/%.0f/%.0f cap=%.0fm/s^3 current=%.1f dir_err=%.1fdeg full_slew=%.1fdeg/s",
			 (int)LOS_RATE_NOTCH_ENABLED, (double)LOS_RATE_NOTCH_FREQ_HZ, (double)LOS_RATE_NOTCH_Q,
			 (double)BPN_JERK_LOW, (double)BPN_JERK_MID, (double)BPN_JERK_HIGH, (double)BPN_JERK_ABS_MAX,
			 (double)_last_bpn_jerk_limit, (double)_last_bpn_jerk_dir_error_deg,
			 (double)GUIDANCE_ATTITUDE_SLEW_DEG_S);
		PX4_INFO("V22.05 terminal: lead=%lums adaptive_slew=%.1fdeg/s centering=%.2fm/s^2 lead_cap=%lums slew_cap=%.1fdeg/s",
			 (unsigned long)(_last_control_extra_lead_us / 1000ULL), (double)_last_adaptive_slew_deg_s,
			 (double)_last_centering_assist_norm, (unsigned long)(LOS_CONTROL_EXTRA_LEAD_MAX_US / 1000ULL),
			 (double)GUIDANCE_ATTITUDE_SLEW_HIGH_DEG_S);
		PX4_INFO("V22.06 GIMBAL_LOAD state=%u risk=%.2f geo=%.2f dyn=%.2f ctrl=%.2f offaxis=%.1fdeg rate=%.1fdeg/s accel=%.0fdeg/s2 caps axial=%.2f slew=%.1f lead=%.1f",
			 (unsigned)_gimbal_load_state, (double)_gimbal_load, (double)_gimbal_geometry_load,
			 (double)_gimbal_dynamic_load, (double)_gimbal_control_load, (double)_gimbal_offaxis_deg,
			 (double)_gimbal_rate_mag_deg_s, (double)_gimbal_accel_mag_filtered_deg_s2,
			 (double)_gimbal_supervisor_axial_cap, (double)_gimbal_supervisor_slew_cap_deg_s,
			 (double)_gimbal_supervisor_lead_cap_deg);
		PX4_INFO("V22.06 ATT_RESET_GUARD active=%d last_angle=%.1fdeg remaining=%.0fms; recommended EKF2_MAG_TYPE=6 for HW test",
			 (int)attitude_reset_guard_active(status_now), (double)_last_attitude_reset_angle_deg,
			 (double)(attitude_reset_guard_active(status_now) ? (_attitude_reset_guard_until - status_now) * 1e-3f : 0.f));
		PX4_INFO("V22.04 GIMBAL_PROTECT window=%.2fs phase=%.2f axial_cap=%.2f bpn_scale=%.2f slew_cap=%.1fdeg/s",
			 (double)(GIMBAL_PROTECT_DURATION_US * 1e-6f), (double)_last_gimbal_protect_phase,
			 (double)_last_gimbal_protect_axial_cap, (double)_last_gimbal_protect_bpn_scale,
			 (double)_last_gimbal_protect_slew_deg_s);
		PX4_INFO("V22.04 thrust_sync=FINAL_Q vertical-priority; axial=%.1f degraded=%.1f; vc_blend=%.2fs axial_blend=%.2fs",
			 (double)AXIAL_ACCEL_MAX, (double)AXIAL_ACCEL_DEGRADED_NO_VEL,
			 (double)VC_FALLBACK_BLEND_TAU, (double)AXIAL_DEGRADED_BLEND_TAU);
		PX4_INFO("VEL SRC=%s EKF_valid=%d IMU_valid=%d IMU=%.2fm/s fallback_max=%.1fs accel_age=%.1fms",
			 guidance_velocity_source_name(), (int)velocity_valid(), (int)imu_velocity_available(),
			 (double)(imu_velocity_available() ? _imu_velocity_ned.norm() : NAN),
			 (double)(IMU_VEL_FALLBACK_MAX_US * 1e-6f),
			 (double)((_last_accel_rx_time > 0 && status_now >= _last_accel_rx_time) ? (status_now - _last_accel_rx_time) * 1e-3f : -1.f));
		PX4_INFO("V21.14 EKF VEL GUARD active=%d blend=%d alpha=%.2f diff=%.2f hold=%.2fs blend_t=%.2fs max=%.2fs jump=%.1fm/s reacq=%.1fm/s",
			 (int)_ekf_velocity_guard_active, (int)_ekf_velocity_blend_active,
			 (double)_ekf_velocity_blend_alpha, (double)_last_ekf_imu_velocity_diff,
			 (double)(EKF_RESET_IMU_HOLD_US * 1e-6f),
			 (double)(EKF_RESET_BLEND_BACK_US * 1e-6f),
			 (double)(EKF_RESET_GUARD_MAX_US * 1e-6f),
			 (double)EKF_VEL_JUMP_TRIGGER_MPS, (double)EKF_REACQUIRE_MAX_DIFF_MPS);

		PX4_INFO("EKF VEL GUARD force_reacquire_diff<=%.1fm/s; larger mismatch stays on IMU/fixed fallback",
			 (double)EKF_REACQUIRE_FORCE_MAX_DIFF_MPS);

		PX4_INFO("BPN_ACCEL_MAX=%.2f m/s^2", (double)BPN_ACCEL_MAX);

		PX4_INFO("GUIDANCE SPEED cruise=%.1f hard=%.1f m/s",
			 (double)GUIDANCE_SPEED_CRUISE,
			 (double)GUIDANCE_SPEED_HARD_LIMIT);

		PX4_INFO("TERMINAL SPEED los_rate %.2f/%.2f/%.2f/%.2f rad/s -> %.1f/%.1f/%.1f/%.1f m/s",
			 (double)LOS_RATE_SPEED_POINT_1,
			 (double)LOS_RATE_SPEED_POINT_2,
			 (double)LOS_RATE_SPEED_POINT_3,
			 (double)LOS_RATE_SPEED_POINT_4,
			 (double)GUIDANCE_SPEED_CRUISE,
			 (double)GUIDANCE_SPEED_MID_1,
			 (double)GUIDANCE_SPEED_MID_2,
			 (double)GUIDANCE_SPEED_MIN);

		PX4_INFO("SPEED brake kp=%.1f max=%.1f, hard_start=%.1f hard_kp=%.1f hard_max=%.1f",
			 (double)SPEED_BRAKE_KP,
			 (double)SPEED_BRAKE_ACCEL_MAX,
			 (double)HARD_SPEED_BRAKE_START,
			 (double)HARD_SPEED_BRAKE_KP,
			 (double)HARD_SPEED_BRAKE_ACCEL_MAX);

		PX4_INFO("GUIDANCE thrust budget=%.2f clearance=%.2f physical_max=%.2f", (double)GUIDANCE_THRUST_LIMIT, (double)CLEARANCE_THRUST_LIMIT, (double)MAX_THRUST);
		PX4_INFO("TEST limits: preview_budget=%.2f attitude_cage=%.1f deg real_thrust=0 (DISARMED only)",
			 (double)BPN_TEST_THRUST_LIMIT, (double)BPN_TEST_ATTITUDE_MAX_DEG);

		PX4_INFO("ALT floor soft=%.2f hard=%.2f m up_max=%.1f m/s^2",
			 (double)ALT_FLOOR_SOFT_DROP,
			 (double)ALT_FLOOR_HARD_DROP,
			 (double)ALT_FLOOR_UP_ACCEL_MAX);

		PX4_INFO("MISS trigger: arm Vc>%.1f, then raw Vc<%.1f for %.2f s",
			 (double)MISS_ARM_VC,
			 (double)MISS_TRIGGER_VC,
			 (double)(MISS_TRIGGER_HOLD_US * 1e-6f));

		PX4_INFO("SAFE_HOVER brake_xy=%.1f brake_z=%.1f m/s^2",
			 (double)SAFE_BRAKE_ACCEL_XY_MAX,
			 (double)SAFE_BRAKE_ACCEL_Z_MAX);

		PX4_INFO("BENCH realtime=%d rate=%.1f Hz count=%u command='pn_guidance bench on|off|once'",
			 (int)_bench_realtime_enabled.load(), (double)(1e6f / (float)BENCH_PRINT_INTERVAL_US), (unsigned)_bench_rt_count.load());

		PX4_INFO("TEST enabled=%d (one-command disarmed zero-thrust attitude-setpoint test)", (int)_sp_interface_test_enabled.load());
		PX4_INFO("SHADOW enabled=%d (observer only, no control publication)", (int)_shadow_mode_enabled.load());
		PX4_INFO("RIG stage=%u active=%d vc_test=%.1f cage=%.1fdeg s2_thr=%.2f s3_max=%.2f",
			 (unsigned)_rig_stage.load(), (int)_rig_release_latched, (double)RIG_VC_TEST,
			 (double)RIG_ATTITUDE_CAGE_DEG, (double)RIG_STAGE2_THRUST, (double)RIG_STAGE3_THRUST_MAX);
		PX4_INFO("ATT_SP uORB calls=%u ok=%u fail=%u last_ok=%d advertised=%d orb_exists=%d",
			 (unsigned)_att_sp_pub_calls.load(), (unsigned)_att_sp_pub_ok.load(),
			 (unsigned)_att_sp_pub_fail.load(), (int)_att_sp_last_ok.load(),
			 (int)_attitude_setpoint_pub.advertised(),
			 (int)(orb_exists(ORB_ID(vehicle_attitude_setpoint), 0) == PX4_OK));

		// Reuse status_now declared above in this print_status() scope.
		PX4_INFO("RELEASE mode=%s low<=%.2f high>=%.2f confirm=%ums", SIMULATION_MODE ? "SIMULATION" : "REAL_RC",
			 (double)RC_RELEASE_LOW_THRESHOLD, (double)RC_RELEASE_HIGH_THRESHOLD, (unsigned)(RC_RELEASE_CONFIRM_US / 1000ULL));
		if (SIMULATION_MODE) {
			PX4_INFO("SIM release: aux1=%+.2f guards=%d ready_hold=%ums low_seen=%d high_pending=%d",
				 (double)_sim_release_aux1, (int)simulation_release_guards_ready(status_now),
				 (unsigned)(SIM_RELEASE_READY_HOLD_US / 1000ULL), (int)_release_low_seen, (int)_release_high_pending);
		} else {
			PX4_INFO("RC release: fresh_valid=%d source=%u aux1=%+.3f low_seen=%d high_pending=%d timeout=%ums",
				 (int)rc_release_input_valid(status_now), (unsigned)_manual_control.data_source, (double)_manual_control.aux1,
				 (int)_release_low_seen, (int)_release_high_pending, (unsigned)(RC_RELEASE_TIMEOUT_US / 1000ULL));
		}

		// status最后附带一帧台架快照；与连续BENCH_RT使用完全相同的计算路径，且只打印、不发布控制指令。
		print_bench_snapshot(hrt_absolute_time());
		return 0;
	}

private:
	// ============================================================
	// 状态机
	// ============================================================

	enum class GuidanceState : uint8_t
	{
		WAIT_LOS         = 0,
		LAUNCH           = 1,
		BPN              = 2,
		SAFE_HOVER       = 3,
		LAUNCH_CLEARANCE = 4,
		TRANSITION       = 5,
		WAIT_RELEASE     = 6
	};

	// V22.06: supervisory state derived from gimbal off-axis angle, rate and angular acceleration.
	// It is diagnostic/coordination state only; target_valid still comes from the vision/gimbal board.
	enum class GimbalLoadState : uint8_t
	{
		NORMAL   = 0,
		CAUTION  = 1,
		HIGH     = 2,
		CRITICAL = 3
	};

	// ============================================================
	// 基本周期
	// ============================================================

	static constexpr uint32_t LOOP_INTERVAL_US = 20000; // 50 Hz
	static constexpr uint64_t LOS_TIMEOUT_US = 200000;
	static constexpr uint64_t BODY_RATE_TIMEOUT_US = 100000; // V16用于健康检查、BENCH诊断以及短时姿态/LOS时间对齐
	static constexpr float LOS_RECON_DT_MIN = 0.005f;
	static constexpr float LOS_RECON_DT_MAX = 0.100f;
	// ============================================================
	// V19 平台自动模式
	//
	// SITL:
	//   - TRUTH只用于闭环控制，V20 Body-relative observer只做shadow A/B；
	//   - Python发送真实camera miss angle，先单独验证observer的相对旋转/时间一致性。
	//
	// V6X/其它实机:
	//   - HARDWARE_OBSERVER_ONLY=true：非RIG时绝不发布姿态/推力控制；
	//   - 真机LOS来源为cgtd070_status，使用板卡直接给出的raw rate + gimbal角 + miss angle + target_valid；
	//   - pn_guidance入口完成deg/deg/s -> rad/rad/s和协议符号统一；
	//   - direct miss angle存在，因此真机observer不再依赖raw rate长期积分估计miss。
	// ============================================================
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	static constexpr bool SIMULATION_MODE = true;
	static constexpr bool SIM_USE_TRUTH_LOS_CONTROL = true;
	static constexpr bool HARDWARE_OBSERVER_ONLY = false;
	static constexpr bool LOS_INPUT_HAS_MISS_ANGLE = true;
#else
	static constexpr bool SIMULATION_MODE = false;
	static constexpr bool SIM_USE_TRUTH_LOS_CONTROL = false;
	// static constexpr bool HARDWARE_OBSERVER_ONLY = true;
	static constexpr bool HARDWARE_OBSERVER_ONLY = false;
	static constexpr bool LOS_INPUT_HAS_MISS_ANGLE = true;
#endif

	// CGTD070 V2.12 -> pn_guidance内部统一语义。
	// Cgtd070Status单位：角度deg，角速度deg/s；pn_guidance内部统一rad/rad/s。
	static constexpr float CGTD070_DEG_TO_RAD = M_PI_F / 180.f;
	// 协议51~54：LOS rate左/下为负、右/上为正，与pn_guidance一致。
	static constexpr float CGTD070_LOS_AZ_RATE_SIGN = 1.f;
	static constexpr float CGTD070_LOS_EL_RATE_SIGN = 1.f;
	// 当前云台角约定：方位向右为正、俯仰向上为正，与pn_guidance一致。
	static constexpr float CGTD070_GIMBAL_AZ_SIGN = 1.f;
	static constexpr float CGTD070_GIMBAL_PITCH_SIGN = 1.f;
	// 协议59~62：miss方位左正右负、miss俯仰下正上负；pn_guidance统一为右正、上正，因此反号。
	static constexpr float CGTD070_MISS_AZ_SIGN = -1.f;
	static constexpr float CGTD070_MISS_EL_SIGN = -1.f;
	// 【V21修改】真机板卡los_az_rate/el_rate仅用于诊断；正式IMG角速度由direct miss angle历史在PX4内重构。

	// ============================================================
	// V20.2 真机RC两步流程预览
	//
	// ARM:
	//   仍完全交给PX4 Commander / RC_MAP_ARM_SW。
	//   pn_guidance只读取vehicle_status的armed状态。
	//
	// RELEASE:
	//   使用RC_MAP_AUX1 -> manual_control_setpoint.aux1。
	//   真机仍执行LOW->HIGH + 100ms互锁以及释放前guard检查，
	//   但只锁存“RELEASE PREVIEW ACCEPTED”，不进入真实推力/姿态发射控制。
	//
	// armed-idle由PX4/ESC标准机制负责，不由本模块主动给推力。
	// ============================================================
	// static constexpr bool HARDWARE_RELEASE_PREVIEW_ONLY = true;
	static constexpr bool HARDWARE_RELEASE_PREVIEW_ONLY = false;

	static constexpr bool SIM_KEEP_RELEASE_ATTITUDE_CLEARANCE = false;
	static constexpr uint64_t TRUTH_LOS_TIMEOUT_US = 120000; // SITL truth 120 ms
	static constexpr uint64_t V20_RESET_METRIC_WINDOW_US = 100000; // EKF reset后100ms误差单独统计
	static constexpr float V20_METRIC_TRUTH_RATE_MAX = 0.50f; // 排除极近距离/越靶LOS几何奇异区
	// V21.10: 64帧body-rate历史，为约100~120ms视觉预测提供足够旋转传播余量。
	static constexpr uint8_t BODY_RATE_HISTORY_LEN = 64;
	static constexpr uint64_t BODY_RATE_INTERP_MAX_GAP_US = 60000;
	static constexpr uint64_t BODY_RATE_NEAREST_MAX_US = 30000;

	// 【V20.9修改】真机已确认视觉capture到PX4完整收包约100ms；只在真机CGTD070入口使用，SITL保持原timestamp语义。
	static constexpr uint64_t VISION_FIXED_DELAY_US = 100000ULL;
	// 【V20.9修改】建立云台角历史缓存；50Hz输入下32帧约覆盖640ms，足够回查100ms前的capture时刻。
	static constexpr uint8_t GIMBAL_HISTORY_LEN = 32;
	static constexpr uint64_t GIMBAL_INTERP_MAX_GAP_US = 60000ULL;
	static constexpr uint64_t GIMBAL_NEAREST_MAX_US = 30000ULL;
	// 【V20.9修改】云台正式角速度用capture时刻前后各40ms的中心差分；80ms窗口兼顾量化噪声与动态相位。
	static constexpr uint64_t GIMBAL_RATE_CENTER_HALF_WINDOW_US = 40000ULL;

	// 【V21修改】保存direct miss angle历史；50Hz下32帧约640ms，足够完成40ms中心差分和时间插值。
	static constexpr uint8_t MISS_HISTORY_LEN = 32;
	// 【V21修改】miss angle历史插值允许的最大相邻样本间隔与最近样本回退范围。
	static constexpr uint64_t MISS_INTERP_MAX_GAP_US = 60000ULL;
	static constexpr uint64_t MISS_NEAREST_MAX_US = 30000ULL;
	// V21.10: direct miss仍以约20ms帧周期保存；正式IMG rate改为capture时刻三点后向二阶差分，
	// 不再为了中心差分把observer额外退后20ms。该常量保留给诊断字段/旧工具兼容。
	static constexpr uint64_t MISS_RATE_CENTER_HALF_WINDOW_US = 20000ULL;
	static constexpr uint64_t MISS_RATE_BACKWARD_STEP_US = 20000ULL;
	static constexpr uint8_t MISS_RATE_POLYFIT_POINTS = 6;

	// V21.10: 只补偿已确认的视觉/observer历史时间，不允许对陈旧数据无限外推。
	static constexpr uint64_t LOS_PREDICT_MAX_US = 170000ULL; // V21.13: allow nominal ~100-115ms age + ~2 missed 50Hz frames
	// V22.05: after reconstructing LOS from capture time to "now", lead it a little further to account for
	// the measured downstream attitude/rate/airframe response. This is dynamic, not a blind fixed 100ms lead.
	static constexpr uint64_t LOS_CONTROL_EXTRA_LEAD_MIN_US = 35000ULL; // 35ms in calm geometry
	static constexpr uint64_t LOS_CONTROL_EXTRA_LEAD_MID_US = 55000ULL; // ~55ms moderate LOS dynamics
	static constexpr uint64_t LOS_CONTROL_EXTRA_LEAD_MAX_US = 80000ULL; // 80ms terminal cap
	static constexpr uint64_t LOS_PREDICT_TOTAL_MAX_US = 250000ULL;     // age + extra lead hard cap
	static constexpr float LOS_CONTROL_LEAD_RATE_P1 = 0.10f; // rad/s
	static constexpr float LOS_CONTROL_LEAD_RATE_P2 = 0.25f; // rad/s
	static constexpr float LOS_CONTROL_LEAD_RATE_P3 = 0.45f; // rad/s
	static constexpr float LOS_CONTROL_LEAD_ATT_ERR_START_DEG = 6.0f;
	static constexpr float LOS_CONTROL_LEAD_ATT_ERR_FULL_DEG = 18.0f;
	static constexpr uint64_t LOS_CONTROL_LEAD_ATT_BOOST_MAX_US = 15000ULL; // +15ms when attitude is lagging
	static constexpr uint64_t LOS_PREDICT_INTEGRATION_STEP_US = 5000ULL;

	// V21.12 shadow-only full-LOS discrete observer. It never feeds BPN in this build.
	static constexpr float SHADOW_LOS_RATE_LPF_TAU = 0.012f;
	static constexpr float SHADOW_LOS_MIN_ANGLE_RAD = 1e-6f;

	static constexpr float LOS_MISS_EST_LIMIT_RAD = 0.34906585f; // +/-20 deg
	static constexpr uint64_t LOS_REACQUIRE_GRACE_US = 120000; // EKF姿态reset后允许LOS observer最多120ms重新建立差分基准
	static constexpr uint64_t TARGET_LOSS_GRACE_US = 250000; // V21.9.1 target/control LOS短暂掉锁宽限250ms
	static constexpr uint64_t LOS_ATTITUDE_SYNC_MAX_US = 40000; // 最多用body rate补偿+/-40ms姿态/LOS采样时差
	// V21.12: poly6降低IMG微分噪声后，将image+gimbal+body融合后的最终低通缩短为10ms。
	static constexpr float INERTIAL_LOS_RATE_LPF_TAU = 0.010f; // 保持低延迟主LPF

	// V22.02: fixed-wing实飞中formal LOS-rate/BPN/collective出现约4Hz窄带振荡。
	// 在10ms主LPF之后追加一个窄带notch，避免用重低通增加整体PN相位滞后。
	// 系数按50Hz制导循环设计；Q=2.5仅压制约3~5Hz附近，不主动削弱低频真实目标机动。
	static constexpr bool LOS_RATE_NOTCH_ENABLED = true;
	static constexpr float LOS_RATE_NOTCH_FREQ_HZ = 4.0f;
	static constexpr float LOS_RATE_NOTCH_Q = 2.5f;
	static constexpr float LOS_RATE_NOTCH_FS_HZ = 50.0f;

	static constexpr uint64_t LAUNCH_DURATION_US = 100000;

	// V20.7：所有连续PX4_INFO诊断统一限制为5 Hz，主制导循环仍保持50 Hz不变。
	static constexpr uint64_t DIAG_PRINT_INTERVAL_US = 200000; // 200 ms = 5 Hz
	// BENCH_RT只用于台架诊断，默认关闭并通过“pn_guidance bench on”显式开启。
	static constexpr uint64_t BENCH_PRINT_INTERVAL_US = DIAG_PRINT_INTERVAL_US;

	// ============================================================
	// V21.5 正式飞行入口选择（A/B回退版）
	// true：AUX1 RELEASE通过后直接进入BPN ACTIVE。
	// false：WAIT_RELEASE -> LAUNCH_CLEARANCE -> TRANSITION -> BPN。
	// 当前设为false，用于恢复缓冲阶段并与direct-BPN版本做A/B对照。
	// ============================================================
	static constexpr bool DIRECT_BPN_AFTER_RELEASE = false;

	// ============================================================
	// Rotor-only launch clearance（V21.5正式路径重新启用）
	// ============================================================

	// 原CLEARANCE ramp/hold保留给SITL旧A/B路径；V21.7真机不再使用其“拉向水平”逻辑。
	static constexpr uint64_t CLEARANCE_RAMP_DURATION_US = 100000; // SITL legacy: 0.10 s
	static constexpr uint64_t CLEARANCE_HOLD_DURATION_US = 100000; // SITL legacy: 0.10 s

	// V21.7真机：RELEASE后固定保持RELEASE瞬间姿态100ms，再进入TRANSITION。
	static constexpr uint64_t HW_CLEARANCE_HOLD_DURATION_US = 100000; // 0.10 s

	static constexpr float CLEARANCE_START_THRUST = 0.25f; // 保持本次实飞实际参数
	static constexpr float CLEARANCE_THRUST_LIMIT = 0.30f; // 保持本次实飞实际参数
	static constexpr float CLEARANCE_FORWARD_ACCEL = 0.35f; // 保持本次实飞实际参数；V21.7真机hold路径不使用

	// ============================================================
	// CLEARANCE -> BPN transition
	// ============================================================
	// V21.7: 真机RELEASE accepted -> 0.10 s hold + 0.30 s transition = 0.40 s；
	// 加上既有AUX1 HIGH 0.10 s确认，物理拨杆HIGH -> full BPN约0.50 s。
	static constexpr uint64_t TRANSITION_DURATION_US = 300000;

	// WAIT_RELEASE阶段guidance本身不给主动推力。
	// 真机armed-idle由PX4输出/ESC机制负责。
	static constexpr float WAIT_RELEASE_THRUST = 0.0f;

	// ========================================================================
	// >>>>>>>>>>>>>>> V21.3 航拍跟随 / BPN 主要调参区 <<<<<<<<<<<<<<<<
	//
	// 法向BPN：
	//   a_BPN = PN_N * Vc * LOS_rate，并由BPN_ACCEL_MAX限幅。
	//   PN_N                  ：比例导航系数，越大法向修正越积极。
	//   BPN_ACCEL_MAX         ：法向BPN加速度上限 [m/s^2]。
	//   FLIGHT_COMMISSIONING_GAIN：正式飞行法向BPN额外缩放；>=0.999时当前实现不再缩放。
	//
	// 轴向靠近：
	//   AXIAL_ACCEL_MAX       ：沿LOS方向用于快速靠近目标的最大轴向加速度 [m/s^2]；
	//                           实际axial_used还会受速度目标、SPEED_ACCEL_TAPER_BAND和推力预算限制。
	//
	// 推力标定：
	//   HOVER_THRUST          ：5.5寸桨实测悬停归一化推力，本版=0.182。
	//
	// 其余与快速靠近直接相关的Vc、速度目标、speed brake、thrust limit参数紧随本区之后。
	// ========================================================================

	static constexpr float GRAVITY = 9.80665f;
	static constexpr float HOVER_THRUST = 0.182f;

	// 【轴向加速度】沿LOS快速靠近目标的最大净加速度。
	// V22.02: 按固定翼跟踪需求提高约30%%：5.0->6.5m/s^2；速度无效降级值同步3.0->3.9m/s^2。
	static constexpr float AXIAL_ACCEL_MAX = 6.5f;
	static constexpr float AXIAL_ACCEL_DEGRADED_NO_VEL = 3.9f;

	// V22.04: BPN法向加速度变化率限制改为LOS-rate自适应jerk。
	// 仍在NED固定坐标系限制矢量delta，避免Body轴旋转本身被误判为jerk。
	// 50Hz下：90/160/240m/s^3分别对应每周期约1.8/3.2/4.8m/s^2；
	// raw-vs-shaped方向严重失配时最高允许300m/s^3（6.0m/s^2/周期）快速追赶。
	static constexpr float BPN_JERK_LOW = 90.0f;   // m/s^3
	static constexpr float BPN_JERK_MID = 160.0f;  // m/s^3
	static constexpr float BPN_JERK_HIGH = 240.0f; // m/s^3
	static constexpr float BPN_JERK_ABS_MAX = 300.0f; // m/s^3

	static constexpr float BPN_JERK_LOS_RATE_P1 = 0.12f; // rad/s
	static constexpr float BPN_JERK_LOS_RATE_P2 = 0.30f; // rad/s
	static constexpr float BPN_JERK_LOS_RATE_P3 = 0.50f; // rad/s

	static constexpr float BPN_JERK_DIR_BOOST_START_DEG = 35.0f;
	static constexpr float BPN_JERK_DIR_BOOST_FULL_DEG = 75.0f;
	static constexpr float BPN_JERK_DIR_BOOST_MAX = 1.25f;

	// ============================================================
	// V22.03 云台保护型起飞动态释放
	// 从RELEASE真正接受时刻开始计时；CLEARANCE/TRANSITION也占用这1.5s窗口。
	// 目的：起飞初期不要同时给满轴向、满BPN权限和110deg/s姿态命令，
	// 降低机体50~100deg/s快速摆动迫使稳定云台高速反补偿而掉锁的概率。
	// ============================================================
	static constexpr uint64_t GIMBAL_PROTECT_DURATION_US = 1500000ULL; // 1.5 s
	static constexpr float GIMBAL_PROTECT_AXIAL_START = 3.5f;          // m/s^2 -> 6.5
	static constexpr float GIMBAL_PROTECT_BPN_SCALE_START = 0.75f;     // 75% -> 100%
	static constexpr float GIMBAL_PROTECT_SLEW_START_DEG_S = 80.0f;    // deg/s -> 110

	// ============================================================
	// V22.06 continuous gimbal-load supervisor
	//
	// Safe region: no intervention.  When the two-axis gimbal is both far off-axis and/or moving hard,
	// progressively reduce forward acceleration and attitude-command aggressiveness while preserving
	// lateral BPN authority.  Thresholds are software protection-envelope seeds from recent flight logs,
	// not claimed hardware limits; they should be calibrated from further logs.
	// ============================================================
	static constexpr float GIMBAL_LOAD_OFFAXIS_SAFE_DEG = 30.0f;
	static constexpr float GIMBAL_LOAD_OFFAXIS_CRITICAL_DEG = 50.0f;
	static constexpr float GIMBAL_LOAD_RATE_SAFE_DEG_S = 70.0f;
	static constexpr float GIMBAL_LOAD_RATE_CRITICAL_DEG_S = 110.0f;
	static constexpr float GIMBAL_LOAD_ACCEL_SAFE_DEG_S2 = 600.0f;
	static constexpr float GIMBAL_LOAD_ACCEL_CRITICAL_DEG_S2 = 1200.0f;
	static constexpr float GIMBAL_LOAD_ACCEL_LPF_TAU = 0.08f;
	static constexpr float GIMBAL_LOAD_LPF_TAU = 0.12f;
	static constexpr float GIMBAL_LOAD_NO_ACTION = 0.25f;

	// Supervisor outputs.  Normal/safe load leaves V22.05 behaviour unchanged.
	static constexpr float GIMBAL_SUPERVISOR_AXIAL_MIN = 3.0f;
	static constexpr float GIMBAL_SUPERVISOR_SLEW_MIN_DEG_S = 90.0f;
	static constexpr float GIMBAL_SUPERVISOR_LEAD_HIGH_DEG = 18.0f;
	static constexpr float GIMBAL_SUPERVISOR_LEAD_CRITICAL_DEG = 7.0f;

	// Hysteresis for diagnostic state labels.  Continuous control scheduling uses the filtered load directly.
	static constexpr float GIMBAL_LOAD_CAUTION_ENTER = 0.30f;
	static constexpr float GIMBAL_LOAD_CAUTION_EXIT = 0.18f;
	static constexpr float GIMBAL_LOAD_HIGH_ENTER = 0.60f;
	static constexpr float GIMBAL_LOAD_HIGH_EXIT = 0.45f;
	static constexpr float GIMBAL_LOAD_CRITICAL_ENTER = 0.85f;
	static constexpr float GIMBAL_LOAD_CRITICAL_EXIT = 0.70f;

	// EKF attitude/yaw-reset transient guard.  A reset changes the estimated reference instantly although
	// the airframe did not physically rotate.  Re-anchor quaternion setpoints as before, reset the NED BPN
	// jerk state, and temporarily reduce dynamic demand so the correction is not amplified into a real motion.
	static constexpr float ATT_RESET_GUARD_TRIGGER_DEG = 5.0f;
	static constexpr uint64_t ATT_RESET_GUARD_DURATION_US = 180000ULL;
	static constexpr float ATT_RESET_GUARD_AXIAL_CAP = 3.0f;
	static constexpr float ATT_RESET_GUARD_SLEW_CAP_DEG_S = 85.0f;
	static constexpr float ATT_RESET_GUARD_LEAD_CAP_DEG = 7.0f;

	// 【法向BPN】比例导航系数 N。
	static constexpr float PN_N = 3.0f;
	// 【法向BPN】正式飞行commissioning缩放。当前1.0，正式BPN不再额外缩小。
	// RIG stage1/2/3不使用此参数；1.0f代表完整正式BPN权限。
	// static constexpr float FLIGHT_COMMISSIONING_GAIN = 0.25f;
	static constexpr float FLIGHT_COMMISSIONING_GAIN = 1.0f;

	// 【法向BPN】法向加速度绝对上限 [m/s^2]。
	// static constexpr float BPN_ACCEL_MAX = 12.0f;
	// static constexpr float BPN_ACCEL_MAX = 4.0f;
	static constexpr float BPN_ACCEL_MAX = 12.0f;

	// ============================================================
	// 【BPN调参】闭合速度 Vc（直接决定法向BPN幅值 PN_N * Vc * LOS_rate）
	// ============================================================

	static constexpr float VC_FALLBACK = 10.0f;
	static constexpr float VC_MAX = 25.0f;

	// V22.01 fixed-wing tracking: target velocity is unknown, therefore do NOT guess Vt.
	// Equivalent target model for control is Vt_NED=[0,0,0], so Vc_est = Va_NED dot LOS_NED.
	// The former balloon-only [0,0,-3]m/s compensation is explicitly disabled.
	static constexpr bool TARGET_VELOCITY_COMPENSATION_ENABLED = false;
	static constexpr float TARGET_CLIMB_SPEED_MPS = 0.0f; // compatibility field; inactive when compensation=false
	// static constexpr float VC_FALLBACK = 20.0f;
	// static constexpr float VC_MAX = 120.0f;
	static constexpr float VC_LPF_TAU_RISE = 0.12f;
	static constexpr float VC_LPF_TAU_FALL = 0.03f;
	// V22.01: when all velocity sources are unavailable, blend toward fixed fallback rather than step to it.
	static constexpr float VC_FALLBACK_BLEND_TAU = 0.35f;

	// ============================================================
	// V21.11 EKF速度失效时的短时IMU惯导速度回退
	// vehicle_acceleration.xyz为FRD Body下bias-corrected acceleration（含重力效应）。
	// 惯性NED加速度 = R_nb * accel_body + [0,0,g]。
	// 只作为短时速度参考，绝不作为长期导航解。
	// ============================================================
	static constexpr uint64_t IMU_ACCEL_TIMEOUT_US = 100000ULL;
	static constexpr uint64_t IMU_VEL_FALLBACK_MAX_US = 6000000ULL; // V22.01: 最多6s纯IMU积分
	static constexpr float IMU_ACCEL_LPF_TAU = 0.040f;              // 40ms，抑制振动积分漂移
	static constexpr float IMU_ACCEL_MAX_NORM = 35.0f;              // m/s^2，异常脉冲限幅
	static constexpr float IMU_VELOCITY_MAX = 35.0f;                // m/s，防积分发散
	static constexpr float IMU_DT_MIN = 0.001f;
	static constexpr float IMU_DT_MAX = 0.080f;
	// V22.01: EKF/IMU速度不可用时，轴向能力从当前值平滑退到3m/s^2，避免5->3瞬时台阶。
	static constexpr float AXIAL_DEGRADED_BLEND_TAU = 0.35f;

	// ============================================================
	// V21.14 EKF速度reset保护：reset/单帧大跳变后，短时保持连续的IMU惯导速度，
	// 防止EKF新参考系速度瞬间跳变直接把Vc翻负、让BPN在终端段清零。
	// HOLD结束后仅当EKF与IMU重新接近时才开始平滑回切；到GUARD_MAX后仅放宽到FORCE阈值，
	// 若差异仍很大则继续保持IMU/固定fallback，绝不强行吃入明显跳变的EKF速度。
	// ============================================================
	static constexpr uint64_t EKF_RESET_IMU_HOLD_US = 800000ULL;       // reset后至少0.8s纯IMU速度
	static constexpr uint64_t EKF_RESET_BLEND_BACK_US = 400000ULL;     // EKF恢复后0.4s平滑融合回EKF
	static constexpr uint64_t EKF_RESET_GUARD_MAX_US = 1500000ULL;     // 最长1.5s等待EKF重新一致
	static constexpr uint64_t EKF_VEL_JUMP_MAX_DT_US = 100000ULL;      // 仅检测100ms内的瞬时速度跳变
	static constexpr float EKF_VEL_JUMP_TRIGGER_MPS = 3.0f;            // 单次更新|dV|>3m/s触发保护
	static constexpr float EKF_REACQUIRE_MAX_DIFF_MPS = 2.5f;          // EKF/IMU速度差<=2.5m/s允许正常回切
	static constexpr float EKF_REACQUIRE_FORCE_MAX_DIFF_MPS = 5.0f;    // 超过GUARD_MAX后也仅在差<=5m/s时允许强制回切

	// ============================================================
	// 【BPN调参】轴向速度目标 / 快速靠近速度管理
	//
	// <=0.05rad/s -> 25m/s
	//  0.05~0.15  -> 25 -> 24m/s
	//  0.15~0.35  -> 24 -> 22m/s
	//  0.35~0.60  -> 22 -> 20m/s
	// >=0.60      -> 20m/s
	//
	// 28m/s为当前名义硬速度保护上限；26m/s开始硬制动。
	// ============================================================

	// Stable:
	// static constexpr float GUIDANCE_SPEED_CRUISE = 10.0f;
	// static constexpr float GUIDANCE_SPEED_MID_1 = 9.0f;
	// static constexpr float GUIDANCE_SPEED_MID_2 = 8.0f;
	// static constexpr float GUIDANCE_SPEED_MIN = 6.0f;
	// static constexpr float GUIDANCE_SPEED_HARD_LIMIT = 12.0f;

	static constexpr float GUIDANCE_SPEED_CRUISE = 25.0f;
	static constexpr float GUIDANCE_SPEED_MID_1 = 24.0f;
	static constexpr float GUIDANCE_SPEED_MID_2 = 22.0f;
	static constexpr float GUIDANCE_SPEED_MIN = 20.0f;
	static constexpr float GUIDANCE_SPEED_HARD_LIMIT = 28.0f;

	// NOW:
	// static constexpr float GUIDANCE_SPEED_CRUISE = 120.0f;
	// static constexpr float GUIDANCE_SPEED_MID_1 = 110.0f;
	// static constexpr float GUIDANCE_SPEED_MID_2 = 105.0f;
	// static constexpr float GUIDANCE_SPEED_MIN = 105.0f;
	// static constexpr float GUIDANCE_SPEED_HARD_LIMIT = 104.0f;

	// static constexpr float GUIDANCE_SPEED_CRUISE = 120.0f;
	// static constexpr float GUIDANCE_SPEED_MID_1 = 80.0f;
	// static constexpr float GUIDANCE_SPEED_MID_2 = 65.0f;
	// static constexpr float GUIDANCE_SPEED_MIN = 65.0f;
	// static constexpr float GUIDANCE_SPEED_HARD_LIMIT = 121.0f;



	static constexpr float LOS_RATE_SPEED_POINT_1 = 0.05f;
	static constexpr float LOS_RATE_SPEED_POINT_2 = 0.15f;
	static constexpr float LOS_RATE_SPEED_POINT_3 = 0.35f;
	// V21.3：消除0.35rad/s处13->6m/s阶跃；0.35~0.60rad/s连续降到GUIDANCE_SPEED_MIN。
	static constexpr float LOS_RATE_SPEED_POINT_4 = 0.60f;

	// 在目标速度前4m/s开始逐渐收掉轴向加速。
	static constexpr float SPEED_ACCEL_TAPER_BAND = 4.0f;
	// static constexpr float SPEED_ACCEL_TAPER_BAND = 5.0f;

	// 一旦超过动态目标速度就开始主动制动。
	static constexpr float SPEED_BRAKE_DEADBAND = 0.0f;
	static constexpr float SPEED_BRAKE_KP = 3.0f;
	static constexpr float SPEED_BRAKE_ACCEL_MAX = 6.0f;

	// 26m/s开始渐进硬制动；达到/超过28m/s名义硬上限时直接使用最大硬制动。
	static constexpr float HARD_SPEED_BRAKE_START = 26.0f;
	static constexpr float HARD_SPEED_BRAKE_KP = 4.0f;
	static constexpr float HARD_SPEED_BRAKE_ACCEL_MAX = 8.0f;

	// static constexpr float HARD_SPEED_BRAKE_START = 121.0f;
	// static constexpr float HARD_SPEED_BRAKE_KP = 5.0f;
	// static constexpr float HARD_SPEED_BRAKE_ACCEL_MAX = 10.0f;

	// static constexpr float HARD_SPEED_BRAKE_START = 85.0f;
	// static constexpr float HARD_SPEED_BRAKE_KP = 5.0f;
	// static constexpr float HARD_SPEED_BRAKE_ACCEL_MAX = 10.0f;

	// ============================================================
	// 【BPN调参】总推力预算上限
	// ============================================================

	static constexpr float GUIDANCE_THRUST_LIMIT = 0.50f;
	// V21.3：高度保护介入时绝不能把可用推力预算从正常BPN的0.50反向降低。
	// 当前先保持与正常BPN相同的0.50预算；后续若日志证明需要更多掉高恢复余量，可再单独提高。
	static constexpr float ALT_FLOOR_THRUST_LIMIT = GUIDANCE_THRUST_LIMIT;

	// TEST/SP_IFACE只用于未解锁零真实推力测试：候选计算沿用正常BPN的当前推力预算，避免低预算提前把BPN裁掉；真正发布的thrust始终为0。
	static constexpr float BPN_TEST_THRUST_LIMIT = GUIDANCE_THRUST_LIMIT;
	// 台架姿态笼：以执行“pn_guidance test on”瞬间姿态为中心，最终CMD最多偏离15deg，便于直观看清目标移动对应的Roll/Pitch方向。
	static constexpr float BPN_TEST_ATTITUDE_MAX_DEG = 15.0f;
	static constexpr float BPN_TEST_ATTITUDE_MAX_RAD = BPN_TEST_ATTITUDE_MAX_DEG * M_PI_F / 180.f;

	// ============================================================
	// 固定机械台架三阶段测试参数
	// ============================================================
	// 固定台架没有真实闭合速度，因此只在RIG路径使用一个明确的测试Vc，
	// 用来验证LOS->BPN的符号、比例和动态；不反馈到正常制导路径。
	static constexpr float RIG_VC_TEST = 20.0f;
	// RIG只保留较小BPN用于姿态台架，RAW仍打印完整BPN，USED按此限幅。
	static constexpr float RIG_BPN_ACCEL_MAX = 3.0f;
	// 最终姿态命令相对启用瞬间机械台架锚点的三维夹角上限。
	static constexpr float RIG_ATTITUDE_CAGE_DEG = 12.0f;
	static constexpr float RIG_ATTITUDE_CAGE_RAD = RIG_ATTITUDE_CAGE_DEG * M_PI_F / 180.f;
	static constexpr float RIG_ATTITUDE_ABORT_DEG = 16.0f;
	static constexpr float RIG_ATTITUDE_ABORT_RAD = RIG_ATTITUDE_ABORT_DEG * M_PI_F / 180.f;
	// 在正式guidance 60deg/s之外，再套一层更慢的台架指令变化限制。
	static constexpr float RIG_ATTITUDE_SLEW_DEG_S = 25.0f;
	static constexpr float RIG_ATTITUDE_SLEW_RAD_S = RIG_ATTITUDE_SLEW_DEG_S * M_PI_F / 180.f;
	// Stage2：无桨/测功台，仅给足以看到控制分配差动的低集体推力。
	static constexpr float RIG_STAGE2_THRUST = 0.12f;
	// 【V21.1修改】Stage3固定机械台架专用BPN缩放：只把Stage3法向BPN降到20%，
	// 用于减弱1m近距离目标+相机偏心导致的台架视差闭环；正式飞行、Stage1、Stage2完全不使用该系数。
	static constexpr float RIG_STAGE3_BPN_SCALE = 0.10f;
	// Stage3：真实BPN控制链只在最终发布端限制推力；姿态/加速度候选仍按正式BPN 0.75预算计算。
	static constexpr float RIG_STAGE3_THRUST_MAX = 0.20f;
	// 软件监测限值；硬件急停/总断能必须独立保留。
	static constexpr float RIG_BODY_RATE_ABORT_RAD_S = 2.0f;
	static constexpr float RIG_MOTOR_OUTPUT_ABORT = 0.50f;
	static constexpr uint64_t RIG_PRINT_INTERVAL_US = DIAG_PRINT_INTERVAL_US; // V20.7: RIG print rate 5Hz

	// 5.5寸桨当前实测悬停约0.182；MIN_THRUST=0.10低于悬停值，不会把悬停推力夹高。
	static constexpr float MIN_THRUST = 0.10f;
	static constexpr float MAX_THRUST = 0.95f;

	// 发射初段允许姿态目标快速变化
	static constexpr float CLEARANCE_ATTITUDE_SLEW_DEG_S = 110.0f;
	static constexpr float CLEARANCE_ATTITUDE_SLEW_RAD_S = CLEARANCE_ATTITUDE_SLEW_DEG_S * M_PI_F / 180.f;

	// Guidance姿态setpoint限制,正常BPN继续保持比较平滑
	static constexpr float GUIDANCE_ATTITUDE_SLEW_DEG_S = 110.0f;
	static constexpr float GUIDANCE_ATTITUDE_SLEW_RAD_S = GUIDANCE_ATTITUDE_SLEW_DEG_S * M_PI_F / 180.f;

	// V22.05: only terminal/high-LOS-rate geometry is allowed to exceed the nominal 110deg/s.
	static constexpr float GUIDANCE_ATTITUDE_SLEW_MID_DEG_S = 123.0f;
	static constexpr float GUIDANCE_ATTITUDE_SLEW_HIGH_DEG_S = 135.0f;
	static constexpr float GUIDANCE_SLEW_LOS_RATE_P1 = 0.15f; // rad/s
	static constexpr float GUIDANCE_SLEW_LOS_RATE_P2 = 0.30f; // rad/s
	static constexpr float GUIDANCE_SLEW_LOS_RATE_P3 = 0.50f; // rad/s

	// V22.05 weak camera-boresight centering assist. It is intentionally much smaller than BPN_MAX.
	// This term is not a replacement for PN: it only reduces accumulated camera miss during rapid terminal geometry.
	static constexpr float CENTERING_ASSIST_GAIN = 10.0f;       // m/s^2 per rad tangent error
	static constexpr float CENTERING_ASSIST_MAX = 1.20f;        // m/s^2
	static constexpr float CENTERING_ASSIST_START_DEG = 0.50f;  // deadband
	static constexpr float CENTERING_ASSIST_FULL_DEG = 3.50f;
	static constexpr float CENTERING_ASSIST_MIN_LOS_RATE = 0.10f; // rad/s


	static constexpr float GUIDANCE_ATTITUDE_LEAD_MAX_DEG = 65.0f;
	static constexpr float GUIDANCE_ATTITUDE_LEAD_MAX_RAD = GUIDANCE_ATTITUDE_LEAD_MAX_DEG * M_PI_F / 180.f;

	// V21.14 推力矢量可行域：正推力四旋翼不允许acceleration_to_attitude()
	// 穿过90deg进入倒飞分支。将期望Body +Z限制在NED +Z周围75deg圆锥内。
	// 这不是普通姿态lead/slew，而是上游“物理可实现性”约束。
	static constexpr float GUIDANCE_THRUST_VECTOR_MAX_TILT_DEG = 75.0f;
	static constexpr float GUIDANCE_THRUST_VECTOR_MAX_TILT_RAD = GUIDANCE_THRUST_VECTOR_MAX_TILT_DEG * M_PI_F / 180.f;

	// V21.7过渡期专用lead限制；正式BPN使用GUIDANCE_ATTITUDE_LEAD_MAX_DEG（当前65deg）。
	static constexpr float CLEARANCE_ATTITUDE_LEAD_MAX_DEG = 8.0f;
	static constexpr float TRANSITION_ATTITUDE_LEAD_START_DEG = 8.0f;
	static constexpr float TRANSITION_ATTITUDE_LEAD_END_DEG = 10.0f;

	// ============================================================
	// RELEASE高度底线保护
	// PX4 NED：z增加 = 向下
	// CLEARANCE不再固定施加向上5m/s^2。只有掉高或预测会继续掉高时，这个保护才介入。
	// ============================================================

	// ============================================================
	// RELEASE高度单边保护
	// ============================================================
	// static constexpr float ALT_FLOOR_KP = 6.0f;
	// static constexpr float ALT_FLOOR_KD = 2.5f;
	static constexpr float ALT_FLOOR_KP = 15.0f;
	static constexpr float ALT_FLOOR_KD = 2.5f;

	// 实际掉高15cm以内基本认为正常。stable：0.15f;0.10f
	static constexpr float ALT_FLOOR_SOFT_DROP = 0.10f;

	// 根据当前下降速度预测最终会掉超过20cm时提前介入。stable：0.30f
	static constexpr float ALT_FLOOR_PREDICT_TRIGGER = 0.15f;

	// 希望尽量在掉高50cm附近把下降速度刹住。stable：0.50f
	static constexpr float ALT_FLOOR_HARD_DROP = 0.25f;

	// 不再使用原来的“恢复release高度”位置P,改成对“预测越界量”的P。stable：6
	static constexpr float ALT_FLOOR_PRED_KP = 9.0f;

	// 对向下速度的阻尼。stable：2.5f
	static constexpr float ALT_FLOOR_VZ_KD = 3.5f;

	// 下降速度已经接近0时立即退出高度保护
	static constexpr float ALT_FLOOR_RELEASE_VZ = 0.05f;

	// 原来4.0，先提高到6.0
	static constexpr float ALT_FLOOR_UP_ACCEL_MAX = 6.0f;

	// 理论制动加速度上的余量
	static constexpr float ALT_FLOOR_ACCEL_MARGIN = 0.8f;
	// ============================================================
	// 越靶判定
	// ============================================================

	static constexpr float MISS_ARM_VC = 8.0f;
	static constexpr float MISS_TRIGGER_VC = -3.0f;
	static constexpr float MISS_MIN_SPEED = 5.0f;
	static constexpr uint64_t MISS_TRIGGER_HOLD_US = 120000;

	// ============================================================
	// SAFE_HOVER
	// ============================================================

	static constexpr float SAFE_BRAKE_KV_XY = 1.00f;
	static constexpr float SAFE_BRAKE_KV_Z = 1.20f;

	static constexpr float SAFE_BRAKE_ACCEL_XY_MAX = 8.0f;
	static constexpr float SAFE_BRAKE_ACCEL_Z_MAX = 4.5f;

	static constexpr float SAFE_LATCH_SPEED_XY = 1.0f;
	static constexpr float SAFE_LATCH_SPEED_Z = 0.5f;
	static constexpr uint64_t SAFE_LATCH_HOLD_US = 500000;

	static constexpr float SAFE_HOLD_KP_XY = 0.8f;
	static constexpr float SAFE_HOLD_KD_XY = 1.4f;
	static constexpr float SAFE_HOLD_KP_Z = 1.0f;
	static constexpr float SAFE_HOLD_KD_Z = 1.5f;

	static constexpr float SAFE_HOLD_ACCEL_XY_MAX = 4.0f;
	static constexpr float SAFE_HOLD_ACCEL_Z_MAX = 3.5f;

	// ============================================================
	// RELEASE模式 + RC安全互锁
	// ============================================================
	// 真机推荐把遥控器独立两段开关通过RC_MAP_AUX1映射到AUX1：LOW约-1，HIGH约+1。
	// 无论仿真还是真机，都保留“先LOW，再HIGH连续100ms”的互锁语义，禁止单个HIGH电平直接触发。
	static constexpr float RC_RELEASE_LOW_THRESHOLD = -0.50f;
	static constexpr float RC_RELEASE_HIGH_THRESHOLD = 0.50f;
	static constexpr uint64_t RC_RELEASE_CONFIRM_US = 100000; // HIGH连续100ms才接受RELEASE
	static constexpr uint64_t RC_RELEASE_TIMEOUT_US = 500000; // 真机RC消息超过500ms未更新视为失效
	static constexpr uint64_t RC_CONTROL_MODE_TIMEOUT_US = 500000; // V21.4: 模式状态超过500ms不参与RC所有权判定

	// 仿真虚拟拨杆参数：进入WAIT_RELEASE后先保持LOW；只有释放条件连续稳定300ms，才把虚拟AUX1拨到HIGH。
	// 这样不是“ARM后立刻发射”，而是尽量复现真机操作者确认条件后再拨RELEASE开关的顺序。
	static constexpr uint64_t SIM_RELEASE_READY_HOLD_US = 300000;
	static constexpr float SIM_RELEASE_AUX_LOW = -1.0f;
	static constexpr float SIM_RELEASE_AUX_HIGH = 1.0f;

	// ============================================================
	// uORB
	// ============================================================

	// SITL仍使用vehicle_target_los；真机直接使用CGTD070驱动发布的cgtd070_status。
	uORB::Subscription _los_sub{ORB_ID(vehicle_target_los)};
	uORB::Subscription _cgtd070_status_sub{ORB_ID(cgtd070_status)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _truth_los_sub{ORB_ID(debug_vect)};

	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_guidance_acceleration_s> _guidance_accel_pub{ORB_ID(vehicle_guidance_acceleration)};
	uORB::Publication<pn_guidance_los_diag_s> _pn_guidance_los_diag_pub{ORB_ID(pn_guidance_los_diag)};
	uORB::Publication<pn_guidance_guidance_diag_s> _pn_guidance_guidance_diag_pub{ORB_ID(pn_guidance_guidance_diag)};
	uORB::Publication<pn_guidance_control_diag_s> _pn_guidance_control_diag_pub{ORB_ID(pn_guidance_control_diag)};
	uORB::Publication<pn_guidance_observer_diag_s> _pn_guidance_observer_diag_pub{ORB_ID(pn_guidance_observer_diag)};

	// ============================================================
	// 消息缓存
	// ============================================================

	// _los是pn_guidance内部统一LOS缓存：SITL直接拷贝vehicle_target_los，真机由Cgtd070Status转换填充。
	vehicle_target_los_s _los{};
	cgtd070_status_s _cgtd070_status{};
	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_acceleration_s _vehicle_acceleration{};
	vehicle_local_position_s _local_position{};
	vehicle_status_s _vehicle_status{};
	manual_control_setpoint_s _manual_control{};
	vehicle_rates_setpoint_s _vehicle_rates_setpoint{};
	actuator_motors_s _actuator_motors{};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_torque_setpoint_s _vehicle_torque_setpoint{};
	vehicle_thrust_setpoint_s _vehicle_thrust_setpoint{};
	debug_vect_s _truth_los_msg{};

	// V18 Gazebo真值旁路，仅SITL A/B使用。
	Vector3f _truth_los_dot_body{0.f, 0.f, 0.f};
	bool _truth_los_valid{false};
	uint64_t _last_truth_los_rx_time{0};

	// 同帧A/B诊断缓存。
	float _last_truth_los_rate_mag{0.f};
	float _last_recon_los_rate_mag{0.f};
	float _last_truth_recon_error_mag{0.f};
	Vector3f _last_bpn_truth_body_raw{0.f, 0.f, 0.f};
	Vector3f _last_bpn_recon_body_raw{0.f, 0.f, 0.f};

	// ============================================================
	// 状态
	// ============================================================

	GuidanceState _state{GuidanceState::WAIT_LOS};

	uint64_t _state_enter_time{0};
	uint64_t _last_los_rx_time{0};
	uint64_t _last_body_rate_rx_time{0};

	// 【V21修改】惯性LOS-rate结构仍为IMG + gimbal motion + body gyro；真机IMG已改为PX4根据direct miss历史自行重构。
	// 不再把完整LOS的20ms单帧位置差分作为BPN主角速度来源，以降低真机云台角/脱靶角量化噪声。
	bool _los_recon_initialized{false};
	bool _inertial_los_rate_valid{false};
	uint64_t _prev_los_recon_time{0};
	float _last_los_recon_dt_s{0.f};
	float _los_miss_az_est{0.f}; // 由RK3576 raw az rate积分得到的相机残余方位角估计
	float _los_miss_el_est{0.f}; // 由RK3576 raw el rate积分得到的相机残余俯仰角估计
	Vector3f _prev_los_ned{0.f, 0.f, 0.f}; // V19仅BENCH兼容保留，不参与observer控制
	Vector3f _prev_los_body{0.f, 0.f, -1.f};
	Vector3f _prev_body_rate_at_los{0.f, 0.f, 0.f};
	bool _prev_body_rate_at_los_valid{false};

	struct BodyRateHistorySample {
		uint64_t t{0};
		Vector3f omega{0.f, 0.f, 0.f};
	};
	BodyRateHistorySample _body_rate_history[BODY_RATE_HISTORY_LEN]{};
	uint8_t _body_rate_history_head{0};
	uint8_t _body_rate_history_count{0};
	int64_t _last_body_rate_align_error_us{0};
	bool _last_body_rate_interpolated{false};

	// 【V20.9修改】缓存“每个CGTD070包在RX时刻携带的最新云台角”，用于回查图像capture时刻对应的云台姿态。
	struct GimbalHistorySample {
		uint64_t t{0};
		float azimuth{0.f};
		float pitch{0.f};
	};
	GimbalHistorySample _gimbal_history[GIMBAL_HISTORY_LEN]{};
	uint8_t _gimbal_history_head{0};
	uint8_t _gimbal_history_count{0};
	int64_t _last_gimbal_align_error_us{0};
	bool _last_gimbal_interpolated{false};

	// 【V21修改】保存按capture_time标记的direct miss angle历史；正式IMG角速度由该历史在PX4内自行重构。
	struct MissHistorySample {
		uint64_t t{0};
		float azimuth{0.f};
		float elevation{0.f};
	};
	MissHistorySample _miss_history[MISS_HISTORY_LEN]{};
	uint8_t _miss_history_head{0};
	uint8_t _miss_history_count{0};

	uint64_t _last_los_sample_time{0};
	uint64_t _last_los_sample_age_us{0};
	bool _last_los_time_fallback{true};

	float _last_v19_body_rotation_deg{0.f};
	uint64_t _last_attitude_reset_time{0};
	uint64_t _attitude_reset_guard_until{0};
	float _last_attitude_reset_angle_deg{0.f};
	uint64_t _last_v19_metric_los_rx_time{0};
	uint32_t _v19_metric_count{0};
	float _v19_error_sq_sum{0.f};
	float _v19_error_rms{0.f};
	float _v19_error_max{0.f};
	float _v19_reset100_error_max{0.f};
	float _v19_direction_error_deg{0.f};
	bool _v20_metric_active{false};
	uint32_t _v20_metric_rejected_high_rate{0};
	uint64_t _last_hw_observer_print_time{0};
	float _prev_gimbal_azimuth{0.f};
	float _prev_gimbal_pitch{0.f};
	// 【V20.9修改】字段名为兼容现有ULog保持filtered不变，但正式语义已改为“capture时刻80ms中心差分得到的云台角速度”；单位rad/s。
	float _gimbal_az_rate_filtered{0.f};
	float _gimbal_pitch_rate_filtered{0.f};
	bool _gimbal_rate_filter_initialized{false};

	// V22.06 gimbal-load supervisor state.
	uint64_t _gimbal_load_last_los_rx_time{0};
	uint64_t _gimbal_load_last_update_time{0};
	float _gimbal_load_prev_az_rate{0.f};
	float _gimbal_load_prev_pitch_rate{0.f};
	bool _gimbal_load_rate_initialized{false};
	float _gimbal_offaxis_deg{0.f};
	float _gimbal_rate_mag_deg_s{0.f};
	float _gimbal_accel_mag_deg_s2{0.f};
	float _gimbal_accel_mag_filtered_deg_s2{0.f};
	float _gimbal_geometry_load{0.f};
	float _gimbal_dynamic_load{0.f};
	float _gimbal_control_load{0.f};
	float _gimbal_load_raw{0.f};
	float _gimbal_load{0.f};
	GimbalLoadState _gimbal_load_state{GimbalLoadState::NORMAL};
	float _gimbal_supervisor_axial_cap{AXIAL_ACCEL_MAX};
	float _gimbal_supervisor_slew_cap_deg_s{GUIDANCE_ATTITUDE_SLEW_HIGH_DEG_S};
	float _gimbal_supervisor_lead_cap_deg{GUIDANCE_ATTITUDE_LEAD_MAX_DEG};
	// 【V20.5修改】三项现在直接参与惯性LOS-rate融合；fused_raw保留低通前结果用于BENCH/HW诊断。
	Vector3f _los_dot_image_body{0.f, 0.f, 0.f};
	Vector3f _los_dot_gimbal_body{0.f, 0.f, 0.f};
	Vector3f _los_dot_body_motion{0.f, 0.f, 0.f};
	Vector3f _los_dot_fused_raw_body{0.f, 0.f, 0.f};
	// V22.02 keeps LPF state separate from the final notched formal LOS-rate used by BPN.
	Vector3f _los_dot_inertial_lpf_body{0.f, 0.f, 0.f};
	Vector3f _los_dot_inertial_body{0.f, 0.f, 0.f};
	bool _los_rate_filter_initialized{false};
	bool _los_rate_notch_initialized{false};
	Vector3f _los_rate_notch_x1{0.f, 0.f, 0.f};
	Vector3f _los_rate_notch_x2{0.f, 0.f, 0.f};
	Vector3f _los_rate_notch_y1{0.f, 0.f, 0.f};
	Vector3f _los_rate_notch_y2{0.f, 0.f, 0.f};

	// V21.10 current-time LOS prediction diagnostics/state.
	Vector3f _los_control_body{0.f, 0.f, -1.f};
	Vector3f _los_dot_control_body{0.f, 0.f, 0.f};
	bool _los_prediction_valid{false};
	uint64_t _los_prediction_age_us{0};
	float _los_prediction_angle_deg{0.f};

	// V21.12 complete-LOS discrete observer shadow state. Diagnostic only.
	bool _shadow_los_initialized{false};
	bool _shadow_los_rate_valid{false};
	bool _shadow_los_rate_filter_initialized{false};
	uint64_t _shadow_prev_sample_time{0};
	float _shadow_dt_s{0.f};
	float _shadow_residual_angle_deg{0.f};
	float _shadow_filter_alpha{0.f};
	Vector3f _shadow_prev_los_body{0.f, 0.f, -1.f};
	Vector3f _shadow_los_dot_raw_body{0.f, 0.f, 0.f};
	Vector3f _shadow_los_dot_filt_body{0.f, 0.f, 0.f};
	bool _shadow_prediction_valid{false};
	uint64_t _shadow_prediction_age_us{0};
	float _shadow_prediction_angle_deg{0.f};
	Vector3f _shadow_los_pred_body{0.f, 0.f, -1.f};
	Vector3f _shadow_los_dot_pred_body{0.f, 0.f, 0.f};
	float _shadow_formal_rate_angle_deg{0.f};
	float _shadow_formal_pred_angle_deg{0.f};

	// ============================================================
	// 【V20.8新增】PnGuidanceDiag 50Hz ULog专用缓存
	// 这些变量只复制现有计算链中的中间量，禁止反向参与任何控制计算。
	// ============================================================
	uint32_t _diag_seq{0};

	// 预留给未来RK3576真实图像capture timestamp/age/frame sequence。
	uint64_t _diag_capture_time{0};
	uint32_t _diag_capture_age_us{0};
	uint32_t _diag_vision_frame_seq{0};
	bool _diag_capture_time_valid{false};

	// 【V21修改】板卡黑盒LOS-rate仅保留诊断；正式IMG使用PX4根据miss历史重构的rate。
	float _diag_board_los_az_rate{0.f};
	float _diag_board_los_el_rate{0.f};
	float _diag_miss_az_rate_px4{0.f};
	float _diag_miss_el_rate_px4{0.f};
	float _diag_miss_az_rate_backward3{0.f};
	float _diag_miss_el_rate_backward3{0.f};
	uint8_t _diag_miss_rate_mode{0}; // 0 invalid, 1 backward3 bootstrap, 2 causal poly6
	bool _diag_miss_rate_valid{false};

	float _diag_gimbal_delta_az{0.f};
	float _diag_gimbal_delta_pitch{0.f};
	float _diag_gimbal_az_rate_raw{0.f};
	float _diag_gimbal_pitch_rate_raw{0.f};

	Vector3f _diag_los_body{0.f, 0.f, -1.f};
	Vector3f _diag_omega_aligned{0.f, 0.f, 0.f};

	Vector3f _diag_bpn_raw_body{0.f, 0.f, 0.f};
	Vector3f _diag_bpn_used_body{0.f, 0.f, 0.f};
	Vector3f _diag_accel_cmd_ned{0.f, 0.f, 0.f};

	Quatf _diag_q_raw{1.f, 0.f, 0.f, 0.f};
	Quatf _diag_q_slew{1.f, 0.f, 0.f, 0.f};
	Quatf _diag_q_cmd{1.f, 0.f, 0.f, 0.f};

	float _diag_thrust_preview{0.f};
	float _diag_thrust_out{0.f};

	bool _diag_guidance_valid{false};
	bool _diag_control_published{false};
	bool _diag_slew_limited{false};
	bool _diag_lead_limited{false};
	bool _diag_cage_limited{false};
	bool _los_reacquire_active{false};
	uint64_t _los_reacquire_start_time{0};
	bool _los_reacquire_hold_logged{false};

	uint64_t _target_loss_start_time{0};
	bool _target_loss_hold_logged{false};

	float _last_los_attitude_sync_dt_s{0.f};
	bool _last_valid_guidance_cmd_valid{false};
	uint64_t _last_valid_guidance_cmd_time{0};
	Vector3f _last_valid_guidance_accel_ned{0.f, 0.f, 0.f};
	Vector3f _last_valid_guidance_bpn_body{0.f, 0.f, 0.f};

	// 最近一次manual_control_setpoint更新时间；只用于RC RELEASE输入新鲜度判断。
	uint64_t _last_manual_control_rx_time{0};

	uint64_t _last_rates_setpoint_rx_time{0};
	uint64_t _last_actuator_motors_rx_time{0};
	uint64_t _last_control_mode_rx_time{0};
	uint64_t _last_torque_setpoint_rx_time{0};

	// V21.4 RC绝对优先级仲裁：一旦正式制导后发生飞行模式接管，锁存到本次DISARM。
	bool _rc_override_latched{false};
	bool _guidance_release_nav_state_valid{false};
	uint8_t _guidance_release_nav_state{0};
	uint64_t _last_rc_override_warn_time{0};
	uint64_t _last_thrust_setpoint_rx_time{0};

	// RC RELEASE互锁状态全部由Run()工作队列维护。
	// low_seen：ARM进入WAIT_RELEASE以后是否已经确认过一次安全低位。
	// high_pending：已经从LOW进入HIGH，正在累计100ms确认时间。
	bool _release_low_seen{false};
	bool _release_high_pending{false};
	bool _release_high_block_warned{false};
	uint64_t _release_high_start_time{0};

	// 真机两步RC流程预览状态：只用于台架/无桨验证，不触发真实发射控制。
	bool _hardware_release_preview_latched{false};
	uint64_t _hardware_release_preview_time{0};

	// 仿真虚拟AUX1状态。仅SIMULATION_MODE=true时参与RELEASE；真机模式完全忽略。
	float _sim_release_aux1{SIM_RELEASE_AUX_LOW};
	uint64_t _sim_release_ready_start_time{0};
	bool _sim_release_ready_logged{false};

	uint64_t _last_print_time{0};
	uint64_t _last_los_rate_print_time{0};
	uint64_t _last_setpoint_pub_time{0};
	// bench命令来自NSH线程，Run()来自nav_and_controllers WorkQueue，因此该开关必须是原子的；它只控制调试打印，不参与制导计算。
	px4::atomic_bool _bench_realtime_enabled{false};
	px4::atomic<uint32_t> _bench_rt_count{0}; // WorkQueue每成功输出一帧BENCH_RT就+1，用于确认连续打印路径是否真正运行。
	// 一键TEST与旧SP_IFACE共用该开关：50Hz发布候选q_cmd到vehicle_attitude_setpoint，但推力始终强制为0。
	px4::atomic_bool _sp_interface_test_enabled{false};
	uint64_t _last_test_print_time{0}; // TEST模式每5Hz打印一行关键结果，移动目标时无需反复执行sp once。
	// TEST姿态笼锚点：test on瞬间捕获真实姿态，候选q_cmd最多偏离该姿态BPN_TEST_ATTITUDE_MAX_DEG。
	Quatf _sp_test_anchor_q{1.f, 0.f, 0.f, 0.f};
	bool _sp_test_anchor_valid{false};
	// SHADOW为只读观察模式：人工/常规PX4控制时可观察BPN候选，但pn_guidance不发布任何控制输出。
	px4::atomic_bool _shadow_mode_enabled{false};
	uint64_t _last_shadow_print_time{0};
	// 固定台架测试状态。stage: 0=off, 1=zero-thrust, 2=armed low-output, 3=limited-thrust.
	px4::atomic<uint32_t> _rig_stage{0};
	px4::atomic_bool _rig_stop_requested{false};
	bool _rig_fault_latched{false};
	bool _rig_release_latched{false};
	bool _rig_low_seen{false};
	bool _rig_high_pending{false};
	uint64_t _rig_high_start_time{0};
	uint64_t _last_rig_print_time{0};
	Quatf _rig_anchor_q{1.f, 0.f, 0.f, 0.f};
	bool _rig_anchor_valid{false};

	// vehicle_attitude_setpoint发布诊断：用于区分“内部q_cmd在更新”与“uORB真正advertise/publish成功”。
	px4::atomic<uint32_t> _att_sp_pub_calls{0};
	px4::atomic<uint32_t> _att_sp_pub_ok{0};
	px4::atomic<uint32_t> _att_sp_pub_fail{0};
	px4::atomic_bool _att_sp_last_ok{false};
	uint64_t _last_att_sp_pub_warn_time{0};

	bool _yaw_locked{false};
	float _yaw_lock{0.f};

	bool _was_armed{false};
	bool _launch_sequence_complete{false};

	// ============================================================
	// 发射姿态 / clearance / transition
	// ============================================================

	Quatf _hold_q{1.f, 0.f, 0.f, 0.f};

	Vector3f _clearance_start_accel_ned{0.f, 0.f, 0.f};
	Vector3f _transition_start_accel_ned{0.f, 0.f, 0.f};

	float _transition_phase{1.f};

	// ============================================================
	// Vc状态
	// ============================================================

	bool _vc_filter_initialized{false};
	bool _vc_valid{false};

	float _vc_raw{0.f};
	float _vc_filtered{0.f};
	float _vc_used{VC_FALLBACK};
	// V21.13 decomposition for diagnostics: Vc_rel = Vc_aircraft - Vtarget dot LOS.
	float _vc_aircraft_only{0.f};
	float _vc_target_projection{0.f};
	float _vc_relative_raw{0.f};

	uint64_t _last_vc_update_time{0};

	// V21.11: short-term strapdown velocity fallback in NED.
	Vector3f _imu_velocity_ned{0.f, 0.f, 0.f};
	Vector3f _imu_accel_ned_filtered{0.f, 0.f, 0.f};
	bool _imu_velocity_initialized{false};
	bool _imu_velocity_valid{false};
	bool _imu_accel_filter_initialized{false};
	bool _imu_velocity_seeded_from_ekf{false};
	uint64_t _imu_velocity_anchor_time{0};
	uint64_t _imu_velocity_last_sample_time{0};
	uint64_t _last_accel_rx_time{0};

	// V21.14: EKF reset/velocity jump continuity guard.
	bool _ekf_velocity_guard_active{false};
	bool _ekf_velocity_blend_active{false};
	bool _ekf_velocity_guard_timeout_warned{false};
	uint64_t _ekf_velocity_guard_start_time{0};
	uint64_t _ekf_velocity_blend_start_time{0};
	float _ekf_velocity_blend_alpha{0.f};
	float _last_ekf_imu_velocity_diff{NAN};
	uint64_t _last_local_position_rx_time{0};

	bool _had_positive_closing{false};
	uint64_t _negative_closing_start_time{0};

	// ============================================================
	// 速度控制debug
	// ============================================================

	float _last_los_rate_mag{0.f};
	float _last_speed_target{GUIDANCE_SPEED_CRUISE};
	float _last_speed_axial_scale{1.f};
	float _last_speed_brake{0.f};
	float _last_axial_limit_cmd{AXIAL_ACCEL_MAX};
	bool _last_guidance_speed_valid{false}; // V22.01: smooth only velocity-source validity handovers

	// V22.03: post-RELEASE gimbal-protection scheduling state/diagnostics.
	uint64_t _guidance_release_time{0};
	float _last_gimbal_protect_phase{1.f};
	float _last_gimbal_protect_axial_cap{AXIAL_ACCEL_MAX};
	float _last_gimbal_protect_bpn_scale{1.f};
	float _last_gimbal_protect_slew_deg_s{GUIDANCE_ATTITUDE_SLEW_DEG_S};

	// V22.05 terminal-response diagnostics.
	uint64_t _last_control_extra_lead_us{LOS_CONTROL_EXTRA_LEAD_MIN_US};
	float _last_adaptive_slew_deg_s{GUIDANCE_ATTITUDE_SLEW_DEG_S};
	float _last_centering_assist_norm{0.f};

	// ============================================================
	// 高度底线保护
	// ============================================================

	bool _release_alt_ref_valid{false};
	float _release_alt_ref_z_ned{0.f};

	bool _alt_floor_active{false};
	float _last_alt_floor_drop{0.f};
	float _last_alt_floor_predicted_drop{0.f};
	float _last_alt_floor_up_accel{0.f};

	// ============================================================
	// 最后一次guidance结果/debug
	// ============================================================

	Quatf _last_raw_q_sp{1.f, 0.f, 0.f, 0.f};
	Quatf _last_q_sp{1.f, 0.f, 0.f, 0.f};

	float _last_thrust_sp{0.f};
	float _last_attitude_slew_deg_s{GUIDANCE_ATTITUDE_SLEW_DEG_S};
	bool _last_thrust_vector_feasibility_limited{false};
	float _last_thrust_vector_raw_tilt_deg{0.f};
	float _last_thrust_vector_cmd_tilt_deg{0.f};
	bool _slew_active{false};
	float _last_raw_q_step_deg{0.f};

	bool _lead_active{false};
	float _last_attitude_lead_deg{0.f};

	Vector3f _last_bpn_body{0.f, 0.f, 0.f};

	// V22.02 BPN jerk limiter state in NED.
	Vector3f _bpn_accel_ned_jerk_limited{0.f, 0.f, 0.f};
	bool _bpn_jerk_initialized{false};
	uint64_t _last_bpn_jerk_update_time{0};
	float _last_bpn_jerk_limit{BPN_JERK_LOW};
	float _last_bpn_jerk_dir_error_deg{0.f};

	float _last_axial_used{0.f};
	float _last_bpn_norm_raw{0.f};
	float _last_bpn_norm_used{0.f};
	float _last_predicted_thrust{0.f};
	float _last_bpn_scale{1.f};

	// ============================================================
	// SAFE_HOVER状态
	// ============================================================

	bool _safe_hold_latched{false};
	uint64_t _safe_low_speed_start_time{0};
	Vector3f _safe_hold_position_ned{0.f, 0.f, 0.f};

	// ============================================================
	// EKF reset监控
	// ============================================================

	bool _ekf_reset_counter_initialized{false};
	uint8_t _last_xy_reset_counter{0};
	uint8_t _last_z_reset_counter{0};
	uint8_t _last_vxy_reset_counter{0};
	uint8_t _last_vz_reset_counter{0};

	bool _attitude_reset_counter_initialized{false};
	uint8_t _last_quat_reset_counter{0};

	// ============================================================
	// 一键TEST模式
	// ============================================================
	int enable_test_mode()
	{
		if (armed()) { PX4_ERR("TEST refused: vehicle is armed"); return PX4_ERROR; }
		if (_state != GuidanceState::BPN || _launch_sequence_complete || !_yaw_locked) { PX4_ERR("TEST refused: require BPN_READY + yaw lock + disarmed"); return PX4_ERROR; }
		if (!los_fresh(hrt_absolute_time()) || !_los.target_valid || !attitude_valid()) { PX4_ERR("TEST refused: require fresh valid LOS + valid attitude"); return PX4_ERROR; }

		// TEST优先占用姿态setpoint测试路径；关闭连续BENCH/SHADOW，避免重复刷屏或只读SHADOW抢先return。
		_bench_realtime_enabled.store(false); _shadow_mode_enabled.store(false);
		_att_sp_pub_calls.store(0); _att_sp_pub_ok.store(0); _att_sp_pub_fail.store(0); _att_sp_last_ok.store(false);

		// 记录当前真实姿态作为15deg cage中心，并从该姿态重新开始slew，避免继承旧setpoint造成瞬时跳变。
		_sp_test_anchor_q = current_attitude_q(); _sp_test_anchor_q.normalize(); _sp_test_anchor_valid = true;
		_last_q_sp = _sp_test_anchor_q; _last_raw_q_sp = _sp_test_anchor_q; _last_test_print_time = 0;
		_sp_interface_test_enabled.store(true);
		PX4_INFO("TEST enabled: BPN preview budget=%.2f cage=%.1fdeg, 50Hz attitude SP, ZERO thrust, DISARMED only", (double)BPN_TEST_THRUST_LIMIT, (double)BPN_TEST_ATTITUDE_MAX_DEG);
		PX4_INFO("TEST note: stop mc_pos_control before pn_guidance to avoid multiple vehicle_attitude_setpoint publishers");
		return PX4_OK;
	}

	int disable_test_mode()
	{
		_sp_interface_test_enabled.store(false); _sp_test_anchor_valid = false; _last_test_print_time = 0;
		// 退出TEST后把hold/setpoint重新锚定到当前真实姿态，使BPN_READY恢复为无跳变保持。
		if (attitude_valid()) { _hold_q = current_attitude_q(); _last_q_sp = _hold_q; _last_raw_q_sp = _hold_q; }
		PX4_INFO("TEST disabled");
		return PX4_OK;
	}

	int print_test_mode_status()
	{
		const bool orb_visible = orb_exists(ORB_ID(vehicle_attitude_setpoint), 0) == PX4_OK;
		PX4_INFO("TEST enabled=%d armed=%d state=%u launch_complete=%d anchor_valid=%d preview_thr=%.2f cage=%.1fdeg real_thrust=0",
			 (int)_sp_interface_test_enabled.load(), (int)armed(), (unsigned)_state, (int)_launch_sequence_complete,
			 (int)_sp_test_anchor_valid, (double)BPN_TEST_THRUST_LIMIT, (double)BPN_TEST_ATTITUDE_MAX_DEG);
		PX4_INFO("TEST uORB calls=%u ok=%u fail=%u last_ok=%d advertised=%d orb_exists=%d",
			 (unsigned)_att_sp_pub_calls.load(), (unsigned)_att_sp_pub_ok.load(), (unsigned)_att_sp_pub_fail.load(),
			 (int)_att_sp_last_ok.load(), (int)_attitude_setpoint_pub.advertised(), (int)orb_visible);
		return PX4_OK;
	}

	// ============================================================
	// 固定机械台架三阶段控制律验证
	// ============================================================
	void reset_rig_release_interlock()
	{
		_rig_release_latched = false;
		_rig_low_seen = false;
		_rig_high_pending = false;
		_rig_high_start_time = 0;
	}

	bool rig_control_mode_ready() const
	{
		return _vehicle_control_mode.flag_control_attitude_enabled
		       && _vehicle_control_mode.flag_control_rates_enabled;
	}

	bool rig_rc_valid(const uint64_t now) const
	{
		return rc_release_input_valid(now);
	}

	bool update_rig_release_interlock(const uint64_t now, const bool is_armed)
	{
		if (!is_armed) {
			reset_rig_release_interlock();
			return false;
		}

		if (!rig_rc_valid(now)) {
			reset_rig_release_interlock();
			return false;
		}

		const float aux1 = _manual_control.aux1;

		// 运行中把AUX1拨回LOW，立即退出台架控制；下一次必须重新LOW->HIGH。
		if (_rig_release_latched && aux1 <= RC_RELEASE_LOW_THRESHOLD) {
			PX4_WARN("RIG STOP: AUX1 LOW");
			reset_rig_release_interlock();
			_rig_low_seen = true;
			return false;
		}

		if (_rig_release_latched) {
			return true;
		}

		if (aux1 <= RC_RELEASE_LOW_THRESHOLD) {
			_rig_low_seen = true;
			_rig_high_pending = false;
			_rig_high_start_time = 0;
			return false;
		}

		if (!_rig_low_seen) {
			return false;
		}

		if (aux1 >= RC_RELEASE_HIGH_THRESHOLD) {
			if (!_rig_high_pending) {
				_rig_high_pending = true;
				_rig_high_start_time = now;
				return false;
			}

			if (now >= _rig_high_start_time && now - _rig_high_start_time >= RC_RELEASE_CONFIRM_US) {
				_rig_release_latched = true;
				_rig_high_pending = false;
				_rig_anchor_q = current_attitude_q();
				_rig_anchor_q.normalize();
				_rig_anchor_valid = true;
				_last_q_sp = _rig_anchor_q;
				_last_raw_q_sp = _rig_anchor_q;
				PX4_WARN("RIG RUN accepted: AUX1 LOW->HIGH held %ums", (unsigned)(RC_RELEASE_CONFIRM_US / 1000ULL));
				return true;
			}
		}

		return false;
	}

	int set_rig_stage(const uint32_t stage)
	{
		if (stage > 3u) { return PX4_ERROR; }

		if (stage == 1u && armed()) {
			PX4_ERR("RIG stage1 refused: DISARM first");
			return PX4_ERROR;
		}

		_sp_interface_test_enabled.store(false);
		_shadow_mode_enabled.store(false);
		_bench_realtime_enabled.store(false);
		reset_rig_release_interlock();
		_rig_fault_latched = false;
		_rig_anchor_valid = false;
		_last_rig_print_time = 0;

		if (stage == 0u) {
			// 如果当前已解锁/正在台架控制，不直接让硬件分支消失；
			// 由WorkQueue先发布一帧当前姿态+zero thrust，再真正退出RIG。
			if (armed() && _rig_stage.load() != 0u) {
				_rig_stop_requested.store(true);
			} else {
				_rig_stage.store(0u);
			}
			PX4_WARN("RIG OFF requested; hardware kill/disarm/power cut remains the emergency stop");
			return PX4_OK;
		}
		_rig_stop_requested.store(false);

		if (!attitude_valid()) {
			PX4_ERR("RIG refused: attitude invalid");
			return PX4_ERROR;
		}

		_rig_stage.store(stage);
		if (stage == 1u) {
			_rig_anchor_q = current_attitude_q();
			_rig_anchor_q.normalize();
			_rig_anchor_valid = true;
			_last_q_sp = _rig_anchor_q;
			_last_raw_q_sp = _rig_anchor_q;
			PX4_INFO("RIG STAGE1: DISARMED, ZERO thrust, BPN->attitude-setpoint only");
		} else {
			PX4_INFO("RIG STAGE%u armed test selected; ARM -> AUX1 LOW -> HIGH >=%ums to run",
				 (unsigned)stage, (unsigned)(RC_RELEASE_CONFIRM_US / 1000ULL));
			PX4_INFO("RIG: ensure pn_guidance is the only vehicle_attitude_setpoint publisher; stop mc_pos_control for this bench test");
			PX4_INFO("RIG limits: cage=%.1fdeg s12_slew=%.1fdeg/s s3_slew=%.1fdeg/s s3_bpn_scale=%.2f s2_thr=%.2f s3_max=%.2f motor_abort=%.2f",
				 (double)RIG_ATTITUDE_CAGE_DEG, (double)RIG_ATTITUDE_SLEW_DEG_S,
				 (double)GUIDANCE_ATTITUDE_SLEW_DEG_S, (double)RIG_STAGE3_BPN_SCALE, (double)RIG_STAGE2_THRUST,
				 (double)RIG_STAGE3_THRUST_MAX, (double)RIG_MOTOR_OUTPUT_ABORT);
		}
		return PX4_OK;
	}

	int print_rig_status()
	{
		const uint64_t now = hrt_absolute_time();
		PX4_INFO("RIG stage=%u armed=%d run=%d fault=%d stop_req=%d anchor=%d RC=%d aux1=%+.2f ctrl_att=%d ctrl_rate=%d",
			 (unsigned)_rig_stage.load(), (int)armed(), (int)_rig_release_latched, (int)_rig_fault_latched,
			 (int)_rig_stop_requested.load(), (int)_rig_anchor_valid,
			 (int)rig_rc_valid(now), (double)_manual_control.aux1,
			 (int)_vehicle_control_mode.flag_control_attitude_enabled,
			 (int)_vehicle_control_mode.flag_control_rates_enabled);
		PX4_INFO("RIG vc_test=%.1f s12_bpn_max=%.1f s3_bpn_max=%.1f s3_bpn_scale=%.2f cage=%.1fdeg s12_slew=%.1fdeg/s",
			 (double)RIG_VC_TEST, (double)RIG_BPN_ACCEL_MAX, (double)BPN_ACCEL_MAX, (double)RIG_STAGE3_BPN_SCALE,
			 (double)RIG_ATTITUDE_CAGE_DEG, (double)RIG_ATTITUDE_SLEW_DEG_S);
		PX4_INFO("RIG cont s3_slew=%.1fdeg/s s2_thr=%.2f s3_max=%.2f",
			 (double)GUIDANCE_ATTITUDE_SLEW_DEG_S,
			 (double)RIG_STAGE2_THRUST, (double)RIG_STAGE3_THRUST_MAX);
		return PX4_OK;
	}

	// ============================================================
	// V18 Gazebo真值LOS旁路
	// ============================================================

	bool truth_los_fresh(const uint64_t now) const
	{
		return _truth_los_valid
		       && _last_truth_los_rx_time > 0
		       && now >= _last_truth_los_rx_time
		       && now - _last_truth_los_rx_time <= TRUTH_LOS_TIMEOUT_US;
	}

	bool control_los_rate_ready(const uint64_t now) const
	{
		if (SIMULATION_MODE && SIM_USE_TRUTH_LOS_CONTROL) {
			return truth_los_fresh(now);
		}

		return _inertial_los_rate_valid;
	}

	void reset_v20_truth_metrics()
	{
		_last_v19_metric_los_rx_time = 0;
		_v19_metric_count = 0;
		_v19_error_sq_sum = 0.f;
		_v19_error_rms = 0.f;
		_v19_error_max = 0.f;
		_v19_reset100_error_max = 0.f;
		_v19_direction_error_deg = 0.f;
		_v20_metric_rejected_high_rate = 0;
	}

	void update_v19_truth_metrics(const uint64_t now)
	{
		if (!SIMULATION_MODE || !_v20_metric_active || !truth_los_fresh(now) || !_inertial_los_rate_valid
		    || _last_los_rx_time == 0 || _last_los_rx_time == _last_v19_metric_los_rx_time) {
			return;
		}

		_last_v19_metric_los_rx_time = _last_los_rx_time;
		const float truth_mag = _truth_los_dot_body.norm();
		if (!PX4_ISFINITE(truth_mag) || truth_mag > V20_METRIC_TRUTH_RATE_MAX) {
			_v20_metric_rejected_high_rate++;
			return;
		}
		const Vector3f err = _los_dot_inertial_body - _truth_los_dot_body;
		const float err_mag = err.norm();

		if (!PX4_ISFINITE(err_mag)) {
			return;
		}

		_v19_metric_count++;
		_v19_error_sq_sum += err_mag * err_mag;
		_v19_error_rms = sqrtf(_v19_error_sq_sum / (float)_v19_metric_count);
		if (err_mag > _v19_error_max) { _v19_error_max = err_mag; }

		if (_last_attitude_reset_time > 0 && now >= _last_attitude_reset_time
		    && now - _last_attitude_reset_time <= V20_RESET_METRIC_WINDOW_US) {
			if (err_mag > _v19_reset100_error_max) { _v19_reset100_error_max = err_mag; }
		}

		const float nt = _truth_los_dot_body.norm();
		const float nr = _los_dot_inertial_body.norm();
		if (nt > 1e-4f && nr > 1e-4f) {
			const float c = math::constrain(
				_truth_los_dot_body.dot(_los_dot_inertial_body) / (nt * nr), -1.f, 1.f);
			_v19_direction_error_deg = acosf(c) * 180.f / M_PI_F;
		} else {
			_v19_direction_error_deg = 0.f;
		}
	}

	void run_hardware_observer_only(const uint64_t now)
	{
		// V20.7：实机observer-only只打印，不发布vehicle_attitude_setpoint/推力/arm命令。
		// BENCH_RT开启时由BENCH5负责输出更完整的5Hz诊断，避免两套日志叠加刷屏。
		if (_bench_realtime_enabled.load()) {
			return;
		}
		if (now - _last_hw_observer_print_time < DIAG_PRINT_INTERVAL_US) {
			return;
		}
		_last_hw_observer_print_time = now;

		const float rate_mag = _inertial_los_rate_valid ? _los_dot_inertial_body.norm() : 0.f;
		const bool rc_ok = rc_release_input_valid(now);
		const bool body_ok = body_rate_valid(now);
		const float ekf_speed = get_ekf_speed();
		const float guidance_speed = get_guidance_speed();

		PX4_INFO("HW5 H arm=%d RC=%d tgt=%d fresh=%d rate=%d body=%d age=%.1fms gyro=%+.1fms dt=%.1fms",
			 (int)armed(), (int)rc_ok, (int)_los.target_valid, (int)los_fresh(now),
			 (int)_inertial_los_rate_valid, (int)body_ok,
			 (double)(_last_los_sample_age_us * 1e-3f),
			 (double)(_last_body_rate_align_error_us * 1e-3f),
			 (double)(_last_los_recon_dt_s * 1000.f));

		PX4_INFO("HW5 LOS G=[%+.1f %+.1f] M=[%+.2f %+.2f] R=[%+.4f %+.4f] IN=[%+.4f %+.4f %+.4f]|%.4f",
			 (double)(_los.gimbal_roll * 180.f / M_PI_F), (double)(_los.gimbal_pitch * 180.f / M_PI_F),
			 (double)(_los.los_azimuth * 180.f / M_PI_F), (double)(_los.los_elevation * 180.f / M_PI_F),
			 (double)_los.los_az_rate, (double)_los.los_el_rate,
			 (double)_los_dot_inertial_body(0), (double)_los_dot_inertial_body(1),
			 (double)_los_dot_inertial_body(2), (double)rate_mag);

		PX4_INFO("HW5 SRC I=%.4f G=%.4f B=%.4f F=%.4f GRate=[%+.3f %+.3f] VC=%.2f/%.2f/%.2f VaLOS=%.2f VtLOS=%.2f v=%d src=%s spd=%.2f EKF=%.2f NO_CONTROL",
			 (double)_los_dot_image_body.norm(), (double)_los_dot_gimbal_body.norm(),
			 (double)_los_dot_body_motion.norm(), (double)_los_dot_fused_raw_body.norm(),
			 (double)_gimbal_az_rate_filtered, (double)_gimbal_pitch_rate_filtered,
			 (double)_vc_raw, (double)_vc_filtered, (double)_vc_used,
			 (double)_vc_aircraft_only, (double)_vc_target_projection,
			 (int)_vc_valid, guidance_velocity_source_name(),
			 (double)guidance_speed, (double)ekf_speed);
	}

	// ============================================================
	// RELEASE状态/互锁：真机RC + SITL虚拟AUX1
	// ============================================================

	bool rc_input_valid(const uint64_t now) const
	{
		// RC所有权只依赖“真实RC链路是否新鲜有效”，绝不依赖AUX1数值。
		// 这样即使AUX1映射异常/NaN，飞手切入Manual/Stabilized时仍然拥有最高控制优先级。
		if (_last_manual_control_rx_time == 0 || now < _last_manual_control_rx_time
		    || now - _last_manual_control_rx_time > RC_RELEASE_TIMEOUT_US) { return false; }
		return _manual_control.valid
		       && _manual_control.data_source == manual_control_setpoint_s::SOURCE_RC;
	}

	bool rc_release_input_valid(const uint64_t now) const
	{
		// RELEASE在真实RC链路有效基础上，再额外要求AUX1本身为有限值。
		// QGC joystick / MAVLink MANUAL_CONTROL不能冒充正式RELEASE输入。
		return rc_input_valid(now) && PX4_ISFINITE(_manual_control.aux1);
	}


	// V21.9: RC人工控制所有权覆盖Manual/Stabilized/Acro。
	// 只要manual+rates有效，且altitude/velocity/position均未接管，就视为飞手直接控制；
	// Stabilized通常attitude=true，Acro通常attitude=false，但两者都必须让pn_guidance完全让权。
	bool rc_manual_attitude_owner_active(const uint64_t now) const
	{
		if (SIMULATION_MODE || !armed() || _rig_stage.load() != 0u) {
			return false;
		}

		if (_last_control_mode_rx_time == 0 || now < _last_control_mode_rx_time
		    || now - _last_control_mode_rx_time > RC_CONTROL_MODE_TIMEOUT_US) {
			return false;
		}

		if (!rc_input_valid(now)) {
			return false;
		}

		return _vehicle_control_mode.flag_control_rates_enabled
		       && _vehicle_control_mode.flag_control_manual_enabled
		       && !_vehicle_control_mode.flag_control_altitude_enabled
		       && !_vehicle_control_mode.flag_control_velocity_enabled
		       && !_vehicle_control_mode.flag_control_position_enabled;
	}

	bool formal_guidance_active() const
	{
		return (_state == GuidanceState::BPN && _launch_sequence_complete)
		       || _state == GuidanceState::LAUNCH_CLEARANCE
		       || _state == GuidanceState::TRANSITION;
	}

	void reset_rc_override_latch()
	{
		_rc_override_latched = false;
		_guidance_release_nav_state_valid = false;
		_guidance_release_nav_state = 0;
		_last_rc_override_warn_time = 0;
	}

	void latch_rc_override(const uint64_t now, const char *reason)
	{
		if (!_rc_override_latched) {
			_rc_override_latched = true;
			PX4_WARN("RC OVERRIDE LATCHED: %s; pn_guidance yields attitude setpoint until DISARM", reason);
		}
		_last_rc_override_warn_time = now;
	}

	void update_rc_authority(const uint64_t now, const bool is_armed)
	{
		if (!is_armed) {
			reset_rc_override_latch();
			return;
		}

		// RELEASE建立nav_state基线后，无论当前处于CLEARANCE/TRANSITION/BPN还是SAFE_HOVER，
		// 都持续监视人工接管，直到DISARM清除基线与override latch。
		if (_rc_override_latched || !_guidance_release_nav_state_valid) {
			return;
		}

		if (rc_manual_attitude_owner_active(now)) {
			latch_rc_override(now, "Manual/Stabilized/Acro RC mode");
			return;
		}

		// RELEASE之后任何Commander nav_state变化都视为飞手/系统级模式接管。
		if (_guidance_release_nav_state_valid
		    && _vehicle_status.nav_state != _guidance_release_nav_state) {
			latch_rc_override(now, "vehicle nav_state changed after RELEASE");
		}
	}

	bool release_altitude_reference_ready() const
	{
		// RELEASE以后高度底线保护依赖z和vz，因此正式释放前就要求它们有效。
		return _local_position.z_valid && _local_position.v_z_valid
		       && PX4_ISFINITE(_local_position.z) && PX4_ISFINITE(_local_position.vz);
	}

	bool release_core_guards_ready(const uint64_t now) const
	{
		// 真机与仿真共同的最低释放条件。最终RELEASE前还会再检查一次，防止同一周期状态突变。
		return los_fresh(now) && _los.target_valid && attitude_valid() && release_altitude_reference_ready();
	}

	bool simulation_release_guards_ready(const uint64_t now) const
	{
		// SITL自动拨杆比真机人工拨杆再多等两项：惯性LOS重构已经有效、机体角速度输入新鲜。
		// 这样仿真不会在V12新加入的LOS惯性化链路尚未建立时提前RELEASE。
		return release_core_guards_ready(now) && control_los_rate_ready(now) && body_rate_valid(now);
	}

	void reset_rc_release_interlock()
	{
		_release_low_seen = false;
		_release_high_pending = false;
		_release_high_block_warned = false;
		_release_high_start_time = 0;
		_sim_release_aux1 = SIM_RELEASE_AUX_LOW;
		_sim_release_ready_start_time = 0;
		_sim_release_ready_logged = false;
	}

	// 真机：保持V12原有逻辑不变。只识别真实RC AUX1的一次安全LOW->HIGH事件。
	bool update_rc_release_interlock(const uint64_t now, const bool is_armed)
	{
		if (!is_armed || _state != GuidanceState::WAIT_RELEASE) {
			reset_rc_release_interlock();
			return false;
		}

		if (!rc_release_input_valid(now)) {
			// RC失联/来源错误/aux1无效后必须重新LOW->HIGH，禁止恢复连接时直接HIGH触发。
			reset_rc_release_interlock();
			return false;
		}

		const float aux1 = _manual_control.aux1;

		if (aux1 <= RC_RELEASE_LOW_THRESHOLD) {
			if (!_release_low_seen) {
				PX4_INFO("RC RELEASE armed: AUX1 LOW seen (%+.2f); next LOW->HIGH can release", (double)aux1);
			}
			_release_low_seen = true;
			_release_high_pending = false;
			_release_high_block_warned = false;
			_release_high_start_time = 0;
			return false;
		}

		if (!_release_low_seen) {
			if (aux1 >= RC_RELEASE_HIGH_THRESHOLD && !_release_high_block_warned) {
				_release_high_block_warned = true;
				PX4_WARN("RC RELEASE blocked: AUX1 already HIGH (%+.2f); move LOW first", (double)aux1);
			}
			_release_high_pending = false;
			_release_high_start_time = 0;
			return false;
		}

		if (aux1 >= RC_RELEASE_HIGH_THRESHOLD) {
			if (!_release_high_pending) {
				_release_high_pending = true;
				_release_high_start_time = now;
				return false;
			}
			if (now >= _release_high_start_time && now - _release_high_start_time >= RC_RELEASE_CONFIRM_US) {
				PX4_WARN("RC RELEASE accepted: AUX1 %+.2f HIGH held %ums",
					 (double)aux1, (unsigned)(RC_RELEASE_CONFIRM_US / 1000ULL));
				reset_rc_release_interlock();
				return true;
			}
			return false;
		}

		// AUX1处在两个阈值之间：取消HIGH计时，但保留ARM以后已经确认过LOW。
		_release_high_pending = false;
		_release_high_start_time = 0;
		return false;
	}

	// SITL：不伪造manual_control_setpoint，也不发布RC topic；只在pn_guidance内部模拟同样的AUX1互锁状态。
	// 流程：ARM进入WAIT_RELEASE -> 虚拟AUX1 LOW -> 条件连续稳定300ms -> 虚拟AUX1 HIGH -> HIGH保持100ms -> RELEASE事件。
	bool update_simulated_release_interlock(const uint64_t now, const bool is_armed)
	{
		if (!is_armed || _state != GuidanceState::WAIT_RELEASE) {
			reset_rc_release_interlock();
			return false;
		}

		// 至少先经过一个50Hz周期的LOW状态，严格保留“ARM以后先见LOW”的语义。
		if (!_release_low_seen) {
			_sim_release_aux1 = SIM_RELEASE_AUX_LOW;
			_release_low_seen = true;
			_release_high_pending = false;
			_release_high_start_time = 0;
			_sim_release_ready_start_time = 0;
			_sim_release_ready_logged = false;
			PX4_INFO("SIM RELEASE armed: virtual AUX1 LOW (%+.1f); waiting release guards", (double)_sim_release_aux1);
			return false;
		}

		// 条件只要中途失效，就立即把虚拟拨杆退回LOW并重新累计稳定时间。
		if (!simulation_release_guards_ready(now)) {
			_sim_release_aux1 = SIM_RELEASE_AUX_LOW;
			_release_high_pending = false;
			_release_high_start_time = 0;
			_sim_release_ready_start_time = 0;
			_sim_release_ready_logged = false;
			return false;
		}

		if (_sim_release_ready_start_time == 0) {
			_sim_release_ready_start_time = now;
			if (!_sim_release_ready_logged) {
				_sim_release_ready_logged = true;
				PX4_INFO("SIM RELEASE guards ready: hold %.0fms, then virtual AUX1 HIGH",
					 (double)(SIM_RELEASE_READY_HOLD_US * 1e-3f));
			}
			return false;
		}

		if (now < _sim_release_ready_start_time || now - _sim_release_ready_start_time < SIM_RELEASE_READY_HOLD_US) {
			return false;
		}

		_sim_release_aux1 = SIM_RELEASE_AUX_HIGH;
		if (!_release_high_pending) {
			_release_high_pending = true;
			_release_high_start_time = now;
			PX4_WARN("SIM RELEASE virtual AUX1 LOW->HIGH (%+.1f); confirming %ums",
				 (double)_sim_release_aux1, (unsigned)(RC_RELEASE_CONFIRM_US / 1000ULL));
			return false;
		}

		// HIGH确认期间再次确认释放条件；如果条件掉线，上面的guard分支会把拨杆退回LOW。
		if (now >= _release_high_start_time && now - _release_high_start_time >= RC_RELEASE_CONFIRM_US) {
			PX4_WARN("SIM RELEASE accepted: virtual AUX1 HIGH held %ums",
				 (unsigned)(RC_RELEASE_CONFIRM_US / 1000ULL));
			reset_rc_release_interlock();
			return true;
		}

		return false;
	}

	// ============================================================
	// V20.2 真机ARM + RELEASE两步RC流程预览
	// ============================================================
	void run_hardware_arm_release_preview(const uint64_t now)
	{
		const bool is_armed = armed();

		// DISARM -> ARM边沿：只有已经BPN_READY时才进入WAIT_RELEASE。
		if (is_armed && !_was_armed) {
			_hardware_release_preview_latched = false;
			_hardware_release_preview_time = 0;

			if (_state == GuidanceState::BPN && !_launch_sequence_complete) {
				enter_wait_release(now);
				PX4_INFO("HW RC FLOW: ARM accepted by Commander -> WAIT_RELEASE");
			} else {
				PX4_WARN("HW RC FLOW: armed, but pn_guidance not BPN_READY (state=%u)",
					 (unsigned)_state);
			}
		}

		// ARM -> DISARM边沿：复位RC释放互锁与preview锁存。
		if (!is_armed && _was_armed) {
			reset_rc_release_interlock();
			_hardware_release_preview_latched = false;
			_hardware_release_preview_time = 0;
			_launch_sequence_complete = false;

			if (los_fresh(now) && _los.target_valid && attitude_valid()) {
				_hold_q = current_attitude_q();
				_state = GuidanceState::BPN;
			} else {
				_state = GuidanceState::WAIT_LOS;
			}

			PX4_INFO("HW RC FLOW: DISARM -> release preview reset");
		}

		_was_armed = is_armed;

		// 只有ARM后处于WAIT_RELEASE时，AUX1 LOW->HIGH才有意义。
		if (!is_armed || _state != GuidanceState::WAIT_RELEASE) {
			return;
		}

		const bool release_event = update_rc_release_interlock(now, is_armed);

		if (!release_event) {
			return;
		}

		// RELEASE事件被接受后再跑一次完整guard。
		if (!los_fresh(now) || !_los.target_valid) {
			PX4_ERR("HW RELEASE PREVIEW refused: LOS invalid/stale");
			return;
		}

		if (!attitude_valid()) {
			PX4_ERR("HW RELEASE PREVIEW refused: attitude invalid");
			return;
		}

		if (!release_altitude_reference_ready()) {
			PX4_ERR("HW RELEASE PREVIEW refused: local Z/VZ invalid");
			return;
		}

		if (!_inertial_los_rate_valid || !body_rate_valid(now)) {
			PX4_ERR("HW RELEASE PREVIEW refused: LOS/body-rate observer not ready");
			return;
		}

		_hardware_release_preview_latched = true;
		_hardware_release_preview_time = now;

		PX4_WARN("HW RELEASE PREVIEW ACCEPTED: ARM + AUX1 LOW->HIGH sequence OK; NO CONTROL OUTPUT");
	}

	int print_release_status()
	{
		const uint64_t now = hrt_absolute_time();
		const bool core_guards = release_core_guards_ready(now);

		PX4_INFO("RELEASE mode=%s; shell command is status-only", SIMULATION_MODE ? "SIMULATION" : "REAL_RC");
		if (SIMULATION_MODE) {
			PX4_INFO("SIM RELEASE state=%u armed=%d virtual_aux1=%+.2f sim_guards=%d ready_hold=%ums",
				 (unsigned)_state, (int)armed(), (double)_sim_release_aux1,
				 (int)simulation_release_guards_ready(now), (unsigned)(SIM_RELEASE_READY_HOLD_US / 1000ULL));
		} else {
			const bool rc_valid = rc_release_input_valid(now);
			PX4_INFO("RC RELEASE state=%u armed=%d rc_valid=%d source=%u aux1=%+.3f age_ms=%.1f",
				 (unsigned)_state, (int)armed(), (int)rc_valid, (unsigned)_manual_control.data_source,
				 (double)_manual_control.aux1,
				 _last_manual_control_rx_time > 0 && now >= _last_manual_control_rx_time
				 ? (double)(now - _last_manual_control_rx_time) * 1e-3 : -1.0);
		}

		PX4_INFO("RELEASE interlock low_seen=%d high_pending=%d low<=%.2f high>=%.2f confirm=%ums",
			 (int)_release_low_seen, (int)_release_high_pending,
			 (double)RC_RELEASE_LOW_THRESHOLD, (double)RC_RELEASE_HIGH_THRESHOLD,
			 (unsigned)(RC_RELEASE_CONFIRM_US / 1000ULL));
		if (!SIMULATION_MODE) {
			const double preview_age_ms = (_hardware_release_preview_latched && _hardware_release_preview_time > 0
				&& now >= _hardware_release_preview_time)
				? (double)(now - _hardware_release_preview_time) * 1e-3 : -1.0;
			PX4_INFO("HW RELEASE PREVIEW latched=%d age_ms=%.1f no_control=1",
				 (int)_hardware_release_preview_latched, preview_age_ms);
		}
		PX4_INFO("RELEASE guards core=%d LOS=%d attitude=%d altitude_z_vz=%d inertial_los=%d body_rate=%d start_thr=%.2f clearance_limit=%.2f",
			 (int)core_guards, (int)(los_fresh(now) && _los.target_valid), (int)attitude_valid(),
			 (int)release_altitude_reference_ready(), (int)_inertial_los_rate_valid, (int)body_rate_valid(now),
			 (double)CLEARANCE_START_THRUST, (double)CLEARANCE_THRUST_LIMIT);
		return PX4_OK;
	}

	// ============================================================
	// 工具函数
	// ============================================================

	float wrap_pi_local(float angle) const
	{
		while (angle > M_PI_F) { angle -= 2.f * M_PI_F; }
		while (angle < -M_PI_F) { angle += 2.f * M_PI_F; }
		return angle;
	}

	float smoothstep01(const float input) const
	{
		const float p = math::constrain(input, 0.f, 1.f);
		return p * p * (3.f - 2.f * p);
	}

	float lerp(const float a, const float b, const float t) const
	{
		return a + (b - a) * math::constrain(t, 0.f, 1.f);
	}

	// ============================================================
	// V22.03 post-RELEASE gimbal-protection scheduler
	// phase=0 at RELEASE acceptance, phase=1 after 1.5s.
	// smoothstep avoids derivative discontinuities at both ends.
	// ============================================================
	float gimbal_protect_phase(const uint64_t now) const
	{
		if (_guidance_release_time == 0 || now <= _guidance_release_time) {
			return (_guidance_release_time == 0) ? 1.f : 0.f;
		}

		const float raw = (float)(now - _guidance_release_time) / (float)GIMBAL_PROTECT_DURATION_US;
		return smoothstep01(raw);
	}

	void update_gimbal_protect_schedule(const uint64_t now)
	{
		const float p = gimbal_protect_phase(now);
		_last_gimbal_protect_phase = p;
		_last_gimbal_protect_axial_cap = lerp(GIMBAL_PROTECT_AXIAL_START, AXIAL_ACCEL_MAX, p);
		_last_gimbal_protect_bpn_scale = lerp(GIMBAL_PROTECT_BPN_SCALE_START, 1.f, p);
		_last_gimbal_protect_slew_deg_s = lerp(GIMBAL_PROTECT_SLEW_START_DEG_S, GUIDANCE_ATTITUDE_SLEW_DEG_S, p);
	}

	void start_gimbal_protect_window(const uint64_t now)
	{
		_guidance_release_time = now;
		_last_gimbal_protect_phase = 0.f;
		_last_gimbal_protect_axial_cap = GIMBAL_PROTECT_AXIAL_START;
		_last_gimbal_protect_bpn_scale = GIMBAL_PROTECT_BPN_SCALE_START;
		_last_gimbal_protect_slew_deg_s = GIMBAL_PROTECT_SLEW_START_DEG_S;
	}

	void reset_gimbal_protect_window()
	{
		_guidance_release_time = 0;
		_last_gimbal_protect_phase = 1.f;
		_last_gimbal_protect_axial_cap = AXIAL_ACCEL_MAX;
		_last_gimbal_protect_bpn_scale = 1.f;
		_last_gimbal_protect_slew_deg_s = GUIDANCE_ATTITUDE_SLEW_DEG_S;
	}

	float quat_angle_rad(const Quatf &qa, const Quatf &qb) const
	{
		float dot = qa(0) * qb(0) + qa(1) * qb(1) + qa(2) * qb(2) + qa(3) * qb(3);
		dot = fabsf(dot);
		dot = math::constrain(dot, 0.f, 1.f);
		return 2.f * acosf(dot);
	}

	Quatf slerp_limited(const Quatf &q_from_in, const Quatf &q_to_in, const float max_angle_rad,
			   bool &limited, float &raw_angle_rad) const
	{
		Quatf q_from = q_from_in;
		Quatf q_to = q_to_in;

		q_from.normalize();
		q_to.normalize();

		float dot = q_from(0) * q_to(0) + q_from(1) * q_to(1) + q_from(2) * q_to(2) + q_from(3) * q_to(3);

		if (dot < 0.f) {
			dot = -dot;
			q_to = Quatf{-q_to(0), -q_to(1), -q_to(2), -q_to(3)};
		}

		dot = math::constrain(dot, 0.f, 1.f);
		raw_angle_rad = 2.f * acosf(dot);

		if (!PX4_ISFINITE(raw_angle_rad) || raw_angle_rad <= max_angle_rad || raw_angle_rad < 1e-6f) {
			limited = false;
			return q_to;
		}

		limited = true;

		const float fraction = math::constrain(max_angle_rad / raw_angle_rad, 0.f, 1.f);
		const float theta = acosf(dot);
		const float sin_theta = sinf(theta);

		Quatf q_out;

		if (fabsf(sin_theta) < 1e-6f) {
			q_out = Quatf{
				(1.f - fraction) * q_from(0) + fraction * q_to(0),
				(1.f - fraction) * q_from(1) + fraction * q_to(1),
				(1.f - fraction) * q_from(2) + fraction * q_to(2),
				(1.f - fraction) * q_from(3) + fraction * q_to(3)
			};

		} else {
			const float w0 = sinf((1.f - fraction) * theta) / sin_theta;
			const float w1 = sinf(fraction * theta) / sin_theta;

			q_out = Quatf{
				w0 * q_from(0) + w1 * q_to(0),
				w0 * q_from(1) + w1 * q_to(1),
				w0 * q_from(2) + w1 * q_to(2),
				w0 * q_from(3) + w1 * q_to(3)
			};
		}

		q_out.normalize();
		return q_out;
	}

	void constrain_xy(Vector3f &accel, const float max_xy) const
	{
		const float norm_xy = sqrtf(accel(0) * accel(0) + accel(1) * accel(1));

		if (PX4_ISFINITE(norm_xy) && norm_xy > max_xy && norm_xy > FLT_EPSILON) {
			const float scale = max_xy / norm_xy;
			accel(0) *= scale;
			accel(1) *= scale;
		}
	}

	// ============================================================
	// 基本判断
	// ============================================================

	bool attitude_valid() const
	{
		float norm_sq = 0.f;

		for (int i = 0; i < 4; ++i) {
			if (!PX4_ISFINITE(_attitude.q[i])) { return false; }
			norm_sq += _attitude.q[i] * _attitude.q[i];
		}

		return norm_sq > 0.25f;
	}

	bool los_fresh(const uint64_t now) const
	{
		return (_last_los_rx_time > 0) && (now - _last_los_rx_time < LOS_TIMEOUT_US);
	}

	bool armed() const
	{
		return _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	bool velocity_valid() const
	{
		return _local_position.v_xy_valid && _local_position.v_z_valid
		       && PX4_ISFINITE(_local_position.vx)
		       && PX4_ISFINITE(_local_position.vy)
		       && PX4_ISFINITE(_local_position.vz);
	}

	bool position_valid() const
	{
		return _local_position.xy_valid && _local_position.z_valid
		       && PX4_ISFINITE(_local_position.x)
		       && PX4_ISFINITE(_local_position.y)
		       && PX4_ISFINITE(_local_position.z);
	}

	Quatf current_attitude_q() const
	{
		Quatf q{_attitude.q};
		const float norm = q.norm();

		if (PX4_ISFINITE(norm) && norm > FLT_EPSILON) {
			q.normalize();

		} else {
			q = Quatf{1.f, 0.f, 0.f, 0.f};
		}

		return q;
	}

	float get_ekf_speed() const
	{
		if (!velocity_valid()) { return NAN; }

		return sqrtf(
			       _local_position.vx * _local_position.vx
			       + _local_position.vy * _local_position.vy
			       + _local_position.vz * _local_position.vz
		       );
	}

	float get_ekf_speed_xy() const
	{
		if (!velocity_valid()) { return NAN; }

		return sqrtf(
			       _local_position.vx * _local_position.vx
			       + _local_position.vy * _local_position.vy
		       );
	}

	bool imu_velocity_available() const
	{
		return _imu_velocity_initialized && _imu_velocity_valid
		       && PX4_ISFINITE(_imu_velocity_ned(0))
		       && PX4_ISFINITE(_imu_velocity_ned(1))
		       && PX4_ISFINITE(_imu_velocity_ned(2));
	}

	bool get_guidance_velocity_ned(Vector3f &velocity_ned, bool *using_imu = nullptr) const
	{
		// V21.14: EKF reset保护优先级高于“EKF valid”标志。
		// reset后的EKF即便仍然声明v_xy/v_z valid，也可能已经发生参考系/速度跳变；
		// 在保护窗口内继续使用连续的IMU积分速度，回切阶段再与EKF平滑融合。
		if (_ekf_velocity_guard_active) {
			if (!imu_velocity_available()) {
				// reset保护期间IMU速度也不可用时，不直接相信可能刚跳变的EKF；
				// 返回false让上层退到固定VC_FALLBACK。
				if (using_imu) { *using_imu = false; }
				return false;
			}

			if (_ekf_velocity_blend_active && velocity_valid()) {
				const Vector3f ekf_velocity{_local_position.vx, _local_position.vy, _local_position.vz};
				const float a = math::constrain(_ekf_velocity_blend_alpha, 0.f, 1.f);
				velocity_ned = _imu_velocity_ned * (1.f - a) + ekf_velocity * a;
				if (using_imu) { *using_imu = a < 0.999f; }
				return true;
			}

			velocity_ned = _imu_velocity_ned;
			if (using_imu) { *using_imu = true; }
			return true;
		}

		if (velocity_valid()) {
			velocity_ned = Vector3f{_local_position.vx, _local_position.vy, _local_position.vz};
			if (using_imu) { *using_imu = false; }
			return true;
		}

		if (imu_velocity_available()) {
			velocity_ned = _imu_velocity_ned;
			if (using_imu) { *using_imu = true; }
			return true;
		}

		if (using_imu) { *using_imu = false; }
		return false;
	}

	float get_guidance_speed() const
	{
		Vector3f velocity_ned;
		if (!get_guidance_velocity_ned(velocity_ned)) { return NAN; }
		return velocity_ned.norm();
	}

	float get_guidance_speed_xy() const
	{
		Vector3f velocity_ned;
		if (!get_guidance_velocity_ned(velocity_ned)) { return NAN; }
		return sqrtf(velocity_ned(0) * velocity_ned(0) + velocity_ned(1) * velocity_ned(1));
	}

	const char *guidance_velocity_source_name() const
	{
		if (_ekf_velocity_guard_active) {
			if (!imu_velocity_available()) { return "FIXED_RST"; }
			return _ekf_velocity_blend_active ? "EKF_BLEND" : "IMU_RST";
		}
		if (velocity_valid()) { return "EKF"; }
		if (imu_velocity_available()) { return "IMU"; }
		return "FIXED";
	}

	void reset_ekf_velocity_guard()
	{
		_ekf_velocity_guard_active = false;
		_ekf_velocity_blend_active = false;
		_ekf_velocity_guard_timeout_warned = false;
		_ekf_velocity_guard_start_time = 0;
		_ekf_velocity_blend_start_time = 0;
		_ekf_velocity_blend_alpha = 0.f;
		_last_ekf_imu_velocity_diff = NAN;
	}

	void seed_imu_velocity(const Vector3f &velocity_ned, const uint64_t now, const bool from_ekf)
	{
		_imu_velocity_ned = velocity_ned;
		_imu_velocity_initialized = true;
		_imu_velocity_valid = PX4_ISFINITE(velocity_ned(0))
			&& PX4_ISFINITE(velocity_ned(1))
			&& PX4_ISFINITE(velocity_ned(2));
		_imu_velocity_seeded_from_ekf = from_ekf;
		_imu_velocity_anchor_time = now;
		_imu_accel_filter_initialized = false;

		const uint64_t accel_time = (_vehicle_acceleration.timestamp_sample > 0)
			? _vehicle_acceleration.timestamp_sample : _vehicle_acceleration.timestamp;
		_imu_velocity_last_sample_time = accel_time;
	}

	void start_ekf_velocity_guard(const uint64_t now, const Vector3f &preferred_seed, const char *reason)
	{
		if (!armed() || !_launch_sequence_complete) {
			return;
		}

		Vector3f seed = preferred_seed;
		bool seed_valid = PX4_ISFINITE(seed(0)) && PX4_ISFINITE(seed(1)) && PX4_ISFINITE(seed(2));

		// 若已经处于保护中，又发生新的EKF reset，优先保持当前连续IMU速度，
		// 绝不重新用刚发生跳变的EKF状态污染惯导速度。
		if (_ekf_velocity_guard_active && imu_velocity_available()) {
			seed = _imu_velocity_ned;
			seed_valid = true;
		} else if (!seed_valid && imu_velocity_available()) {
			seed = _imu_velocity_ned;
			seed_valid = true;
		} else if (!seed_valid && velocity_valid()) {
			seed = Vector3f{_local_position.vx, _local_position.vy, _local_position.vz};
			seed_valid = true;
		}

		if (!seed_valid) {
			PX4_WARN("EKF VEL GUARD requested but no velocity seed (%s)", reason ? reason : "unknown");
			return;
		}

		seed_imu_velocity(seed, now, true);
		_ekf_velocity_guard_active = true;
		_ekf_velocity_blend_active = false;
		_ekf_velocity_guard_timeout_warned = false;
		_ekf_velocity_guard_start_time = now;
		_ekf_velocity_blend_start_time = 0;
		_ekf_velocity_blend_alpha = 0.f;
		_last_ekf_imu_velocity_diff = NAN;

		PX4_WARN("EKF VEL GUARD START reason=%s seed=[%.2f %.2f %.2f] hold=%.2fs max=%.2fs",
			 reason ? reason : "unknown",
			 (double)seed(0), (double)seed(1), (double)seed(2),
			 (double)(EKF_RESET_IMU_HOLD_US * 1e-6f),
			 (double)(EKF_RESET_GUARD_MAX_US * 1e-6f));
	}

	void update_ekf_velocity_guard_state(const uint64_t now)
	{
		if (!_ekf_velocity_guard_active || _ekf_velocity_guard_start_time == 0
		    || now < _ekf_velocity_guard_start_time) {
			return;
		}

		const uint64_t elapsed = now - _ekf_velocity_guard_start_time;

		if (_ekf_velocity_blend_active) {
			if (_ekf_velocity_blend_start_time == 0 || now < _ekf_velocity_blend_start_time) {
				_ekf_velocity_blend_start_time = now;
			}

			const float blend_t = (float)(now - _ekf_velocity_blend_start_time);
			_ekf_velocity_blend_alpha = math::constrain(
				blend_t / (float)EKF_RESET_BLEND_BACK_US, 0.f, 1.f);

			if (_ekf_velocity_blend_alpha >= 0.999f) {
				PX4_INFO("EKF VEL GUARD END blend complete diff=%.2fm/s",
					 (double)_last_ekf_imu_velocity_diff);
				reset_ekf_velocity_guard();
			}
			return;
		}

		if (elapsed < EKF_RESET_IMU_HOLD_US) {
			return;
		}

		if (!velocity_valid() || !imu_velocity_available()) {
			if (elapsed >= EKF_RESET_GUARD_MAX_US && !velocity_valid()) {
				// EKF仍无效时结束“reset专用”标志，但普通IMU fallback继续按V22.01的6s逻辑工作。
				PX4_WARN("EKF VEL GUARD timeout with EKF invalid; continue normal IMU fallback");
				reset_ekf_velocity_guard();
			}
			return;
		}

		const Vector3f ekf_velocity{_local_position.vx, _local_position.vy, _local_position.vz};
		_last_ekf_imu_velocity_diff = (ekf_velocity - _imu_velocity_ned).norm();
		const bool consistent = PX4_ISFINITE(_last_ekf_imu_velocity_diff)
			&& _last_ekf_imu_velocity_diff <= EKF_REACQUIRE_MAX_DIFF_MPS;
		const bool force_reacquire = elapsed >= EKF_RESET_GUARD_MAX_US
			&& PX4_ISFINITE(_last_ekf_imu_velocity_diff)
			&& _last_ekf_imu_velocity_diff <= EKF_REACQUIRE_FORCE_MAX_DIFF_MPS;

		if (consistent || force_reacquire) {
			_ekf_velocity_blend_active = true;
			_ekf_velocity_blend_start_time = now;
			_ekf_velocity_blend_alpha = 0.f;
			PX4_INFO("EKF VEL GUARD BLEND start diff=%.2fm/s forced=%d duration=%.2fs",
				 (double)_last_ekf_imu_velocity_diff, (int)force_reacquire,
				 (double)(EKF_RESET_BLEND_BACK_US * 1e-6f));
		} else if (elapsed >= EKF_RESET_GUARD_MAX_US && !_ekf_velocity_guard_timeout_warned) {
			_ekf_velocity_guard_timeout_warned = true;
			PX4_WARN("EKF VEL GUARD keeps IMU: EKF/IMU diff %.2fm/s > %.2f",
				 (double)_last_ekf_imu_velocity_diff, (double)EKF_REACQUIRE_FORCE_MAX_DIFF_MPS);
		}
	}

	void reset_imu_velocity_fallback()
	{
		_imu_velocity_ned.zero();
		_imu_accel_ned_filtered.zero();
		_imu_velocity_initialized = false;
		_imu_velocity_valid = false;
		_imu_accel_filter_initialized = false;
		_imu_velocity_seeded_from_ekf = false;
		_imu_velocity_anchor_time = 0;
		_imu_velocity_last_sample_time = 0;
		reset_ekf_velocity_guard();
	}

	void update_imu_velocity_estimator(const uint64_t now)
	{
		// 先推进reset保护/回切状态机。保护有效期间，即使EKF仍声明velocity_valid，
		// 也禁止每周期把IMU速度重新锚定到已经跳变的新EKF速度。
		update_ekf_velocity_guard_state(now);

		if (!_ekf_velocity_guard_active && velocity_valid()) {
			seed_imu_velocity(Vector3f{_local_position.vx, _local_position.vy, _local_position.vz}, now, true);
			return;
		}

		// 如果起飞前EKF速度就无效，WAIT_RELEASE/未解锁阶段可以安全假设飞机静止，
		// 每周期把0m/s作为最新锚点；RELEASE后不再清零，转为纯IMU短时积分。
		if (!_ekf_velocity_guard_active
		    && (!armed() || (_state == GuidanceState::WAIT_RELEASE && !_launch_sequence_complete))) {
			seed_imu_velocity(Vector3f{0.f, 0.f, 0.f}, now, false);
			return;
		}

		if (!_imu_velocity_initialized || _imu_velocity_anchor_time == 0
		    || now < _imu_velocity_anchor_time
		    || now - _imu_velocity_anchor_time > IMU_VEL_FALLBACK_MAX_US) {
			_imu_velocity_valid = false;
			return;
		}

		if (_last_accel_rx_time == 0 || now < _last_accel_rx_time
		    || now - _last_accel_rx_time > IMU_ACCEL_TIMEOUT_US || !attitude_valid()) {
			_imu_velocity_valid = false;
			return;
		}

		const uint64_t sample_time = (_vehicle_acceleration.timestamp_sample > 0)
			? _vehicle_acceleration.timestamp_sample : _vehicle_acceleration.timestamp;
		if (sample_time == 0) {
			_imu_velocity_valid = false;
			return;
		}

		if (_imu_velocity_last_sample_time == 0 || sample_time <= _imu_velocity_last_sample_time) {
			_imu_velocity_last_sample_time = sample_time;
			return;
		}

		const float dt = (float)(sample_time - _imu_velocity_last_sample_time) * 1e-6f;
		_imu_velocity_last_sample_time = sample_time;
		if (!PX4_ISFINITE(dt) || dt < IMU_DT_MIN || dt > IMU_DT_MAX) {
			_imu_velocity_valid = false;
			return;
		}

		const Vector3f accel_body{
			_vehicle_acceleration.xyz[0],
			_vehicle_acceleration.xyz[1],
			_vehicle_acceleration.xyz[2]
		};
		if (!PX4_ISFINITE(accel_body(0)) || !PX4_ISFINITE(accel_body(1)) || !PX4_ISFINITE(accel_body(2))) {
			_imu_velocity_valid = false;
			return;
		}

		const Quatf q_sample = attitude_q_at_los_sample(sample_time, now);
		Vector3f accel_ned = Dcmf{q_sample} * accel_body + Vector3f{0.f, 0.f, GRAVITY};
		float accel_norm = accel_ned.norm();
		if (!PX4_ISFINITE(accel_norm)) {
			_imu_velocity_valid = false;
			return;
		}
		if (accel_norm > IMU_ACCEL_MAX_NORM && accel_norm > FLT_EPSILON) {
			accel_ned *= IMU_ACCEL_MAX_NORM / accel_norm;
		}

		if (!_imu_accel_filter_initialized) {
			_imu_accel_ned_filtered = accel_ned;
			_imu_accel_filter_initialized = true;
		} else {
			const float alpha = math::constrain(dt / (IMU_ACCEL_LPF_TAU + dt), 0.f, 1.f);
			_imu_accel_ned_filtered += alpha * (accel_ned - _imu_accel_ned_filtered);
		}

		_imu_velocity_ned += _imu_accel_ned_filtered * dt;
		const float speed = _imu_velocity_ned.norm();
		if (!PX4_ISFINITE(speed)) {
			_imu_velocity_valid = false;
			return;
		}
		if (speed > IMU_VELOCITY_MAX && speed > FLT_EPSILON) {
			_imu_velocity_ned *= IMU_VELOCITY_MAX / speed;
		}
		_imu_velocity_valid = true;
	}



	// ============================================================
	// V22.06 gimbal-load supervisor
	// ============================================================
	float normalize_gimbal_load_component(const float value, const float safe, const float critical) const
	{
		if (!PX4_ISFINITE(value) || critical <= safe) { return 0.f; }
		const float u = math::constrain((value - safe) / (critical - safe), 0.f, 1.f);
		return smoothstep01(u);
	}

	bool attitude_reset_guard_active(const uint64_t now) const
	{
		return _attitude_reset_guard_until > 0 && now < _attitude_reset_guard_until;
	}

	void update_gimbal_load_state_hysteresis()
	{
		switch (_gimbal_load_state) {
		case GimbalLoadState::NORMAL:
			if (_gimbal_load >= GIMBAL_LOAD_CAUTION_ENTER) { _gimbal_load_state = GimbalLoadState::CAUTION; }
			break;
		case GimbalLoadState::CAUTION:
			if (_gimbal_load >= GIMBAL_LOAD_HIGH_ENTER) { _gimbal_load_state = GimbalLoadState::HIGH; }
			else if (_gimbal_load <= GIMBAL_LOAD_CAUTION_EXIT) { _gimbal_load_state = GimbalLoadState::NORMAL; }
			break;
		case GimbalLoadState::HIGH:
			if (_gimbal_load >= GIMBAL_LOAD_CRITICAL_ENTER) { _gimbal_load_state = GimbalLoadState::CRITICAL; }
			else if (_gimbal_load <= GIMBAL_LOAD_HIGH_EXIT) { _gimbal_load_state = GimbalLoadState::CAUTION; }
			break;
		case GimbalLoadState::CRITICAL:
			if (_gimbal_load <= GIMBAL_LOAD_CRITICAL_EXIT) { _gimbal_load_state = GimbalLoadState::HIGH; }
			break;
		}
	}

	void update_gimbal_load_supervisor(const uint64_t now)
	{
		// Update only when a new gimbal/vision packet has been accepted.
		if (_last_los_rx_time == 0 || _last_los_rx_time == _gimbal_load_last_los_rx_time) { return; }
		_gimbal_load_last_los_rx_time = _last_los_rx_time;

		float dt = LOOP_INTERVAL_US * 1e-6f;
		if (_gimbal_load_last_update_time > 0 && now > _gimbal_load_last_update_time) {
			dt = math::constrain((float)(now - _gimbal_load_last_update_time) * 1e-6f, 0.005f, 0.10f);
		}
		_gimbal_load_last_update_time = now;

		// Off-axis angle of the actual gimbal boresight relative to the zero-gimbal Body -Z boresight.
		const Vector3f center_guidance = calculate_los_body(_los.gimbal_roll, _los.gimbal_pitch, 0.f, 0.f);
		const Vector3f center_body = guidance_to_px4_body(center_guidance);
		const Vector3f zero_body{0.f, 0.f, -1.f};
		_gimbal_offaxis_deg = vector_angle_deg(center_body, zero_body);

		const float rate_mag_rad_s = sqrtf(_gimbal_az_rate_filtered * _gimbal_az_rate_filtered
			+ _gimbal_pitch_rate_filtered * _gimbal_pitch_rate_filtered);
		_gimbal_rate_mag_deg_s = PX4_ISFINITE(rate_mag_rad_s) ? rate_mag_rad_s * 180.f / M_PI_F : 0.f;

		float accel_mag_deg_s2 = 0.f;
		if (_gimbal_load_rate_initialized && dt > 0.f) {
			const float daz = (_gimbal_az_rate_filtered - _gimbal_load_prev_az_rate) / dt;
			const float dp = (_gimbal_pitch_rate_filtered - _gimbal_load_prev_pitch_rate) / dt;
			const float accel_mag_rad_s2 = sqrtf(daz * daz + dp * dp);
			if (PX4_ISFINITE(accel_mag_rad_s2)) { accel_mag_deg_s2 = accel_mag_rad_s2 * 180.f / M_PI_F; }
		}
		_gimbal_load_prev_az_rate = _gimbal_az_rate_filtered;
		_gimbal_load_prev_pitch_rate = _gimbal_pitch_rate_filtered;
		_gimbal_load_rate_initialized = true;
		_gimbal_accel_mag_deg_s2 = accel_mag_deg_s2;

		const float alpha_acc = dt / (GIMBAL_LOAD_ACCEL_LPF_TAU + dt);
		_gimbal_accel_mag_filtered_deg_s2 += alpha_acc * (accel_mag_deg_s2 - _gimbal_accel_mag_filtered_deg_s2);

		const float l_theta = normalize_gimbal_load_component(_gimbal_offaxis_deg,
			GIMBAL_LOAD_OFFAXIS_SAFE_DEG, GIMBAL_LOAD_OFFAXIS_CRITICAL_DEG);
		const float l_rate = normalize_gimbal_load_component(_gimbal_rate_mag_deg_s,
			GIMBAL_LOAD_RATE_SAFE_DEG_S, GIMBAL_LOAD_RATE_CRITICAL_DEG_S);
		const float l_acc = normalize_gimbal_load_component(_gimbal_accel_mag_filtered_deg_s2,
			GIMBAL_LOAD_ACCEL_SAFE_DEG_S2, GIMBAL_LOAD_ACCEL_CRITICAL_DEG_S2);

		// Geometry risk and dynamic load must not be treated as the same thing.  A large off-axis angle with
		// low gimbal rate is a field-of-regard warning, but blindly reducing axial acceleration can actually
		// reduce body tilt and make the off-axis angle larger.  Therefore control scheduling is driven by
		// rate/acceleration, with off-axis only amplifying an already-dynamic condition.
		_gimbal_geometry_load = l_theta;
		_gimbal_dynamic_load = fmaxf(l_rate, l_acc);
		_gimbal_load_raw = fmaxf(_gimbal_geometry_load, _gimbal_dynamic_load);

		const float alpha_load = dt / (GIMBAL_LOAD_LPF_TAU + dt);
		_gimbal_load += alpha_load * (_gimbal_load_raw - _gimbal_load);
		_gimbal_load = math::constrain(_gimbal_load, 0.f, 1.f);
		update_gimbal_load_state_hysteresis();

		// Control load: dynamic stress is primary.  Large off-axis geometry can raise its severity by up to 35%,
		// but geometry alone never commands the aircraft to slow down.
		_gimbal_control_load = math::constrain(
			_gimbal_dynamic_load * (1.f + 0.35f * _gimbal_geometry_load), 0.f, 1.f);

		// In the dynamically safe region the supervisor is intentionally transparent even if geometric off-axis
		// is large.  The latter is logged as a risk that requires future body/gimbal unloading logic.
		if (_gimbal_control_load <= GIMBAL_LOAD_NO_ACTION) {
			_gimbal_supervisor_axial_cap = AXIAL_ACCEL_MAX;
			_gimbal_supervisor_slew_cap_deg_s = GUIDANCE_ATTITUDE_SLEW_HIGH_DEG_S;
			_gimbal_supervisor_lead_cap_deg = GUIDANCE_ATTITUDE_LEAD_MAX_DEG;
			return;
		}

		const float u = math::constrain((_gimbal_control_load - GIMBAL_LOAD_NO_ACTION)
			/ fmaxf(1.f - GIMBAL_LOAD_NO_ACTION, 0.01f), 0.f, 1.f);
		const float us = smoothstep01(u);
		_gimbal_supervisor_axial_cap = lerp(AXIAL_ACCEL_MAX, GIMBAL_SUPERVISOR_AXIAL_MIN, us);
		_gimbal_supervisor_slew_cap_deg_s = lerp(GUIDANCE_ATTITUDE_SLEW_HIGH_DEG_S,
			GIMBAL_SUPERVISOR_SLEW_MIN_DEG_S, us);

		// Lead is intentionally tightened more strongly only after dynamic load has clearly left the safe region.
		if (_gimbal_control_load <= 0.65f) {
			const float ul = math::constrain((_gimbal_control_load - GIMBAL_LOAD_NO_ACTION) / (0.65f - GIMBAL_LOAD_NO_ACTION), 0.f, 1.f);
			_gimbal_supervisor_lead_cap_deg = lerp(GUIDANCE_ATTITUDE_LEAD_MAX_DEG,
				GIMBAL_SUPERVISOR_LEAD_HIGH_DEG, smoothstep01(ul));
		} else {
			const float ul = math::constrain((_gimbal_control_load - 0.65f) / 0.35f, 0.f, 1.f);
			_gimbal_supervisor_lead_cap_deg = lerp(GIMBAL_SUPERVISOR_LEAD_HIGH_DEG,
				GIMBAL_SUPERVISOR_LEAD_CRITICAL_DEG, smoothstep01(ul));
		}
	}

	// ============================================================
	// Guidance/光轴坐标 -> PX4标准Body FRD
	// ============================================================
	Vector3f guidance_to_px4_body(const Vector3f &v_g) const
	{
		// 当前真实安装：Camera +X->Body +Y，Camera +Y->Body -X，Camera +Z(光轴)->Body -Z。
		// 当前Guidance定义：Gx=-CameraY，Gy=CameraX，Gz=CameraZ，因此Bx=Gx、By=Gy、Bz=-Gz；这里只改变坐标表达，不改变物理向量。
		return Vector3f{v_g(0), v_g(1), -v_g(2)};
	}

	// ============================================================
	// LOS几何
	// ============================================================

	Vector3f rotate_zero_to_guidance(const Vector3f &v_zero, const float gimbal_azimuth, const float gimbal_pitch) const
	{
		// 与原有LOS几何完全相同：先Ry(-pitch)，再Rx(-azimuth)。
		const float sa = sinf(gimbal_azimuth), ca = cosf(gimbal_azimuth);
		const float sb = sinf(gimbal_pitch), cb = cosf(gimbal_pitch);
		const Vector3f after_pitch{cb * v_zero(0) - sb * v_zero(2), v_zero(1), sb * v_zero(0) + cb * v_zero(2)};
		return Vector3f{after_pitch(0), ca * after_pitch(1) + sa * after_pitch(2), -sa * after_pitch(1) + ca * after_pitch(2)};
	}

	Vector3f calculate_los_body(const float gimbal_azimuth, const float gimbal_pitch,
				    const float miss_az, const float miss_el) const
	{
		// 历史函数名保留；输出实际为Guidance/光轴坐标。Camera: +X右、+Y上、+Z光轴。
		const float saz = sinf(miss_az), caz = cosf(miss_az), sel = sinf(miss_el), cel = cosf(miss_el);
		const Vector3f los_camera{saz * cel, sel, caz * cel};
		Vector3f los_guidance = rotate_zero_to_guidance(Vector3f{-los_camera(1), los_camera(0), los_camera(2)},
							       gimbal_azimuth, gimbal_pitch);
		const float norm = los_guidance.norm();
		if (!PX4_ISFINITE(norm) || norm <= FLT_EPSILON) { return Vector3f{}; }
		los_guidance /= norm;
		return los_guidance;
	}

	Vector3f px4_body_to_guidance(const Vector3f &v_b) const
	{
		// G->B为[Bx,By,Bz]=[Gx,Gy,-Gz]，该变换自身就是逆变换。
		return Vector3f{v_b(0), v_b(1), -v_b(2)};
	}

	bool body_rate_valid(const uint64_t now) const
	{
		if (_last_body_rate_rx_time == 0 || now < _last_body_rate_rx_time
		    || now - _last_body_rate_rx_time > BODY_RATE_TIMEOUT_US) { return false; }
		return PX4_ISFINITE(_angular_velocity.xyz[0]) && PX4_ISFINITE(_angular_velocity.xyz[1])
		       && PX4_ISFINITE(_angular_velocity.xyz[2]);
	}

	uint64_t body_rate_measurement_time(const uint64_t now) const
	{
		if (_angular_velocity.timestamp_sample > 0 && _angular_velocity.timestamp_sample <= now
		    && now - _angular_velocity.timestamp_sample <= BODY_RATE_TIMEOUT_US) {
			return _angular_velocity.timestamp_sample;
		}
		if (_angular_velocity.timestamp > 0 && _angular_velocity.timestamp <= now
		    && now - _angular_velocity.timestamp <= BODY_RATE_TIMEOUT_US) {
			return _angular_velocity.timestamp;
		}
		return now;
	}

	void push_body_rate_history(const uint64_t now)
	{
		if (!PX4_ISFINITE(_angular_velocity.xyz[0]) || !PX4_ISFINITE(_angular_velocity.xyz[1])
		    || !PX4_ISFINITE(_angular_velocity.xyz[2])) { return; }
		BodyRateHistorySample &s = _body_rate_history[_body_rate_history_head];
		s.t = body_rate_measurement_time(now);
		s.omega = Vector3f{_angular_velocity.xyz[0], _angular_velocity.xyz[1], _angular_velocity.xyz[2]};
		_body_rate_history_head = (uint8_t)((_body_rate_history_head + 1U) % BODY_RATE_HISTORY_LEN);
		if (_body_rate_history_count < BODY_RATE_HISTORY_LEN) { _body_rate_history_count++; }
	}

	bool body_rate_at_time(const uint64_t sample_time, Vector3f &omega_out,
	                      int64_t &align_error_us, bool &interpolated) const
	{
		if (sample_time == 0 || _body_rate_history_count == 0) { return false; }
		const BodyRateHistorySample *before = nullptr;
		const BodyRateHistorySample *after = nullptr;
		const BodyRateHistorySample *nearest = nullptr;
		uint64_t nearest_abs_dt = UINT64_MAX;

		for (uint8_t i = 0; i < _body_rate_history_count; i++) {
			const BodyRateHistorySample &s = _body_rate_history[i];
			if (s.t == 0) { continue; }
			const uint64_t abs_dt = s.t > sample_time ? s.t - sample_time : sample_time - s.t;
			if (abs_dt < nearest_abs_dt) { nearest_abs_dt = abs_dt; nearest = &s; }
			if (s.t <= sample_time && (before == nullptr || s.t > before->t)) { before = &s; }
			if (s.t >= sample_time && (after == nullptr || s.t < after->t)) { after = &s; }
		}

		if (before != nullptr && after != nullptr && after->t > before->t
		    && after->t - before->t <= BODY_RATE_INTERP_MAX_GAP_US) {
			const float a = (float)(sample_time - before->t) / (float)(after->t - before->t);
			omega_out = before->omega * (1.f - a) + after->omega * a;
			align_error_us = 0;
			interpolated = true;
			return PX4_ISFINITE(omega_out(0)) && PX4_ISFINITE(omega_out(1)) && PX4_ISFINITE(omega_out(2));
		}

		if (nearest != nullptr && nearest_abs_dt <= BODY_RATE_NEAREST_MAX_US) {
			omega_out = nearest->omega;
			align_error_us = (int64_t)nearest->t - (int64_t)sample_time;
			interpolated = false;
			return PX4_ISFINITE(omega_out(0)) && PX4_ISFINITE(omega_out(1)) && PX4_ISFINITE(omega_out(2));
		}
		return false;
	}

	// 【V20.9修改】把当前CGTD070包里的“最新云台角”按RX-HRT写入历史；后续只从历史回查capture时刻对应角度。
	void push_gimbal_history(const uint64_t sample_time, const float azimuth, const float pitch)
	{
		if (sample_time == 0 || !PX4_ISFINITE(azimuth) || !PX4_ISFINITE(pitch)) {
			return;
		}

		GimbalHistorySample &s = _gimbal_history[_gimbal_history_head];
		s.t = sample_time;
		s.azimuth = azimuth;
		s.pitch = pitch;
		_gimbal_history_head = (uint8_t)((_gimbal_history_head + 1U) % GIMBAL_HISTORY_LEN);

		if (_gimbal_history_count < GIMBAL_HISTORY_LEN) {
			_gimbal_history_count++;
		}
	}

	// 【V20.9修改】按PX4 HRT时刻在线性插值云台历史；azimuth使用最短角差插值，避免跨+-pi时产生跳变。
	bool gimbal_at_time(const uint64_t sample_time, float &azimuth_out, float &pitch_out,
			    int64_t &align_error_us, bool &interpolated) const
	{
		if (sample_time == 0 || _gimbal_history_count == 0) {
			return false;
		}

		const GimbalHistorySample *before = nullptr;
		const GimbalHistorySample *after = nullptr;
		const GimbalHistorySample *nearest = nullptr;
		uint64_t nearest_abs_dt = UINT64_MAX;

		for (uint8_t i = 0; i < _gimbal_history_count; ++i) {
			const GimbalHistorySample &s = _gimbal_history[i];

			if (s.t == 0) {
				continue;
			}

			const uint64_t abs_dt = s.t > sample_time ? s.t - sample_time : sample_time - s.t;

			if (abs_dt < nearest_abs_dt) {
				nearest_abs_dt = abs_dt;
				nearest = &s;
			}

			if (s.t <= sample_time && (before == nullptr || s.t > before->t)) {
				before = &s;
			}

			if (s.t >= sample_time && (after == nullptr || s.t < after->t)) {
				after = &s;
			}
		}

		if (before != nullptr && after != nullptr && after->t > before->t
		    && after->t - before->t <= GIMBAL_INTERP_MAX_GAP_US) {
			const float a = (float)(sample_time - before->t) / (float)(after->t - before->t);
			const float da = atan2f(sinf(after->azimuth - before->azimuth),
					       cosf(after->azimuth - before->azimuth));
			azimuth_out = before->azimuth + a * da;
			azimuth_out = atan2f(sinf(azimuth_out), cosf(azimuth_out));
			pitch_out = before->pitch + a * (after->pitch - before->pitch);
			align_error_us = 0;
			interpolated = true;
			return PX4_ISFINITE(azimuth_out) && PX4_ISFINITE(pitch_out);
		}

		if (nearest != nullptr && nearest_abs_dt <= GIMBAL_NEAREST_MAX_US) {
			azimuth_out = nearest->azimuth;
			pitch_out = nearest->pitch;
			align_error_us = (int64_t)nearest->t - (int64_t)sample_time;
			interpolated = false;
			return PX4_ISFINITE(azimuth_out) && PX4_ISFINITE(pitch_out);
		}

		return false;
	}

	// 【V20.9修改】利用已知100ms视觉延迟，在capture时刻前后各40ms取历史云台角做中心差分；相位中心就在capture时刻。
	bool gimbal_rate_at_time_centered(const uint64_t sample_time, float &az_rate_out, float &pitch_rate_out) const
	{
		if (sample_time <= GIMBAL_RATE_CENTER_HALF_WINDOW_US) {
			return false;
		}

		const uint64_t t0 = sample_time - GIMBAL_RATE_CENTER_HALF_WINDOW_US;
		const uint64_t t1 = sample_time + GIMBAL_RATE_CENTER_HALF_WINDOW_US;

		float az0 = 0.f;
		float pitch0 = 0.f;
		float az1 = 0.f;
		float pitch1 = 0.f;
		int64_t err0 = 0;
		int64_t err1 = 0;
		bool interp0 = false;
		bool interp1 = false;

		if (!gimbal_at_time(t0, az0, pitch0, err0, interp0)
		    || !gimbal_at_time(t1, az1, pitch1, err1, interp1)) {
			return false;
		}

		const float dt = (float)(t1 - t0) * 1e-6f;

		if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
			return false;
		}

		const float delta_az = atan2f(sinf(az1 - az0), cosf(az1 - az0));
		az_rate_out = delta_az / dt;
		pitch_rate_out = (pitch1 - pitch0) / dt;

		return PX4_ISFINITE(az_rate_out) && PX4_ISFINITE(pitch_rate_out);
	}


	// 【V21修改】把当前有效direct miss angle按图像capture时刻写入历史；历史仅保存PX4内部统一后的右正/上正rad语义。
	void push_miss_history(const uint64_t sample_time, const float azimuth, const float elevation)
	{
		if (sample_time == 0 || !PX4_ISFINITE(azimuth) || !PX4_ISFINITE(elevation)) {
			return;
		}

		MissHistorySample &s = _miss_history[_miss_history_head];
		s.t = sample_time;
		s.azimuth = azimuth;
		s.elevation = elevation;
		_miss_history_head = (uint8_t)((_miss_history_head + 1U) % MISS_HISTORY_LEN);

		if (_miss_history_count < MISS_HISTORY_LEN) {
			_miss_history_count++;
		}
	}

	// 【V21修改】目标无效/重捕获时清空miss历史，禁止跨目标丢失区间做差分产生伪IMG角速度。
	void reset_miss_history()
	{
		for (uint8_t i = 0; i < MISS_HISTORY_LEN; ++i) {
			_miss_history[i] = MissHistorySample{};
		}
		_miss_history_head = 0;
		_miss_history_count = 0;
		_diag_miss_az_rate_px4 = 0.f;
		_diag_miss_el_rate_px4 = 0.f;
		_diag_miss_az_rate_backward3 = 0.f;
		_diag_miss_el_rate_backward3 = 0.f;
		_diag_miss_rate_mode = 0;
		_diag_miss_rate_valid = false;
	}

	// 【V21修改】按PX4 HRT/capture时间插值direct miss angle；方位角按最短角差插值，俯仰角线性插值。
	bool miss_at_time(const uint64_t sample_time, float &azimuth_out, float &elevation_out,
			  int64_t &align_error_us, bool &interpolated) const
	{
		if (sample_time == 0 || _miss_history_count == 0) {
			return false;
		}

		const MissHistorySample *before = nullptr;
		const MissHistorySample *after = nullptr;
		const MissHistorySample *nearest = nullptr;
		uint64_t nearest_abs_dt = UINT64_MAX;

		for (uint8_t i = 0; i < _miss_history_count; ++i) {
			const MissHistorySample &s = _miss_history[i];

			if (s.t == 0) {
				continue;
			}

			const uint64_t abs_dt = s.t > sample_time ? s.t - sample_time : sample_time - s.t;

			if (abs_dt < nearest_abs_dt) {
				nearest_abs_dt = abs_dt;
				nearest = &s;
			}

			if (s.t <= sample_time && (before == nullptr || s.t > before->t)) {
				before = &s;
			}

			if (s.t >= sample_time && (after == nullptr || s.t < after->t)) {
				after = &s;
			}
		}

		if (before != nullptr && after != nullptr && after->t > before->t
		    && after->t - before->t <= MISS_INTERP_MAX_GAP_US) {
			const float a = (float)(sample_time - before->t) / (float)(after->t - before->t);
			const float da = atan2f(sinf(after->azimuth - before->azimuth),
					       cosf(after->azimuth - before->azimuth));
			azimuth_out = before->azimuth + a * da;
			azimuth_out = atan2f(sinf(azimuth_out), cosf(azimuth_out));
			elevation_out = before->elevation + a * (after->elevation - before->elevation);
			align_error_us = 0;
			interpolated = true;
			return PX4_ISFINITE(azimuth_out) && PX4_ISFINITE(elevation_out);
		}

		if (nearest != nullptr && nearest_abs_dt <= MISS_NEAREST_MAX_US) {
			azimuth_out = nearest->azimuth;
			elevation_out = nearest->elevation;
			align_error_us = (int64_t)nearest->t - (int64_t)sample_time;
			interpolated = false;
			return PX4_ISFINITE(azimuth_out) && PX4_ISFINITE(elevation_out);
		}

		return false;
	}

	// 【V21修改】用observer时刻前后各20ms的direct miss angle做中心差分，得到PX4自行重构的相机相对LOS角速度。
	bool miss_rate_at_time_centered(const uint64_t sample_time, float &az_rate_out, float &el_rate_out) const
	{
		if (sample_time <= MISS_RATE_CENTER_HALF_WINDOW_US) {
			return false;
		}

		const uint64_t t0 = sample_time - MISS_RATE_CENTER_HALF_WINDOW_US;
		const uint64_t t1 = sample_time + MISS_RATE_CENTER_HALF_WINDOW_US;

		float az0 = 0.f;
		float el0 = 0.f;
		float az1 = 0.f;
		float el1 = 0.f;
		int64_t err0 = 0;
		int64_t err1 = 0;
		bool interp0 = false;
		bool interp1 = false;

		if (!miss_at_time(t0, az0, el0, err0, interp0)
		    || !miss_at_time(t1, az1, el1, err1, interp1)) {
			return false;
		}

		const float dt = (float)(t1 - t0) * 1e-6f;

		if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
			return false;
		}

		const float delta_az = atan2f(sinf(az1 - az0), cosf(az1 - az0));
		az_rate_out = delta_az / dt;
		el_rate_out = (el1 - el0) / dt;

		return PX4_ISFINITE(az_rate_out) && PX4_ISFINITE(el_rate_out);
	}

	// V21.12: small pivoted Gaussian elimination for the 3x3 normal equation used by causal quadratic fit.
	bool solve_3x3(float A[3][3], float b[3], float x[3]) const
	{
		float aug[3][4]{};
		for (int r = 0; r < 3; ++r) {
			for (int c = 0; c < 3; ++c) { aug[r][c] = A[r][c]; }
			aug[r][3] = b[r];
		}
		for (int col = 0; col < 3; ++col) {
			int pivot = col;
			float pivot_abs = fabsf(aug[col][col]);
			for (int r = col + 1; r < 3; ++r) {
				const float a = fabsf(aug[r][col]);
				if (a > pivot_abs) { pivot_abs = a; pivot = r; }
			}
			if (!PX4_ISFINITE(pivot_abs) || pivot_abs < 1e-7f) { return false; }
			if (pivot != col) {
				for (int c = col; c < 4; ++c) { const float tmp = aug[col][c]; aug[col][c] = aug[pivot][c]; aug[pivot][c] = tmp; }
			}
			const float inv = 1.f / aug[col][col];
			for (int c = col; c < 4; ++c) { aug[col][c] *= inv; }
			for (int r = 0; r < 3; ++r) {
				if (r == col) { continue; }
				const float f = aug[r][col];
				for (int c = col; c < 4; ++c) { aug[r][c] -= f * aug[col][c]; }
			}
		}
		for (int i = 0; i < 3; ++i) { x[i] = aug[i][3]; }
		return PX4_ISFINITE(x[0]) && PX4_ISFINITE(x[1]) && PX4_ISFINITE(x[2]);
	}

	// V21.12: six-sample causal quadratic least-squares derivative evaluated at the newest capture time.
	// It uses no future image sample. Actual capture timestamps are used, so 50Hz jitter does not bias the derivative.
	bool miss_rate_at_time_causal_polyfit(const uint64_t sample_time, float &az_rate_out, float &el_rate_out) const
	{
		if (sample_time == 0 || _miss_history_count < MISS_RATE_POLYFIT_POINTS) { return false; }

		MissHistorySample selected[MISS_RATE_POLYFIT_POINTS]{};
		uint8_t selected_count = 0;
		// Repeatedly select the newest sample <= cursor. N=6 and history=32, so this is cheap at 50Hz.
		uint64_t cursor = sample_time;
		for (uint8_t n = 0; n < MISS_RATE_POLYFIT_POINTS; ++n) {
			const MissHistorySample *best = nullptr;
			for (uint8_t i = 0; i < _miss_history_count; ++i) {
				const MissHistorySample &m = _miss_history[i];
				if (m.t > 0 && m.t <= cursor && (best == nullptr || m.t > best->t)) { best = &m; }
			}
			if (best == nullptr) { return false; }
			selected[MISS_RATE_POLYFIT_POINTS - 1U - n] = *best;
			selected_count++;
			if (best->t == 0) { return false; }
			cursor = best->t - 1ULL;
		}
		if (selected_count != MISS_RATE_POLYFIT_POINTS) { return false; }

		for (uint8_t i = 1; i < MISS_RATE_POLYFIT_POINTS; ++i) {
			if (selected[i].t <= selected[i - 1].t) { return false; }
			const uint64_t gap = selected[i].t - selected[i - 1].t;
			if (gap > MISS_INTERP_MAX_GAP_US) { return false; }
		}
		const float mean_step_s = (selected[MISS_RATE_POLYFIT_POINTS - 1U].t - selected[0].t)
			* 1e-6f / (float)(MISS_RATE_POLYFIT_POINTS - 1U);
		if (!PX4_ISFINITE(mean_step_s) || mean_step_s < 0.008f || mean_step_s > 0.040f) { return false; }

		float az_unwrapped[MISS_RATE_POLYFIT_POINTS]{};
		az_unwrapped[MISS_RATE_POLYFIT_POINTS - 1U] = selected[MISS_RATE_POLYFIT_POINTS - 1U].azimuth;
		for (int i = (int)MISS_RATE_POLYFIT_POINTS - 2; i >= 0; --i) {
			const float d = atan2f(sinf(selected[i].azimuth - selected[i + 1].azimuth),
					       cosf(selected[i].azimuth - selected[i + 1].azimuth));
			az_unwrapped[i] = az_unwrapped[i + 1] + d;
		}

		float S0 = 0.f, S1 = 0.f, S2 = 0.f, S3 = 0.f, S4 = 0.f;
		float Yaz0 = 0.f, Yaz1 = 0.f, Yaz2 = 0.f;
		float Yel0 = 0.f, Yel1 = 0.f, Yel2 = 0.f;
		const uint64_t t_ref = selected[MISS_RATE_POLYFIT_POINTS - 1U].t;

		for (uint8_t i = 0; i < MISS_RATE_POLYFIT_POINTS; ++i) {
			const float u = ((int64_t)selected[i].t - (int64_t)t_ref) * 1e-6f / mean_step_s;
			const float u2 = u * u;
			S0 += 1.f; S1 += u; S2 += u2; S3 += u2 * u; S4 += u2 * u2;
			Yaz0 += az_unwrapped[i]; Yaz1 += u * az_unwrapped[i]; Yaz2 += u2 * az_unwrapped[i];
			Yel0 += selected[i].elevation; Yel1 += u * selected[i].elevation; Yel2 += u2 * selected[i].elevation;
		}

		float Aaz[3][3]{{S0, S1, S2}, {S1, S2, S3}, {S2, S3, S4}};
		float Ael[3][3]{{S0, S1, S2}, {S1, S2, S3}, {S2, S3, S4}};
		float baz[3]{Yaz0, Yaz1, Yaz2};
		float bel[3]{Yel0, Yel1, Yel2};
		float xaz[3]{};
		float xel[3]{};
		if (!solve_3x3(Aaz, baz, xaz) || !solve_3x3(Ael, bel, xel)) { return false; }

		az_rate_out = xaz[1] / mean_step_s;
		el_rate_out = xel[1] / mean_step_s;
		return PX4_ISFINITE(az_rate_out) && PX4_ISFINITE(el_rate_out);
	}

	// V21.10: 在当前capture时刻使用三点后向二阶差分，避免中心差分必须等待“未来20ms”而把observer再退后20ms。
	// f'(t) ~= [3f(t)-4f(t-h)+f(t-2h)]/(2h)。az使用逐段最短角差展开，避免跨+-pi时跳变。
	bool miss_rate_at_time_backward(const uint64_t sample_time, float &az_rate_out, float &el_rate_out) const
	{
		const uint64_t h_us = MISS_RATE_BACKWARD_STEP_US;

		if (sample_time <= 2ULL * h_us) {
			return false;
		}

		float az0 = 0.f, el0 = 0.f;
		float az1 = 0.f, el1 = 0.f;
		float az2 = 0.f, el2 = 0.f;
		int64_t err0 = 0, err1 = 0, err2 = 0;
		bool interp0 = false, interp1 = false, interp2 = false;

		if (!miss_at_time(sample_time, az0, el0, err0, interp0)
		    || !miss_at_time(sample_time - h_us, az1, el1, err1, interp1)
		    || !miss_at_time(sample_time - 2ULL * h_us, az2, el2, err2, interp2)) {
			return false;
		}

		// 将历史az围绕当前az连续展开。
		const float d01 = atan2f(sinf(az0 - az1), cosf(az0 - az1));
		const float az1_unwrapped = az0 - d01;
		const float d12 = atan2f(sinf(az1 - az2), cosf(az1 - az2));
		const float az2_unwrapped = az1_unwrapped - d12;

		const float h = (float)h_us * 1e-6f;
		if (!PX4_ISFINITE(h) || h <= FLT_EPSILON) {
			return false;
		}

		az_rate_out = (3.f * az0 - 4.f * az1_unwrapped + az2_unwrapped) / (2.f * h);
		el_rate_out = (3.f * el0 - 4.f * el1 + el2) / (2.f * h);

		return PX4_ISFINITE(az_rate_out) && PX4_ISFINITE(el_rate_out);
	}

	bool update_los_from_cgtd070(const cgtd070_status_s &src, const uint64_t now)
	{
		// 【V21修改】先定义当前CGTD070完整包解析完成的RX-HRT；真机图像capture仍固定认为比RX早100ms。
		const bool rx_timestamp_valid =
			src.timestamp > 0 && src.timestamp <= now && now - src.timestamp <= LOS_TIMEOUT_US;
		const uint64_t rx_time = rx_timestamp_valid ? src.timestamp : now;

		// 【V21修改】板卡黑盒LOS-rate不再决定observer有效性，只转换后保存到ULog用于与PX4 miss-rate长期A/B比较。
		_diag_board_los_az_rate = PX4_ISFINITE(src.los_az_rate)
			? src.los_az_rate * CGTD070_DEG_TO_RAD * CGTD070_LOS_AZ_RATE_SIGN : NAN;
		_diag_board_los_el_rate = PX4_ISFINITE(src.los_el_rate)
			? src.los_el_rate * CGTD070_DEG_TO_RAD * CGTD070_LOS_EL_RATE_SIGN : NAN;

		// 【V21修改】B方案保持不变：同包gimbal_roll/pitch是RX时刻最新云台角，先按RX时刻写入云台历史。
		const float gimbal_az_rx =
			src.gimbal_roll * CGTD070_DEG_TO_RAD * CGTD070_GIMBAL_AZ_SIGN;
		const float gimbal_pitch_rx =
			src.gimbal_pitch * CGTD070_DEG_TO_RAD * CGTD070_GIMBAL_PITCH_SIGN;

		if (PX4_ISFINITE(gimbal_az_rx) && PX4_ISFINITE(gimbal_pitch_rx)) {
			push_gimbal_history(rx_time, gimbal_az_rx, gimbal_pitch_rx);
		}

		// 【V21修改】capture时刻仍统一定义为RX-HRT减固定100ms；启动早期时间不足时等待历史建立。
		if (rx_time <= VISION_FIXED_DELAY_US) {
			_los = vehicle_target_los_s{};
			_los.target_valid = false;
			_last_los_rx_time = now;
			_diag_capture_time = 0;
			_diag_capture_age_us = (uint32_t)VISION_FIXED_DELAY_US;
			_diag_capture_time_valid = false;
			_diag_miss_rate_valid = false;
			reset_inertial_los_reconstruction();
			return false;
		}

		const uint64_t capture_time = rx_time - VISION_FIXED_DELAY_US;

		// 【V21修改】direct miss angle仍来自当前视觉包，但其物理时间标记为capture_time，并在入口统一为右正/上正rad。
		const float miss_az_capture =
			src.los_azimuth * CGTD070_DEG_TO_RAD * CGTD070_MISS_AZ_SIGN;
		const float miss_el_capture =
			src.los_elevation * CGTD070_DEG_TO_RAD * CGTD070_MISS_EL_SIGN;

		_diag_capture_time = capture_time;
		_diag_capture_age_us = (uint32_t)VISION_FIXED_DELAY_US;
		_diag_vision_frame_seq = 0;
		_diag_capture_time_valid = rx_timestamp_valid;

		// 【V21修改】目标无效或direct miss本身无效时清空miss历史，禁止下一次重捕获跨无效区间做差分。
		if (!src.target_valid || !PX4_ISFINITE(miss_az_capture) || !PX4_ISFINITE(miss_el_capture)) {
			reset_miss_history();
			_los = vehicle_target_los_s{};
			_los.timestamp = capture_time;
			_los.target_valid = false;
			_last_los_rx_time = now;
			reset_inertial_los_reconstruction();
			return false;
		}

		// 【V21修改】把当前有效direct miss写入capture时间轴历史，作为PX4自行重构IMG角速度的唯一正式视觉输入。
		push_miss_history(capture_time, miss_az_capture, miss_el_capture);

		// V21.12: observer仍直接位于capture_time。正式miss-rate优先使用6点因果二次最小二乘端点导数；
		// 重捕获初期历史不足时回退三点后向差分，因此仍只要求至少3帧即可启动。
		if (capture_time <= 2ULL * MISS_RATE_BACKWARD_STEP_US) {
			_los = vehicle_target_los_s{};
			_los.timestamp = capture_time;
			_los.target_valid = false;
			_last_los_rx_time = now;
			_diag_miss_rate_valid = false;
			reset_inertial_los_reconstruction();
			return false;
		}

		const uint64_t observer_time = capture_time;

		// V21.12: miss角与formal miss-rate都严格标记为capture_time；板卡raw rate继续只作诊断。
		float miss_az_observer = 0.f;
		float miss_el_observer = 0.f;
		int64_t miss_align_error_us = 0;
		bool miss_interpolated = false;
		float miss_az_rate_px4 = 0.f;
		float miss_el_rate_px4 = 0.f;

		const bool miss_ok =
			miss_at_time(observer_time, miss_az_observer, miss_el_observer,
				     miss_align_error_us, miss_interpolated);
		const bool miss_rate_ok =
			miss_rate_at_time_robust(observer_time, miss_az_rate_px4, miss_el_rate_px4);

		_diag_miss_az_rate_px4 = miss_az_rate_px4;
		_diag_miss_el_rate_px4 = miss_el_rate_px4;
		_diag_miss_rate_valid = miss_ok && miss_rate_ok;

		// 【V21修改】云台几何也从RX时间轴历史回查到同一个observer_time，确保IMG/GIM/BODY都处于同一物理时刻。
		float gimbal_az_observer = 0.f;
		float gimbal_pitch_observer = 0.f;
		int64_t gimbal_align_error_us = 0;
		bool gimbal_interpolated = false;
		const bool gimbal_ok =
			gimbal_at_time(observer_time, gimbal_az_observer, gimbal_pitch_observer,
				       gimbal_align_error_us, gimbal_interpolated);
		_last_gimbal_align_error_us = gimbal_align_error_us;
		_last_gimbal_interpolated = gimbal_interpolated;

		// 【V21修改】历史不足时只等待历史建立，不回退使用板卡黑盒rate或当前时刻gimbal，避免重新引入时间错位。
		if (!miss_ok || !miss_rate_ok || !gimbal_ok) {
			_los = vehicle_target_los_s{};
			_los.timestamp = observer_time;
			_los.target_valid = false;
			_last_los_rx_time = now;
			reset_inertial_los_reconstruction();
			return false;
		}

		// V21.10: 从这里开始_los全部表示capture_time：miss、PX4 miss-rate、gimbal角与body gyro严格同一物理时刻。
		_los.timestamp = observer_time;
		_los.target_valid = true;
		_los.los_az_rate = miss_az_rate_px4;
		_los.los_el_rate = miss_el_rate_px4;
		_los.gimbal_roll = gimbal_az_observer;
		_los.gimbal_pitch = gimbal_pitch_observer;
		_los.los_azimuth = miss_az_observer;
		_los.los_elevation = miss_el_observer;

		_last_los_rx_time = now;
		update_inertial_los_rate(now);
		return true;
	}

	// V21.12 formal miss-rate selection: poly6 when history is established, backward3 only during bootstrap/reacquire.
	bool miss_rate_at_time_robust(const uint64_t sample_time, float &az_rate_out, float &el_rate_out)
	{
		float back_az = 0.f, back_el = 0.f;
		const bool back_ok = miss_rate_at_time_backward(sample_time, back_az, back_el);
		_diag_miss_az_rate_backward3 = back_ok ? back_az : 0.f;
		_diag_miss_el_rate_backward3 = back_ok ? back_el : 0.f;

		float poly_az = 0.f, poly_el = 0.f;
		if (miss_rate_at_time_causal_polyfit(sample_time, poly_az, poly_el)) {
			az_rate_out = poly_az;
			el_rate_out = poly_el;
			_diag_miss_rate_mode = 2;
			return true;
		}
		if (back_ok) {
			az_rate_out = back_az;
			el_rate_out = back_el;
			_diag_miss_rate_mode = 1;
			return true;
		}
		_diag_miss_rate_mode = 0;
		return false;
	}

	bool calculate_raw_image_los_dot_body(Vector3f &los_dot_image_body) const
	{
		// 【V21修改】真机时_los.los_az_rate/el_rate已改为PX4根据direct miss历史自行重构的相机相对LOS角速度；
		// SITL仍沿用vehicle_target_los上游rate。先对相机LOS单位向量 l_C=[sin(az)cos(el), sin(el), cos(az)cos(el)] 求导，
		// 再经过与LOS方向完全相同的 Camera->Guidance->PX4 Body FRD 旋转。
		const float az = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_azimuth : _los_miss_az_est;
		const float el = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_elevation : _los_miss_el_est;
		const float azd = _los.los_az_rate, eld = _los.los_el_rate;
		if (!PX4_ISFINITE(az) || !PX4_ISFINITE(el) || !PX4_ISFINITE(azd) || !PX4_ISFINITE(eld)
		    || !PX4_ISFINITE(_los.gimbal_roll) || !PX4_ISFINITE(_los.gimbal_pitch)) { return false; }

		const float sa = sinf(az), ca = cosf(az), se = sinf(el), ce = cosf(el);
		const Vector3f los_dot_camera{
			ca * ce * azd - sa * se * eld,
			ce * eld,
			-sa * ce * azd - ca * se * eld
		};
		const Vector3f dot_zero{-los_dot_camera(1), los_dot_camera(0), los_dot_camera(2)};
		const Vector3f dot_guidance = rotate_zero_to_guidance(dot_zero, _los.gimbal_roll, _los.gimbal_pitch);
		los_dot_image_body = guidance_to_px4_body(dot_guidance);
		return PX4_ISFINITE(los_dot_image_body(0)) && PX4_ISFINITE(los_dot_image_body(1))
		       && PX4_ISFINITE(los_dot_image_body(2));
	}


	// 【V20.5修改】解析计算“仅由云台方位/俯仰角速度引起”的LOS方向导数；miss角在该项中保持当前值不变，避免再次对miss做数值微分。
	bool calculate_gimbal_los_dot_body(const float gimbal_az_rate, const float gimbal_pitch_rate,
					  Vector3f &los_dot_gimbal_body) const
	{
		if (!PX4_ISFINITE(gimbal_az_rate) || !PX4_ISFINITE(gimbal_pitch_rate)
		    || !PX4_ISFINITE(_los.gimbal_roll) || !PX4_ISFINITE(_los.gimbal_pitch)) {
			return false;
		}

		const float miss_az = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_azimuth : _los_miss_az_est;
		const float miss_el = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_elevation : _los_miss_el_est;
		if (!PX4_ISFINITE(miss_az) || !PX4_ISFINITE(miss_el)) { return false; }

		const float saz = sinf(miss_az), caz = cosf(miss_az);
		const float sel = sinf(miss_el), cel = cosf(miss_el);
		const Vector3f los_camera{saz * cel, sel, caz * cel};
		const Vector3f v_zero{-los_camera(1), los_camera(0), los_camera(2)};

		const float a = _los.gimbal_roll;
		const float b = _los.gimbal_pitch;
		const float sa = sinf(a), ca = cosf(a);
		const float sb = sinf(b), cb = cosf(b);
		const float xp = cb * v_zero(0) - sb * v_zero(2);
		const float yp = v_zero(1);
		const float zp = sb * v_zero(0) + cb * v_zero(2);
		const Vector3f los_g{xp, ca * yp + sa * zp, -sa * yp + ca * zp};

		// 对rotate_zero_to_guidance()解析求偏导：dL/da=[0,Lz,-Ly]，dL/db=[-zp,sa*xp,ca*xp]。
		const Vector3f d_da{0.f, los_g(2), -los_g(1)};
		const Vector3f d_db{-zp, sa * xp, ca * xp};
		const Vector3f dot_guidance = d_da * gimbal_az_rate + d_db * gimbal_pitch_rate;
		los_dot_gimbal_body = guidance_to_px4_body(dot_guidance);
		return PX4_ISFINITE(los_dot_gimbal_body(0)) && PX4_ISFINITE(los_dot_gimbal_body(1))
		       && PX4_ISFINITE(los_dot_gimbal_body(2));
	}

	void reset_inertial_los_reconstruction()
	{
		// 完整reset：用于真正LOS/数据无效、dt异常等情况。
		// 这类情况不能假设相机残余角仍可信，因此miss积分也清零。
		_los_recon_initialized = false;
		_inertial_los_rate_valid = false;
		_prev_los_recon_time = 0;
		_last_los_recon_dt_s = 0.f;
		_los_miss_az_est = 0.f;
		_los_miss_el_est = 0.f;
		_prev_los_ned.zero();
		_prev_los_body = Vector3f{0.f, 0.f, -1.f};
		_prev_body_rate_at_los.zero();
		_prev_body_rate_at_los_valid = false;
		_last_v19_body_rotation_deg = 0.f;
		// 【V20.9修改】完整reset清空中心差分rate输出和最终融合rate；云台历史本身保留，避免短暂LOS无效后重新等待100ms历史。
		_gimbal_az_rate_filtered = 0.f;
		_gimbal_pitch_rate_filtered = 0.f;
		_gimbal_rate_filter_initialized = false;
		_los_dot_image_body.zero();
		_los_dot_gimbal_body.zero();
		_los_dot_body_motion.zero();
		_los_dot_fused_raw_body.zero();
		_los_dot_inertial_lpf_body.zero();
		_los_dot_inertial_body.zero();
		_los_rate_filter_initialized = false;
		reset_los_rate_notch();
		_los_control_body = Vector3f{0.f, 0.f, -1.f};
		_los_dot_control_body.zero();
		_los_prediction_valid = false;
		_los_prediction_age_us = 0;
		_los_prediction_angle_deg = 0.f;
		_shadow_los_initialized = false;
		_shadow_los_rate_valid = false;
		_shadow_los_rate_filter_initialized = false;
		_shadow_prev_sample_time = 0;
		_shadow_dt_s = 0.f;
		_shadow_residual_angle_deg = 0.f;
		_shadow_filter_alpha = 0.f;
		_shadow_prev_los_body = Vector3f{0.f, 0.f, -1.f};
		_shadow_los_dot_raw_body.zero();
		_shadow_los_dot_filt_body.zero();
		_shadow_prediction_valid = false;
		_shadow_prediction_age_us = 0;
		_shadow_prediction_angle_deg = 0.f;
		_shadow_los_pred_body = Vector3f{0.f, 0.f, -1.f};
		_shadow_los_dot_pred_body.zero();
		_shadow_formal_rate_angle_deg = 0.f;
		_shadow_formal_pred_angle_deg = 0.f;
		_diag_gimbal_delta_az = 0.f;
		_diag_gimbal_delta_pitch = 0.f;
		_diag_gimbal_az_rate_raw = 0.f;
		_diag_gimbal_pitch_rate_raw = 0.f;
		_diag_omega_aligned.zero();
		_diag_los_body = Vector3f{0.f, 0.f, -1.f};
		// 【V21修改】observer reset只清当前PX4 miss-rate有效标志；miss历史是否清空由目标有效性入口单独决定。
		_diag_miss_az_rate_px4 = 0.f;
		_diag_miss_el_rate_px4 = 0.f;
		_diag_miss_az_rate_backward3 = 0.f;
		_diag_miss_el_rate_backward3 = 0.f;
		_diag_miss_rate_mode = 0;
		_diag_miss_rate_valid = false;
		_los_reacquire_active = false;
		_los_reacquire_start_time = 0;
		_los_reacquire_hold_logged = false;
		_last_los_attitude_sync_dt_s = 0.f;
	}

	void reseed_inertial_los_baseline_preserve_miss(const uint64_t now)
	{
		// V19仅用于真正的数据间断/body-rate不可用后的重新播种。
		// EKF绝对姿态reset不会再调用本函数，因为Body-relative observer不依赖绝对NED姿态。
		_los_recon_initialized = false;
		_inertial_los_rate_valid = false;
		_prev_los_recon_time = 0;
		_last_los_recon_dt_s = 0.f;
		_prev_los_body = Vector3f{0.f, 0.f, -1.f};
		_prev_body_rate_at_los.zero();
		_prev_body_rate_at_los_valid = false;
		_last_v19_body_rotation_deg = 0.f;
		// 【V20.9修改】重播种时清空中心差分rate输出，但保留direct miss及云台历史；下一帧直接从历史重新对齐capture时刻。
		_gimbal_az_rate_filtered = 0.f;
		_gimbal_pitch_rate_filtered = 0.f;
		_gimbal_rate_filter_initialized = false;
		_los_dot_image_body.zero();
		_los_dot_gimbal_body.zero();
		_los_dot_body_motion.zero();
		_los_dot_fused_raw_body.zero();
		_los_dot_inertial_lpf_body.zero();
		_los_dot_inertial_body.zero();
		_los_rate_filter_initialized = false;
		reset_los_rate_notch();
		_los_control_body = Vector3f{0.f, 0.f, -1.f};
		_los_dot_control_body.zero();
		_los_prediction_valid = false;
		_los_prediction_age_us = 0;
		_los_prediction_angle_deg = 0.f;
		_shadow_los_initialized = false;
		_shadow_los_rate_valid = false;
		_shadow_los_rate_filter_initialized = false;
		_shadow_prev_sample_time = 0;
		_shadow_dt_s = 0.f;
		_shadow_residual_angle_deg = 0.f;
		_shadow_filter_alpha = 0.f;
		_shadow_prev_los_body = Vector3f{0.f, 0.f, -1.f};
		_shadow_los_dot_raw_body.zero();
		_shadow_los_dot_filt_body.zero();
		_shadow_prediction_valid = false;
		_shadow_prediction_age_us = 0;
		_shadow_prediction_angle_deg = 0.f;
		_shadow_los_pred_body = Vector3f{0.f, 0.f, -1.f};
		_shadow_los_dot_pred_body.zero();
		_shadow_formal_rate_angle_deg = 0.f;
		_shadow_formal_pred_angle_deg = 0.f;
		_diag_gimbal_delta_az = 0.f;
		_diag_gimbal_delta_pitch = 0.f;
		_diag_gimbal_az_rate_raw = 0.f;
		_diag_gimbal_pitch_rate_raw = 0.f;
		_diag_omega_aligned.zero();
		_diag_los_body = Vector3f{0.f, 0.f, -1.f};
		// 【V21修改】重播种时清当前PX4 miss-rate有效标志，但保留历史，下一有效帧可继续按统一时间轴重建。
		_diag_miss_az_rate_px4 = 0.f;
		_diag_miss_el_rate_px4 = 0.f;
		_diag_miss_az_rate_backward3 = 0.f;
		_diag_miss_el_rate_backward3 = 0.f;
		_diag_miss_rate_mode = 0;
		_diag_miss_rate_valid = false;
		_los_reacquire_active = true;
		_los_reacquire_start_time = now;
		_los_reacquire_hold_logged = false;
	}


	bool los_reacquire_grace_active(const uint64_t now) const
	{
		if (!_los_reacquire_active || _los_reacquire_start_time == 0 || now < _los_reacquire_start_time) {
			return false;
		}
		return (now - _los_reacquire_start_time) <= LOS_REACQUIRE_GRACE_US;
	}

	uint64_t attitude_measurement_time(const uint64_t now) const
	{
		if (_attitude.timestamp_sample > 0 && _attitude.timestamp_sample <= now
		    && now - _attitude.timestamp_sample < LOS_TIMEOUT_US) {
			return _attitude.timestamp_sample;
		}
		if (_attitude.timestamp > 0 && _attitude.timestamp <= now
		    && now - _attitude.timestamp < LOS_TIMEOUT_US) {
			return _attitude.timestamp;
		}
		return now;
	}

	Quatf attitude_q_at_los_sample(const uint64_t sample_time, const uint64_t now)
	{
		Quatf q_sample = current_attitude_q();
		_last_los_attitude_sync_dt_s = 0.f;

		if (!body_rate_valid(now)) {
			return q_sample;
		}

		const uint64_t att_time = attitude_measurement_time(now);
		const int64_t dt_us_signed = (int64_t)sample_time - (int64_t)att_time;
		if (dt_us_signed > (int64_t)LOS_ATTITUDE_SYNC_MAX_US
		    || dt_us_signed < -(int64_t)LOS_ATTITUDE_SYNC_MAX_US) {
			return q_sample;
		}

		const float dt = (float)dt_us_signed * 1e-6f;
		_last_los_attitude_sync_dt_s = dt;
		if (fabsf(dt) < 1e-6f) {
			return q_sample;
		}

		Vector3f omega_body{0.f, 0.f, 0.f};
		int64_t align_error_us = 0;
		bool interpolated = false;
		const uint64_t mid_time = (uint64_t)(((int64_t)att_time + (int64_t)sample_time) / 2);
		if (!body_rate_at_time(mid_time, omega_body, align_error_us, interpolated)) {
			omega_body = Vector3f{_angular_velocity.xyz[0], _angular_velocity.xyz[1], _angular_velocity.xyz[2]};
		}
		const float omega_norm = omega_body.norm();
		if (!PX4_ISFINITE(omega_norm) || omega_norm <= FLT_EPSILON) {
			return q_sample;
		}

		// R_nb_dot = R_nb * skew(omega_B)，因此从att_time传播到LOS sample_time时右乘body增量旋转。
		const float angle = omega_norm * dt;
		const float half = 0.5f * angle;
		const float scale = sinf(half) / omega_norm;
		const Quatf dq{cosf(half), omega_body(0) * scale, omega_body(1) * scale, omega_body(2) * scale};
		q_sample = q_sample * dq;
		q_sample.normalize();
		return q_sample;
	}

	uint64_t los_measurement_time(const uint64_t now)
	{
		// 【V21修改】内部_los.timestamp始终位于PX4 HRT时钟域：SITL沿用上游sample timestamp；
		// 真机V21.10为observer_time = capture_time，其中capture_time = CGTD070 RX-HRT - 固定100ms；随后预测到now。
		if (_los.timestamp > 0 && _los.timestamp <= now && now - _los.timestamp <= LOS_TIMEOUT_US) {
			_last_los_sample_time = _los.timestamp;
			_last_los_sample_age_us = now - _los.timestamp;
			_last_los_time_fallback = false;
			return _los.timestamp;
		}
		_last_los_sample_time = now;
		_last_los_sample_age_us = 0;
		_last_los_time_fallback = true;
		return now;
	}

	Vector3f rotate_vector_rodrigues(const Vector3f &v, const Vector3f &rotation_vector) const
	{
		// rotation_vector = axis * angle，右手正方向。
		// V19传播同一惯性向量的Body坐标时使用 -omega_B*dt。
		const float angle = rotation_vector.norm();

		if (!PX4_ISFINITE(angle) || angle <= 1e-7f) {
			// 一阶近似在极小角度下数值更稳定。
			return v + rotation_vector.cross(v);
		}

		const Vector3f axis = rotation_vector / angle;
		const float c = cosf(angle);
		const float s = sinf(angle);
		return v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.f - c));
	}

	void update_inertial_los_rate(const uint64_t now)
	{
		_inertial_los_rate_valid = false;

		if (!_los.target_valid
		    || !PX4_ISFINITE(_los.gimbal_roll) || !PX4_ISFINITE(_los.gimbal_pitch)
		    || !PX4_ISFINITE(_los.los_az_rate) || !PX4_ISFINITE(_los.los_el_rate)
		    || (LOS_INPUT_HAS_MISS_ANGLE
		        && (!PX4_ISFINITE(_los.los_azimuth) || !PX4_ISFINITE(_los.los_elevation)))) {
			reset_inertial_los_reconstruction();
			return;
		}

		uint64_t sample_time = los_measurement_time(now);

		// V20按LOS测量时刻从gyro历史中插值omega_B；不再使用WorkQueue当前时刻的最新gyro。
		Vector3f omega_now{0.f, 0.f, 0.f};
		int64_t omega_align_error_us = 0;
		bool omega_interpolated = false;
		const bool omega_now_valid = body_rate_at_time(sample_time, omega_now,
			omega_align_error_us, omega_interpolated);
		_last_body_rate_align_error_us = omega_align_error_us;
		_last_body_rate_interpolated = omega_interpolated;

		// 当前相机残余角：V20.4起SITL和真机都优先直接使用上游真实miss angle。
		// 保留积分分支仅作为未来无direct-miss输入源的兼容路径。
		if (_los_recon_initialized) {
			// 【V20.9修改】capture时间不允许回退到now；若时间戳重复/倒退则直接重播种，避免把100ms前图像突然与当前gyro/gimbal混用。
			if (sample_time <= _prev_los_recon_time) {
				reseed_inertial_los_baseline_preserve_miss(now);
				return;
			}

			const float dt_pre = sample_time > _prev_los_recon_time
				? (sample_time - _prev_los_recon_time) * 1e-6f
				: 0.f;

			if (!PX4_ISFINITE(dt_pre) || dt_pre < LOS_RECON_DT_MIN || dt_pre > LOS_RECON_DT_MAX) {
				reseed_inertial_los_baseline_preserve_miss(now);
				return;
			}

			if (LOS_INPUT_HAS_MISS_ANGLE
			    && PX4_ISFINITE(_los.los_azimuth)
			    && PX4_ISFINITE(_los.los_elevation)) {
				_los_miss_az_est = _los.los_azimuth;
				_los_miss_el_est = _los.los_elevation;
			} else {
				_los_miss_az_est += _los.los_az_rate * dt_pre;
				_los_miss_el_est += _los.los_el_rate * dt_pre;
				_los_miss_az_est = math::constrain(
					_los_miss_az_est, -LOS_MISS_EST_LIMIT_RAD, LOS_MISS_EST_LIMIT_RAD);
				_los_miss_el_est = math::constrain(
					_los_miss_el_est, -LOS_MISS_EST_LIMIT_RAD, LOS_MISS_EST_LIMIT_RAD);
			}
		} else {
			if (LOS_INPUT_HAS_MISS_ANGLE
			    && PX4_ISFINITE(_los.los_azimuth)
			    && PX4_ISFINITE(_los.los_elevation)) {
				_los_miss_az_est = _los.los_azimuth;
				_los_miss_el_est = _los.los_elevation;
			} else if (!_los_reacquire_active) {
				_los_miss_az_est = 0.f;
				_los_miss_el_est = 0.f;
			}
		}

		const Vector3f los_guidance = calculate_los_body(
			_los.gimbal_roll,
			_los.gimbal_pitch,
			_los_miss_az_est,
			_los_miss_el_est);

		Vector3f los_body = guidance_to_px4_body(los_guidance);
		const float los_norm = los_body.norm();

		if (!PX4_ISFINITE(los_norm) || los_norm <= FLT_EPSILON) {
			reset_inertial_los_reconstruction();
			return;
		}

		los_body /= los_norm;
		_diag_los_body = los_body;

		// V21.12 shadow observer consumes the same capture-time complete LOS and gyro history.
		// It is diagnostic-only and never changes _los_dot_inertial_body or any control output.
		update_full_los_shadow_observer(sample_time, los_body);

		// 【V20.9修改】第一帧建立capture对齐后的gimbal/body-rate基准；云台正式rate后续优先由历史中心差分得到。
		if (!_los_recon_initialized) {
			_prev_los_body = los_body;
			_prev_los_recon_time = sample_time;
			_last_los_recon_dt_s = 0.f;
			_last_v19_body_rotation_deg = 0.f;
			_prev_gimbal_azimuth = _los.gimbal_roll;
			_prev_gimbal_pitch = _los.gimbal_pitch;
			_gimbal_az_rate_filtered = 0.f;
			_gimbal_pitch_rate_filtered = 0.f;
			_gimbal_rate_filter_initialized = true;
			_diag_gimbal_delta_az = 0.f;
			_diag_gimbal_delta_pitch = 0.f;
			_diag_gimbal_az_rate_raw = 0.f;
			_diag_gimbal_pitch_rate_raw = 0.f;
			_diag_omega_aligned = omega_now_valid ? omega_now : Vector3f{0.f, 0.f, 0.f};
			_los_dot_fused_raw_body.zero();
			_los_dot_inertial_lpf_body.zero();
			_los_dot_inertial_body.zero();
			_los_rate_filter_initialized = true;
			reset_los_rate_notch();

			if (omega_now_valid) {
				_prev_body_rate_at_los = omega_now;
				_prev_body_rate_at_los_valid = true;
			} else {
				_prev_body_rate_at_los.zero();
				_prev_body_rate_at_los_valid = false;
			}

			_los_recon_initialized = true;
			return;
		}

		const float dt = sample_time > _prev_los_recon_time
			? (sample_time - _prev_los_recon_time) * 1e-6f
			: 0.f;
		_last_los_recon_dt_s = dt;

		if (!PX4_ISFINITE(dt) || dt < LOS_RECON_DT_MIN || dt > LOS_RECON_DT_MAX || !omega_now_valid) {
			// 【V20.9修改】时间间隔异常时仅更新capture对齐后的几何/gyro基准并清空rate输出，避免跨大间隔计算。
			_prev_los_body = los_body;
			_prev_los_recon_time = sample_time;
			_prev_gimbal_azimuth = _los.gimbal_roll;
			_prev_gimbal_pitch = _los.gimbal_pitch;
			_prev_body_rate_at_los = omega_now;
			_prev_body_rate_at_los_valid = omega_now_valid;
			_gimbal_az_rate_filtered = 0.f;
			_gimbal_pitch_rate_filtered = 0.f;
			_gimbal_rate_filter_initialized = true;
			_diag_gimbal_delta_az = 0.f;
			_diag_gimbal_delta_pitch = 0.f;
			_diag_gimbal_az_rate_raw = 0.f;
			_diag_gimbal_pitch_rate_raw = 0.f;
			_diag_omega_aligned = omega_now_valid ? omega_now : Vector3f{0.f, 0.f, 0.f};
			_los_dot_fused_raw_body.zero();
			_los_dot_inertial_lpf_body.zero();
			_los_dot_inertial_body.zero();
			_los_rate_filter_initialized = true;
			reset_los_rate_notch();
			return;
		}

		Vector3f omega_avg = omega_now;
		if (_prev_body_rate_at_los_valid) {
			omega_avg = 0.5f * (_prev_body_rate_at_los + omega_now);
		}
		_diag_omega_aligned = omega_avg;
		_last_v19_body_rotation_deg = (omega_avg * dt).norm() * 180.f / M_PI_F;

		// 【V21修改】正式IMG角速度使用PX4根据direct miss历史重构的_los.los_az_rate/el_rate；板卡黑盒raw rate只保留ULog诊断。
		if (!calculate_raw_image_los_dot_body(_los_dot_image_body)) {
			_los_dot_image_body.zero();
		}

		// 【V21修改】相邻observer帧云台差分只保留为ULog raw诊断；正式GIM仍使用历史中心差分。
		const float delta_az = atan2f(sinf(_los.gimbal_roll - _prev_gimbal_azimuth),
					      cosf(_los.gimbal_roll - _prev_gimbal_azimuth));
		const float delta_pitch = _los.gimbal_pitch - _prev_gimbal_pitch;
		const float gimbal_az_rate_raw = delta_az / dt;
		const float gimbal_pitch_rate_raw = delta_pitch / dt;

		_diag_gimbal_delta_az = delta_az;
		_diag_gimbal_delta_pitch = delta_pitch;
		_diag_gimbal_az_rate_raw = gimbal_az_rate_raw;
		_diag_gimbal_pitch_rate_raw = gimbal_pitch_rate_raw;

		// 【V21修改】正式GIM优先使用observer时刻±40ms历史中心差分；若极少数帧历史不足，则安全回退到相邻observer帧raw差分而不中断observer。
		float gimbal_az_rate_used = gimbal_az_rate_raw;
		float gimbal_pitch_rate_used = gimbal_pitch_rate_raw;
		const bool centered_rate_valid =
			gimbal_rate_at_time_centered(sample_time, gimbal_az_rate_used, gimbal_pitch_rate_used);

		_gimbal_az_rate_filtered = gimbal_az_rate_used;
		_gimbal_pitch_rate_filtered = gimbal_pitch_rate_used;
		_gimbal_rate_filter_initialized = centered_rate_valid;

		// 【V21修改】用observer时刻的LOS几何和同一时刻中心差分gimbal rate计算GIM，确保IMG/GIM/BODY处于同一时间基准。
		if (!calculate_gimbal_los_dot_body(_gimbal_az_rate_filtered, _gimbal_pitch_rate_filtered,
						 _los_dot_gimbal_body)) {
			_los_dot_gimbal_body.zero();
		}

		// 【V21修改】BODY使用observer_time插值得到的gyro，并与observer LOS在同一Body时刻融合。
		_los_dot_body_motion = omega_avg.cross(los_body);

		// 【V20.5修改】三项相加得到低通前惯性LOS-rate，并严格投影到当前LOS切平面，去掉任何数值径向分量。
		_los_dot_fused_raw_body = _los_dot_image_body + _los_dot_gimbal_body + _los_dot_body_motion;
		_los_dot_fused_raw_body -= los_body * _los_dot_fused_raw_body.dot(los_body);

		// V21.12: final causal LOS-rate LPF. poly6端点拟合负责主要降噪，因此tau保持10ms降低相位。
		const float los_rate_alpha = math::constrain(dt / (INERTIAL_LOS_RATE_LPF_TAU + dt), 0.f, 1.f);
		if (!_los_rate_filter_initialized) {
			_los_dot_inertial_lpf_body.zero();
			_los_dot_inertial_body.zero();
			_los_rate_filter_initialized = true;
			reset_los_rate_notch();
		}
		_los_dot_inertial_lpf_body += los_rate_alpha * (_los_dot_fused_raw_body - _los_dot_inertial_lpf_body);
		_los_dot_inertial_lpf_body -= los_body * _los_dot_inertial_lpf_body.dot(los_body);

		// V22.02: 4Hz narrow-band notch after the low-latency LPF.
		// The filtered vector is projected back to the LOS tangent plane after the biquad.
		_los_dot_inertial_body = apply_los_rate_notch(_los_dot_inertial_lpf_body);
		_los_dot_inertial_body -= los_body * _los_dot_inertial_body.dot(los_body);

		_inertial_los_rate_valid =
			PX4_ISFINITE(_los_dot_inertial_body(0))
			&& PX4_ISFINITE(_los_dot_inertial_body(1))
			&& PX4_ISFINITE(_los_dot_inertial_body(2));

		if (_inertial_los_rate_valid && _los_reacquire_active) {
			PX4_INFO("V20.5 LOS-rate fusion reacquired in %.1f ms",
				 (double)((now - _los_reacquire_start_time) * 1e-3f));
			_los_reacquire_active = false;
			_los_reacquire_start_time = 0;
			_los_reacquire_hold_logged = false;
		}

		_prev_los_body = los_body;
		_prev_los_recon_time = sample_time;
		_prev_body_rate_at_los = omega_now;
		_prev_body_rate_at_los_valid = true;
		_prev_gimbal_azimuth = _los.gimbal_roll;
		_prev_gimbal_pitch = _los.gimbal_pitch;
	}


	// V21.10: 将“某个惯性向量在sample_time时刻的Body坐标”传播到now时刻的Body坐标。
	// 对固定惯性向量v有 dv_B/dt = -omega_B x v_B；使用body-rate历史按<=5ms小步Rodrigues积分。
	bool propagate_inertial_vector_body_to_now(const uint64_t sample_time, const uint64_t now,
						  const Vector3f &v_body_sample, Vector3f &v_body_now) const
	{
		if (sample_time == 0 || now < sample_time || now - sample_time > LOS_PREDICT_MAX_US
		    || !PX4_ISFINITE(v_body_sample(0)) || !PX4_ISFINITE(v_body_sample(1)) || !PX4_ISFINITE(v_body_sample(2))) {
			return false;
		}

		v_body_now = v_body_sample;
		uint64_t t = sample_time;

		while (t < now) {
			const uint64_t remaining = now - t;
			const uint64_t step_us = remaining > LOS_PREDICT_INTEGRATION_STEP_US
				? LOS_PREDICT_INTEGRATION_STEP_US : remaining;
			const uint64_t t_next = t + step_us;
			const uint64_t t_mid = t + step_us / 2ULL;

			Vector3f omega_body{0.f, 0.f, 0.f};
			int64_t align_error_us = 0;
			bool interpolated = false;

			if (!body_rate_at_time(t_mid, omega_body, align_error_us, interpolated)) {
				return false;
			}

			const float dt = (float)step_us * 1e-6f;
			v_body_now = rotate_vector_rodrigues(v_body_now, omega_body * (-dt));

			if (!PX4_ISFINITE(v_body_now(0)) || !PX4_ISFINITE(v_body_now(1)) || !PX4_ISFINITE(v_body_now(2))) {
				return false;
			}

			t = t_next;
		}

		return true;
	}

	// ============================================================
	// V21.12 complete-LOS discrete observer (shadow only)
	// ============================================================
	void update_full_los_shadow_observer(const uint64_t sample_time, const Vector3f &los_body_sample)
	{
		_shadow_los_rate_valid = false;
		_shadow_dt_s = 0.f;
		_shadow_residual_angle_deg = 0.f;
		_shadow_filter_alpha = 0.f;

		const float sample_norm = los_body_sample.norm();
		if (sample_time == 0 || !PX4_ISFINITE(sample_norm) || sample_norm <= FLT_EPSILON) {
			_shadow_los_initialized = false;
			_shadow_los_rate_filter_initialized = false;
			return;
		}

		Vector3f los_now = los_body_sample / sample_norm;

		if (!_shadow_los_initialized || _shadow_prev_sample_time == 0 || sample_time <= _shadow_prev_sample_time) {
			_shadow_prev_sample_time = sample_time;
			_shadow_prev_los_body = los_now;
			_shadow_los_dot_raw_body.zero();
			_shadow_los_dot_filt_body.zero();
			_shadow_los_rate_filter_initialized = false;
			_shadow_los_initialized = true;
			return;
		}

		const float dt = (sample_time - _shadow_prev_sample_time) * 1e-6f;
		_shadow_dt_s = dt;
		if (!PX4_ISFINITE(dt) || dt < LOS_RECON_DT_MIN || dt > LOS_RECON_DT_MAX) {
			_shadow_prev_sample_time = sample_time;
			_shadow_prev_los_body = los_now;
			_shadow_los_rate_filter_initialized = false;
			return;
		}

		Vector3f prev_fixed_at_now{0.f, 0.f, 0.f};
		if (!propagate_inertial_vector_body_to_now(_shadow_prev_sample_time, sample_time,
						       _shadow_prev_los_body, prev_fixed_at_now)) {
			_shadow_prev_sample_time = sample_time;
			_shadow_prev_los_body = los_now;
			_shadow_los_rate_filter_initialized = false;
			return;
		}

		const float prev_norm = prev_fixed_at_now.norm();
		if (!PX4_ISFINITE(prev_norm) || prev_norm <= FLT_EPSILON) {
			_shadow_prev_sample_time = sample_time;
			_shadow_prev_los_body = los_now;
			_shadow_los_rate_filter_initialized = false;
			return;
		}
		prev_fixed_at_now /= prev_norm;

		const Vector3f cross_v = prev_fixed_at_now.cross(los_now);
		const float cross_norm = cross_v.norm();
		const float dot_v = math::constrain(prev_fixed_at_now.dot(los_now), -1.f, 1.f);
		const float angle = atan2f(cross_norm, dot_v);
		_shadow_residual_angle_deg = angle * 180.f / M_PI_F;

		Vector3f rate_raw{0.f, 0.f, 0.f};
		if (PX4_ISFINITE(angle) && angle > SHADOW_LOS_MIN_ANGLE_RAD && cross_norm > FLT_EPSILON) {
			const Vector3f axis = cross_v / cross_norm;
			const Vector3f omega_los = axis * (angle / dt);
			rate_raw = omega_los.cross(los_now);
		} else {
			rate_raw = (los_now - prev_fixed_at_now) / dt;
		}

		rate_raw -= los_now * rate_raw.dot(los_now);
		if (!PX4_ISFINITE(rate_raw(0)) || !PX4_ISFINITE(rate_raw(1)) || !PX4_ISFINITE(rate_raw(2))) {
			_shadow_prev_sample_time = sample_time;
			_shadow_prev_los_body = los_now;
			_shadow_los_rate_filter_initialized = false;
			return;
		}
		_shadow_los_dot_raw_body = rate_raw;

		const float alpha = math::constrain(dt / (SHADOW_LOS_RATE_LPF_TAU + dt), 0.f, 1.f);
		_shadow_filter_alpha = alpha;
		if (!_shadow_los_rate_filter_initialized) {
			_shadow_los_dot_filt_body = rate_raw;
			_shadow_los_rate_filter_initialized = true;
		} else {
			_shadow_los_dot_filt_body += alpha * (rate_raw - _shadow_los_dot_filt_body);
		}
		_shadow_los_dot_filt_body -= los_now * _shadow_los_dot_filt_body.dot(los_now);
		_shadow_los_rate_valid = PX4_ISFINITE(_shadow_los_dot_filt_body(0))
			&& PX4_ISFINITE(_shadow_los_dot_filt_body(1))
			&& PX4_ISFINITE(_shadow_los_dot_filt_body(2));

		_shadow_prev_sample_time = sample_time;
		_shadow_prev_los_body = los_now;
	}

	bool calculate_shadow_los_prediction(const uint64_t now)
	{
		_shadow_prediction_valid = false;
		_shadow_prediction_age_us = 0;
		_shadow_prediction_angle_deg = 0.f;
		_shadow_los_pred_body = Vector3f{0.f, 0.f, -1.f};
		_shadow_los_dot_pred_body.zero();

		if (SIMULATION_MODE || !_shadow_los_rate_valid || !_los.target_valid) {
			return false;
		}

		const uint64_t sample_time = los_measurement_time(now);
		if (sample_time == 0 || now < sample_time || now - sample_time > LOS_PREDICT_MAX_US) {
			return false;
		}

		const float miss_az = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_azimuth : _los_miss_az_est;
		const float miss_el = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_elevation : _los_miss_el_est;
		const Vector3f los_g = calculate_los_body(_los.gimbal_roll, _los.gimbal_pitch, miss_az, miss_el);
		Vector3f los_sample = guidance_to_px4_body(los_g);
		const float n = los_sample.norm();
		if (!PX4_ISFINITE(n) || n <= FLT_EPSILON) { return false; }
		los_sample /= n;

		Vector3f rate_sample = _shadow_los_dot_filt_body;
		rate_sample -= los_sample * rate_sample.dot(los_sample);
		const Vector3f omega_los_sample = los_sample.cross(rate_sample);

		Vector3f los_fixed_now{0.f, 0.f, 0.f};
		Vector3f omega_los_now{0.f, 0.f, 0.f};
		if (!propagate_inertial_vector_body_to_now(sample_time, now, los_sample, los_fixed_now)
		    || !propagate_inertial_vector_body_to_now(sample_time, now, omega_los_sample, omega_los_now)) {
			return false;
		}

		const float fixed_norm = los_fixed_now.norm();
		if (!PX4_ISFINITE(fixed_norm) || fixed_norm <= FLT_EPSILON) { return false; }
		los_fixed_now /= fixed_norm;

		const uint64_t age_us = now - sample_time;
		const uint64_t extra_lead_us = calculate_control_extra_lead_us(_shadow_los_dot_filt_body.norm());
		const float dt_predict = (float)(age_us + extra_lead_us) * 1e-6f;
		Vector3f pred = rotate_vector_rodrigues(los_fixed_now, omega_los_now * dt_predict);
		const float pred_norm = pred.norm();
		if (!PX4_ISFINITE(pred_norm) || pred_norm <= FLT_EPSILON) { return false; }
		pred /= pred_norm;

		Vector3f pred_rate = omega_los_now.cross(pred);
		pred_rate -= pred * pred_rate.dot(pred);
		if (!PX4_ISFINITE(pred_rate(0)) || !PX4_ISFINITE(pred_rate(1)) || !PX4_ISFINITE(pred_rate(2))) {
			return false;
		}

		_shadow_los_pred_body = pred;
		_shadow_los_dot_pred_body = pred_rate;
		_shadow_prediction_age_us = age_us;
		const float c = math::constrain(los_fixed_now.dot(pred), -1.f, 1.f);
		_shadow_prediction_angle_deg = acosf(c) * 180.f / M_PI_F;
		_shadow_prediction_valid = true;
		return true;
	}

	float vector_angle_deg(const Vector3f &a, const Vector3f &b) const
	{
		const float na = a.norm();
		const float nb = b.norm();
		if (!PX4_ISFINITE(na) || !PX4_ISFINITE(nb) || na <= FLT_EPSILON || nb <= FLT_EPSILON) {
			return 0.f;
		}
		return acosf(math::constrain(a.dot(b) / (na * nb), -1.f, 1.f)) * 180.f / M_PI_F;
	}

	uint64_t calculate_control_extra_lead_us(const float los_rate_mag) const
	{
		const float r = PX4_ISFINITE(los_rate_mag) ? fmaxf(los_rate_mag, 0.f) : 0.f;
		float lead_us = (float)LOS_CONTROL_EXTRA_LEAD_MIN_US;

		if (r <= LOS_CONTROL_LEAD_RATE_P1) {
			lead_us = (float)LOS_CONTROL_EXTRA_LEAD_MIN_US;
		} else if (r <= LOS_CONTROL_LEAD_RATE_P2) {
			const float u = (r - LOS_CONTROL_LEAD_RATE_P1)
				/ fmaxf(LOS_CONTROL_LEAD_RATE_P2 - LOS_CONTROL_LEAD_RATE_P1, 0.001f);
			lead_us = lerp((float)LOS_CONTROL_EXTRA_LEAD_MIN_US, (float)LOS_CONTROL_EXTRA_LEAD_MID_US, smoothstep01(u));
		} else if (r <= LOS_CONTROL_LEAD_RATE_P3) {
			const float u = (r - LOS_CONTROL_LEAD_RATE_P2)
				/ fmaxf(LOS_CONTROL_LEAD_RATE_P3 - LOS_CONTROL_LEAD_RATE_P2, 0.001f);
			lead_us = lerp((float)LOS_CONTROL_EXTRA_LEAD_MID_US, (float)LOS_CONTROL_EXTRA_LEAD_MAX_US, smoothstep01(u));
		} else {
			lead_us = (float)LOS_CONTROL_EXTRA_LEAD_MAX_US;
		}

		const float att_err = attitude_error_angle_deg();
		if (PX4_ISFINITE(att_err) && att_err > LOS_CONTROL_LEAD_ATT_ERR_START_DEG) {
			const float u = math::constrain(
				(att_err - LOS_CONTROL_LEAD_ATT_ERR_START_DEG)
				/ fmaxf(LOS_CONTROL_LEAD_ATT_ERR_FULL_DEG - LOS_CONTROL_LEAD_ATT_ERR_START_DEG, 1.f),
				0.f, 1.f);
			lead_us += (float)LOS_CONTROL_LEAD_ATT_BOOST_MAX_US * smoothstep01(u);
		}

		return (uint64_t)math::constrain(lead_us,
			(float)LOS_CONTROL_EXTRA_LEAD_MIN_US,
			(float)LOS_CONTROL_EXTRA_LEAD_MAX_US);
	}

	float calculate_adaptive_attitude_slew_deg_s(const float los_rate_mag) const
	{
		const float r = PX4_ISFINITE(los_rate_mag) ? fmaxf(los_rate_mag, 0.f) : 0.f;
		if (r <= GUIDANCE_SLEW_LOS_RATE_P1) {
			return GUIDANCE_ATTITUDE_SLEW_DEG_S;
		}
		if (r <= GUIDANCE_SLEW_LOS_RATE_P2) {
			const float u = (r - GUIDANCE_SLEW_LOS_RATE_P1)
				/ fmaxf(GUIDANCE_SLEW_LOS_RATE_P2 - GUIDANCE_SLEW_LOS_RATE_P1, 0.001f);
			return lerp(GUIDANCE_ATTITUDE_SLEW_DEG_S, GUIDANCE_ATTITUDE_SLEW_MID_DEG_S, smoothstep01(u));
		}
		if (r <= GUIDANCE_SLEW_LOS_RATE_P3) {
			const float u = (r - GUIDANCE_SLEW_LOS_RATE_P2)
				/ fmaxf(GUIDANCE_SLEW_LOS_RATE_P3 - GUIDANCE_SLEW_LOS_RATE_P2, 0.001f);
			return lerp(GUIDANCE_ATTITUDE_SLEW_MID_DEG_S, GUIDANCE_ATTITUDE_SLEW_HIGH_DEG_S, smoothstep01(u));
		}
		return GUIDANCE_ATTITUDE_SLEW_HIGH_DEG_S;
	}

	// V21.10: 构造“当前时刻”的LOS与惯性LOS方向导数。
	// 真机：capture时刻LOS + capture时刻惯性LOS rate -> 先补偿capture->now期间Body自身旋转，
	// 再用常惯性LOS角速度模型在球面上外推；SITL保持原当前时刻语义，不额外预测。
	bool calculate_control_los_body(const uint64_t now, Vector3f &los_body_control,
					Vector3f &los_dot_control_body, bool &prediction_used)
	{
		prediction_used = false;
		_los_prediction_valid = false;
		_los_prediction_age_us = 0;
		_los_prediction_angle_deg = 0.f;

		if (!_los.target_valid || !PX4_ISFINITE(_los.gimbal_roll) || !PX4_ISFINITE(_los.gimbal_pitch)
		    || !PX4_ISFINITE(_los.los_azimuth) || !PX4_ISFINITE(_los.los_elevation)) {
			return false;
		}

		const float miss_az = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_azimuth : _los_miss_az_est;
		const float miss_el = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_elevation : _los_miss_el_est;
		const Vector3f los_guidance_sample = calculate_los_body(_los.gimbal_roll, _los.gimbal_pitch, miss_az, miss_el);
		Vector3f los_body_sample = guidance_to_px4_body(los_guidance_sample);
		const float los_sample_norm = los_body_sample.norm();

		if (!PX4_ISFINITE(los_sample_norm) || los_sample_norm <= FLT_EPSILON) {
			return false;
		}
		los_body_sample /= los_sample_norm;

		// SITL保持原来的时间语义；truth控制时truth rate本身就是控制输入。
		if (SIMULATION_MODE) {
			los_body_control = los_body_sample;
			if (SIM_USE_TRUTH_LOS_CONTROL && truth_los_fresh(now)) {
				los_dot_control_body = _truth_los_dot_body;
			} else if (_inertial_los_rate_valid) {
				los_dot_control_body = _los_dot_inertial_body;
			} else {
				los_dot_control_body.zero();
			}
			_los_control_body = los_body_control;
			_los_dot_control_body = los_dot_control_body;
			_los_prediction_valid = true;
			return true;
		}

		// 真机正式控制必须建立capture时刻惯性LOS rate；预测失败时不混用100ms历史Body LOS作为“当前LOS”。
		if (!_inertial_los_rate_valid) {
			return false;
		}

		const uint64_t sample_time = los_measurement_time(now);
		if (sample_time == 0 || now < sample_time) {
			return false;
		}

		const uint64_t age_us = now - sample_time;
		if (age_us > LOS_PREDICT_MAX_US) {
			return false;
		}

		Vector3f los_dot_sample = _los_dot_inertial_body;
		los_dot_sample -= los_body_sample * los_dot_sample.dot(los_body_sample);
		if (!PX4_ISFINITE(los_dot_sample(0)) || !PX4_ISFINITE(los_dot_sample(1)) || !PX4_ISFINITE(los_dot_sample(2))) {
			return false;
		}

		// 最小LOS惯性角速度：omega_LOS = L x Ldot，使 omega_LOS x L = Ldot。
		const Vector3f omega_los_sample = los_body_sample.cross(los_dot_sample);

		Vector3f los_fixed_now{0.f, 0.f, 0.f};
		Vector3f omega_los_now{0.f, 0.f, 0.f};
		if (!propagate_inertial_vector_body_to_now(sample_time, now, los_body_sample, los_fixed_now)
		    || !propagate_inertial_vector_body_to_now(sample_time, now, omega_los_sample, omega_los_now)) {
			return false;
		}

		const float fixed_norm = los_fixed_now.norm();
		if (!PX4_ISFINITE(fixed_norm) || fixed_norm <= FLT_EPSILON) {
			return false;
		}
		los_fixed_now /= fixed_norm;

		const uint64_t extra_lead_us = calculate_control_extra_lead_us(los_dot_sample.norm());
		const uint64_t total_predict_us = age_us + extra_lead_us;
		if (total_predict_us > LOS_PREDICT_TOTAL_MAX_US) {
			return false;
		}
		_last_control_extra_lead_us = extra_lead_us;
		const float dt_predict = (float)total_predict_us * 1e-6f;
		los_body_control = rotate_vector_rodrigues(los_fixed_now, omega_los_now * dt_predict);
		const float pred_norm = los_body_control.norm();
		if (!PX4_ISFINITE(pred_norm) || pred_norm <= FLT_EPSILON) {
			return false;
		}
		los_body_control /= pred_norm;

		los_dot_control_body = omega_los_now.cross(los_body_control);
		los_dot_control_body -= los_body_control * los_dot_control_body.dot(los_body_control);
		if (!PX4_ISFINITE(los_dot_control_body(0)) || !PX4_ISFINITE(los_dot_control_body(1))
		    || !PX4_ISFINITE(los_dot_control_body(2))) {
			return false;
		}

		const float c = math::constrain(los_fixed_now.dot(los_body_control), -1.f, 1.f);
		_los_prediction_angle_deg = acosf(c) * 180.f / M_PI_F;
		_los_prediction_age_us = age_us;
		_los_prediction_valid = true;
		prediction_used = age_us > 0;
		_los_control_body = los_body_control;
		_los_dot_control_body = los_dot_control_body;
		_diag_los_body = los_body_control; // 正式控制诊断显示current-time LOS，而不是100ms历史LOS。

		// V21.12 diagnostic-only shadow prediction and A/B disagreement metrics.
		calculate_shadow_los_prediction(now);
		_shadow_formal_rate_angle_deg = (_shadow_los_rate_valid && _inertial_los_rate_valid)
			? vector_angle_deg(_los_dot_inertial_body, _shadow_los_dot_filt_body) : 0.f;
		_shadow_formal_pred_angle_deg = (_shadow_prediction_valid && _los_prediction_valid)
			? vector_angle_deg(_los_control_body, _shadow_los_pred_body) : 0.f;
		return true;
	}


	// ============================================================
	// V22.02 LOS-rate 4Hz notch
	// RBJ biquad notch, component-wise on the formal LOS-rate vector.
	// Low-frequency/DC gain is unity; initialization uses steady-state values to avoid startup impulse.
	// ============================================================
	void reset_los_rate_notch()
	{
		_los_rate_notch_initialized = false;
		_los_rate_notch_x1.zero();
		_los_rate_notch_x2.zero();
		_los_rate_notch_y1.zero();
		_los_rate_notch_y2.zero();
	}

	Vector3f apply_los_rate_notch(const Vector3f &input)
	{
		if (!LOS_RATE_NOTCH_ENABLED) { return input; }

		const float w0 = 2.f * M_PI_F * LOS_RATE_NOTCH_FREQ_HZ / LOS_RATE_NOTCH_FS_HZ;
		const float c = cosf(w0);
		const float alpha = sinf(w0) / (2.f * LOS_RATE_NOTCH_Q);
		const float a0 = 1.f + alpha;

		const float b0 = 1.f / a0;
		const float b1 = -2.f * c / a0;
		const float b2 = 1.f / a0;
		const float a1 = -2.f * c / a0;
		const float a2 = (1.f - alpha) / a0;

		if (!_los_rate_notch_initialized) {
			_los_rate_notch_x1 = input;
			_los_rate_notch_x2 = input;
			_los_rate_notch_y1 = input;
			_los_rate_notch_y2 = input;
			_los_rate_notch_initialized = true;
			return input;
		}

		const Vector3f output = input * b0
			+ _los_rate_notch_x1 * b1
			+ _los_rate_notch_x2 * b2
			- _los_rate_notch_y1 * a1
			- _los_rate_notch_y2 * a2;

		_los_rate_notch_x2 = _los_rate_notch_x1;
		_los_rate_notch_x1 = input;
		_los_rate_notch_y2 = _los_rate_notch_y1;
		_los_rate_notch_y1 = output;

		return output;
	}

	// ============================================================
	// V22.04 adaptive BPN acceleration jerk limiter in NED
	//
	// 1) LOS-rate越大，说明终端几何变化越快，允许更高jerk；
	// 2) raw与当前shaped BPN方向差越大，额外提高jerk以快速消除相位滞后；
	// 3) 仍限制3-D acceleration delta的模，因此不会单轴突然无界跳变。
	// ============================================================
	void reset_bpn_jerk_limiter()
	{
		_bpn_accel_ned_jerk_limited.zero();
		_bpn_jerk_initialized = false;
		_last_bpn_jerk_update_time = 0;
		_last_bpn_jerk_limit = BPN_JERK_LOW;
		_last_bpn_jerk_dir_error_deg = 0.f;
	}

	float calculate_adaptive_bpn_jerk_limit(const Vector3f &raw_bpn_ned) const
	{
		const float los_rate = PX4_ISFINITE(_last_los_rate_mag) ? fmaxf(_last_los_rate_mag, 0.f) : 0.f;

		float jerk = BPN_JERK_LOW;

		if (los_rate <= BPN_JERK_LOS_RATE_P1) {
			jerk = BPN_JERK_LOW;

		} else if (los_rate <= BPN_JERK_LOS_RATE_P2) {
			const float t = (los_rate - BPN_JERK_LOS_RATE_P1)
				/ fmaxf(BPN_JERK_LOS_RATE_P2 - BPN_JERK_LOS_RATE_P1, 0.001f);
			jerk = lerp(BPN_JERK_LOW, BPN_JERK_MID, math::constrain(t, 0.f, 1.f));

		} else if (los_rate <= BPN_JERK_LOS_RATE_P3) {
			const float t = (los_rate - BPN_JERK_LOS_RATE_P2)
				/ fmaxf(BPN_JERK_LOS_RATE_P3 - BPN_JERK_LOS_RATE_P2, 0.001f);
			jerk = lerp(BPN_JERK_MID, BPN_JERK_HIGH, math::constrain(t, 0.f, 1.f));

		} else {
			jerk = BPN_JERK_HIGH;
		}

		// Severe raw-vs-shaped direction mismatch means the limiter itself is becoming the dominant phase lag.
		// Boost only when both vectors have meaningful magnitude; otherwise the direction angle is numerically meaningless.
		const float raw_norm = raw_bpn_ned.norm();
		const float shaped_norm = _bpn_accel_ned_jerk_limited.norm();

		if (PX4_ISFINITE(raw_norm) && PX4_ISFINITE(shaped_norm)
		    && raw_norm > 0.25f && shaped_norm > 0.25f) {
			const float c = math::constrain(
					raw_bpn_ned.dot(_bpn_accel_ned_jerk_limited) / (raw_norm * shaped_norm),
					-1.f, 1.f);
			const float dir_err_deg = acosf(c) * 180.f / M_PI_F;

			if (PX4_ISFINITE(dir_err_deg) && dir_err_deg > BPN_JERK_DIR_BOOST_START_DEG) {
				const float u = math::constrain(
						(dir_err_deg - BPN_JERK_DIR_BOOST_START_DEG)
						/ fmaxf(BPN_JERK_DIR_BOOST_FULL_DEG - BPN_JERK_DIR_BOOST_START_DEG, 1.f),
						0.f, 1.f);
				const float boost = lerp(1.f, BPN_JERK_DIR_BOOST_MAX, u);
				jerk *= boost;
			}
		}

		return math::constrain(jerk, BPN_JERK_LOW, BPN_JERK_ABS_MAX);
	}

	Vector3f limit_bpn_accel_jerk_ned(const Vector3f &raw_bpn_ned, const uint64_t now)
	{
		if (!_bpn_jerk_initialized) {
			_bpn_accel_ned_jerk_limited = raw_bpn_ned;
			_bpn_jerk_initialized = true;
			_last_bpn_jerk_update_time = now;
			_last_bpn_jerk_limit = BPN_JERK_LOW;
			_last_bpn_jerk_dir_error_deg = 0.f;
			return raw_bpn_ned;
		}

		float dt = LOOP_INTERVAL_US * 1e-6f;
		if (_last_bpn_jerk_update_time > 0 && now > _last_bpn_jerk_update_time) {
			dt = (float)(now - _last_bpn_jerk_update_time) * 1e-6f;
			dt = math::constrain(dt, 0.001f, 0.10f);
		}
		_last_bpn_jerk_update_time = now;

		const float raw_norm = raw_bpn_ned.norm();
		const float shaped_norm = _bpn_accel_ned_jerk_limited.norm();
		_last_bpn_jerk_dir_error_deg = 0.f;

		if (PX4_ISFINITE(raw_norm) && PX4_ISFINITE(shaped_norm)
		    && raw_norm > 0.25f && shaped_norm > 0.25f) {
			const float c = math::constrain(
					raw_bpn_ned.dot(_bpn_accel_ned_jerk_limited) / (raw_norm * shaped_norm),
					-1.f, 1.f);
			_last_bpn_jerk_dir_error_deg = acosf(c) * 180.f / M_PI_F;
		}

		_last_bpn_jerk_limit = calculate_adaptive_bpn_jerk_limit(raw_bpn_ned);

		Vector3f delta = raw_bpn_ned - _bpn_accel_ned_jerk_limited;
		const float delta_norm = delta.norm();
		const float max_delta = _last_bpn_jerk_limit * dt;

		if (PX4_ISFINITE(delta_norm) && delta_norm > max_delta && delta_norm > FLT_EPSILON) {
			delta *= max_delta / delta_norm;
		}

		_bpn_accel_ned_jerk_limited += delta;
		return _bpn_accel_ned_jerk_limited;
	}


	Vector3f calculate_bpn_body(const Vector3f &los_body, const Vector3f &los_dot_body,
				    const float vc_used) const
	{
		Vector3f accel_body = los_dot_body * (PN_N * vc_used);

		// 在Guidance坐标内去掉LOS轴向分量，仅保留法向加速度。
		accel_body -= los_body * accel_body.dot(los_body);

		const float accel_norm = accel_body.norm();

		if (PX4_ISFINITE(accel_norm) && accel_norm > BPN_ACCEL_MAX) {
			accel_body *= BPN_ACCEL_MAX / accel_norm;
		}

		return accel_body;
	}

	bool calculate_los_ned(Vector3f &los_ned)
	{
		// V21.10: 正式真机使用预测到当前时刻的PX4 Body LOS，再用当前vehicle_attitude旋到NED。
		// 这样Vc投影、CLEARANCE和BPN共享同一current-time LOS时间基准。
		if (!attitude_valid()) { return false; }

		Vector3f los_body_control{0.f, 0.f, 0.f};
		Vector3f los_dot_control{0.f, 0.f, 0.f};
		bool prediction_used = false;
		if (!calculate_control_los_body(hrt_absolute_time(), los_body_control, los_dot_control, prediction_used)) {
			return false;
		}

		los_ned = Dcmf{current_attitude_q()} * los_body_control;
		const float norm = los_ned.norm();
		if (!PX4_ISFINITE(norm) || norm <= FLT_EPSILON) { return false; }
		los_ned /= norm;
		return true;
	}


	// ============================================================
	// 动态制导目标速度
	// ============================================================

	float calculate_guidance_speed_target(const float los_rate_mag) const
	{
		if (!PX4_ISFINITE(los_rate_mag)) { return GUIDANCE_SPEED_CRUISE; }

		if (los_rate_mag <= LOS_RATE_SPEED_POINT_1) {
			return GUIDANCE_SPEED_CRUISE;
		}

		if (los_rate_mag <= LOS_RATE_SPEED_POINT_2) {
			const float t = (los_rate_mag - LOS_RATE_SPEED_POINT_1)
					/ (LOS_RATE_SPEED_POINT_2 - LOS_RATE_SPEED_POINT_1);

			return lerp(GUIDANCE_SPEED_CRUISE, GUIDANCE_SPEED_MID_1, t);
		}

		if (los_rate_mag <= LOS_RATE_SPEED_POINT_3) {
			const float t = (los_rate_mag - LOS_RATE_SPEED_POINT_2)
					/ (LOS_RATE_SPEED_POINT_3 - LOS_RATE_SPEED_POINT_2);

			return lerp(GUIDANCE_SPEED_MID_1, GUIDANCE_SPEED_MID_2, t);
		}

		// V21.3：原逻辑在0.35rad/s上方直接13->6m/s，导致目标速度阶跃。
		// 现在保留原端点数值，只增加0.35~0.60rad/s连续过渡。
		if (los_rate_mag <= LOS_RATE_SPEED_POINT_4) {
			const float t = (los_rate_mag - LOS_RATE_SPEED_POINT_3)
					/ (LOS_RATE_SPEED_POINT_4 - LOS_RATE_SPEED_POINT_3);

			return lerp(GUIDANCE_SPEED_MID_2, GUIDANCE_SPEED_MIN, t);
		}

		return GUIDANCE_SPEED_MIN;
	}

	float calculate_axial_speed_scale(const float speed, const float target_speed) const
	{
		if (!PX4_ISFINITE(speed)) { return 1.f; }

		const float taper_start = fmaxf(target_speed - SPEED_ACCEL_TAPER_BAND, 0.f);

		if (speed <= taper_start) { return 1.f; }
		if (speed >= target_speed) { return 0.f; }

		return math::constrain(
			       (target_speed - speed) / fmaxf(target_speed - taper_start, 0.1f),
			       0.f,
			       1.f
		       );
	}

	Vector3f calculate_speed_brake_accel_ned(const float target_speed, float &brake_mag) const
	{
		Vector3f brake{0.f, 0.f, 0.f};
		brake_mag = 0.f;

		Vector3f velocity_ned;
		if (!get_guidance_velocity_ned(velocity_ned)) { return brake; }

		const float speed = velocity_ned.norm();
		const float speed_xy = sqrtf(
				       velocity_ned(0) * velocity_ned(0)
				       + velocity_ned(1) * velocity_ned(1)
			       );

		if (!PX4_ISFINITE(speed) || speed < 0.1f) { return brake; }

		float normal_brake = 0.f;
		const float normal_brake_start = target_speed + SPEED_BRAKE_DEADBAND;

		if (speed > normal_brake_start) {
			normal_brake = math::constrain(
					       SPEED_BRAKE_KP * (speed - normal_brake_start),
					       0.f,
					       SPEED_BRAKE_ACCEL_MAX
				       );
		}

		float hard_brake = 0.f;

		// V21.8：26m/s开始渐进硬制动；达到/超过28m/s名义硬上限时直接打满硬制动。
		if (speed >= GUIDANCE_SPEED_HARD_LIMIT) {
			hard_brake = HARD_SPEED_BRAKE_ACCEL_MAX;

		} else if (speed > HARD_SPEED_BRAKE_START) {
			hard_brake = math::constrain(
					     HARD_SPEED_BRAKE_KP * (speed - HARD_SPEED_BRAKE_START),
					     0.f,
					     HARD_SPEED_BRAKE_ACCEL_MAX
				     );
		}

		// 硬速度保护优先：直接沿三维速度反方向减速。
		if (hard_brake > 0.f && hard_brake >= normal_brake) {
			brake_mag = hard_brake;
			brake = -velocity_ned * (hard_brake / speed);
			return brake;
		}

		// 正常动态目标速度制动仍尽量只使用XY，
		// 把垂向自由度留给BPN和高度保护。
		if (normal_brake > 0.f && PX4_ISFINITE(speed_xy) && speed_xy > 0.1f) {
			brake_mag = normal_brake;
			// V21.14: brake方向必须与当前guidance velocity源一致；
			// EKF reset保护期间不能一边用IMU速度算speed，一边又用跳变后的EKF vx/vy决定制动方向。
			brake(0) = -normal_brake * velocity_ned(0) / speed_xy;
			brake(1) = -normal_brake * velocity_ned(1) / speed_xy;
			brake(2) = 0.f;
		}

		return brake;
	}

	// ============================================================
	// 相对闭合速度 Vc
	//
	// V22.01 fixed-wing tracking: unknown target velocity is not guessed.
	// TARGET_VELOCITY_COMPENSATION_ENABLED=false, therefore Vt=[0,0,0] and Vc_est=Va·LOS.
	// 正：接近目标；负：目标沿LOS方向远离得更快。
	// ============================================================

	void update_closing_speed(const uint64_t now)
	{
		Vector3f los_ned;
		Vector3f velocity_ned;

		float dt = LOOP_INTERVAL_US * 1e-6f;
		if (_last_vc_update_time > 0 && now > _last_vc_update_time) {
			dt = (float)(now - _last_vc_update_time) * 1e-6f;
			dt = math::constrain(dt, 0.001f, 0.10f);
		}
		_last_vc_update_time = now;

		const bool valid = get_guidance_velocity_ned(velocity_ned)
				   && calculate_los_ned(los_ned);

		if (!valid) {
			// V22.01: do not jump vc_used directly to VC_FALLBACK when EKF/IMU source expires.
			// Preserve the previous filtered value and exponentially blend toward fallback.
			_vc_valid = false;
			_vc_aircraft_only = 0.f;
			_vc_target_projection = 0.f;
			_vc_relative_raw = VC_FALLBACK;
			_vc_raw = VC_FALLBACK;

			if (!_vc_filter_initialized || !PX4_ISFINITE(_vc_filtered)) {
				_vc_filtered = VC_FALLBACK;
				_vc_filter_initialized = true;
			} else {
				const float alpha = dt / (VC_FALLBACK_BLEND_TAU + dt);
				_vc_filtered += alpha * (VC_FALLBACK - _vc_filtered);
			}

			_vc_used = math::constrain(fmaxf(_vc_filtered, 0.f), 0.f, VC_MAX);
			return;
		}

		// V22.01 fixed-wing UNKNOWN target velocity mode:
		// L points aircraft -> target; Vt is intentionally not guessed, therefore Vc_est = Va dot L.
		_vc_aircraft_only = velocity_ned.dot(los_ned);

		Vector3f target_velocity_ned{0.f, 0.f, 0.f};
		if (!SIMULATION_MODE && TARGET_VELOCITY_COMPENSATION_ENABLED) {
			target_velocity_ned(2) = -TARGET_CLIMB_SPEED_MPS;
		}

		_vc_target_projection = target_velocity_ned.dot(los_ned);
		_vc_relative_raw = _vc_aircraft_only - _vc_target_projection;
		_vc_raw = _vc_relative_raw;

		if (!PX4_ISFINITE(_vc_raw) || !PX4_ISFINITE(_vc_aircraft_only) || !PX4_ISFINITE(_vc_target_projection)) {
			_vc_valid = false;
			const float alpha = dt / (VC_FALLBACK_BLEND_TAU + dt);
			if (!_vc_filter_initialized || !PX4_ISFINITE(_vc_filtered)) {
				_vc_filtered = VC_FALLBACK;
				_vc_filter_initialized = true;
			} else {
				_vc_filtered += alpha * (VC_FALLBACK - _vc_filtered);
			}
			_vc_used = math::constrain(fmaxf(_vc_filtered, 0.f), 0.f, VC_MAX);
			return;
		}

		if (!_vc_filter_initialized) {
			_vc_filtered = _vc_raw;
			_vc_filter_initialized = true;

		} else {
			const float tau = (_vc_raw < _vc_filtered) ? VC_LPF_TAU_FALL : VC_LPF_TAU_RISE;
			const float alpha = dt / (tau + dt);
			_vc_filtered += alpha * (_vc_raw - _vc_filtered);
		}

		_vc_valid = true;
		_vc_used = math::constrain(fmaxf(_vc_filtered, 0.f), 0.f, VC_MAX);

		if (_vc_raw > MISS_ARM_VC) {
			_had_positive_closing = true;
		}
	}

	// ============================================================
	// attitude + thrust -> 净惯性加速度
	// ============================================================

	Vector3f attitude_thrust_to_net_accel(const Quatf &q_nb, const float thrust_norm) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};
		const Vector3f thrust_axis_body{0.f, 0.f, -1.f};

		const Dcmf R_nb{q_nb};
		const Vector3f thrust_axis_ned = R_nb * thrust_axis_body;

		const float thrust_accel = GRAVITY * thrust_norm / HOVER_THRUST;

		return gravity_ned + thrust_axis_ned * thrust_accel;
	}

	// ============================================================
	// 推力预算
	// ============================================================

	float thrust_accel_budget(const float thrust_limit) const
	{
		return GRAVITY * thrust_limit / HOVER_THRUST;
	}

	float predicted_thrust_norm_from_accel(const Vector3f &accel_sp_ned) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};
		const Vector3f thrust_accel = accel_sp_ned - gravity_ned;

		return HOVER_THRUST * thrust_accel.norm() / GRAVITY;
	}

	// ============================================================
	// V22.02 final-q thrust synchronization with vertical support priority
	//
	// 1) projection term: least-squares realization of desired thrust vector along q_cmd thrust axis;
	// 2) vertical term: minimum collective required to realize requested NED vertical acceleration with q_cmd;
	// 3) final collective = max(projection, vertical_required), capped by the active flight-state thrust budget.
	// This avoids both V21.14's "slewed direction + raw magnitude" excess and V22.01's collapse to MIN_THRUST
	// when q_raw and q_cmd temporarily differ by a large angle.
	// ============================================================
	float active_final_thrust_limit() const
	{
		if (_state == GuidanceState::LAUNCH_CLEARANCE) {
			return CLEARANCE_THRUST_LIMIT;
		}

		if (_state == GuidanceState::TRANSITION) {
			return _alt_floor_active ? CLEARANCE_THRUST_LIMIT : GUIDANCE_THRUST_LIMIT;
		}

		if (_state == GuidanceState::BPN) {
			return _alt_floor_active ? ALT_FLOOR_THRUST_LIMIT : GUIDANCE_THRUST_LIMIT;
		}

		return MAX_THRUST;
	}

	float commanded_thrust_norm_from_accel_and_attitude(const Vector3f &accel_sp_ned, const Quatf &q_cmd) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};
		const Vector3f desired_thrust_accel = accel_sp_ned - gravity_ned;

		const Dcmf R_nb{q_cmd};
		Vector3f thrust_axis_ned = R_nb * Vector3f{0.f, 0.f, -1.f};
		const float axis_norm = thrust_axis_ned.norm();
		const float thrust_limit = math::constrain(active_final_thrust_limit(), MIN_THRUST, MAX_THRUST);

		if (!PX4_ISFINITE(axis_norm) || axis_norm < FLT_EPSILON
		    || !PX4_ISFINITE(desired_thrust_accel(0)) || !PX4_ISFINITE(desired_thrust_accel(1))
		    || !PX4_ISFINITE(desired_thrust_accel(2))) {
			return math::constrain(predicted_thrust_norm_from_accel(accel_sp_ned), MIN_THRUST, thrust_limit);
		}

		thrust_axis_ned /= axis_norm;

		// Least-squares projection along the physically achievable q_cmd thrust axis.
		const float projection_accel = fmaxf(desired_thrust_accel.dot(thrust_axis_ned), 0.f);

		// Vertical-support requirement. In NED, thrust_axis_ned(2)<0 for upright flight and
		// a_z = g + thrust_accel * axis_z. Solve for thrust_accel that realizes accel_sp_ned.z.
		float vertical_required_accel = 0.f;
		if (thrust_axis_ned(2) < -0.05f) {
			vertical_required_accel =
				fmaxf((accel_sp_ned(2) - GRAVITY) / thrust_axis_ned(2), 0.f);
		}

		const float commanded_accel = fmaxf(projection_accel, vertical_required_accel);
		const float thrust_norm = HOVER_THRUST * commanded_accel / GRAVITY;
		return math::constrain(thrust_norm, MIN_THRUST, thrust_limit);
	}

	// ============================================================
	// 垂向优先推力预算
	//
	// thrust_limit由调用者决定：
	// CLEARANCE = CLEARANCE_THRUST_LIMIT（legacy路径）
	// BPN/TRANSITION = GUIDANCE_THRUST_LIMIT
	// ============================================================

	void constrain_net_accel_vertical_priority(Vector3f &accel_sp_ned, const float thrust_limit) const
	{
		const float budget = thrust_accel_budget(thrust_limit);

		float thrust_z = accel_sp_ned(2) - GRAVITY;
		float thrust_xy = sqrtf(
				  accel_sp_ned(0) * accel_sp_ned(0)
				  + accel_sp_ned(1) * accel_sp_ned(1)
			  );

		if (!PX4_ISFINITE(thrust_z) || !PX4_ISFINITE(thrust_xy)) { return; }

		if (fabsf(thrust_z) >= budget) {
			thrust_z = math::constrain(thrust_z, -budget, budget);

			accel_sp_ned(0) = 0.f;
			accel_sp_ned(1) = 0.f;
			accel_sp_ned(2) = thrust_z + GRAVITY;
			return;
		}

		const float xy_available = sqrtf(fmaxf(budget * budget - thrust_z * thrust_z, 0.f));

		if (thrust_xy > xy_available && thrust_xy > FLT_EPSILON) {
			const float scale = xy_available / thrust_xy;
			accel_sp_ned(0) *= scale;
			accel_sp_ned(1) *= scale;
		}
	}

	float scale_bpn_to_budget(const Vector3f &bpn_accel_ned, const float budget_accel) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};

		if ((bpn_accel_ned - gravity_ned).norm() <= budget_accel) {
			return 1.f;
		}

		float low = 0.f;
		float high = 1.f;

		for (int i = 0; i < 12; ++i) {
			const float mid = 0.5f * (low + high);
			const float required = (bpn_accel_ned * mid - gravity_ned).norm();

			if (required <= budget_accel) {
				low = mid;

			} else {
				high = mid;
			}
		}

		return math::constrain(low, 0.f, 1.f);
	}

	void allocate_guidance_acceleration(const Vector3f &los_ned, const Vector3f &bpn_accel_ned_raw, const float axial_accel_limit,
					    const float thrust_limit, Vector3f &accel_sp_ned, float &axial_used, float &bpn_scale, float &predicted_thrust) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};
		// 推力预算由调用路径明确传入：正式BPN与零推力TEST候选当前都使用GUIDANCE_THRUST_LIMIT；接口保留参数化，便于后续独立诊断。
		const float budget = thrust_accel_budget(thrust_limit);

		// 第一级：BPN优先。
		bpn_scale = scale_bpn_to_budget(bpn_accel_ned_raw, budget);

		const Vector3f bpn_accel_ned = bpn_accel_ned_raw * bpn_scale;

		// 第二级：剩余推力分配给LOS轴向加速度。
		const Vector3f c = bpn_accel_ned - gravity_ned;

		const float d = c.dot(los_ned);
		const float c2 = c.norm_squared();

		const float discriminant = d * d - (c2 - budget * budget);

		axial_used = 0.f;

		if (discriminant >= 0.f && PX4_ISFINITE(discriminant)) {
			const float root = -d + sqrtf(discriminant);

			if (PX4_ISFINITE(root) && root > 0.f) {
				axial_used = math::constrain(root, 0.f, axial_accel_limit);
			}
		}

		accel_sp_ned = bpn_accel_ned + los_ned * axial_used;
		predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);
	}

	// ============================================================
	// RELEASE高度底线
	// ============================================================

	void apply_release_altitude_floor_guard(Vector3f &accel_sp_ned)
	{
	    _alt_floor_active = false;
	    _last_alt_floor_up_accel = 0.f;

	    if (!_release_alt_ref_valid
		|| !_local_position.z_valid
		|| !_local_position.v_z_valid
		|| !PX4_ISFINITE(_local_position.z)
		|| !PX4_ISFINITE(_local_position.vz)) {
		return;
	    }

	    // PX4 NED：
	    // z增大 = 向下
	    // vz > 0 = 正在下降
	    const float drop =
		_local_position.z - _release_alt_ref_z_ned;

	    const float vz_down =
		fmaxf(_local_position.vz, 0.f);

	    // 已经停止下降或开始上升：
	    // ALT_FLOOR立即退出，不负责把飞机拉回release高度。
	    if (_local_position.vz <= ALT_FLOOR_RELEASE_VZ) {
		_last_alt_floor_drop = drop;
		_last_alt_floor_predicted_drop = drop;
		return;
	    }

	    // 按最大允许向上加速度估算：
	    // 以当前下降速度刹到0，还需要多少距离。
	    const float stop_distance =
		(vz_down * vz_down)
		/ fmaxf(
		    2.f * ALT_FLOOR_UP_ACCEL_MAX,
		    0.1f
		);

	    const float predicted_drop =
		drop + stop_distance;

	    _last_alt_floor_drop = drop;
	    _last_alt_floor_predicted_drop = predicted_drop;

	    // 当前掉高不大，同时预测也不会越过警戒线：
	    // 不需要高度保护。
	    if (drop <= ALT_FLOOR_SOFT_DROP
		&& predicted_drop <= ALT_FLOOR_PREDICT_TRIGGER) {
		return;
	    }

	    _alt_floor_active = true;

	    // 距离HARD_DROP还剩多少制动空间。
	    const float remaining =
		fmaxf(
		    ALT_FLOOR_HARD_DROP - drop,
		    0.05f
		);

	    // 基于物理停车距离计算最小制动加速度。
	    const float accel_stop =
		(vz_down * vz_down)
		/ (2.f * remaining)
		+ ALT_FLOOR_ACCEL_MARGIN;

	    // 预测越过SOFT线多少。
	    const float predicted_violation =
		fmaxf(
		    predicted_drop - ALT_FLOOR_SOFT_DROP,
		    0.f
		);

	    // 单边barrier控制：
	    // P项看“预测越界量”
	    // D项看“当前下降速度”
	    const float accel_barrier =
		ALT_FLOOR_PRED_KP * predicted_violation
		+ ALT_FLOOR_VZ_KD * vz_down;

	    // ★关键就在这里： accel_barrier必须实际参与最终计算。
	    const float accel_up =
	    math::constrain(
		fmaxf(accel_stop, accel_barrier),
		0.f,
		ALT_FLOOR_UP_ACCEL_MAX
	    );

	    _last_alt_floor_up_accel = accel_up;

	    // NED中负Z方向是向上。
	    // 只允许ALT_FLOOR增加向上控制，
	    // 不削弱原本已经更强的向上控制。
	    accel_sp_ned(2) =
		fminf(
		    accel_sp_ned(2),
		    -accel_up
		);
	}
	// ============================================================
	// acceleration -> attitude
	// ============================================================

	void acceleration_to_attitude(const Vector3f &accel_sp_ned, const float yaw_sp,
				      Quatf &q_sp, float &thrust_norm,
				      bool *feasibility_limited = nullptr,
				      float *raw_tilt_deg = nullptr,
				      float *cmd_tilt_deg = nullptr) const
	{
		const Vector3f gravity_ned{0.f, 0.f, GRAVITY};

		const Vector3f thrust_accel_ned = accel_sp_ned - gravity_ned;
		const float thrust_accel_mag = thrust_accel_ned.norm();

		Vector3f body_z = -thrust_accel_ned;

		if (body_z.norm_squared() < FLT_EPSILON) {
			body_z = Vector3f{0.f, 0.f, 1.f};
		}

		body_z.normalize();

		const float raw_tilt = acosf(math::constrain(body_z(2), -1.f, 1.f));
		bool limited = false;

		// V21.14: positive-thrust multirotor feasibility cone.
		// body_z is Body +Z expressed in NED. Upright hover is [0,0,+1].
		// Any requested body_z with tilt > MAX_TILT (including body_z.z<0 inverted solutions)
		// is projected to the nearest direction on the upright cone while preserving horizontal azimuth.
		const float min_body_z_z = cosf(GUIDANCE_THRUST_VECTOR_MAX_TILT_RAD);
		if (body_z(2) < min_body_z_z) {
			const float xy_norm = sqrtf(body_z(0) * body_z(0) + body_z(1) * body_z(1));

			if (PX4_ISFINITE(xy_norm) && xy_norm > FLT_EPSILON) {
				const float sin_tilt = sinf(GUIDANCE_THRUST_VECTOR_MAX_TILT_RAD);
				body_z(0) = body_z(0) * (sin_tilt / xy_norm);
				body_z(1) = body_z(1) * (sin_tilt / xy_norm);
				body_z(2) = min_body_z_z;
			} else {
				// Pure inverted/ill-conditioned vertical request has no meaningful horizontal azimuth.
				// Safest feasible projection is upright rather than choosing an arbitrary 75deg direction.
				body_z = Vector3f{0.f, 0.f, 1.f};
			}
			body_z.normalize();
			limited = true;
		}

		const float cmd_tilt = acosf(math::constrain(body_z(2), -1.f, 1.f));

		const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

		Vector3f body_x = y_C % body_z;

		// body_z经过可行域约束后始终保持upright hemisphere；保留数值防护。
		if (body_z(2) < 0.f) {
			body_x = -body_x;
		}

		if (fabsf(body_z(2)) < 0.000001f) {
			body_x.zero();
			body_x(2) = 1.f;
		}

		if (body_x.norm_squared() < FLT_EPSILON) {
			body_x = Vector3f{cosf(yaw_sp), sinf(yaw_sp), 0.f};
		}

		body_x.normalize();

		const Vector3f body_y = body_z % body_x;

		Dcmf R_sp;

		for (int i = 0; i < 3; ++i) {
			R_sp(i, 0) = body_x(i);
			R_sp(i, 1) = body_y(i);
			R_sp(i, 2) = body_z(i);
		}

		q_sp = Quatf{R_sp};
		q_sp.normalize();

		// Magnitude follows the original requested net thrust acceleration; only direction is made feasible.
		// This prevents q_raw 180deg flips without silently increasing demanded thrust magnitude.
		thrust_norm = HOVER_THRUST * thrust_accel_mag / GRAVITY;
		thrust_norm = math::constrain(thrust_norm, MIN_THRUST, MAX_THRUST);

		if (feasibility_limited) { *feasibility_limited = limited; }
		if (raw_tilt_deg) { *raw_tilt_deg = raw_tilt * 180.f / M_PI_F; }
		if (cmd_tilt_deg) { *cmd_tilt_deg = cmd_tilt * 180.f / M_PI_F; }
	}


	// ============================================================
	// 发布
	// ============================================================

	// 【V20.8新增】ULog诊断消息中的缓存年龄统一以本Run周期now为基准。
	float diag_age_ms(const uint64_t now, const uint64_t timestamp) const
	{
		if (timestamp == 0 || now < timestamp) {
			return NAN;
		}

		return (float)(now - timestamp) * 1e-3f;
	}

	// 【V20.8新增】只清除“本50Hz周期发生了什么”的瞬态flag。
	void begin_diag_cycle()
	{
		_diag_guidance_valid = false;
		_diag_control_published = false;
		_diag_slew_limited = false;
		_diag_lead_limited = false;
		_diag_cage_limited = false;
	}

	// 【V20.8修订】原单一PnGuidanceDiag消息字段过多，会超过PX4 Logger的
	// uORB untokenized format buffer上限。因此拆成3个50Hz topic，并使用完全相同的
	// timestamp/seq进行离线对齐：
	//   pn_guidance_los_diag      : 时间链 + IMG/GIM/BODY/FRAW/IN observer
	//   pn_guidance_guidance_diag : Vc/BPN/加速度/q_raw/q_slew/q_cmd
	//   pn_guidance_control_diag  : PX4内环/电机/台架状态
	// 三个topic都只读取现有缓存，不参与任何控制计算。
	void publish_pn_guidance_diag(const uint64_t now)
	{
		const uint32_t seq = ++_diag_seq;

		// ========================================================
		// 1) LOS / Observer
		// ========================================================
		pn_guidance_los_diag_s los_msg{};
		los_msg.timestamp = now;
		los_msg.seq = seq;
		los_msg.version = 3;

		los_msg.los_sample_time = _last_los_sample_time;
		los_msg.los_rx_time = _last_los_rx_time;
		los_msg.source_timestamp = SIMULATION_MODE ? _los.timestamp : _cgtd070_status.timestamp;

		los_msg.capture_time = _diag_capture_time;
		los_msg.capture_age_us = _diag_capture_age_us;
		los_msg.vision_frame_seq = _diag_vision_frame_seq;

		if (_last_body_rate_align_error_us > INT32_MAX) {
			los_msg.body_rate_align_error_us = INT32_MAX;
		} else if (_last_body_rate_align_error_us < INT32_MIN) {
			los_msg.body_rate_align_error_us = INT32_MIN;
		} else {
			los_msg.body_rate_align_error_us = (int32_t)_last_body_rate_align_error_us;
		}

		los_msg.los_sample_age_ms = (float)_last_los_sample_age_us * 1e-3f;
		los_msg.recon_dt_ms = _last_los_recon_dt_s * 1000.f;
		los_msg.att_sync_dt_ms = _last_los_attitude_sync_dt_s * 1000.f;
		los_msg.body_rotation_deg = _last_v19_body_rotation_deg;

		los_msg.gimbal_angle[0] = _los.gimbal_roll;
		los_msg.gimbal_angle[1] = _los.gimbal_pitch;
		los_msg.gimbal_delta[0] = _diag_gimbal_delta_az;
		los_msg.gimbal_delta[1] = _diag_gimbal_delta_pitch;
		los_msg.gimbal_rate_raw[0] = _diag_gimbal_az_rate_raw;
		los_msg.gimbal_rate_raw[1] = _diag_gimbal_pitch_rate_raw;
		los_msg.gimbal_rate_filt[0] = _gimbal_az_rate_filtered;
		los_msg.gimbal_rate_filt[1] = _gimbal_pitch_rate_filtered;

		los_msg.miss_angle[0] = _los.los_azimuth;
		los_msg.miss_angle[1] = _los.los_elevation;
		// 【V21修改】真机raw_los_rate固定记录RK3576黑盒输出用于A/B；SITL继续记录vehicle_target_los上游rate，保持原诊断语义。
		los_msg.raw_los_rate[0] = SIMULATION_MODE ? _los.los_az_rate : _diag_board_los_az_rate;
		los_msg.raw_los_rate[1] = SIMULATION_MODE ? _los.los_el_rate : _diag_board_los_el_rate;
		// 【V21修改】新增miss_rate_px4记录正式参与IMG计算的PX4中心差分rate。
		los_msg.miss_rate_px4[0] = _diag_miss_az_rate_px4;
		los_msg.miss_rate_px4[1] = _diag_miss_el_rate_px4;
		los_msg.miss_rate_window_ms = (_diag_miss_rate_mode == 2)
			? (float)((MISS_RATE_POLYFIT_POINTS - 1U) * MISS_RATE_BACKWARD_STEP_US) * 1e-3f
			: (float)(2ULL * MISS_RATE_CENTER_HALF_WINDOW_US) * 1e-3f;
		los_msg.miss_rate_valid = _diag_miss_rate_valid ? 1 : 0;

		for (int i = 0; i < 3; ++i) {
			los_msg.los_body[i] = _diag_los_body(i);
			los_msg.omega_aligned[i] = _diag_omega_aligned(i);
			los_msg.los_dot_img[i] = _los_dot_image_body(i);
			los_msg.los_dot_gim[i] = _los_dot_gimbal_body(i);
			los_msg.los_dot_body[i] = _los_dot_body_motion(i);
			los_msg.los_dot_fraw[i] = _los_dot_fused_raw_body(i);
			los_msg.los_dot_in[i] = _los_dot_inertial_body(i);
		}

		los_msg.target_valid = _los.target_valid ? 1 : 0;
		los_msg.los_fresh = los_fresh(now) ? 1 : 0;
		los_msg.attitude_valid = attitude_valid() ? 1 : 0;
		los_msg.inertial_rate_valid = _inertial_los_rate_valid ? 1 : 0;
		los_msg.body_rate_valid = body_rate_valid(now) ? 1 : 0;
		los_msg.body_rate_interpolated = _last_body_rate_interpolated ? 1 : 0;
		los_msg.los_time_fallback = _last_los_time_fallback ? 1 : 0;
		los_msg.capture_time_valid = _diag_capture_time_valid ? 1 : 0;
		los_msg.observer_initialized = _los_recon_initialized ? 1 : 0;
		los_msg.reacquire_active = _los_reacquire_active ? 1 : 0;

		_pn_guidance_los_diag_pub.publish(los_msg);

		// ========================================================
		// V21.12 Observer A/B shadow diagnostics (50 Hz, logging only).
		// ========================================================
		pn_guidance_observer_diag_s observer_msg{};
		observer_msg.timestamp = now;
		observer_msg.sample_time = _last_los_sample_time;
		observer_msg.seq = seq;
		observer_msg.version = 1;
		observer_msg.sample_age_ms = (float)_last_los_sample_age_us * 1e-3f;
		observer_msg.shadow_dt_ms = _shadow_dt_s * 1000.f;
		observer_msg.shadow_residual_angle_deg = _shadow_residual_angle_deg;
		observer_msg.shadow_filter_alpha = _shadow_filter_alpha;

		const float formal_rate_mag = _los_dot_inertial_body.norm();
		const float shadow_raw_mag = _shadow_los_dot_raw_body.norm();
		const float shadow_filt_mag = _shadow_los_dot_filt_body.norm();
		observer_msg.formal_rate_mag = PX4_ISFINITE(formal_rate_mag) ? formal_rate_mag : 0.f;
		observer_msg.shadow_rate_raw_mag = PX4_ISFINITE(shadow_raw_mag) ? shadow_raw_mag : 0.f;
		observer_msg.shadow_rate_filt_mag = PX4_ISFINITE(shadow_filt_mag) ? shadow_filt_mag : 0.f;
		observer_msg.formal_shadow_rate_angle_deg = _shadow_formal_rate_angle_deg;
		observer_msg.formal_prediction_age_ms = (float)_los_prediction_age_us * 1e-3f;
		observer_msg.shadow_prediction_age_ms = (float)_shadow_prediction_age_us * 1e-3f;
		observer_msg.formal_shadow_pred_angle_deg = _shadow_formal_pred_angle_deg;
		observer_msg.miss_rate_formal[0] = _diag_miss_az_rate_px4;
		observer_msg.miss_rate_formal[1] = _diag_miss_el_rate_px4;
		observer_msg.miss_rate_backward3[0] = _diag_miss_az_rate_backward3;
		observer_msg.miss_rate_backward3[1] = _diag_miss_el_rate_backward3;
		observer_msg.miss_rate_mode = _diag_miss_rate_mode;

		for (int i = 0; i < 3; ++i) {
			observer_msg.formal_rate[i] = _los_dot_inertial_body(i);
			observer_msg.shadow_rate_raw[i] = _shadow_los_dot_raw_body(i);
			observer_msg.shadow_rate_filt[i] = _shadow_los_dot_filt_body(i);
			observer_msg.formal_pred_los[i] = _los_control_body(i);
			observer_msg.shadow_pred_los[i] = _shadow_los_pred_body(i);
		}
		observer_msg.formal_rate_valid = _inertial_los_rate_valid ? 1 : 0;
		observer_msg.shadow_rate_valid = _shadow_los_rate_valid ? 1 : 0;
		observer_msg.formal_prediction_valid = _los_prediction_valid ? 1 : 0;
		observer_msg.shadow_prediction_valid = _shadow_prediction_valid ? 1 : 0;
		observer_msg.target_valid = _los.target_valid ? 1 : 0;
		_pn_guidance_observer_diag_pub.publish(observer_msg);

		// ========================================================
		// 2) Guidance
		// ========================================================
		pn_guidance_guidance_diag_s guidance_msg{};
		guidance_msg.timestamp = now;
		guidance_msg.seq = seq;
		guidance_msg.version = 4;

		guidance_msg.vc_raw = _vc_raw;
		guidance_msg.vc_filtered = _vc_filtered;
		guidance_msg.vc_used = _vc_used;

		guidance_msg.ekf_velocity[0] = _local_position.vx;
		guidance_msg.ekf_velocity[1] = _local_position.vy;
		guidance_msg.ekf_velocity[2] = _local_position.vz;
		guidance_msg.ekf_speed = get_ekf_speed();

		guidance_msg.speed_target = _last_speed_target;
		guidance_msg.axial_used = _last_axial_used;
		guidance_msg.axial_limit = _last_axial_limit_cmd;
		guidance_msg.speed_brake = _last_speed_brake;
		guidance_msg.bpn_scale = _last_bpn_scale;

		for (int i = 0; i < 3; ++i) {
			guidance_msg.bpn_raw_body[i] = _diag_bpn_raw_body(i);
			guidance_msg.bpn_used_body[i] = _diag_bpn_used_body(i);
			guidance_msg.accel_cmd_ned[i] = _diag_accel_cmd_ned(i);
		}

		_diag_q_raw.copyTo(guidance_msg.q_raw);
		_diag_q_slew.copyTo(guidance_msg.q_slew);
		_diag_q_cmd.copyTo(guidance_msg.q_cmd);

		const Quatf q_actual = current_attitude_q();
		q_actual.copyTo(guidance_msg.q_actual);

		guidance_msg.thrust_preview = _diag_thrust_preview;
		guidance_msg.thrust_out = _diag_thrust_out;

		guidance_msg.state = (uint8_t)_state;
		guidance_msg.rig_stage = (uint8_t)_rig_stage.load();
		guidance_msg.vc_valid = _vc_valid ? 1 : 0;
		guidance_msg.velocity_valid = velocity_valid() ? 1 : 0;
		guidance_msg.guidance_valid = _diag_guidance_valid ? 1 : 0;
		guidance_msg.slew_limited = _diag_slew_limited ? 1 : 0;
		guidance_msg.lead_limited = _diag_lead_limited ? 1 : 0;
		guidance_msg.cage_limited = _diag_cage_limited ? 1 : 0;
		guidance_msg.alt_floor_active = _alt_floor_active ? 1 : 0;

		_pn_guidance_guidance_diag_pub.publish(guidance_msg);

		// ========================================================
		// 3) PX4 inner-loop / rig state
		// ========================================================
		pn_guidance_control_diag_s control_msg{};
		control_msg.timestamp = now;
		control_msg.seq = seq;
		control_msg.version = 3;

		control_msg.rate_sp[0] = _vehicle_rates_setpoint.roll;
		control_msg.rate_sp[1] = _vehicle_rates_setpoint.pitch;
		control_msg.rate_sp[2] = _vehicle_rates_setpoint.yaw;

		control_msg.rate_actual[0] = _angular_velocity.xyz[0];
		control_msg.rate_actual[1] = _angular_velocity.xyz[1];
		control_msg.rate_actual[2] = _angular_velocity.xyz[2];

		for (int i = 0; i < 3; ++i) {
			control_msg.torque_sp[i] = _vehicle_torque_setpoint.xyz[i];
			control_msg.thrust_sp[i] = _vehicle_thrust_setpoint.xyz[i];
		}

		for (int i = 0; i < 4; ++i) {
			control_msg.motor[i] = _actuator_motors.control[i];
		}

		control_msg.rates_sp_age_ms = diag_age_ms(now, _last_rates_setpoint_rx_time);
		control_msg.torque_sp_age_ms = diag_age_ms(now, _last_torque_setpoint_rx_time);
		control_msg.thrust_sp_age_ms = diag_age_ms(now, _last_thrust_setpoint_rx_time);
		control_msg.motor_age_ms = diag_age_ms(now, _last_actuator_motors_rx_time);

		control_msg.aux1 = _manual_control.aux1;
		control_msg.attitude_error_deg = attitude_valid() ? attitude_error_angle_deg() : NAN;
		control_msg.rig_att_from_anchor_deg =
			(_rig_anchor_valid && attitude_valid())
			? quat_angle_rad(_rig_anchor_q, current_attitude_q()) * 180.f / M_PI_F
			: NAN;

		float max_motor_abs = 0.f;
		for (int i = 0; i < 4; ++i) {
			if (PX4_ISFINITE(_actuator_motors.control[i])) {
				max_motor_abs = fmaxf(max_motor_abs, fabsf(_actuator_motors.control[i]));
			}
		}
		control_msg.max_motor_abs = max_motor_abs;

		const uint32_t stage = _rig_stage.load();
		control_msg.state = (uint8_t)_state;
		control_msg.rig_stage = (uint8_t)stage;
		control_msg.armed = armed() ? 1 : 0;
		control_msg.rig_run = (stage == 1u || _rig_release_latched) ? 1 : 0;
		control_msg.rig_fault = _rig_fault_latched ? 1 : 0;
		control_msg.control_published = _diag_control_published ? 1 : 0;
		control_msg.rc_valid = rc_release_input_valid(now) ? 1 : 0;
		control_msg.control_attitude_enabled =
			_vehicle_control_mode.flag_control_attitude_enabled ? 1 : 0;
		control_msg.control_rates_enabled =
			_vehicle_control_mode.flag_control_rates_enabled ? 1 : 0;

		_pn_guidance_control_diag_pub.publish(control_msg);
	}

	void publish_attitude_setpoint_internal(const Quatf &q_d, const float thrust_norm, const uint64_t now)
	{
		// V21.4最终防线：RC拥有姿态控制权或RC_OVERRIDE已锁存时，pn_guidance绝不再写vehicle_attitude_setpoint。
		// RIG固定台架与DISARMED接口测试保留原行为。
		if (!SIMULATION_MODE && armed() && _rig_stage.load() == 0u
		    && (_rc_override_latched || rc_manual_attitude_owner_active(now))) {
			_diag_control_published = false;
			_diag_thrust_out = 0.f;
			_last_thrust_sp = 0.f;
			return;
		}

		vehicle_attitude_setpoint_s sp{};

		sp.timestamp = now;
		q_d.copyTo(sp.q_d);
		sp.yaw_sp_move_rate = 0.f;

		sp.thrust_body[0] = 0.f;
		sp.thrust_body[1] = 0.f;
		sp.thrust_body[2] = -math::constrain(thrust_norm, 0.f, MAX_THRUST);

		// 显式检查uORB发布结果。内部_last_q_sp更新并不等价于topic真的被advertise，
		// 因此把publish返回值和orb_exists分开记录，专门诊断“listener never published”。
		_att_sp_pub_calls.fetch_add(1);
		const bool pub_ok = _attitude_setpoint_pub.publish(sp);
		_diag_q_cmd = q_d;
		_diag_thrust_out = math::constrain(thrust_norm, 0.f, MAX_THRUST);
		_diag_control_published = pub_ok;
		_att_sp_last_ok.store(pub_ok);
		if (pub_ok) {
			_att_sp_pub_ok.fetch_add(1);
		} else {
			_att_sp_pub_fail.fetch_add(1);
			if (now - _last_att_sp_pub_warn_time > 1000000ULL) {
				_last_att_sp_pub_warn_time = now;
				PX4_ERR("vehicle_attitude_setpoint publish() failed");
			}
		}

		_last_q_sp = q_d;
		_last_thrust_sp = math::constrain(thrust_norm, 0.f, MAX_THRUST);
		_last_setpoint_pub_time = now;
	}

	void publish_direct_attitude(const Quatf &q_d, const float thrust_norm, const uint64_t now)
	{
		_last_raw_q_sp = q_d;
		_last_thrust_vector_feasibility_limited = false;
		_last_thrust_vector_raw_tilt_deg = 0.f;
		_last_thrust_vector_cmd_tilt_deg = 0.f;
		_slew_active = false;
		_last_raw_q_step_deg = 0.f;
		_lead_active = false;
		_last_attitude_lead_deg = 0.f;

		_diag_q_raw = q_d;
		_diag_q_slew = q_d;
		_diag_q_cmd = q_d;
		_diag_thrust_preview = thrust_norm;
		_diag_slew_limited = false;
		_diag_lead_limited = false;
		_diag_cage_limited = false;

		publish_attitude_setpoint_internal(q_d, thrust_norm, now);
	}

	void publish_accel_attitude_setpoint(const Vector3f &accel_sp_ned, const uint64_t now)
	{
		Quatf q_raw;
		float thrust_norm = 0.f;
		bool feasibility_limited = false;
		float raw_tilt_deg = 0.f;
		float cmd_tilt_deg = 0.f;

		acceleration_to_attitude(accel_sp_ned, _yaw_lock, q_raw, thrust_norm,
				       &feasibility_limited, &raw_tilt_deg, &cmd_tilt_deg);

		_last_thrust_vector_feasibility_limited = feasibility_limited;
		_last_thrust_vector_raw_tilt_deg = raw_tilt_deg;
		_last_thrust_vector_cmd_tilt_deg = cmd_tilt_deg;
		_last_raw_q_sp = q_raw;

		float dt = LOOP_INTERVAL_US * 1e-6f;

		if (_last_setpoint_pub_time > 0 && now > _last_setpoint_pub_time) {
			dt = (float)(now - _last_setpoint_pub_time) * 1e-6f;
			dt = math::constrain(dt, 0.001f, 0.10f);
		}

		// const float max_step_rad = GUIDANCE_ATTITUDE_SLEW_RAD_S * dt;

		// ============================================================
		// 根据当前制导状态选择姿态setpoint最大变化速度
		// LAUNCH_CLEARANCE：100 deg/s
		// TRANSITION：100 -> 60 deg/s
		// BPN/其他状态：60 deg/s
		// ============================================================

		float attitude_slew_deg_s = GUIDANCE_ATTITUDE_SLEW_DEG_S;
		if (_state == GuidanceState::LAUNCH_CLEARANCE) {
		    // 发射初段允许姿态目标更快变化，尽快把推力方向从大俯仰姿态转向有效方向。
		    attitude_slew_deg_s =
			CLEARANCE_ATTITUDE_SLEW_DEG_S;
		} else if (_state == GuidanceState::TRANSITION) {
		    // _transition_phase:
		    // 0 -> 刚进入TRANSITION
		    // 1 -> TRANSITION结束
		    // 因此：
		    // phase=0 -> 100 deg/s
		    // phase=0.5 -> 80 deg/s
		    // phase=1 -> 60 deg/s
		    attitude_slew_deg_s =
			CLEARANCE_ATTITUDE_SLEW_DEG_S
			+ _transition_phase
			* (GUIDANCE_ATTITUDE_SLEW_DEG_S
			   - CLEARANCE_ATTITUDE_SLEW_DEG_S);
		}

		// V22.05: after the launch-protection state logic, allow higher slew only when the current LOS-rate justifies it.
		// The V22.03 gimbal-protection cap below still wins during the first 1.5s.
		if (_state == GuidanceState::BPN) {
			_last_adaptive_slew_deg_s = calculate_adaptive_attitude_slew_deg_s(_last_los_rate_mag);
			attitude_slew_deg_s = fmaxf(attitude_slew_deg_s, _last_adaptive_slew_deg_s);
		} else {
			_last_adaptive_slew_deg_s = attitude_slew_deg_s;
		}

		// V22.03: early post-RELEASE attitude slew cap (80 -> 110deg/s over 1.5s).
		// This only reduces the normal state-specific slew; it never increases a tighter state limit.
		if (_guidance_release_time > 0
		    && (_state == GuidanceState::TRANSITION || _state == GuidanceState::BPN)) {
			update_gimbal_protect_schedule(now);
			attitude_slew_deg_s = fminf(attitude_slew_deg_s, _last_gimbal_protect_slew_deg_s);
		}

		// V22.06: when gimbal load is high, do not let terminal adaptive-slew outrun the two-axis stabilizer.
		// Safe load leaves the V22.05 110->135deg/s adaptive behaviour unchanged.
		if (_state == GuidanceState::TRANSITION || _state == GuidanceState::BPN) {
			attitude_slew_deg_s = fminf(attitude_slew_deg_s, _gimbal_supervisor_slew_cap_deg_s);
		}
		if (attitude_reset_guard_active(now)) {
			attitude_slew_deg_s = fminf(attitude_slew_deg_s, ATT_RESET_GUARD_SLEW_CAP_DEG_S);
		}

		const float attitude_slew_rad_s =
		    attitude_slew_deg_s * M_PI_F / 180.f;

		const float max_step_rad =
		    attitude_slew_rad_s * dt;

		_last_attitude_slew_deg_s = attitude_slew_deg_s;

		float raw_step_rad = 0.f;

		// 第一级：限制setpoint本身角速度。
		const Quatf q_slew = slerp_limited(
					     _last_q_sp,
					     q_raw,
					     max_step_rad,
					     _slew_active,
					     raw_step_rad
				     );

		_last_raw_q_step_deg = raw_step_rad * 180.f / M_PI_F;

		// 第二级：限制setpoint相对于飞机真实姿态的领先量。
		// V21.9.2：CLEARANCE/TRANSITION单独收紧，正式BPN使用GUIDANCE_ATTITUDE_LEAD_MAX_DEG（当前65deg）。
		float attitude_lead_max_deg = GUIDANCE_ATTITUDE_LEAD_MAX_DEG;

		if (_state == GuidanceState::LAUNCH_CLEARANCE) {
			attitude_lead_max_deg = CLEARANCE_ATTITUDE_LEAD_MAX_DEG;

		} else if (_state == GuidanceState::TRANSITION) {
			attitude_lead_max_deg = TRANSITION_ATTITUDE_LEAD_START_DEG
				+ _transition_phase * (TRANSITION_ATTITUDE_LEAD_END_DEG - TRANSITION_ATTITUDE_LEAD_START_DEG);
		}

		// V22.06: q_cmd-to-actual lead protection directly limits the rate demand generated by the attitude P loop.
		// It is the main response to the flight where body rate reached ~110deg/s while the gimbal transiently lagged.
		if (_state == GuidanceState::BPN) {
			attitude_lead_max_deg = fminf(attitude_lead_max_deg, _gimbal_supervisor_lead_cap_deg);
		}
		if (attitude_reset_guard_active(now)) {
			attitude_lead_max_deg = fminf(attitude_lead_max_deg, ATT_RESET_GUARD_LEAD_CAP_DEG);
		}

		const float attitude_lead_max_rad = attitude_lead_max_deg * M_PI_F / 180.f;
		float lead_angle_rad = 0.f;

		const Quatf q_cmd = slerp_limited(
					    current_attitude_q(),
					    q_slew,
					    attitude_lead_max_rad,
					    _lead_active,
					    lead_angle_rad
				    );

		_last_attitude_lead_deg = lead_angle_rad * 180.f / M_PI_F;

		// V22.02: raw thrust is retained as preview only. Final collective uses q_cmd with vertical-support priority,
		// preventing the V22.01 projection-only collapse toward MIN_THRUST when q_raw and q_cmd differ greatly.
		const float thrust_cmd = commanded_thrust_norm_from_accel_and_attitude(accel_sp_ned, q_cmd);

		_diag_accel_cmd_ned = accel_sp_ned;
		_diag_q_raw = q_raw;
		_diag_q_slew = q_slew;
		_diag_q_cmd = q_cmd;
		_diag_thrust_preview = thrust_norm; // pre-slew/raw acceleration magnitude
		_diag_slew_limited = _slew_active;
		_diag_lead_limited = _lead_active;
		_diag_cage_limited = false;
		_diag_guidance_valid = true;

		publish_attitude_setpoint_internal(q_cmd, thrust_cmd, now);
	}

	void publish_guidance_acceleration(const Vector3f &bpn_accel_body, const bool valid, const uint64_t now)
	{
		vehicle_guidance_acceleration_s msg{};

		msg.timestamp = now;

		msg.acceleration[0] = bpn_accel_body(0);
		msg.acceleration[1] = bpn_accel_body(1);
		msg.acceleration[2] = bpn_accel_body(2);

		msg.valid = valid;

		_guidance_accel_pub.publish(msg);
	}

	// ============================================================
	// BPN目标净加速度
	// ============================================================

	bool calculate_bpn_target(Vector3f &accel_sp_ned, Vector3f &bpn_accel_body)
	{
		if (!attitude_valid() || !_los.target_valid
		    || !PX4_ISFINITE(_los.gimbal_roll) || !PX4_ISFINITE(_los.gimbal_pitch)
		    || !PX4_ISFINITE(_los.los_azimuth) || !PX4_ISFINITE(_los.los_elevation)
		    || !PX4_ISFINITE(_los.los_az_rate) || !PX4_ISFINITE(_los.los_el_rate)) {
			return false;
		}

		const uint64_t now = hrt_absolute_time();
		const bool use_truth = SIMULATION_MODE && SIM_USE_TRUTH_LOS_CONTROL;
		const bool truth_ready = use_truth && truth_los_fresh(now);
		if (use_truth && !truth_ready) {
			return false;
		}
		if (!use_truth && !_inertial_los_rate_valid) {
			return false;
		}

		Vector3f los_body_control{0.f, 0.f, 0.f};
		Vector3f los_dot_control_body{0.f, 0.f, 0.f};
		bool prediction_used = false;

		if (use_truth) {
			// SITL保持原语义：几何LOS来自当前vehicle_target_los，LOS rate直接使用truth。
			const float miss_az = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_azimuth : _los_miss_az_est;
			const float miss_el = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_elevation : _los_miss_el_est;
			const Vector3f los_g = calculate_los_body(_los.gimbal_roll, _los.gimbal_pitch, miss_az, miss_el);
			los_body_control = guidance_to_px4_body(los_g);
			const float n = los_body_control.norm();
			if (!PX4_ISFINITE(n) || n <= FLT_EPSILON) { return false; }
			los_body_control /= n;
			los_dot_control_body = _truth_los_dot_body;
		} else {
			// 真机：BPN只使用预测到当前时刻的LOS/Ldot；预测链失败则由上层重捕获/target-loss保护接管。
			if (!calculate_control_los_body(now, los_body_control, los_dot_control_body, prediction_used)) {
				return false;
			}
		}

		const Vector3f los_guidance = px4_body_to_guidance(los_body_control);
		const Vector3f los_dot_guidance = px4_body_to_guidance(los_dot_control_body);
		const float los_guidance_norm = los_guidance.norm();
		if (!PX4_ISFINITE(los_guidance_norm) || los_guidance_norm <= FLT_EPSILON) {
			return false;
		}

		_last_truth_los_rate_mag = truth_ready ? _truth_los_dot_body.norm() : 0.f;
		_last_recon_los_rate_mag = _inertial_los_rate_valid ? _los_dot_inertial_body.norm() : 0.f;
		_last_truth_recon_error_mag =
			(truth_ready && _inertial_los_rate_valid)
			? (_los_dot_inertial_body - _truth_los_dot_body).norm()
			: 0.f;

		// A/B原始BPN诊断：SITL保留truth/recon比较；真机RECON使用实际current-time预测链。
		_last_bpn_truth_body_raw.zero();
		_last_bpn_recon_body_raw.zero();

		if (truth_ready) {
			const Vector3f truth_guidance = px4_body_to_guidance(_truth_los_dot_body);
			const Vector3f bpn_truth_guidance = calculate_bpn_body(los_guidance, truth_guidance, _vc_used);
			_last_bpn_truth_body_raw = guidance_to_px4_body(bpn_truth_guidance);
		}

		if (_inertial_los_rate_valid) {
			Vector3f recon_los_body = los_body_control;
			Vector3f recon_dot_body = _los_dot_inertial_body;
			if (!use_truth) {
				recon_dot_body = los_dot_control_body;
			}
			const Vector3f recon_los_guidance = px4_body_to_guidance(recon_los_body);
			const Vector3f recon_dot_guidance = px4_body_to_guidance(recon_dot_body);
			const Vector3f bpn_recon_guidance = calculate_bpn_body(recon_los_guidance, recon_dot_guidance, _vc_used);
			_last_bpn_recon_body_raw = guidance_to_px4_body(bpn_recon_guidance);
		}

		_last_los_rate_mag = los_dot_guidance.norm();
		_last_speed_target =
			calculate_guidance_speed_target(_last_los_rate_mag);

		const float guidance_speed = get_guidance_speed();

		// V22.03: update the post-RELEASE gimbal-protection schedule once per BPN target calculation.
		update_gimbal_protect_schedule(now);

		if (PX4_ISFINITE(guidance_speed)) {
			_last_speed_axial_scale =
				calculate_axial_speed_scale(
					guidance_speed,
					_last_speed_target);

			const float desired_axial_limit = AXIAL_ACCEL_MAX * _last_speed_axial_scale;
			// If velocity validity has just recovered, blend back instead of stepping 3->5m/s^2.
			if (!_last_guidance_speed_valid && PX4_ISFINITE(_last_axial_limit_cmd) && _last_axial_limit_cmd > 0.f) {
				const float dt_ax = LOOP_INTERVAL_US * 1e-6f;
				const float alpha_ax = dt_ax / (AXIAL_DEGRADED_BLEND_TAU + dt_ax);
				_last_axial_limit_cmd += alpha_ax * (desired_axial_limit - _last_axial_limit_cmd);
			} else {
				_last_axial_limit_cmd = desired_axial_limit;
			}
			_last_guidance_speed_valid = true;

		} else {
			// V22.01: velocity source unavailable. Keep the conservative 3m/s^2 target,
			// but blend toward it so EKF/IMU expiry does not create a collective-thrust step.
			const float degraded_scale =
				math::constrain(AXIAL_ACCEL_DEGRADED_NO_VEL / fmaxf(AXIAL_ACCEL_MAX, 0.1f), 0.f, 1.f);
			_last_speed_axial_scale = degraded_scale;

			if (!PX4_ISFINITE(_last_axial_limit_cmd) || _last_axial_limit_cmd <= 0.f) {
				_last_axial_limit_cmd = AXIAL_ACCEL_DEGRADED_NO_VEL;
			} else {
				const float dt_ax = LOOP_INTERVAL_US * 1e-6f;
				const float alpha_ax = dt_ax / (AXIAL_DEGRADED_BLEND_TAU + dt_ax);
				_last_axial_limit_cmd += alpha_ax * (AXIAL_ACCEL_DEGRADED_NO_VEL - _last_axial_limit_cmd);
			}
			_last_guidance_speed_valid = false;
		}

		// V22.03: early post-RELEASE axial cap. This is an upper bound only; speed taper/degraded logic may command less.
		_last_axial_limit_cmd = fminf(_last_axial_limit_cmd, _last_gimbal_protect_axial_cap);

		// V22.06: gimbal-load protection is transparent in the safe region and progressively lowers only
		// the forward/axial demand when the gimbal is approaching its high-dynamic envelope.
		_last_axial_limit_cmd = fminf(_last_axial_limit_cmd, _gimbal_supervisor_axial_cap);

		// EKF attitude/yaw reset guard: keep the airframe calm for a short window while all reference-linked
		// states settle.  Lateral BPN remains available; the first action is simply to stop accelerating hard.
		if (attitude_reset_guard_active(now)) {
			_last_axial_limit_cmd = fminf(_last_axial_limit_cmd, ATT_RESET_GUARD_AXIAL_CAP);
		}

		// 当前真正用于控制的BPN。
		const Vector3f bpn_accel_guidance =
			calculate_bpn_body(
				los_guidance,
				los_dot_guidance,
				_vc_used);

		_last_bpn_norm_raw =
			bpn_accel_guidance.norm();

		const Vector3f los_body = los_body_control;

		const Vector3f bpn_accel_body_raw =
			guidance_to_px4_body(
				bpn_accel_guidance);

		const Dcmf R_nb{current_attitude_q()};

		Vector3f los_ned = R_nb * los_body;
		const float los_ned_norm = los_ned.norm();

		if (!PX4_ISFINITE(los_ned_norm)
		    || los_ned_norm < FLT_EPSILON) {
			return false;
		}

		los_ned /= los_ned_norm;

		const Vector3f bpn_accel_ned_raw =
			R_nb * bpn_accel_body_raw;

		// V22.03: during the first 1.5s after RELEASE, release lateral authority from 75% to 100%.
		// Apply this before the existing NED jerk limiter so the limiter state follows the actually permitted command.
		const Vector3f bpn_accel_ned_release_limited =
			bpn_accel_ned_raw * _last_gimbal_protect_bpn_scale;

		// V22.02: shape only the lateral BPN vector; axial acceleration and speed-brake remain independent.
		Vector3f bpn_accel_ned_shaped =
			limit_bpn_accel_jerk_ned(bpn_accel_ned_release_limited, now);

		// V22.05 weak boresight-centering assist. The gimbal center LOS and target LOS are compared in the
		// same Guidance frame; only the tangent component is used, and the term is hard-limited to 1.2m/s^2.
		_last_centering_assist_norm = 0.f;
		if (_state == GuidanceState::BPN && _last_los_rate_mag >= CENTERING_ASSIST_MIN_LOS_RATE) {
			const float miss_az = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_azimuth : _los_miss_az_est;
			const float miss_el = LOS_INPUT_HAS_MISS_ANGLE ? _los.los_elevation : _los_miss_el_est;
			const float miss_mag_rad = sqrtf(miss_az * miss_az + miss_el * miss_el);
			const float miss_deg = miss_mag_rad * 180.f / M_PI_F;
			if (PX4_ISFINITE(miss_deg) && miss_deg > CENTERING_ASSIST_START_DEG) {
				const Vector3f center_guidance = calculate_los_body(_los.gimbal_roll, _los.gimbal_pitch, 0.f, 0.f);
				const Vector3f center_body = guidance_to_px4_body(center_guidance);
				Vector3f center_ned = R_nb * center_body;
				const float cn = center_ned.norm();
				if (PX4_ISFINITE(cn) && cn > FLT_EPSILON) {
					center_ned /= cn;
					Vector3f tangent = los_ned - center_ned;
					tangent -= los_ned * tangent.dot(los_ned);
					const float tn = tangent.norm();
					if (PX4_ISFINITE(tn) && tn > FLT_EPSILON) {
						const float u = math::constrain((miss_deg - CENTERING_ASSIST_START_DEG)
							/ fmaxf(CENTERING_ASSIST_FULL_DEG - CENTERING_ASSIST_START_DEG, 0.1f), 0.f, 1.f);
						const float mag = fminf(CENTERING_ASSIST_GAIN * miss_mag_rad, CENTERING_ASSIST_MAX)
							* smoothstep01(u);
						const Vector3f assist_ned = tangent * (mag / tn);
						bpn_accel_ned_shaped += assist_ned;
						_last_centering_assist_norm = mag;
					}
				}
			}
		}

		// Keep the total lateral command inside the existing BPN envelope even after the weak centering assist.
		const float shaped_total_norm = bpn_accel_ned_shaped.norm();
		if (PX4_ISFINITE(shaped_total_norm) && shaped_total_norm > BPN_ACCEL_MAX && shaped_total_norm > FLT_EPSILON) {
			bpn_accel_ned_shaped *= BPN_ACCEL_MAX / shaped_total_norm;
		}

		const Vector3f bpn_accel_body_shaped =
			R_nb.transpose() * bpn_accel_ned_shaped;

		float axial_used = 0.f;
		float bpn_scale = 1.f;
		float predicted_thrust = 0.f;

		allocate_guidance_acceleration(
			los_ned,
			bpn_accel_ned_shaped,
			_last_axial_limit_cmd,
			GUIDANCE_THRUST_LIMIT,
			accel_sp_ned,
			axial_used,
			bpn_scale,
			predicted_thrust);

		float speed_brake_mag = 0.f;

		accel_sp_ned +=
			calculate_speed_brake_accel_ned(
				_last_speed_target,
				speed_brake_mag);

		_last_speed_brake = speed_brake_mag;

		bpn_accel_body =
			bpn_accel_body_shaped * bpn_scale;

		_last_axial_used = axial_used;
		// Include V22.03 release-authority scheduling in the logged total scale.
		_last_bpn_scale = bpn_scale * _last_gimbal_protect_bpn_scale;
		_last_bpn_norm_used = bpn_accel_body.norm();
		_last_predicted_thrust = predicted_thrust;

		_diag_bpn_raw_body = bpn_accel_body_raw;
		_diag_bpn_used_body = bpn_accel_body;

		return true;
	}


	// ============================================================
	// 正式飞行 commissioning gain
	// 正式BPN调用；legacy TRANSITION也保留调用；RIG Stage3继续使用独立RIG_STAGE3_BPN_SCALE。
	// 只缩放法向BPN分量，不缩放轴向加速度、speed brake、重力补偿和高度保护。
	// ============================================================
	void apply_flight_commissioning_gain(Vector3f &accel_sp_ned, Vector3f &bpn_accel_body)
	{
		if (FLIGHT_COMMISSIONING_GAIN >= 0.999f) {
			return;
		}

		const Dcmf R_nb{current_attitude_q()};
		const Vector3f bpn_ned_before = R_nb * bpn_accel_body;

		bpn_accel_body *= FLIGHT_COMMISSIONING_GAIN;

		const Vector3f bpn_ned_after = R_nb * bpn_accel_body;
		accel_sp_ned += bpn_ned_after - bpn_ned_before;

		_last_bpn_scale *= FLIGHT_COMMISSIONING_GAIN;
		_last_bpn_norm_used = bpn_accel_body.norm();
		_diag_bpn_used_body = bpn_accel_body;
	}


	// ============================================================
	// LAUNCH_CLEARANCE目标
	//
	// 只给小幅水平前向净加速度。
	// z=0表示默认不主动制造爬升/下降净加速度。
	// 垂向掉高由release altitude floor guard单独处理。
	// ============================================================

	bool calculate_clearance_target(Vector3f &accel_sp_ned)
	{
		Vector3f los_ned;

		if (!calculate_los_ned(los_ned)) { return false; }

		Vector3f horizontal_dir{los_ned(0), los_ned(1), 0.f};

		const float horizontal_norm = horizontal_dir.norm();

		if (PX4_ISFINITE(horizontal_norm) && horizontal_norm > FLT_EPSILON) {
			horizontal_dir /= horizontal_norm;

		} else {
			horizontal_dir.zero();
		}

		accel_sp_ned = horizontal_dir * CLEARANCE_FORWARD_ACCEL;
		accel_sp_ned(2) = 0.f;

		return true;
	}

	// ============================================================
	// 发射阶段
	// ============================================================

	void enter_wait_release(const uint64_t now)
	{
		reset_bpn_jerk_limiter();
		reset_gimbal_protect_window();
		_hold_q = current_attitude_q();

		_state = GuidanceState::WAIT_RELEASE;
		_state_enter_time = now;
		_transition_phase = 0.f;

		// ARM只代表进入等待释放状态；无论仿真还是真机，每次ARM都重新锁住RELEASE互锁。
		reset_rc_release_interlock();

		PX4_INFO("ARM -> WAIT_RELEASE");
		if (SIMULATION_MODE) {
			PX4_INFO("SIM RELEASE: virtual AUX1 starts LOW; guards stable %ums -> HIGH held %ums",
				 (unsigned)(SIM_RELEASE_READY_HOLD_US / 1000ULL), (unsigned)(RC_RELEASE_CONFIRM_US / 1000ULL));
		} else {
			PX4_INFO("RC RELEASE interlock: after ARM require real AUX1 LOW, then LOW->HIGH held %ums",
				 (unsigned)(RC_RELEASE_CONFIRM_US / 1000ULL));
		}
	}

	// ============================================================
	// V21.9.1: RELEASE真正接受时重新锁定当前Yaw
	//
	// WAIT_LOS阶段的yaw_lock可能比实际AUX1 RELEASE早很多；如果期间机体被搬动/转向，
	// 继续沿用旧yaw_lock会在制导接管后额外产生大幅偏航命令，迫使两轴云台高速反向补偿。
	// 因此只在AUX1 HIGH完成100ms确认且所有RELEASE guard均通过后刷新一次。
	// CLEARANCE/TRANSITION/BPN的其余逻辑均保持v21.9不变。
	// ============================================================
	void relock_yaw_at_release(const char *source)
	{
		const float old_yaw = _yaw_lock;
		const Eulerf euler_now{current_attitude_q()};
		_yaw_lock = euler_now.psi();
		_yaw_locked = true;

		PX4_INFO("%s accepted: YAW RELOCK %.1f -> %.1f deg",
			 source,
			 (double)(old_yaw * 180.f / M_PI_F),
			 (double)(_yaw_lock * 180.f / M_PI_F));
	}

	// ============================================================
	// V21.2 正式飞行：RELEASE后直接进入BPN ACTIVE
	//
	// 不经过LAUNCH_CLEARANCE和TRANSITION，也不做BPN gain ramp。
	// 但把原CLEARANCE入口中真正需要的“任务边界初始化”保留下来：
	//   1) 当前真实姿态作为BPN姿态slew起点；
	//   2) RELEASE时刻高度作为ALT floor基准；
	//   3) 清理上一轮越靶/reacquire/缓存状态；
	//   4) 从RELEASE重新开始A/B统计。
	// ============================================================
	void enter_bpn_active(const uint64_t now)
	{
		reset_bpn_jerk_limiter();
		start_gimbal_protect_window(now);
		_hold_q = current_attitude_q();
		_hold_q.normalize();

		// 直接BPN第一帧从当前真实姿态开始slew，避免继承BPN_READY旧setpoint造成姿态跳变。
		_last_q_sp = _hold_q;
		_last_raw_q_sp = _hold_q;
		_last_setpoint_pub_time = now;
		_slew_active = false;
		_lead_active = false;
		_last_raw_q_step_deg = 0.f;
		_last_attitude_lead_deg = 0.f;

		// RELEASE高度继续作为掉高保护参考；虽然不再经过CLEARANCE，ALT floor仍在run_bpn()中生效。
		if (_local_position.z_valid && PX4_ISFINITE(_local_position.z)) {
			_release_alt_ref_z_ned = _local_position.z;
			_release_alt_ref_valid = true;
		} else {
			_release_alt_ref_valid = false;
		}

		_alt_floor_active = false;
		_last_alt_floor_drop = 0.f;
		_last_alt_floor_predicted_drop = 0.f;
		_last_alt_floor_up_accel = 0.f;

		_had_positive_closing = false;
		_negative_closing_start_time = 0;

		_los_reacquire_active = false;
		_los_reacquire_start_time = 0;
		_los_reacquire_hold_logged = false;
		reset_target_loss_grace();
		_last_valid_guidance_cmd_valid = false;
		_last_valid_guidance_cmd_time = 0;

		reset_v20_truth_metrics();
		_v20_metric_active = SIMULATION_MODE;

		_state = GuidanceState::BPN;
		_state_enter_time = now;
		_transition_phase = 1.f;
		_launch_sequence_complete = true;

		// V21.4: RELEASE瞬间锁存当前Commander模式，后续任何模式变化都触发RC/系统接管。
		_rc_override_latched = false;
		_guidance_release_nav_state = _vehicle_status.nav_state;
		_guidance_release_nav_state_valid = true;

		PX4_INFO("RELEASE SIGNAL -> BPN ACTIVE (DIRECT, no CLEARANCE/TRANSITION)");
		PX4_INFO("V22.04 GIMBAL PROTECT: %.2fs axial %.1f->%.1f m/s2 BPN %.0f->100%% slew %.0f->%.0f deg/s",
			 (double)(GIMBAL_PROTECT_DURATION_US * 1e-6f),
			 (double)GIMBAL_PROTECT_AXIAL_START, (double)AXIAL_ACCEL_MAX,
			 (double)(GIMBAL_PROTECT_BPN_SCALE_START * 100.f),
			 (double)GIMBAL_PROTECT_SLEW_START_DEG_S, (double)GUIDANCE_ATTITUDE_SLEW_DEG_S);
		PX4_INFO("BPN direct cfg: hover=%.3f PN_N=%.2f BPN_MAX=%.2f axial_max=%.2f thrust=%.2f alt_floor=%.2f slew=%.1fdeg/s lead=%.1fdeg",
			 (double)HOVER_THRUST, (double)PN_N, (double)BPN_ACCEL_MAX,
			 (double)AXIAL_ACCEL_MAX, (double)GUIDANCE_THRUST_LIMIT, (double)ALT_FLOOR_THRUST_LIMIT,
			 (double)GUIDANCE_ATTITUDE_SLEW_DEG_S, (double)GUIDANCE_ATTITUDE_LEAD_MAX_DEG);
		if (_release_alt_ref_valid) {
			PX4_INFO("BPN direct ALT floor reference z=%.2f", (double)_release_alt_ref_z_ned);
		}
	}

	// Legacy path retained for rollback only when DIRECT_BPN_AFTER_RELEASE=false.
	void enter_launch_clearance(const uint64_t now)
	{
		reset_bpn_jerk_limiter();
		start_gimbal_protect_window(now);
		_hold_q = current_attitude_q();

		_clearance_start_accel_ned =
			attitude_thrust_to_net_accel(
				_hold_q,
				CLEARANCE_START_THRUST
			);

		// RELEASE高度作为掉高保护参考。
		if (_local_position.z_valid && PX4_ISFINITE(_local_position.z)) {
			_release_alt_ref_z_ned = _local_position.z;
			_release_alt_ref_valid = true;

		} else {
			_release_alt_ref_valid = false;
		}

		_alt_floor_active = false;
		_last_alt_floor_drop = 0.f;
		_last_alt_floor_predicted_drop = 0.f;
		_last_alt_floor_up_accel = 0.f;

		_state = GuidanceState::LAUNCH_CLEARANCE;
		_state_enter_time = now;
		_transition_phase = 0.f;

		// V21.4: legacy RELEASE路径同样建立模式基线。
		_rc_override_latched = false;
		_guidance_release_nav_state = _vehicle_status.nav_state;
		_guidance_release_nav_state_valid = true;

		_had_positive_closing = false;
		_negative_closing_start_time = 0;
		reset_target_loss_grace();

		// V20 A/B统计从真正RELEASE重新开始，排除PrelaunchTracker发射前整体转动阶段。
		reset_v20_truth_metrics();
		_v20_metric_active = SIMULATION_MODE;

		PX4_INFO("RELEASE SIGNAL -> LAUNCH_CLEARANCE");
		PX4_INFO("V22.04 GIMBAL PROTECT: %.2fs axial %.1f->%.1f m/s2 BPN %.0f->100%% slew %.0f->%.0f deg/s",
			 (double)(GIMBAL_PROTECT_DURATION_US * 1e-6f),
			 (double)GIMBAL_PROTECT_AXIAL_START, (double)AXIAL_ACCEL_MAX,
			 (double)(GIMBAL_PROTECT_BPN_SCALE_START * 100.f),
			 (double)GIMBAL_PROTECT_SLEW_START_DEG_S, (double)GUIDANCE_ATTITUDE_SLEW_DEG_S);
		PX4_INFO("V20 A/B metrics reset at RELEASE; reject truth rate > %.2f rad/s", (double)V20_METRIC_TRUTH_RATE_MAX);

		if (!SIMULATION_MODE) {
			PX4_INFO("CLEARANCE HW hold release attitude %.2f s start_thr %.2f limit %.2f",
				 (double)(HW_CLEARANCE_HOLD_DURATION_US * 1e-6f),
				 (double)CLEARANCE_START_THRUST, (double)CLEARANCE_THRUST_LIMIT);
		} else {
			PX4_INFO("CLEARANCE SITL start_thr %.2f limit %.2f ramp %.2f s hold %.2f s",
				 (double)CLEARANCE_START_THRUST, (double)CLEARANCE_THRUST_LIMIT,
				 (double)(CLEARANCE_RAMP_DURATION_US * 1e-6f),
				 (double)(CLEARANCE_HOLD_DURATION_US * 1e-6f));
			PX4_INFO("CLEARANCE SITL forward %.2f m/s^2, no fixed climb accel", (double)CLEARANCE_FORWARD_ACCEL);
		}

		PX4_INFO("GUIDANCE slew %.1f deg/s lead %.1f deg",
			 (double)GUIDANCE_ATTITUDE_SLEW_DEG_S,
			 (double)GUIDANCE_ATTITUDE_LEAD_MAX_DEG);

		if (_release_alt_ref_valid) {
			PX4_INFO("ALT floor reference z=%.2f", (double)_release_alt_ref_z_ned);
		}
	}

	void enter_transition(const uint64_t now, const Vector3f &start_accel_ned)
	{
		_transition_start_accel_ned = start_accel_ned;

		_state = GuidanceState::TRANSITION;
		_state_enter_time = now;
		_transition_phase = 0.f;

		PX4_INFO("LAUNCH_CLEARANCE -> TRANSITION");
		PX4_INFO("TRANSITION duration %.2f s", (double)(TRANSITION_DURATION_US * 1e-6f));
	}

	void run_launch_clearance(const uint64_t now)
	{
		// V21.7真机：CLEARANCE仅保持RELEASE瞬间姿态100ms，降低机体角速度/云台补偿负担。
		if (!SIMULATION_MODE) {
			const uint64_t elapsed = now - _state_enter_time;
			Vector3f accel_equiv_ned = attitude_thrust_to_net_accel(_hold_q, CLEARANCE_START_THRUST);

			apply_release_altitude_floor_guard(accel_equiv_ned);

			if (_alt_floor_active) {
				constrain_net_accel_vertical_priority(accel_equiv_ned, CLEARANCE_THRUST_LIMIT);
				publish_accel_attitude_setpoint(accel_equiv_ned, now);
			} else {
				publish_direct_attitude(_hold_q, CLEARANCE_START_THRUST, now);
			}

			_last_bpn_body.zero();
			_last_axial_used = 0.f;
			_last_bpn_norm_raw = 0.f;
			_last_bpn_norm_used = 0.f;
			_last_bpn_scale = 1.f;
			_last_speed_brake = 0.f;
			_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_equiv_ned);
			publish_guidance_acceleration(_last_bpn_body, false, now);
			cache_valid_guidance_command(accel_equiv_ned, _last_bpn_body, now);

			if (elapsed >= HW_CLEARANCE_HOLD_DURATION_US) {
				enter_transition(now, accel_equiv_ned);
			}
			return;
		}

		// V18第二阶段诊断开关，默认false。
		// 若第一轮TRUTH闭环仍有系统性上方越靶，可改true：
		// CLEARANCE期间保持RELEASE瞬间姿态，不再主动拉向近水平姿态。
		if (SIMULATION_MODE && SIM_KEEP_RELEASE_ATTITUDE_CLEARANCE) {
			const uint64_t elapsed = now - _state_enter_time;

			const float raw_phase =
				math::constrain(
					(float)elapsed
					/ (float)CLEARANCE_RAMP_DURATION_US,
					0.f,
					1.f);

			const float phase = smoothstep01(raw_phase);
			_transition_phase = phase;

			const float thrust_cmd =
				CLEARANCE_START_THRUST * (1.f - phase)
				+ GUIDANCE_THRUST_LIMIT * phase;

			Vector3f accel_equiv_ned =
				attitude_thrust_to_net_accel(
					_hold_q,
					thrust_cmd);

			// 仍保留原掉高保护；只有真正掉高才允许它改变姿态。
			apply_release_altitude_floor_guard(accel_equiv_ned);

			if (_alt_floor_active) {
				constrain_net_accel_vertical_priority(
					accel_equiv_ned,
					CLEARANCE_THRUST_LIMIT);

				publish_accel_attitude_setpoint(
					accel_equiv_ned,
					now);
			} else {
				publish_direct_attitude(
					_hold_q,
					thrust_cmd,
					now);
			}

			_last_bpn_body.zero();
			_last_axial_used = 0.f;
			_last_bpn_norm_raw = 0.f;
			_last_bpn_norm_used = 0.f;
			_last_bpn_scale = 1.f;
			_last_speed_brake = 0.f;
			_last_predicted_thrust = thrust_cmd;

			publish_guidance_acceleration(
				_last_bpn_body,
				false,
				now);

			const uint64_t total_clearance_time =
				CLEARANCE_RAMP_DURATION_US
				+ CLEARANCE_HOLD_DURATION_US;

			if (elapsed >= total_clearance_time) {
				enter_transition(
					now,
					accel_equiv_ned);
			}

			return;
		}

		Vector3f clearance_target_ned;

		if (!calculate_clearance_target(clearance_target_ned)) {
			enter_safe_hover(now, "invalid clearance target");
			return;
		}

		const uint64_t elapsed = now - _state_enter_time;

		Vector3f accel_sp_ned;

		if (elapsed < CLEARANCE_RAMP_DURATION_US) {
			const float raw_phase = math::constrain(
							(float)elapsed
							/ (float)CLEARANCE_RAMP_DURATION_US,
							0.f,
							1.f
						);

			const float phase = smoothstep01(raw_phase);

			_transition_phase = phase;

			accel_sp_ned =
				_clearance_start_accel_ned * (1.f - phase)
				+ clearance_target_ned * phase;

		} else {
			_transition_phase = 1.f;
			accel_sp_ned = clearance_target_ned;
		}

		// 只有发生/预测发生掉高时才向上干预。
		apply_release_altitude_floor_guard(accel_sp_ned);

		// CLEARANCE使用独立0.95推力预算，而不是BPN的0.75。
		constrain_net_accel_vertical_priority(accel_sp_ned, CLEARANCE_THRUST_LIMIT);

		_last_bpn_body.zero();
		_last_axial_used = 0.f;
		_last_bpn_norm_raw = 0.f;
		_last_bpn_norm_used = 0.f;
		_last_bpn_scale = 1.f;
		_last_speed_brake = 0.f;

		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);

		publish_accel_attitude_setpoint(accel_sp_ned, now);
		publish_guidance_acceleration(_last_bpn_body, false, now);

		const uint64_t total_clearance_time =
			CLEARANCE_RAMP_DURATION_US
			+ CLEARANCE_HOLD_DURATION_US;

		if (elapsed >= total_clearance_time) {
			enter_transition(now, accel_sp_ned);
		}
	}

	void cache_valid_guidance_command(const Vector3f &accel_sp_ned, const Vector3f &bpn_accel_body, const uint64_t now)
	{
		if (!PX4_ISFINITE(accel_sp_ned(0)) || !PX4_ISFINITE(accel_sp_ned(1)) || !PX4_ISFINITE(accel_sp_ned(2))
		    || !PX4_ISFINITE(bpn_accel_body(0)) || !PX4_ISFINITE(bpn_accel_body(1)) || !PX4_ISFINITE(bpn_accel_body(2))) {
			return;
		}
		_last_valid_guidance_accel_ned = accel_sp_ned;
		_last_valid_guidance_bpn_body = bpn_accel_body;
		_last_valid_guidance_cmd_time = now;
		_last_valid_guidance_cmd_valid = true;
	}

	void reset_target_loss_grace()
	{
		_target_loss_start_time = 0;
		_target_loss_hold_logged = false;
	}

	bool hold_last_guidance_during_target_loss(const uint64_t now)
	{
		if (!_last_valid_guidance_cmd_valid || _last_valid_guidance_cmd_time == 0
		    || now < _last_valid_guidance_cmd_time
		    || now - _last_valid_guidance_cmd_time > TARGET_LOSS_GRACE_US + LOOP_INTERVAL_US) {
			return false;
		}

		Vector3f accel_hold = _last_valid_guidance_accel_ned;
		apply_release_altitude_floor_guard(accel_hold);

		float active_thrust_limit = GUIDANCE_THRUST_LIMIT;
		if (_state == GuidanceState::LAUNCH_CLEARANCE) {
			active_thrust_limit = CLEARANCE_THRUST_LIMIT;
		} else if (_state == GuidanceState::TRANSITION && _alt_floor_active) {
			active_thrust_limit = CLEARANCE_THRUST_LIMIT;
		} else if (_alt_floor_active) {
			active_thrust_limit = ALT_FLOOR_THRUST_LIMIT;
		}

		constrain_net_accel_vertical_priority(accel_hold, active_thrust_limit);
		_last_bpn_body = _last_valid_guidance_bpn_body;
		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_hold);
		publish_accel_attitude_setpoint(accel_hold, now);
		publish_guidance_acceleration(_last_valid_guidance_bpn_body, false, now);

		if (!_target_loss_hold_logged) {
			PX4_WARN("TARGET LOSS grace: hold last valid guidance for <=%ums",
				 (unsigned)(TARGET_LOSS_GRACE_US / 1000ULL));
			_target_loss_hold_logged = true;
		}
		return true;
	}

	bool hold_last_guidance_during_los_reacquire(const uint64_t now, const char *phase_name)
	{
		if (!los_reacquire_grace_active(now) || !los_fresh(now) || !_los.target_valid || !attitude_valid()) {
			return false;
		}
		if (!_last_valid_guidance_cmd_valid || _last_valid_guidance_cmd_time == 0
		    || now < _last_valid_guidance_cmd_time
		    || now - _last_valid_guidance_cmd_time > LOS_REACQUIRE_GRACE_US) {
			return false;
		}

		Vector3f accel_hold = _last_valid_guidance_accel_ned;
		apply_release_altitude_floor_guard(accel_hold);
		const float active_thrust_limit = _alt_floor_active ? ALT_FLOOR_THRUST_LIMIT : GUIDANCE_THRUST_LIMIT;
		constrain_net_accel_vertical_priority(accel_hold, active_thrust_limit);

		_last_bpn_body = _last_valid_guidance_bpn_body;
		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_hold);
		publish_accel_attitude_setpoint(accel_hold, now);
		publish_guidance_acceleration(_last_valid_guidance_bpn_body, true, now);

		if (!_los_reacquire_hold_logged) {
			PX4_WARN("%s: LOS observer reacquiring, hold last valid guidance for <=%ums",
				 phase_name, (unsigned)(LOS_REACQUIRE_GRACE_US / 1000));
			_los_reacquire_hold_logged = true;
		}
		return true;
	}

	void run_transition(const uint64_t now)
	{
		Vector3f target_accel_ned;
		Vector3f bpn_accel_body;

		if (!calculate_bpn_target(target_accel_ned, bpn_accel_body)) {
			if (hold_last_guidance_during_los_reacquire(now, "TRANSITION")) { return; }
			if (_los_reacquire_active && !los_reacquire_grace_active(now)) {
				enter_safe_hover(now, "LOS observer reacquire timeout in transition");
			} else {
				enter_safe_hover(now, "invalid BPN target in transition");
			}
			return;
		}

		apply_flight_commissioning_gain(target_accel_ned, bpn_accel_body);

		const float raw_phase = math::constrain(
						(float)(now - _state_enter_time)
						/ (float)TRANSITION_DURATION_US,
						0.f,
						1.f
					);

		const float phase = smoothstep01(raw_phase);

		_transition_phase = phase;

		Vector3f accel_sp_ned =
			_transition_start_accel_ned * (1.f - phase)
			+ target_accel_ned * phase;

		apply_release_altitude_floor_guard(accel_sp_ned);

		// Legacy TRANSITION：正常使用GUIDANCE_THRUST_LIMIT；高度保护时沿用legacy CLEARANCE预算。
		// 注意：这是“最大允许推力”，不是强制输出0.95。
		const float active_thrust_limit =
		    _alt_floor_active
		    ? CLEARANCE_THRUST_LIMIT
		    : GUIDANCE_THRUST_LIMIT;

		constrain_net_accel_vertical_priority(
		    accel_sp_ned,
		    active_thrust_limit
		);

		// TRANSITION回到正常BPN推力预算。
		// constrain_net_accel_vertical_priority(accel_sp_ned, GUIDANCE_THRUST_LIMIT);

		_last_bpn_body = bpn_accel_body;
		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);

		publish_accel_attitude_setpoint(accel_sp_ned, now);
		publish_guidance_acceleration(bpn_accel_body, true, now);
		cache_valid_guidance_command(accel_sp_ned, bpn_accel_body, now);

		if (raw_phase >= 1.f) {
			_state = GuidanceState::BPN;
			_state_enter_time = now;

			_launch_sequence_complete = true;
			_transition_phase = 1.f;

			PX4_INFO("TRANSITION -> BPN");
		}
	}

	// ============================================================
	// 正常BPN
	// ============================================================

	void run_bpn(const uint64_t now)
	{
		Vector3f accel_sp_ned;
		Vector3f bpn_accel_body;

		if (!calculate_bpn_target(accel_sp_ned, bpn_accel_body)) {
			if (hold_last_guidance_during_los_reacquire(now, "BPN")) { return; }
			if (_los_reacquire_active && !los_reacquire_grace_active(now)) {
				enter_safe_hover(now, "LOS observer reacquire timeout");
			} else {
				enter_safe_hover(now, "invalid BPN target");
			}
			return;
		}

		apply_flight_commissioning_gain(accel_sp_ned, bpn_accel_body);

		apply_release_altitude_floor_guard(accel_sp_ned);

		// 正常BPN使用GUIDANCE_THRUST_LIMIT；若高度底线保护介入，则临时使用ALT_FLOOR_THRUST_LIMIT。
		const float active_thrust_limit =
		    _alt_floor_active
		    ? ALT_FLOOR_THRUST_LIMIT
		    : GUIDANCE_THRUST_LIMIT;

		constrain_net_accel_vertical_priority(
		    accel_sp_ned,
		    active_thrust_limit
		);

		_diag_accel_cmd_ned = accel_sp_ned;
		_diag_guidance_valid = true;

		// 正常BPN推力预算由GUIDANCE_THRUST_LIMIT决定，并优先保留Z方向能力。
		// constrain_net_accel_vertical_priority(accel_sp_ned, GUIDANCE_THRUST_LIMIT);

		_last_bpn_body = bpn_accel_body;
		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);

		publish_accel_attitude_setpoint(accel_sp_ned, now);
		publish_guidance_acceleration(bpn_accel_body, true, now);
		cache_valid_guidance_command(accel_sp_ned, bpn_accel_body, now);
	}

	// ============================================================
	// 越靶检测
	// ============================================================

	void monitor_target_pass(const uint64_t now)
	{
		if (_state != GuidanceState::TRANSITION
		    && !(_state == GuidanceState::BPN && _launch_sequence_complete)) {

			_negative_closing_start_time = 0;
			return;
		}

		if (!los_fresh(now) || !_los.target_valid || !_vc_valid || !_had_positive_closing) {
			_negative_closing_start_time = 0;
			return;
		}

		const float speed = get_guidance_speed();

		if (!PX4_ISFINITE(speed) || speed < MISS_MIN_SPEED) {
			_negative_closing_start_time = 0;
			return;
		}

		if (_vc_raw < MISS_TRIGGER_VC) {
			if (_negative_closing_start_time == 0) {
				_negative_closing_start_time = now;
			}

			if (now - _negative_closing_start_time >= MISS_TRIGGER_HOLD_US) {
				enter_safe_hover(now, "target passed: Vc became negative");
			}

		} else {
			_negative_closing_start_time = 0;
		}
	}

	// ============================================================
	// SAFE_HOVER
	// ============================================================

	void enter_safe_hover(const uint64_t now, const char *reason)
	{
		if (_state == GuidanceState::SAFE_HOVER) { return; }
		reset_bpn_jerk_limiter();

		PX4_WARN("-> SAFE_HOVER: %s", reason);

		_state = GuidanceState::SAFE_HOVER;
		_state_enter_time = now;
		_transition_phase = 1.f;

		_safe_hold_latched = false;
		_safe_low_speed_start_time = 0;

		_last_bpn_body.zero();
		_last_axial_used = 0.f;
		_last_bpn_norm_raw = 0.f;
		_last_bpn_norm_used = 0.f;
		_last_bpn_scale = 1.f;

		_last_speed_brake = 0.f;
		_alt_floor_active = false;
		reset_target_loss_grace();

		if (attitude_valid()) {
			_last_q_sp = current_attitude_q();
			_last_raw_q_sp = _last_q_sp;
		}

		PX4_WARN("SAFE_HOVER braking with EKF velocity");
	}

	Vector3f calculate_safe_brake_accel() const
	{
		Vector3f accel_sp{0.f, 0.f, 0.f};

		Vector3f velocity_ned;
		if (!get_guidance_velocity_ned(velocity_ned)) { return accel_sp; }

		accel_sp(0) = -SAFE_BRAKE_KV_XY * velocity_ned(0);
		accel_sp(1) = -SAFE_BRAKE_KV_XY * velocity_ned(1);
		accel_sp(2) = -SAFE_BRAKE_KV_Z * velocity_ned(2);

		constrain_xy(accel_sp, SAFE_BRAKE_ACCEL_XY_MAX);

		accel_sp(2) = math::constrain(
				      accel_sp(2),
				      -SAFE_BRAKE_ACCEL_Z_MAX,
				      SAFE_BRAKE_ACCEL_Z_MAX
			      );

		return accel_sp;
	}

	Vector3f calculate_safe_hold_accel() const
	{
		Vector3f accel_sp{0.f, 0.f, 0.f};

		if (!position_valid() || !velocity_valid()) {
			return calculate_safe_brake_accel();
		}

		const float ex = _local_position.x - _safe_hold_position_ned(0);
		const float ey = _local_position.y - _safe_hold_position_ned(1);
		const float ez = _local_position.z - _safe_hold_position_ned(2);

		accel_sp(0) =
			-SAFE_HOLD_KP_XY * ex
			-SAFE_HOLD_KD_XY * _local_position.vx;

		accel_sp(1) =
			-SAFE_HOLD_KP_XY * ey
			-SAFE_HOLD_KD_XY * _local_position.vy;

		accel_sp(2) =
			-SAFE_HOLD_KP_Z * ez
			-SAFE_HOLD_KD_Z * _local_position.vz;

		constrain_xy(accel_sp, SAFE_HOLD_ACCEL_XY_MAX);

		accel_sp(2) = math::constrain(
				      accel_sp(2),
				      -SAFE_HOLD_ACCEL_Z_MAX,
				      SAFE_HOLD_ACCEL_Z_MAX
			      );

		return accel_sp;
	}

	void run_safe_hover(const uint64_t now)
	{
		const float speed_xy = get_ekf_speed_xy();

		const float vz =
			velocity_valid()
			? fabsf(_local_position.vz)
			: NAN;

		if (!_safe_hold_latched) {
			if (PX4_ISFINITE(speed_xy)
			    && PX4_ISFINITE(vz)
			    && speed_xy < SAFE_LATCH_SPEED_XY
			    && vz < SAFE_LATCH_SPEED_Z
			    && position_valid()) {

				if (_safe_low_speed_start_time == 0) {
					_safe_low_speed_start_time = now;
				}

				if (now - _safe_low_speed_start_time >= SAFE_LATCH_HOLD_US) {
					_safe_hold_position_ned = Vector3f{
						_local_position.x,
						_local_position.y,
						_local_position.z
					};

					_safe_hold_latched = true;

					PX4_WARN(
						"SAFE_HOVER hold latched x=%.1f y=%.1f z=%.1f",
						(double)_safe_hold_position_ned(0),
						(double)_safe_hold_position_ned(1),
						(double)_safe_hold_position_ned(2)
					);
				}

			} else {
				_safe_low_speed_start_time = 0;
			}
		}

		const Vector3f accel_sp_ned =
			_safe_hold_latched
			? calculate_safe_hold_accel()
			: calculate_safe_brake_accel();

		_last_bpn_body.zero();

		_last_axial_used = 0.f;
		_last_bpn_norm_raw = 0.f;
		_last_bpn_norm_used = 0.f;
		_last_bpn_scale = 1.f;

		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_sp_ned);

		publish_accel_attitude_setpoint(accel_sp_ned, now);
		publish_guidance_acceleration(_last_bpn_body, false, now);
	}

	// ============================================================
	// EKF local position reset监控
	// ============================================================

	void monitor_ekf_resets(const uint64_t now, const bool prev_velocity_valid,
				const Vector3f &prev_velocity_ned, const uint64_t prev_lpos_rx_time)
	{
		bool velocity_reset_event = false;

		if (!_ekf_reset_counter_initialized) {
			_last_xy_reset_counter = _local_position.xy_reset_counter;
			_last_z_reset_counter = _local_position.z_reset_counter;
			_last_vxy_reset_counter = _local_position.vxy_reset_counter;
			_last_vz_reset_counter = _local_position.vz_reset_counter;

			_ekf_reset_counter_initialized = true;
			return;
		}

		if (_local_position.xy_reset_counter != _last_xy_reset_counter) {
			PX4_WARN(
				"EKF XY RESET %u -> %u",
				(unsigned)_last_xy_reset_counter,
				(unsigned)_local_position.xy_reset_counter
			);

			if (_safe_hold_latched) {
				_safe_hold_position_ned(0) += _local_position.delta_xy[0];
				_safe_hold_position_ned(1) += _local_position.delta_xy[1];
			}

			_last_xy_reset_counter = _local_position.xy_reset_counter;
		}

		if (_local_position.z_reset_counter != _last_z_reset_counter) {
			PX4_WARN(
				"EKF Z RESET %u -> %u",
				(unsigned)_last_z_reset_counter,
				(unsigned)_local_position.z_reset_counter
			);

			if (_safe_hold_latched) {
				_safe_hold_position_ned(2) += _local_position.delta_z;
			}

			if (_release_alt_ref_valid && PX4_ISFINITE(_local_position.delta_z)) {
				_release_alt_ref_z_ned += _local_position.delta_z;
			}

			_last_z_reset_counter = _local_position.z_reset_counter;
		}

		if (_local_position.vxy_reset_counter != _last_vxy_reset_counter) {
			PX4_WARN(
				"EKF VXY RESET %u -> %u",
				(unsigned)_last_vxy_reset_counter,
				(unsigned)_local_position.vxy_reset_counter
			);

			velocity_reset_event = true;
			_last_vxy_reset_counter = _local_position.vxy_reset_counter;
		}

		if (_local_position.vz_reset_counter != _last_vz_reset_counter) {
			PX4_WARN(
				"EKF VZ RESET %u -> %u",
				(unsigned)_last_vz_reset_counter,
				(unsigned)_local_position.vz_reset_counter
			);

			velocity_reset_event = true;
			_last_vz_reset_counter = _local_position.vz_reset_counter;
		}

		bool jump_event = false;
		float velocity_jump = 0.f;
		if (prev_velocity_valid && velocity_valid() && prev_lpos_rx_time > 0 && now >= prev_lpos_rx_time
		    && now - prev_lpos_rx_time <= EKF_VEL_JUMP_MAX_DT_US) {
			const Vector3f new_velocity{_local_position.vx, _local_position.vy, _local_position.vz};
			velocity_jump = (new_velocity - prev_velocity_ned).norm();
			jump_event = PX4_ISFINITE(velocity_jump) && velocity_jump > EKF_VEL_JUMP_TRIGGER_MPS;
		}

		if (jump_event) {
			PX4_WARN("EKF velocity jump %.2fm/s in %.1fms",
				 (double)velocity_jump,
				 (double)((now - prev_lpos_rx_time) * 1e-3f));
		}

		if (velocity_reset_event || jump_event) {
			start_ekf_velocity_guard(now, prev_velocity_ned, velocity_reset_event ? "RESET_COUNTER" : "DV_JUMP");
		}
	}

	// ============================================================
	// EKF attitude quaternion reset
	// ============================================================

	void monitor_attitude_reset(const uint64_t now)
	{
		if (!_attitude_reset_counter_initialized) {
			_last_quat_reset_counter = _attitude.quat_reset_counter;
			_attitude_reset_counter_initialized = true;
			return;
		}

		if (_attitude.quat_reset_counter == _last_quat_reset_counter) {
			return;
		}

		const uint8_t old_counter = _last_quat_reset_counter;
		const uint8_t new_counter = _attitude.quat_reset_counter;

		_last_quat_reset_counter = new_counter;

		// V20 Body-relative observer不依赖绝对NED姿态，因此EKF quaternion/yaw reset绝不重播种observer。
		// 这里只记录reset时刻用于SITL误差统计；yaw_lock/hold_q仍按delta_q_reset同步到新的EKF参考。
		_last_attitude_reset_time = now;
		PX4_WARN("V20 LOS observer preserved across EKF attitude reset (body-relative)");

		Quatf delta_q{_attitude.delta_q_reset};

		const float delta_norm = delta_q.norm();

		if (!PX4_ISFINITE(delta_norm) || delta_norm < FLT_EPSILON) {
			PX4_WARN(
				"EKF ATT RESET %u -> %u invalid delta_q",
				(unsigned)old_counter,
				(unsigned)new_counter
			);

			return;
		}

		delta_q.normalize();

		const Eulerf delta_euler{delta_q};
		const float delta_yaw = delta_euler.psi();
		const float delta_angle_rad = 2.f * acosf(math::constrain(fabsf(delta_q(0)), 0.f, 1.f));
		_last_attitude_reset_angle_deg = delta_angle_rad * 180.f / M_PI_F;

		PX4_WARN(
			"EKF ATT RESET %u -> %u dYaw=%.1f deg dAngle=%.1f deg",
			(unsigned)old_counter,
			(unsigned)new_counter,
			(double)(delta_yaw * 180.f / M_PI_F),
			(double)_last_attitude_reset_angle_deg
		);

		// V22.06: a quaternion reset is a reference correction, not a physical body impulse.
		// The BPN jerk state lives in the previous estimated geometry; re-seed it from the next raw BPN sample
		// instead of forcing the limiter to slew across an artificial coordinate discontinuity.
		reset_bpn_jerk_limiter();
		if (_last_attitude_reset_angle_deg >= ATT_RESET_GUARD_TRIGGER_DEG
		    || fabsf(delta_yaw) * 180.f / M_PI_F >= ATT_RESET_GUARD_TRIGGER_DEG) {
			_attitude_reset_guard_until = now + ATT_RESET_GUARD_DURATION_US;
			PX4_WARN("V22.06 ATT RESET GUARD %.0fms axial<=%.1f slew<=%.0f lead<=%.0f",
				(double)(ATT_RESET_GUARD_DURATION_US * 1e-3f),
				(double)ATT_RESET_GUARD_AXIAL_CAP, (double)ATT_RESET_GUARD_SLEW_CAP_DEG_S,
				(double)ATT_RESET_GUARD_LEAD_CAP_DEG);
		}

		if (_yaw_locked) {
			const float old_yaw = _yaw_lock;

			_yaw_lock = wrap_pi_local(_yaw_lock + delta_yaw);

			PX4_WARN(
				"YAW LOCK RESET %.1f -> %.1f deg",
				(double)(old_yaw * 180.f / M_PI_F),
				(double)(_yaw_lock * 180.f / M_PI_F)
			);
		}

		_hold_q = delta_q * _hold_q;
		_hold_q.normalize();

		_last_q_sp = delta_q * _last_q_sp;
		_last_q_sp.normalize();

		_last_raw_q_sp = delta_q * _last_raw_q_sp;
		_last_raw_q_sp.normalize();

		if (_state == GuidanceState::LAUNCH_CLEARANCE) {
			_clearance_start_accel_ned =
				attitude_thrust_to_net_accel(
					_last_q_sp,
					_last_thrust_sp
				);

			_state_enter_time = now;
			_transition_phase = 0.f;

			PX4_WARN("restart LAUNCH_CLEARANCE after ATT reset");
		}

		if (_state == GuidanceState::TRANSITION) {
			_transition_start_accel_ned =
				attitude_thrust_to_net_accel(
					_last_q_sp,
					_last_thrust_sp
				);

			_state_enter_time = now;
			_transition_phase = 0.f;

			PX4_WARN("restart TRANSITION after ATT reset");
		}
	}

	// ============================================================
	// 台架诊断：完整链路只读预览，不修改状态机、不发布vehicle_attitude_setpoint、不发布任何额外控制指令。
	// BENCH_RT同时给出Rraw、旧三项诊断以及V16离散观测器最终惯性LOS方向导数IN。
	// ============================================================

	bool calculate_bench_preview(Vector3f &los_guidance, Vector3f &los_dot_guidance, Vector3f &bpn_guidance,
				     Eulerf &e_preview, float &thrust_preview, const uint64_t now) const
	{
		const bool input_valid = _los.target_valid && los_fresh(now) && attitude_valid()
			&& PX4_ISFINITE(_los.gimbal_roll) && PX4_ISFINITE(_los.gimbal_pitch)
			&& PX4_ISFINITE(_los.los_azimuth) && PX4_ISFINITE(_los.los_elevation)
			&& PX4_ISFINITE(_los.los_az_rate) && PX4_ISFINITE(_los.los_el_rate);
		if (!input_valid) { return false; }
		los_guidance = calculate_los_body(_los.gimbal_roll, _los.gimbal_pitch, _los_miss_az_est, _los_miss_el_est);
		if (!PX4_ISFINITE(los_guidance.norm()) || los_guidance.norm() <= FLT_EPSILON || !_inertial_los_rate_valid) { return false; }
		los_dot_guidance = px4_body_to_guidance(_los_dot_inertial_body);
		// 固定Vc=20m/s只用于台架验证BPN方向/幅值；候选姿态前先把Guidance BPN转换成PX4 FRD Body。
		bpn_guidance = calculate_bpn_body(los_guidance, los_dot_guidance, VC_FALLBACK);
		const Vector3f bpn_body = guidance_to_px4_body(bpn_guidance);
		const Vector3f bpn_preview_ned = Dcmf{current_attitude_q()} * bpn_body;
		Quatf q_preview; thrust_preview = 0.f;
		acceleration_to_attitude(bpn_preview_ned, _yaw_lock, q_preview, thrust_preview);
		e_preview = Eulerf{q_preview};
		return true;
	}


	void print_bench_snapshot(const uint64_t now) const
	{
		Vector3f los_g, los_dot_g, bpn_g; Eulerf e_preview{0.f, 0.f, 0.f}; float thrust_preview = 0.f;
		const double age_ms = (_last_los_rx_time > 0 && now >= _last_los_rx_time) ? (double)(now - _last_los_rx_time) * 1e-3 : -1.0;
		if (!calculate_bench_preview(los_g, los_dot_g, bpn_g, e_preview, thrust_preview, now)) {
			PX4_INFO("BENCH_SNAP INVALID target=%d fresh=%d att=%d age=%.1fms", (int)_los.target_valid, (int)los_fresh(now), (int)attitude_valid(), age_ms);
			return;
		}
		const Vector3f los_b = guidance_to_px4_body(los_g);
		const Vector3f bpn_b = guidance_to_px4_body(bpn_g);
		PX4_INFO("BENCH_SNAP Gdeg=[%+.1f %+.1f] Mdeg=[%+.2f %+.2f] Rraw=[%+.3f %+.3f] Vused=%.2f recon=%d",
			 (double)(_los.gimbal_roll * 180.f / M_PI_F), (double)(_los.gimbal_pitch * 180.f / M_PI_F),
			 (double)(_los.los_azimuth * 180.f / M_PI_F), (double)(_los.los_elevation * 180.f / M_PI_F),
			 (double)_los.los_az_rate, (double)_los.los_el_rate, (double)_vc_used, (int)_inertial_los_rate_valid);
		// 【V20.5修改】拆成三行短输出，避免NSH截断，并明确区分raw image、gimbal、body、融合前FRAW和最终滤波LDin。
		PX4_INFO("BENCH_LD0 LB=[%+.3f %+.3f %+.3f]", (double)los_b(0), (double)los_b(1), (double)los_b(2));
		PX4_INFO("BENCH_LD1 IMG=[%+.4f %+.4f %+.4f] GIM=[%+.4f %+.4f %+.4f]",
			 (double)_los_dot_image_body(0), (double)_los_dot_image_body(1), (double)_los_dot_image_body(2),
			 (double)_los_dot_gimbal_body(0), (double)_los_dot_gimbal_body(1), (double)_los_dot_gimbal_body(2));
		PX4_INFO("BENCH_LD2 BODY=[%+.4f %+.4f %+.4f] FRAW=[%+.4f %+.4f %+.4f]",
			 (double)_los_dot_body_motion(0), (double)_los_dot_body_motion(1), (double)_los_dot_body_motion(2),
			 (double)_los_dot_fused_raw_body(0), (double)_los_dot_fused_raw_body(1), (double)_los_dot_fused_raw_body(2));
		PX4_INFO("BENCH_LD3 LDin=[%+.4f %+.4f %+.4f] GRate=[%+.3f %+.3f]",
			 (double)_los_dot_inertial_body(0), (double)_los_dot_inertial_body(1), (double)_los_dot_inertial_body(2),
			 (double)_gimbal_az_rate_filtered, (double)_gimbal_pitch_rate_filtered);
		PX4_INFO("BENCH_SNAP B20G(使用固定Vc=20m/s计算的Guidance BPN)=[%+.2f %+.2f %+.2f] B20B(转换成PX4 Body后的BPN)=[%+.2f %+.2f %+.2f] Q(预览姿态)=[%+.1f %+.1f %+.1f] T(预览推力)=%.3f NOT_PUBLISHED",
			 (double)bpn_g(0), (double)bpn_g(1), (double)bpn_g(2), (double)bpn_b(0), (double)bpn_b(1), (double)bpn_b(2),
			 (double)(e_preview.phi() * 180.f / M_PI_F), (double)(e_preview.theta() * 180.f / M_PI_F), (double)(e_preview.psi() * 180.f / M_PI_F), (double)thrust_preview);
	}


	void print_los_rate_realtime(const uint64_t now)
	{
		if (!_bench_realtime_enabled.load() || now - _last_los_rate_print_time < BENCH_PRINT_INTERVAL_US) { return; }
		_last_los_rate_print_time = now;

		Vector3f los_g, los_dot_g, bpn_g;
		Eulerf e_preview{0.f, 0.f, 0.f};
		float thrust_preview = 0.f;

		if (!calculate_bench_preview(los_g, los_dot_g, bpn_g, e_preview, thrust_preview, now)) {
			PX4_INFO("BENCH5 INVALID tgt=%d fresh=%d att=%d rate=%d body=%d age=%.1fms",
				 (int)_los.target_valid, (int)los_fresh(now), (int)attitude_valid(),
				 (int)_inertial_los_rate_valid, (int)body_rate_valid(now),
				 (double)(_last_los_sample_age_us * 1e-3f));
			return;
		}

		const Vector3f bpn_b = guidance_to_px4_body(bpn_g);
		const uint32_t bench_count = _bench_rt_count.fetch_add(1U) + 1U;

		PX4_INFO("BENCH5 #%u H tgt=%d fresh=%d rate=%d body=%d dt=%.1fms sync=%+.1fms VC=%.2f/%.2f/%.2f v=%d",
			 (unsigned)bench_count, (int)_los.target_valid, (int)los_fresh(now),
			 (int)_inertial_los_rate_valid, (int)body_rate_valid(now),
			 (double)(_last_los_recon_dt_s * 1000.f),
			 (double)(_last_body_rate_align_error_us * 1e-3f),
			 (double)_vc_raw, (double)_vc_filtered, (double)_vc_used, (int)_vc_valid);

		PX4_INFO("BENCH5 LOS G=[%+.1f %+.1f] M=[%+.2f %+.2f] R=[%+.4f %+.4f] IN=[%+.4f %+.4f %+.4f]",
			 (double)(_los.gimbal_roll * 180.f / M_PI_F), (double)(_los.gimbal_pitch * 180.f / M_PI_F),
			 (double)(_los.los_azimuth * 180.f / M_PI_F), (double)(_los.los_elevation * 180.f / M_PI_F),
			 (double)_los.los_az_rate, (double)_los.los_el_rate,
			 (double)_los_dot_inertial_body(0), (double)_los_dot_inertial_body(1), (double)_los_dot_inertial_body(2));

		PX4_INFO("BENCH5 SRC1 IMG=[%+.4f %+.4f %+.4f] GIM=[%+.4f %+.4f %+.4f]",
			 (double)_los_dot_image_body(0), (double)_los_dot_image_body(1), (double)_los_dot_image_body(2),
			 (double)_los_dot_gimbal_body(0), (double)_los_dot_gimbal_body(1), (double)_los_dot_gimbal_body(2));

		PX4_INFO("BENCH5 SRC2 BODY=[%+.4f %+.4f %+.4f] FRAW=[%+.4f %+.4f %+.4f]",
			 (double)_los_dot_body_motion(0), (double)_los_dot_body_motion(1), (double)_los_dot_body_motion(2),
			 (double)_los_dot_fused_raw_body(0), (double)_los_dot_fused_raw_body(1), (double)_los_dot_fused_raw_body(2));

		PX4_INFO("BENCH5 B20=[%+.2f %+.2f %+.2f]|%.2f Q=[%+.1f %+.1f %+.1f] Tpre=%.3f GRate=[%+.3f %+.3f] NO_PUB",
			 (double)bpn_b(0), (double)bpn_b(1), (double)bpn_b(2), (double)bpn_b.norm(),
			 (double)(e_preview.phi() * 180.f / M_PI_F), (double)(e_preview.theta() * 180.f / M_PI_F),
			 (double)(e_preview.psi() * 180.f / M_PI_F), (double)thrust_preview,
			 (double)_gimbal_az_rate_filtered, (double)_gimbal_pitch_rate_filtered);
	}



	// ============================================================
	// 最终setpoint只读预览
	//
	// pn_guidance sp once
	//   只计算、只打印，不publish，不改变GuidanceState，也不改变_last_q_sp。
	//
	// 输出链路与正式BPN使用相同几何/姿态算法；TEST候选计算使用BPN_TEST_THRUST_LIMIT（当前等于正常0.75预算）：
	//   LOS -> BPN Body -> Body/NED -> 推力预算/轴向分配 -> speed brake -> 最终预算兜底
	//       -> acceleration_to_attitude -> setpoint slew -> attitude lead -> TEST/SHADOW姿态笼。
	//
	// 为避免台架诊断改变正式状态，这里故意不调用apply_release_altitude_floor_guard()；
	// 因此ACMD_N仅代表“正常BPN加速度命令（无release高度保护附加项）”。
	// ============================================================
	bool calculate_setpoint_preview(Vector3f &bpn_body, Vector3f &accel_pre_ned, Vector3f &accel_cmd_ned,
					Quatf &q_raw, Quatf &q_slew, Quatf &q_cmd, float &thrust_norm,
					float &speed_target, float &axial_used, float &bpn_scale, bool &slew_limited, bool &lead_limited,
					float &raw_step_deg, float &lead_angle_deg, const uint64_t now) const
	{
		if (!_los.target_valid || !los_fresh(now) || !attitude_valid() || !PX4_ISFINITE(_los.gimbal_roll) || !PX4_ISFINITE(_los.gimbal_pitch)
		    || !PX4_ISFINITE(_los.los_azimuth) || !PX4_ISFINITE(_los.los_elevation)
		    || !PX4_ISFINITE(_los.los_az_rate) || !PX4_ISFINITE(_los.los_el_rate)) { return false; }

		const Vector3f los_guidance = calculate_los_body(_los.gimbal_roll, _los.gimbal_pitch, _los_miss_az_est, _los_miss_el_est);
		if (!PX4_ISFINITE(los_guidance.norm()) || los_guidance.norm() <= FLT_EPSILON || !_inertial_los_rate_valid) { return false; }
		const Vector3f los_dot_guidance = px4_body_to_guidance(_los_dot_inertial_body);

		const float los_rate_mag = los_dot_guidance.norm();
		speed_target = calculate_guidance_speed_target(los_rate_mag);
		const float guidance_speed = get_guidance_speed();
		const float axial_scale = PX4_ISFINITE(guidance_speed) ? calculate_axial_speed_scale(guidance_speed, speed_target) : 1.f;
		const float axial_limit = AXIAL_ACCEL_MAX * axial_scale;

		const Vector3f bpn_guidance = calculate_bpn_body(los_guidance, los_dot_guidance, _vc_used);
		const Vector3f los_body = guidance_to_px4_body(los_guidance);
		bpn_body = guidance_to_px4_body(bpn_guidance);

		const Dcmf R_nb{current_attitude_q()};
		Vector3f los_ned = R_nb * los_body;
		const float los_ned_norm = los_ned.norm();
		if (!PX4_ISFINITE(los_ned_norm) || los_ned_norm <= FLT_EPSILON) { return false; }
		los_ned /= los_ned_norm;

		const Vector3f bpn_ned_raw = R_nb * bpn_body;
		float predicted_thrust = 0.f;
		// SP预览属于台架诊断路径，统一按45%推力预算分配BPN和光轴轴向加速度。
		allocate_guidance_acceleration(los_ned, bpn_ned_raw, axial_limit, BPN_TEST_THRUST_LIMIT,
					       accel_pre_ned, axial_used, bpn_scale, predicted_thrust);
		float speed_brake_mag = 0.f;
		accel_pre_ned += calculate_speed_brake_accel_ned(speed_target, speed_brake_mag);

		// 台架SP预览最终再执行一次45%硬预算兜底；这里不调用会写内部状态的release高度保护。
		accel_cmd_ned = accel_pre_ned;
		constrain_net_accel_vertical_priority(accel_cmd_ned, BPN_TEST_THRUST_LIMIT);
		acceleration_to_attitude(accel_cmd_ned, _yaw_lock, q_raw, thrust_norm);

		// sp once固定按正式50Hz的下一周期计算，避免NSH触发相位影响slew结果。
		const float dt = LOOP_INTERVAL_US * 1e-6f;
		float attitude_slew_deg_s = GUIDANCE_ATTITUDE_SLEW_DEG_S;
		if (_state == GuidanceState::LAUNCH_CLEARANCE) { attitude_slew_deg_s = CLEARANCE_ATTITUDE_SLEW_DEG_S; }
		else if (_state == GuidanceState::TRANSITION) { attitude_slew_deg_s = CLEARANCE_ATTITUDE_SLEW_DEG_S + _transition_phase * (GUIDANCE_ATTITUDE_SLEW_DEG_S - CLEARANCE_ATTITUDE_SLEW_DEG_S); }

		float raw_step_rad = 0.f;
		q_slew = slerp_limited(_last_q_sp, q_raw, attitude_slew_deg_s * M_PI_F / 180.f * dt, slew_limited, raw_step_rad);
		raw_step_deg = raw_step_rad * 180.f / M_PI_F;
		float lead_angle_rad = 0.f;
		q_cmd = slerp_limited(current_attitude_q(), q_slew, GUIDANCE_ATTITUDE_LEAD_MAX_RAD, lead_limited, lead_angle_rad);
		lead_angle_deg = lead_angle_rad * 180.f / M_PI_F;

		// TEST/SP_IFACE开启时再套最后一级姿态笼：不论RAW/SLEW/LEAD要求多大，最终q_cmd都不能离test on瞬间姿态超过设定角度。
		if ((_sp_interface_test_enabled.load() || _shadow_mode_enabled.load()) && _sp_test_anchor_valid) {
			bool cage_limited = false;
			float cage_raw_angle_rad = 0.f;
			q_cmd = slerp_limited(_sp_test_anchor_q, q_cmd, BPN_TEST_ATTITUDE_MAX_RAD, cage_limited, cage_raw_angle_rad);
		}
		return PX4_ISFINITE(thrust_norm);
	}


	// ============================================================
	// 固定台架专用：只保留 LOS -> BPN -> attitude，不混入速度规划/高度保护/发射状态机。
	// ============================================================
	bool calculate_rig_attitude_target(Vector3f &bpn_raw_body, Vector3f &bpn_used_body,
					   Quatf &q_raw, Quatf &q_cmd, float &thrust_preview,
					   const uint64_t now)
	{
		if (!_los.target_valid || !los_fresh(now) || !attitude_valid() || !_inertial_los_rate_valid) {
			return false;
		}

		const Vector3f los_g = calculate_los_body(_los.gimbal_roll, _los.gimbal_pitch,
			_los_miss_az_est, _los_miss_el_est);
		const Vector3f los_dot_g = px4_body_to_guidance(_los_dot_inertial_body);
		if (los_g.norm_squared() < FLT_EPSILON) { return false; }

		const Vector3f bpn_g = calculate_bpn_body(los_g, los_dot_g, RIG_VC_TEST);
		bpn_raw_body = guidance_to_px4_body(bpn_g);
		bpn_used_body = bpn_raw_body;

		const float bpn_norm = bpn_used_body.norm();
		if (PX4_ISFINITE(bpn_norm) && bpn_norm > RIG_BPN_ACCEL_MAX) {
			bpn_used_body *= RIG_BPN_ACCEL_MAX / bpn_norm;
		}

		const Dcmf R_nb{current_attitude_q()};
		const Vector3f accel_ned = R_nb * bpn_used_body;

		// 机械台架偏航锁死：每周期使用当前估计yaw，避免锁定旧yaw导致偏航环对机械限位持续施力。
		const Eulerf e_now{current_attitude_q()};
		acceleration_to_attitude(accel_ned, e_now.psi(), q_raw, thrust_preview);

		// 台架专用慢slew。
		const float dt = LOOP_INTERVAL_US * 1e-6f;
		bool rig_slew_limited = false;
		float rig_raw_angle = 0.f;
		q_cmd = slerp_limited(_last_q_sp, q_raw, RIG_ATTITUDE_SLEW_RAD_S * dt,
				      rig_slew_limited, rig_raw_angle);

		_diag_q_slew = q_cmd;
		_diag_slew_limited = rig_slew_limited;
		_diag_lead_limited = false;
		_diag_cage_limited = false;

		// 相对台架锚点的最终姿态笼。
		if (_rig_anchor_valid) {
			bool cage_limited = false;
			float cage_raw = 0.f;
			q_cmd = slerp_limited(_rig_anchor_q, q_cmd, RIG_ATTITUDE_CAGE_RAD,
					      cage_limited, cage_raw);
			_diag_cage_limited = cage_limited;
		}
		q_cmd.normalize();
		return true;
	}

	// ============================================================
	// 【V20.6新增】Stage3真实BPN台架路径
	//
	// 目标：
	//   尽可能复用正式BPN飞行时的“命令侧”链路，只在最终发布推力时做0.25限幅：
	//   LOS -> V21惯性LOS rate -> 正式BPN(N=3, BPN_MAX=12)
	//       -> 正式轴向加速度/速度管理/0.75推力预算分配
	//       -> 【V21.1】仅在固定台架Stage3把法向BPN乘RIG_STAGE3_BPN_SCALE=0.20
	//       -> 正式acceleration_to_attitude()
	//       -> 正式BPN姿态slew(当前85deg/s)
	//       -> 正式attitude lead(当前65deg)
	//       -> 台架最终安全cage(8deg)
	//
	// 与正式run_bpn()有意保留的差异：
	//   1) 固定台架不执行release高度底线保护，避免台架Z/VZ状态污染BPN测试；
	//   2) 【V21.1】仅Stage3把法向BPN乘RIG_STAGE3_BPN_SCALE=0.20，减弱近距离台架视差形成的外环自激；
	//   3) 最终真实发布推力由run_fixed_rig_test()硬限到RIG_STAGE3_THRUST_MAX；
	//   4) 最后一层保留RIG姿态笼/实际姿态/角速度/motor abort，仅作为机械台架安全边界。
	//
	// 注意：无桨台架没有真实姿态动力学响应，因此Stage3能还原“命令与控制器输出链”，
	// 不能等价于真实飞行中的稳态motor分配；持续姿态误差会让PX4姿态/角速度环继续施加力矩。
	// ============================================================
	bool calculate_rig_stage3_realistic_target(Vector3f &bpn_raw_body,
						 Vector3f &bpn_used_body,
						 Vector3f &accel_cmd_ned,
						 Quatf &q_raw,
						 Quatf &q_cmd,
						 float &thrust_preview,
						 bool &slew_limited,
						 bool &lead_limited,
						 bool &cage_limited,
						 const uint64_t now)
	{
		if (!_los.target_valid || !los_fresh(now) || !attitude_valid() || !_inertial_los_rate_valid) {
			return false;
		}

		// 先单独保留“正式BPN原始法向加速度”，便于RIG_RT直接观察目标运动对应的BPN。
		// _vc_used在室内velocity无效时由update_closing_speed()自动回退VC_FALLBACK=10m/s，
		// 与正式BPN代码路径完全一致，而不是Stage2专用RIG_BPN_ACCEL_MAX=3m/s^2。
		const Vector3f los_g = calculate_los_body(_los.gimbal_roll, _los.gimbal_pitch,
			_los_miss_az_est, _los_miss_el_est);
		const Vector3f los_dot_g = px4_body_to_guidance(_los_dot_inertial_body);
		if (!PX4_ISFINITE(los_g.norm()) || los_g.norm_squared() <= FLT_EPSILON) {
			return false;
		}

		const Vector3f bpn_raw_g = calculate_bpn_body(los_g, los_dot_g, _vc_used);
		bpn_raw_body = guidance_to_px4_body(bpn_raw_g);

		// 直接复用正式BPN目标生成器：
		// 包含LOS/BPN、动态目标速度、轴向加速度分配、speed brake和0.75正式推力预算。
		if (!calculate_bpn_target(accel_cmd_ned, bpn_used_body)) {
			return false;
		}

		// 【V21.1修改】只在固定机械台架Stage3缩放“法向BPN分量”，不缩放轴向加速度、speed brake或重力项。
		// calculate_bpn_target()返回的accel_cmd_ned已经包含：BPN_NED + LOS轴向加速度 + speed brake。
		// 因此这里先取出原BPN在NED中的贡献，再用0.20倍BPN替换它，避免错误地把整个加速度命令一起乘0.20。
		const Dcmf R_nb_stage3{current_attitude_q()};
		const Vector3f bpn_used_ned_before_scale = R_nb_stage3 * bpn_used_body;
		bpn_used_body *= RIG_STAGE3_BPN_SCALE;
		const Vector3f bpn_used_ned_after_scale = R_nb_stage3 * bpn_used_body;
		accel_cmd_ned += bpn_used_ned_after_scale - bpn_used_ned_before_scale;

		// 【V21.1修改】让Stage3日志/ULog中的USED BPN和scale反映真正发布前实际使用的20%结果；RAW仍保持完整正式BPN。
		_last_bpn_scale *= RIG_STAGE3_BPN_SCALE;
		_last_bpn_norm_used = bpn_used_body.norm();
		_diag_bpn_used_body = bpn_used_body;

		// 与正式run_bpn()保持一致的最终GUIDANCE_THRUST_LIMIT预算兜底。
		// 固定台架刻意不调用apply_release_altitude_floor_guard()。
		constrain_net_accel_vertical_priority(accel_cmd_ned, GUIDANCE_THRUST_LIMIT);
		_last_predicted_thrust = predicted_thrust_norm_from_accel(accel_cmd_ned);

		// Stage3进入瞬间的台架yaw等价于正式飞行中的yaw_lock；
		// 机械台架偏航被锁死，因此固定使用anchor yaw，避免每周期跟随估计yaw抖动。
		const float yaw_stage3 = _rig_anchor_valid
			? Eulerf{_rig_anchor_q}.psi()
			: Eulerf{current_attitude_q()}.psi();

		acceleration_to_attitude(accel_cmd_ned, yaw_stage3, q_raw, thrust_preview);

		// 正式BPN姿态setpoint变化率：当前85deg/s。
		float dt = LOOP_INTERVAL_US * 1e-6f;
		if (_last_setpoint_pub_time > 0 && now > _last_setpoint_pub_time) {
			dt = (float)(now - _last_setpoint_pub_time) * 1e-6f;
			dt = math::constrain(dt, 0.001f, 0.10f);
		}

		float raw_step_rad = 0.f;
		const Quatf q_slew = slerp_limited(
			_last_q_sp,
			q_raw,
			GUIDANCE_ATTITUDE_SLEW_RAD_S * dt,
			slew_limited,
			raw_step_rad);

		_diag_q_slew = q_slew;

		// 正式BPN attitude lead上限：当前65deg。
		float lead_angle_rad = 0.f;
		Quatf q_real_cmd = slerp_limited(
			current_attitude_q(),
			q_slew,
			GUIDANCE_ATTITUDE_LEAD_MAX_RAD,
			lead_limited,
			lead_angle_rad);

		// 最后一层仅保留机械台架安全cage；cage内完全保持正式BPN姿态命令。
		q_cmd = q_real_cmd;
		cage_limited = false;
		if (_rig_anchor_valid) {
			float cage_raw_angle_rad = 0.f;
			q_cmd = slerp_limited(
				_rig_anchor_q,
				q_real_cmd,
				RIG_ATTITUDE_CAGE_RAD,
				cage_limited,
				cage_raw_angle_rad);
		}

		q_cmd.normalize();
		return PX4_ISFINITE(thrust_preview)
		       && PX4_ISFINITE(accel_cmd_ned(0))
		       && PX4_ISFINITE(accel_cmd_ned(1))
		       && PX4_ISFINITE(accel_cmd_ned(2));
	}

	float rig_max_motor_output() const
	{
		float max_out = 0.f;
		for (int i = 0; i < 4; ++i) {
			if (PX4_ISFINITE(_actuator_motors.control[i])) {
				max_out = fmaxf(max_out, fabsf(_actuator_motors.control[i]));
			}
		}
		return max_out;
	}

	void publish_rig_safe_zero(const uint64_t now)
	{
		if (attitude_valid()) {
			publish_direct_attitude(current_attitude_q(), 0.f, now);
		}
	}

	void run_fixed_rig_test(const uint64_t now)
	{
		const uint32_t stage = _rig_stage.load();
		if (stage == 0u) { return; }

		if (_rig_stop_requested.load()) {
			publish_rig_safe_zero(now);
			reset_rig_release_interlock();
			_rig_fault_latched = false;
			_rig_anchor_valid = false;
			_rig_stage.store(0u);
			_rig_stop_requested.store(false);
			PX4_WARN("RIG OFF completed: zero-thrust setpoint sent");
			return;
		}

		if (_rig_fault_latched) {
			publish_rig_safe_zero(now);
			return;
		}

		const bool is_armed = armed();
		if (stage == 1u && is_armed) {
			PX4_ERR("RIG STAGE1 aborted: vehicle armed");
			publish_rig_safe_zero(now);
			_rig_stage.store(0u);
			return;
		}

		// Stage2/3：ARM后仍保持0 thrust，直到真实RC AUX1完成LOW->HIGH互锁。
		if (stage >= 2u) {
			if (!is_armed) {
				reset_rig_release_interlock();
				return;
			}

			if (!rig_rc_valid(now)) {
				if (_rig_release_latched) { PX4_ERR("RIG STOP: RC invalid/lost"); }
				reset_rig_release_interlock();
				publish_rig_safe_zero(now);
				return;
			}

			const bool run_enabled = update_rig_release_interlock(now, is_armed);
			if (!run_enabled) {
				publish_rig_safe_zero(now);
				return;
			}

			if (!rig_control_mode_ready()) {
				PX4_ERR("RIG STOP: PX4 attitude/rate control flags not enabled");
				reset_rig_release_interlock();
				publish_rig_safe_zero(now);
				return;
			}
		}

		if (!_rig_anchor_valid) {
			_rig_anchor_q = current_attitude_q();
			_rig_anchor_q.normalize();
			_rig_anchor_valid = true;
			_last_q_sp = _rig_anchor_q;
		}

		// 实际姿态/角速度/上一周期电机输出越界则撤掉台架控制输出。
		const float att_from_anchor = quat_angle_rad(_rig_anchor_q, current_attitude_q());
		const float p = _angular_velocity.xyz[0];
		const float q = _angular_velocity.xyz[1];
		const float max_motor = rig_max_motor_output();
		if ((stage >= 2u && att_from_anchor > RIG_ATTITUDE_ABORT_RAD)
		    || (stage >= 2u && (fabsf(p) > RIG_BODY_RATE_ABORT_RAD_S || fabsf(q) > RIG_BODY_RATE_ABORT_RAD_S))
		    || (stage >= 2u && max_motor > RIG_MOTOR_OUTPUT_ABORT)) {
			PX4_ERR("RIG FAULT limit: att=%.1fdeg p=%+.2f q=%+.2f motor=%.2f",
				 (double)(att_from_anchor * 180.f / M_PI_F), (double)p, (double)q, (double)max_motor);
			reset_rig_release_interlock();
			_rig_fault_latched = true;
			publish_rig_safe_zero(now);
			return;
		}

		Vector3f bpn_raw_body{}, bpn_used_body{};
		Vector3f stage3_accel_cmd_ned{};
		Quatf q_raw{}, q_cmd{};
		float thrust_preview = 0.f;
		bool stage3_slew_limited = false;
		bool stage3_lead_limited = false;
		bool stage3_cage_limited = false;

		bool target_ok = false;
		if (stage == 3u) {
			// 【V21.1修改】Stage3仍复用正式BPN“命令侧”链路，但在calculate_rig_stage3_realistic_target()内部
			// 仅把法向BPN乘0.20；Stage2的3m/s^2限幅和25deg/s台架slew仍不参与Stage3。
			target_ok = calculate_rig_stage3_realistic_target(
				bpn_raw_body, bpn_used_body, stage3_accel_cmd_ned,
				q_raw, q_cmd, thrust_preview,
				stage3_slew_limited, stage3_lead_limited, stage3_cage_limited, now);
		} else {
			// Stage1/2保持V20.5原逻辑完全不变，保证前面已经验证通过的测试链不被修改。
			target_ok = calculate_rig_attitude_target(
				bpn_raw_body, bpn_used_body, q_raw, q_cmd, thrust_preview, now);
		}

		if (!target_ok) {
			publish_rig_safe_zero(now);
			return;
		}

		float thrust_out = 0.f;
		if (stage == 2u) {
			thrust_out = RIG_STAGE2_THRUST;
		} else if (stage == 3u) {
			// 【V21.1修改】Stage3法向BPN已在目标生成函数内缩放为20%；这里仍只负责最终真实集体推力上限。
			thrust_out = math::constrain(thrust_preview, 0.f, RIG_STAGE3_THRUST_MAX);
		}

		_diag_bpn_raw_body = bpn_raw_body;
		_diag_bpn_used_body = bpn_used_body;

		if (stage == 3u) {
			_diag_accel_cmd_ned = stage3_accel_cmd_ned;
			_diag_slew_limited = stage3_slew_limited;
			_diag_lead_limited = stage3_lead_limited;
			_diag_cage_limited = stage3_cage_limited;
		} else {
			_diag_accel_cmd_ned = Dcmf{current_attitude_q()} * bpn_used_body;
		}

		_diag_q_raw = q_raw;
		_diag_q_cmd = q_cmd;
		_diag_thrust_preview = thrust_preview;
		_diag_thrust_out = thrust_out;
		_diag_guidance_valid = true;

		_last_raw_q_sp = q_raw;
		publish_attitude_setpoint_internal(q_cmd, thrust_out, now);
		_last_bpn_body = bpn_used_body;

		if (now - _last_rig_print_time >= RIG_PRINT_INTERVAL_US) {
			_last_rig_print_time = now;
			const Eulerf e_raw{q_raw};
			const Eulerf e_cmd{q_cmd};
			const Eulerf e_att{current_attitude_q()};

			float min_motor = 1.f;
			float sum_motor = 0.f;
			int motor_n = 0;
			for (int i = 0; i < 4; ++i) {
				if (PX4_ISFINITE(_actuator_motors.control[i])) {
					min_motor = fminf(min_motor, _actuator_motors.control[i]);
					sum_motor += _actuator_motors.control[i];
					motor_n++;
				}
			}
			const float avg_motor = motor_n > 0 ? sum_motor / (float)motor_n : 0.f;

			PX4_INFO("RIG5 H S%u run=%d tgt=%d fresh=%d rate=%d body=%d VC=%.2f/%.2f/%.2f v=%d spdT=%.1f brake=%.2f",
				 (unsigned)stage, (int)(stage == 1u || _rig_release_latched),
				 (int)_los.target_valid, (int)los_fresh(now), (int)_inertial_los_rate_valid,
				 (int)body_rate_valid(now), (double)_vc_raw, (double)_vc_filtered,
				 (double)_vc_used, (int)_vc_valid, (double)_last_speed_target, (double)_last_speed_brake);

			PX4_INFO("RIG5 LOS G=[%+.1f %+.1f] M=[%+.2f %+.2f] R=[%+.4f %+.4f] IN=[%+.4f %+.4f %+.4f]",
				 (double)(_los.gimbal_roll * 180.f / M_PI_F), (double)(_los.gimbal_pitch * 180.f / M_PI_F),
				 (double)(_los.los_azimuth * 180.f / M_PI_F), (double)(_los.los_elevation * 180.f / M_PI_F),
				 (double)_los.los_az_rate, (double)_los.los_el_rate,
				 (double)_los_dot_inertial_body(0), (double)_los_dot_inertial_body(1), (double)_los_dot_inertial_body(2));
			PX4_INFO("RIG5 LOS2 Braw=[%+.2f %+.2f %+.2f] Buse=[%+.2f %+.2f %+.2f]",
				 (double)bpn_raw_body(0), (double)bpn_raw_body(1), (double)bpn_raw_body(2),
				 (double)bpn_used_body(0), (double)bpn_used_body(1), (double)bpn_used_body(2));

			if (stage == 3u) {
				PX4_INFO("RIG5 CMD A=[%+.2f %+.2f %+.2f] axial=%.2f/%.2f B=%.2f->%.2f s=%.2f rigS=%.2f T=%.3f->%.3f lim[s/l/c]=%d/%d/%d",
					 (double)stage3_accel_cmd_ned(0), (double)stage3_accel_cmd_ned(1), (double)stage3_accel_cmd_ned(2),
					 (double)_last_axial_used, (double)_last_axial_limit_cmd,
					 (double)_last_bpn_norm_raw, (double)_last_bpn_norm_used, (double)_last_bpn_scale, (double)RIG_STAGE3_BPN_SCALE,
					 (double)thrust_preview, (double)thrust_out,
					 (int)stage3_slew_limited, (int)stage3_lead_limited, (int)stage3_cage_limited);
			} else {
				PX4_INFO("RIG5 CMD Tpre=%.3f Tout=%.3f", (double)thrust_preview, (double)thrust_out);
			}

			PX4_INFO("RIG5 CTL rawRP=[%+.1f %+.1f] cmdRP=[%+.1f %+.1f] attRP=[%+.1f %+.1f] rateSP=[%+.2f %+.2f] rate=[%+.2f %+.2f]",
				 (double)(e_raw.phi()*180.f/M_PI_F), (double)(e_raw.theta()*180.f/M_PI_F),
				 (double)(e_cmd.phi()*180.f/M_PI_F), (double)(e_cmd.theta()*180.f/M_PI_F),
				 (double)(e_att.phi()*180.f/M_PI_F), (double)(e_att.theta()*180.f/M_PI_F),
				 (double)_vehicle_rates_setpoint.roll, (double)_vehicle_rates_setpoint.pitch,
				 (double)_angular_velocity.xyz[0], (double)_angular_velocity.xyz[1]);
			PX4_INFO("RIG5 CTL2 TQ=[%+.3f %+.3f] MOT=[%.3f %.3f %.3f %.3f] min/avg/max=%.3f/%.3f/%.3f",
				 (double)_vehicle_torque_setpoint.xyz[0], (double)_vehicle_torque_setpoint.xyz[1],
				 (double)_actuator_motors.control[0], (double)_actuator_motors.control[1],
				 (double)_actuator_motors.control[2], (double)_actuator_motors.control[3],
				 (double)min_motor, (double)avg_motor, (double)max_motor);
		}

	}

	// ============================================================
	// TEST/SP_IFACE：未解锁、零真实推力的姿态接口测试。
	// 验证BPN -> q_cmd -> vehicle_attitude_setpoint -> mc_att_control；任何已解锁状态自动退出。
	// ============================================================
	bool run_setpoint_interface_test(const uint64_t now)
	{
		if (armed()) {
			_sp_interface_test_enabled.store(false);
			_sp_test_anchor_valid = false;
			PX4_WARN("TEST auto-disabled because vehicle became armed");
			return false;
		}

		Vector3f bpn_body, accel_pre_ned, accel_cmd_ned;
		Quatf q_raw, q_slew, q_cmd;
		float thrust_preview = 0.f, speed_target = 0.f, axial_used = 0.f, bpn_scale = 1.f;
		float raw_step_deg = 0.f, lead_angle_deg = 0.f;
		bool slew_limited = false, lead_limited = false;

		if (!calculate_setpoint_preview(bpn_body, accel_pre_ned, accel_cmd_ned, q_raw, q_slew, q_cmd,
						thrust_preview, speed_target, axial_used, bpn_scale,
						slew_limited, lead_limited, raw_step_deg, lead_angle_deg, now)) {
			if (attitude_valid()) { publish_direct_attitude(current_attitude_q(), 0.f, now); }
			return true;
		}

		_diag_bpn_raw_body = bpn_body;
		_diag_bpn_used_body = bpn_body * bpn_scale;
		_diag_accel_cmd_ned = accel_cmd_ned;
		_diag_q_raw = q_raw;
		_diag_q_slew = q_slew;
		_diag_q_cmd = q_cmd;
		_diag_thrust_preview = thrust_preview;
		_diag_thrust_out = 0.f;
		_diag_slew_limited = slew_limited;
		_diag_lead_limited = lead_limited;
		_diag_cage_limited = false;
		_diag_guidance_valid = true;

		_last_raw_q_sp = q_raw;
		_slew_active = slew_limited;
		_last_raw_q_step_deg = raw_step_deg;
		_lead_active = lead_limited;
		_last_attitude_lead_deg = lead_angle_deg;

		// 核心安全约束：候选姿态真实发布，但候选推力绝不发布。
		publish_attitude_setpoint_internal(q_cmd, 0.f, now);

		// 一键TEST自动5Hz打印关键链路：直接移动目标即可观察BPN方向、RAW理想姿态和最终CMD，无需反复执行sp once。
		if (now - _last_test_print_time >= BENCH_PRINT_INTERVAL_US) {
			_last_test_print_time = now;
			const Vector3f bpn_used = bpn_body * bpn_scale;
			const Vector3f accel_body = Dcmf{current_attitude_q()}.transpose() * accel_pre_ned;
			const Eulerf e_raw{q_raw}, e_cmd{q_cmd};
			const float cage_deg = _sp_test_anchor_valid ? quat_angle_rad(_sp_test_anchor_q, q_cmd) * 180.f / M_PI_F : NAN;
			PX4_INFO("TEST_RT BPN_RAW)(BPN最原始算出来的法向加速度需求)=[%+.2f %+.2f %+.2f] USED(经过推力预算以后,BPN真正保留下来的法向加速度)=[%+.2f %+.2f %+.2f] AOPT(沿光轴方向的净加速度)=%+.2f RAW(原始角度设定值)=[%+.1f %+.1f %+.1f] CMD(真正发布的角度设定值)=[%+.1f %+.1f %+.1f] TPRE(推力估算)=%.3f cage(当前与初始时的三维夹角)=%.1f OUT_T(TEST模式实际发布推力为0)=0",
				 (double)bpn_body(0), (double)bpn_body(1), (double)bpn_body(2), (double)bpn_used(0), (double)bpn_used(1), (double)bpn_used(2),
				 (double)(-accel_body(2)), (double)(e_raw.phi()*180.f/M_PI_F), (double)(e_raw.theta()*180.f/M_PI_F), (double)(e_raw.psi()*180.f/M_PI_F),
				 (double)(e_cmd.phi()*180.f/M_PI_F), (double)(e_cmd.theta()*180.f/M_PI_F), (double)(e_cmd.psi()*180.f/M_PI_F), (double)thrust_preview, (double)cage_deg);
		}
		// publish()返回true但orb_exists仍为false时，说明消息并没有形成可见的uORB advertisement；
		// 这类情况常见于publisher rules/advertise路径问题，不能再用内部_last_q_sp判断接口已接通。
		if (_att_sp_pub_calls.load() == 25 && orb_exists(ORB_ID(vehicle_attitude_setpoint), 0) != PX4_OK) {
			PX4_ERR("TEST: publish attempted but vehicle_attitude_setpoint is still not advertised");
		}
		Vector3f zero_accel{};
		publish_guidance_acceleration(zero_accel, false, now);
		return true;
	}

	void print_setpoint_snapshot(const uint64_t now) const
	{
		Vector3f bpn_body, accel_pre_ned, accel_cmd_ned;
		Quatf q_raw, q_slew, q_cmd;
		float thrust_norm = 0.f, speed_target = 0.f, axial_used = 0.f, bpn_scale = 1.f;
		float raw_step_deg = 0.f, lead_angle_deg = 0.f;
		bool slew_limited = false, lead_limited = false;

		// 先沿用正式A方案的完整预览链路，得到BPN、轴向加速度、推力预算以及最终NED净加速度。
		// 注意：下面会仅在本函数内重新计算RAW/SLEW/CMD，使sp once从“当前真实姿态”起步；
		// 不修改_yaw_lock、_last_q_sp、GuidanceState，也不publish任何控制输出。
		if (!calculate_setpoint_preview(bpn_body, accel_pre_ned, accel_cmd_ned,
					       q_raw, q_slew, q_cmd, thrust_norm,
					       speed_target, axial_used, bpn_scale,
					       slew_limited, lead_limited, raw_step_deg, lead_angle_deg, now)) {
			PX4_INFO("SP INVALID target=%d fresh=%d att=%d",
				 (int)_los.target_valid, (int)los_fresh(now), (int)attitude_valid());
			return;
		}

		// ========================================================
		// sp once专用只读姿态预览：
		//   BASE = 当前真实vehicle_attitude
		//   yaw_preview = 当前真实yaw
		//   RAW = 当前ACMD_N通过正常acceleration_to_attitude得到的理想姿态
		//   SLEW/CMD = 假设“此刻刚开始正式BPN”，从BASE按当前状态对应的slew走一个50Hz周期
		//
		// 这样避免真机HARDWARE_OBSERVER_ONLY / WAIT_LOS时_last_q_sp仍接近单位四元数，
		// 导致原sp once把SLEW/CMD错误显示在0deg附近。
		// ========================================================
		Quatf q_preview_start = current_attitude_q();
		q_preview_start.normalize();

		const Eulerf e_base{q_preview_start};
		const float yaw_preview = e_base.psi();

		acceleration_to_attitude(accel_cmd_ned, yaw_preview, q_raw, thrust_norm);

		const float dt = LOOP_INTERVAL_US * 1e-6f;
		float attitude_slew_deg_s = GUIDANCE_ATTITUDE_SLEW_DEG_S;

		if (_state == GuidanceState::LAUNCH_CLEARANCE) {
			attitude_slew_deg_s = CLEARANCE_ATTITUDE_SLEW_DEG_S;

		} else if (_state == GuidanceState::TRANSITION) {
			attitude_slew_deg_s =
				CLEARANCE_ATTITUDE_SLEW_DEG_S
				+ _transition_phase
				* (GUIDANCE_ATTITUDE_SLEW_DEG_S - CLEARANCE_ATTITUDE_SLEW_DEG_S);
		}

		const float slew_step_max_deg = attitude_slew_deg_s * dt;
		float raw_step_rad = 0.f;

		q_slew = slerp_limited(
				 q_preview_start,
				 q_raw,
				 slew_step_max_deg * M_PI_F / 180.f,
				 slew_limited,
				 raw_step_rad);

		raw_step_deg = raw_step_rad * 180.f / M_PI_F;

		float lead_angle_rad = 0.f;

		q_cmd = slerp_limited(
				q_preview_start,
				q_slew,
				GUIDANCE_ATTITUDE_LEAD_MAX_RAD,
				lead_limited,
				lead_angle_rad);

		lead_angle_deg = lead_angle_rad * 180.f / M_PI_F;

		// 如果TEST/SHADOW本身处于开启状态，仍保留原有最后一级姿态笼，只影响本次打印候选。
		if ((_sp_interface_test_enabled.load() || _shadow_mode_enabled.load()) && _sp_test_anchor_valid) {
			bool cage_limited = false;
			float cage_raw_angle_rad = 0.f;

			q_cmd = slerp_limited(
					_sp_test_anchor_q,
					q_cmd,
					BPN_TEST_ATTITUDE_MAX_RAD,
					cage_limited,
					cage_raw_angle_rad);
		}

		const Eulerf e_raw{q_raw};
		const Eulerf e_slew{q_slew};
		const Eulerf e_cmd{q_cmd};
		const Eulerf e_pub{_last_q_sp};

		const float slew_applied_deg =
			quat_angle_rad(q_preview_start, q_slew) * 180.f / M_PI_F;

		const Vector3f bpn_used_body = bpn_body * bpn_scale;

		PX4_INFO("SP BASE=[%+.1f %+.1f %+.1f] yaw_now=%+.1f",
			 (double)(e_base.phi() * 180.f / M_PI_F),
			 (double)(e_base.theta() * 180.f / M_PI_F),
			 (double)(e_base.psi() * 180.f / M_PI_F),
			 (double)(yaw_preview * 180.f / M_PI_F));

		PX4_INFO("SP BRAW=[%+.2f %+.2f %+.2f]",
			 (double)bpn_body(0), (double)bpn_body(1), (double)bpn_body(2));

		PX4_INFO("SP BUSE=[%+.2f %+.2f %+.2f] scale=%.3f",
			 (double)bpn_used_body(0), (double)bpn_used_body(1),
			 (double)bpn_used_body(2), (double)bpn_scale);

		PX4_INFO("SP APRE=[%+.2f %+.2f %+.2f]",
			 (double)accel_pre_ned(0), (double)accel_pre_ned(1), (double)accel_pre_ned(2));

		PX4_INFO("SP ACMD=[%+.2f %+.2f %+.2f]",
			 (double)accel_cmd_ned(0), (double)accel_cmd_ned(1), (double)accel_cmd_ned(2));

		// 把NED净加速度反变换回PX4标准FRD Body。
		// ABX=Body +X；AOPT=-AB.z，AOPT>0表示沿当前Body -Z/光轴方向加速。
		const Dcmf R_nb{current_attitude_q()};
		const Vector3f accel_pre_body = R_nb.transpose() * accel_pre_ned;

		PX4_INFO("SP AB=[%+.2f %+.2f %+.2f] ABX=%+.2f AOPT=%+.2f",
			 (double)accel_pre_body(0), (double)accel_pre_body(1), (double)accel_pre_body(2),
			 (double)accel_pre_body(0), (double)(-accel_pre_body(2)));

		PX4_INFO("SP RAW=[%+.1f %+.1f %+.1f]",
			 (double)(e_raw.phi() * 180.f / M_PI_F),
			 (double)(e_raw.theta() * 180.f / M_PI_F),
			 (double)(e_raw.psi() * 180.f / M_PI_F));

		PX4_INFO("SP SLEW=[%+.1f %+.1f %+.1f]",
			 (double)(e_slew.phi() * 180.f / M_PI_F),
			 (double)(e_slew.theta() * 180.f / M_PI_F),
			 (double)(e_slew.psi() * 180.f / M_PI_F));

		PX4_INFO("SP CMD=[%+.1f %+.1f %+.1f] T=%.3f",
			 (double)(e_cmd.phi() * 180.f / M_PI_F),
			 (double)(e_cmd.theta() * 180.f / M_PI_F),
			 (double)(e_cmd.psi() * 180.f / M_PI_F),
			 (double)thrust_norm);

		PX4_INFO("SP LIM1 slew=%d raw=%.1f appl=%.2f max=%.2f rate=%.1f",
			 (int)slew_limited, (double)raw_step_deg,
			 (double)slew_applied_deg, (double)slew_step_max_deg,
			 (double)attitude_slew_deg_s);

		PX4_INFO("SP LIM2 lead=%d angle=%.1f speed=%.1f axial=%.2f",
			 (int)lead_limited, (double)lead_angle_deg,
			 (double)speed_target, (double)axial_used);

		if ((_sp_interface_test_enabled.load() || _shadow_mode_enabled.load()) && _sp_test_anchor_valid) {
			PX4_INFO("SP BENCH budget=%.2f cage=%.1f from=%.2f mode=%s",
				 (double)BPN_TEST_THRUST_LIMIT, (double)BPN_TEST_ATTITUDE_MAX_DEG,
				 (double)(quat_angle_rad(_sp_test_anchor_q, q_cmd) * 180.f / M_PI_F),
				 _shadow_mode_enabled.load() ? "SHADOW" : "TEST");
		}

		// PUB只显示模块内部当前已发布setpoint，便于与本次只读候选BASE/RAW/SLEW/CMD区分。
		PX4_INFO("SP PUB=[%+.1f %+.1f %+.1f] T=%.3f state=%u done=%d PREVIEW_ONLY",
			 (double)(e_pub.phi() * 180.f / M_PI_F),
			 (double)(e_pub.theta() * 180.f / M_PI_F),
			 (double)(e_pub.psi() * 180.f / M_PI_F),
			 (double)_last_thrust_sp,
			 (unsigned)_state, (int)_launch_sequence_complete);
	}

	// ============================================================
	// SHADOW只读观察：10Hz打印当前预览预算+cage候选姿态，不发布任何控制topic。
	// ============================================================
	void run_shadow_snapshot(const uint64_t now)
	{
		if (now - _last_shadow_print_time < BENCH_PRINT_INTERVAL_US) { return; }
		_last_shadow_print_time = now;
		Vector3f bpn_body, accel_pre_ned, accel_cmd_ned; Quatf q_raw, q_slew, q_cmd;
		float thrust_norm = 0.f, speed_target = 0.f, axial_used = 0.f, bpn_scale = 1.f, raw_step_deg = 0.f, lead_angle_deg = 0.f;
		bool slew_limited = false, lead_limited = false;
		if (!calculate_setpoint_preview(bpn_body, accel_pre_ned, accel_cmd_ned, q_raw, q_slew, q_cmd, thrust_norm, speed_target, axial_used, bpn_scale,
					       slew_limited, lead_limited, raw_step_deg, lead_angle_deg, now)) {
			PX4_INFO("SHADOW INVALID target=%d fresh=%d att=%d", (int)_los.target_valid, (int)los_fresh(now), (int)attitude_valid());
			return;
		}
		const Eulerf e_raw{q_raw}, e_cmd{q_cmd};
		PX4_INFO("SHADOW RAW=[%+.1f %+.1f %+.1f] CMD=[%+.1f %+.1f %+.1f] TPRE=%.3f cage=%.2fdeg BPN_B=[%+.2f %+.2f %+.2f] NO_PUBLISH",
			 (double)(e_raw.phi()*180.f/M_PI_F), (double)(e_raw.theta()*180.f/M_PI_F), (double)(e_raw.psi()*180.f/M_PI_F),
			 (double)(e_cmd.phi()*180.f/M_PI_F), (double)(e_cmd.theta()*180.f/M_PI_F), (double)(e_cmd.psi()*180.f/M_PI_F),
			 (double)thrust_norm, (double)(quat_angle_rad(_sp_test_anchor_q, q_cmd)*180.f/M_PI_F),
			 (double)bpn_body(0), (double)bpn_body(1), (double)bpn_body(2));
	}

	// ============================================================
	// Debug
	// ============================================================

	float attitude_error_angle_deg() const
	{
		return quat_angle_rad(_last_q_sp, current_attitude_q())
		       * 180.f
		       / M_PI_F;
	}

	void print_debug(const uint64_t now)
	{
		if (now - _last_print_time < DIAG_PRINT_INTERVAL_US) { return; }
		_last_print_time = now;

		const float ekf_speed = get_ekf_speed();
		const float guidance_speed = get_guidance_speed();
		const float imu_speed = imu_velocity_available() ? _imu_velocity_ned.norm() : NAN;
		const float imu_age_s = (_imu_velocity_anchor_time > 0 && now >= _imu_velocity_anchor_time)
			? (float)(now - _imu_velocity_anchor_time) * 1e-6f : NAN;
		const Eulerf att_euler{current_attitude_q()};
		const Eulerf sp_euler{_last_q_sp};

		float min_motor = 1.f;
		float max_motor = 0.f;
		float sum_motor = 0.f;
		int motor_n = 0;
		for (int i = 0; i < 4; ++i) {
			if (PX4_ISFINITE(_actuator_motors.control[i])) {
				min_motor = fminf(min_motor, _actuator_motors.control[i]);
				max_motor = fmaxf(max_motor, _actuator_motors.control[i]);
				sum_motor += _actuator_motors.control[i];
				motor_n++;
			}
		}
		const float avg_motor = motor_n > 0 ? sum_motor / (float)motor_n : 0.f;

		const float active_budget =
			(_state == GuidanceState::LAUNCH_CLEARANCE)
			? CLEARANCE_THRUST_LIMIT
			: (_alt_floor_active ? ALT_FLOOR_THRUST_LIMIT : GUIDANCE_THRUST_LIMIT);

		PX4_INFO("DBG5 H st=%u launch=%d tgt=%d fresh=%d rate=%d body=%d VC=%.2f/%.2f/%.2f VaLOS=%.2f VtLOS=%.2f v=%d src=%s guard=%d/%d a=%.2f diff=%.2f spd=%.2f EKF=%.2f IMU=%.2f age=%.2fs tgtSpd=%.1f",
			 (unsigned)_state, (int)_launch_sequence_complete,
			 (int)_los.target_valid, (int)los_fresh(now), (int)_inertial_los_rate_valid, (int)body_rate_valid(now),
			 (double)_vc_raw, (double)_vc_filtered, (double)_vc_used,
			 (double)_vc_aircraft_only, (double)_vc_target_projection,
			 (int)_vc_valid, guidance_velocity_source_name(),
			 (int)_ekf_velocity_guard_active, (int)_ekf_velocity_blend_active,
			 (double)_ekf_velocity_blend_alpha, (double)_last_ekf_imu_velocity_diff,
			 (double)guidance_speed, (double)ekf_speed, (double)imu_speed, (double)imu_age_s, (double)_last_speed_target);

		PX4_INFO("DBG5 LOS IN=[%+.4f %+.4f %+.4f]|%.4f CTRL=[%+.4f %+.4f %+.4f] pred=%d %.0fms/%.1fdeg B=[%+.2f %+.2f %+.2f] norm=%.2f->%.2f scale=%.2f",
			 (double)_los_dot_inertial_body(0), (double)_los_dot_inertial_body(1), (double)_los_dot_inertial_body(2),
			 (double)_los_dot_inertial_body.norm(),
			 (double)_los_dot_control_body(0), (double)_los_dot_control_body(1), (double)_los_dot_control_body(2),
			 (int)_los_prediction_valid, (double)(_los_prediction_age_us * 1e-3f), (double)_los_prediction_angle_deg,
			 (double)_last_bpn_body(0), (double)_last_bpn_body(1), (double)_last_bpn_body(2),
			 (double)_last_bpn_norm_raw, (double)_last_bpn_norm_used, (double)_last_bpn_scale);

		PX4_INFO("DBG5 CMD axial=%.2f/%.2f brake=%.2f Tpred=%.3f Tsp=%.3f budget=%.2f slew=%d lead=%d feas=%d tilt=%.1f->%.1f alt=%d drop=%.2f up=%.2f",
			 (double)_last_axial_used, (double)_last_axial_limit_cmd, (double)_last_speed_brake,
			 (double)_last_predicted_thrust, (double)_last_thrust_sp, (double)active_budget,
			 (int)_slew_active, (int)_lead_active, (int)_last_thrust_vector_feasibility_limited,
			 (double)_last_thrust_vector_raw_tilt_deg, (double)_last_thrust_vector_cmd_tilt_deg,
			 (int)_alt_floor_active, (double)_last_alt_floor_drop, (double)_last_alt_floor_up_accel);

		PX4_INFO("DBG5 CTL att=[%+.1f %+.1f] sp=[%+.1f %+.1f] err=%.1f rateSP=[%+.2f %+.2f] rate=[%+.2f %+.2f] TQ=[%+.3f %+.3f] MOTmin/avg/max=%.3f/%.3f/%.3f",
			 (double)(att_euler.phi()*180.f/M_PI_F), (double)(att_euler.theta()*180.f/M_PI_F),
			 (double)(sp_euler.phi()*180.f/M_PI_F), (double)(sp_euler.theta()*180.f/M_PI_F),
			 (double)attitude_error_angle_deg(),
			 (double)_vehicle_rates_setpoint.roll, (double)_vehicle_rates_setpoint.pitch,
			 (double)_angular_velocity.xyz[0], (double)_angular_velocity.xyz[1],
			 (double)_vehicle_torque_setpoint.xyz[0], (double)_vehicle_torque_setpoint.xyz[1],
			 (double)min_motor, (double)avg_motor, (double)max_motor);
		PX4_INFO("DBG6 GIM risk=%.2f geo=%.2f dyn=%.2f ctrl=%.2f st=%u off=%.1f rate=%.1f acc=%.0f caps A=%.2f S=%.1f L=%.1f reset_guard=%d",
			 (double)_gimbal_load, (double)_gimbal_geometry_load, (double)_gimbal_dynamic_load,
			 (double)_gimbal_control_load, (unsigned)_gimbal_load_state, (double)_gimbal_offaxis_deg,
			 (double)_gimbal_rate_mag_deg_s, (double)_gimbal_accel_mag_filtered_deg_s2,
			 (double)_gimbal_supervisor_axial_cap, (double)_gimbal_supervisor_slew_cap_deg_s,
			 (double)_gimbal_supervisor_lead_cap_deg, (int)attitude_reset_guard_active(now));
	}

	// ============================================================
	// Run
	// ============================================================

	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		const uint64_t now = hrt_absolute_time();
		begin_diag_cycle();

		// --------------------------------------------------------
		// attitude -> body rate -> LOS
		// --------------------------------------------------------
		// V19先更新vehicle_attitude，再处理本周期LOS；离散惯性LOS观测器需要使用尽可能新的Body->NED姿态。
		if (_attitude_sub.update(&_attitude)) {
			monitor_attitude_reset(now);
		}

		// body rate是V20 Body-relative observer的核心输入，同时用于健康检查和BENCH诊断。
		if (_angular_velocity_sub.update(&_angular_velocity)) { _last_body_rate_rx_time = now; push_body_rate_history(now); }
		if (_vehicle_acceleration_sub.update(&_vehicle_acceleration)) { _last_accel_rx_time = now; }

		if (SIMULATION_MODE) {
			// SITL继续使用Gazebo/Python发布的vehicle_target_los，保持原仿真链和TRUTH A/B逻辑不变。
			if (_los_sub.update(&_los)) {
				_last_los_rx_time = now;
				update_inertial_los_rate(now);
			}

		} else {
			// 真机唯一LOS输入改为CGTD070驱动反馈；不再等待仿真/外部vehicle_target_los发布器。
			if (_cgtd070_status_sub.update(&_cgtd070_status)) {
				(void)update_los_from_cgtd070(_cgtd070_status, now);
			}
		}

		// V22.06: update gimbal-load protection after the newest gimbal/LOS packet and observer update.
		update_gimbal_load_supervisor(now);

		// V20 SITL仿真真值旁路：
		// pymavlink DEBUG_VECT(name="LOS_TRUE")由mavlink_receiver转成uORB debug_vect。
		debug_vect_s dbg{};

		if (_truth_los_sub.update(&dbg)) {
			if (std::strncmp(
				    dbg.name,
				    "LOS_TRUE",
				    sizeof(dbg.name)) == 0
			    && PX4_ISFINITE(dbg.x)
			    && PX4_ISFINITE(dbg.y)
			    && PX4_ISFINITE(dbg.z)) {

				_truth_los_msg = dbg;

				_truth_los_dot_body =
					Vector3f{
						dbg.x,
						dbg.y,
						dbg.z};

				_truth_los_valid = true;
				_last_truth_los_rx_time = now;
			}
		}

		update_v19_truth_metrics(now);

		// --------------------------------------------------------
		// local position
		// --------------------------------------------------------

		const bool prev_velocity_valid = velocity_valid();
		const Vector3f prev_velocity_ned = prev_velocity_valid
			? Vector3f{_local_position.vx, _local_position.vy, _local_position.vz}
			: Vector3f{NAN, NAN, NAN};
		const uint64_t prev_lpos_rx_time = _last_local_position_rx_time;

		if (_local_position_sub.update(&_local_position)) {
			_last_local_position_rx_time = now;
			monitor_ekf_resets(now, prev_velocity_valid, prev_velocity_ned, prev_lpos_rx_time);
		}

		_vehicle_status_sub.update(&_vehicle_status);

		// V21.11：EKF有效时锚定；EKF无效时在短时间内用IMU积分速度接管速度参考。
		update_imu_velocity_estimator(now);

		// --------------------------------------------------------
		// manual control / RC RELEASE input
		// --------------------------------------------------------
		// 真机模式使用该输入；仿真模式可以完全没有RC，订阅即使没有数据也不会阻塞虚拟RELEASE逻辑。
		if (_manual_control_sub.update(&_manual_control)) { _last_manual_control_rx_time = now; }

		if (_vehicle_rates_setpoint_sub.update(&_vehicle_rates_setpoint)) { _last_rates_setpoint_rx_time = now; }
		if (_actuator_motors_sub.update(&_actuator_motors)) { _last_actuator_motors_rx_time = now; }
		if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) { _last_control_mode_rx_time = now; }

		if (_vehicle_torque_setpoint_sub.update(&_vehicle_torque_setpoint)) { _last_torque_setpoint_rx_time = now; }
		if (_vehicle_thrust_setpoint_sub.update(&_vehicle_thrust_setpoint)) { _last_thrust_setpoint_rx_time = now; }

		// 每周期根据current-time LOS + EKF/短时IMU速度源更新闭合速度。
		update_closing_speed(now);

		// BENCH_RT默认关闭；执行“pn_guidance bench on”后每200ms(5Hz)打印一次完整只读预览。
		print_los_rate_realtime(now);

		// V20实机第一阶段强制observer-only。到这里已经完成LOS/gyro/RC/EKF订阅和observer计算，
		// 随即return，绝不进入后面的WAIT_RELEASE/BPN状态机，也不发布任何姿态或推力。
		if (HARDWARE_OBSERVER_ONLY) {
			// 固定台架模式是唯一允许真机继续发布vehicle_attitude_setpoint的硬件路径；
			// 它在本分支内执行后立即return，绝不落入LAUNCH_CLEARANCE/TRANSITION/BPN自由运动状态机。
			if (_rig_stage.load() != 0u || _rig_stop_requested.load()) {
				run_fixed_rig_test(now);
				publish_pn_guidance_diag(now);
				return;
			}

			// 非台架模式保持V20.2 observer/release-preview安全边界。
			if (HARDWARE_RELEASE_PREVIEW_ONLY) {
				run_hardware_arm_release_preview(now);
			}
			run_hardware_observer_only(now);
			publish_pn_guidance_diag(now);
			return;
		}

		// SHADOW优先级最高：只做只读候选计算/打印，然后立即return；故意放在所有状态机控制发布之前。
		if (_shadow_mode_enabled.load()) {
			run_shadow_snapshot(now);
			publish_pn_guidance_diag(now);
			return;
		}

		// --------------------------------------------------------
		// 第一次有效LOS
		// WAIT_LOS -> LAUNCH
		// --------------------------------------------------------

		if (_state == GuidanceState::WAIT_LOS
		    && los_fresh(now)
		    && _los.target_valid
		    && attitude_valid()) {

			if (!_yaw_locked) {
				const Eulerf euler{current_attitude_q()};

				_yaw_lock = euler.psi();
				_yaw_locked = true;

				PX4_INFO("WAIT_LOS -> LAUNCH");
				PX4_INFO("LOCK YAW %.2f deg", (double)(_yaw_lock * 180.f / M_PI_F));
			}

			_hold_q = current_attitude_q();

			_state = GuidanceState::LAUNCH;
			_state_enter_time = now;
			_transition_phase = 1.f;
		}

		// --------------------------------------------------------
		// ARM边沿
		// --------------------------------------------------------

		const bool is_armed = armed();

		if (is_armed && !_was_armed) {
			if (_state == GuidanceState::BPN && !_launch_sequence_complete) {
				enter_wait_release(now);
			}
		}

		if (!is_armed && _was_armed) {
			reset_imu_velocity_fallback();
			_launch_sequence_complete = false;
			_transition_phase = 1.f;
			reset_rc_release_interlock();
			reset_rc_override_latch();
			reset_target_loss_grace();

			_had_positive_closing = false;
			_negative_closing_start_time = 0;

			_safe_hold_latched = false;
			_safe_low_speed_start_time = 0;
			_los_reacquire_active = false;
			_los_reacquire_start_time = 0;
			_los_reacquire_hold_logged = false;
			_last_valid_guidance_cmd_valid = false;
			_last_valid_guidance_cmd_time = 0;

			_release_alt_ref_valid = false;
			_alt_floor_active = false;

			if (los_fresh(now)
			    && _los.target_valid
			    && attitude_valid()) {

				_hold_q = current_attitude_q();
				_state = GuidanceState::BPN;

			} else {
				_state = GuidanceState::WAIT_LOS;
			}
		}

		_was_armed = is_armed;

		// --------------------------------------------------------
		// V21.4 RC绝对优先级仲裁
		// --------------------------------------------------------
		update_rc_authority(now, is_armed);

		if (_rc_override_latched) {
			// 这里故意不发布vehicle_attitude_setpoint；mc_att_control/Commander拥有最终控制权。
			_last_bpn_body.zero();
			publish_guidance_acceleration(_last_bpn_body, false, now);
			publish_pn_guidance_diag(now);
			return;
		}

		// --------------------------------------------------------
		// SP_IFACE未解锁零推力接口测试。启用后本周期由该路径唯一发布pn_guidance自己的姿态setpoint，
		// 然后直接return，避免下面BPN_READY hold setpoint在同一周期覆盖q_cmd。
		// --------------------------------------------------------
		if (_sp_interface_test_enabled.load()) {
			if (is_armed) {
				_sp_interface_test_enabled.store(false);
				_sp_test_anchor_valid = false;
				PX4_WARN("TEST auto-disabled: armed state detected");
			} else {
				run_setpoint_interface_test(now);
				publish_pn_guidance_diag(now);
				return;
			}
		}

		// --------------------------------------------------------
		// RELEASE：仿真虚拟AUX1 / 真机真实RC AUX1二选一
		// --------------------------------------------------------
		// SIMULATION_MODE=true：ARM进入WAIT_RELEASE后，内部模拟LOW；条件稳定后自动HIGH并保持100ms。
		// SIMULATION_MODE=false：严格沿用真实RC SOURCE_RC + LOW->HIGH + 100ms确认。
		const bool release_event = SIMULATION_MODE
			? update_simulated_release_interlock(now, is_armed)
			: update_rc_release_interlock(now, is_armed);

		if (release_event && _state == GuidanceState::WAIT_RELEASE && is_armed) {
			// 无论事件来自仿真还是真机，进入正式控制前都跑同一套核心安全条件。
			// 真机若拒绝，本次HIGH事件被消费，必须真实回LOW再HIGH；仿真则自动退回LOW并重新等待条件稳定。
			const char *source = SIMULATION_MODE ? "SIM RELEASE" : "RC RELEASE";
			if (!los_fresh(now) || !_los.target_valid) {
				PX4_ERR("%s refused: LOS invalid/stale", source);
			} else if (!attitude_valid()) {
				PX4_ERR("%s refused: attitude invalid", source);
			} else if (!release_altitude_reference_ready()) {
				PX4_ERR("%s refused: local Z/VZ invalid; altitude floor unavailable", source);
			} else if (!SIMULATION_MODE && rc_manual_attitude_owner_active(now)) {
				PX4_ERR("%s refused: RC Manual/Stabilized/Acro mode owns control; switch to guidance mode then AUX1 LOW->HIGH", source);
			} else if (DIRECT_BPN_AFTER_RELEASE && !control_los_rate_ready(now)) {
				// 旧流程可借CLEARANCE/TRANSITION等待observer建立；直接BPN没有这个缓冲，因此RELEASE前必须确认BPN LOS-rate已经可用。
				PX4_ERR("%s refused: direct BPN LOS-rate not ready", source);
			} else if (DIRECT_BPN_AFTER_RELEASE && !body_rate_valid(now)) {
				PX4_ERR("%s refused: direct BPN body-rate input stale", source);
			} else if (DIRECT_BPN_AFTER_RELEASE) {
				// V21.9.1：RELEASE真正接受后，以此刻真实姿态重新锁Yaw，再进入正式控制。
				relock_yaw_at_release(source);
				// V21.2方案A：RELEASE后同一控制周期直接进入完整BPN ACTIVE。
				enter_bpn_active(now);
			} else {
				// V21.9.1：v21.9正式路径同样在进入CLEARANCE前刷新Yaw；其余0.10s+0.30s过渡不变。
				relock_yaw_at_release(source);
				// Legacy fallback：保留旧CLEARANCE -> TRANSITION -> BPN流程；v21.9默认走此路径。
				enter_launch_clearance(now);
			}
		}

		// --------------------------------------------------------
		// 越靶检测
		// --------------------------------------------------------

		monitor_target_pass(now);

		// --------------------------------------------------------
		// V21.7 LOS/target失效保护：stale立即SAFE；target/control LOS短时false给250ms宽限。
		// --------------------------------------------------------
		if (formal_guidance_active()) {
			const bool control_los_ready_now =
				_los.target_valid
				&& (_state == GuidanceState::LAUNCH_CLEARANCE || control_los_rate_ready(now));

			if (!los_fresh(now)) {
				reset_target_loss_grace();
				enter_safe_hover(now, "LOS stale");

			} else if (!control_los_ready_now) {
				if (_target_loss_start_time == 0) {
					_target_loss_start_time = now;
					_target_loss_hold_logged = false;
				}

				const uint64_t loss_elapsed = now >= _target_loss_start_time
					? now - _target_loss_start_time : TARGET_LOSS_GRACE_US + 1;

				if (loss_elapsed <= TARGET_LOSS_GRACE_US) {
					if (!hold_last_guidance_during_target_loss(now)) {
						if (attitude_valid() && _last_setpoint_pub_time > 0) {
							publish_direct_attitude(_last_q_sp, _last_thrust_sp, now);
						}
						_last_bpn_body.zero();
						publish_guidance_acceleration(_last_bpn_body, false, now);
					}
					publish_pn_guidance_diag(now);
					if (!_bench_realtime_enabled.load()) { print_debug(now); }
					return;
				}

				enter_safe_hover(now, "target/control LOS invalid >250ms");

			} else if (_target_loss_start_time != 0) {
				const uint64_t paused_us = now >= _target_loss_start_time ? now - _target_loss_start_time : 0;
				if ((_state == GuidanceState::LAUNCH_CLEARANCE || _state == GuidanceState::TRANSITION)
				    && paused_us > 0 && paused_us <= TARGET_LOSS_GRACE_US + LOOP_INTERVAL_US) {
					_state_enter_time += paused_us;
				}
				PX4_INFO("TARGET REACQUIRED after %.1fms; resume guidance", (double)paused_us * 1e-3);
				reset_target_loss_grace();
			}
		}

		// --------------------------------------------------------
		// 状态机
		// --------------------------------------------------------

		switch (_state) {

		case GuidanceState::WAIT_LOS:
		{
			_last_bpn_body.zero();

			publish_guidance_acceleration(
				_last_bpn_body,
				false,
				now
			);

			break;
		}

		case GuidanceState::LAUNCH:
		{
			if (attitude_valid()) {
				// _hold_q在进入LAUNCH时已经锁定；这里只重复发布，禁止setpoint追随实际姿态。
				publish_direct_attitude(
					_hold_q,
					WAIT_RELEASE_THRUST,
					now
				);
			}

			_last_bpn_body.zero();

			publish_guidance_acceleration(
				_last_bpn_body,
				false,
				now
			);

			if (now - _state_enter_time >= LAUNCH_DURATION_US) {
				// 进入BPN_READY瞬间锁定一次姿态，READY期间保持该姿态。
				if (attitude_valid()) {
					_hold_q = current_attitude_q();
				}

				_state = GuidanceState::BPN;
				_state_enter_time = now;
				_launch_sequence_complete = false;

				PX4_INFO("LAUNCH -> BPN READY");
			}

			break;
		}

		case GuidanceState::BPN:
		{
			if (_launch_sequence_complete) {
				run_bpn(now);

			} else {
				if (attitude_valid()) {
					// BPN_READY真正保持进入READY时锁定的_hold_q，禁止每周期用实际姿态覆盖。
					publish_direct_attitude(
						_hold_q,
						WAIT_RELEASE_THRUST,
						now
					);
				}

				_last_bpn_body.zero();

				publish_guidance_acceleration(
					_last_bpn_body,
					false,
					now
				);
			}

			break;
		}

		case GuidanceState::WAIT_RELEASE:
		{
			publish_direct_attitude(
				_hold_q,
				WAIT_RELEASE_THRUST,
				now
			);

			_last_bpn_body.zero();

			publish_guidance_acceleration(
				_last_bpn_body,
				false,
				now
			);

			break;
		}

		case GuidanceState::LAUNCH_CLEARANCE:
		{
			run_launch_clearance(now);
			break;
		}

		case GuidanceState::TRANSITION:
		{
			run_transition(now);
			break;
		}

		case GuidanceState::SAFE_HOVER:
		default:
		{
			run_safe_hover(now);
			break;
		}
		}

		// V20.8统一50Hz结构化诊断；正常状态机路径在本周期所有控制计算之后发布。
		publish_pn_guidance_diag(now);

		// 连续BENCH_RT开启时抑制正式DBG5，避免两套5Hz日志互相穿插；关闭BENCH_RT后恢复DBG5。
		if (!_bench_realtime_enabled.load()) { print_debug(now); }
	}
};

extern "C"
__EXPORT
int pn_guidance_main(int argc, char *argv[])
{
	return PnGuidance::main(argc, argv);
}

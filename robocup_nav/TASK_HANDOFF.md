# 导航与原起降交接（2026-09-30）

2026-10-01本次EV联合窗口通过：resident_joint_773e2f73a2fa4f77a7a8aafe24a30d30，总54.303s；service_5fe4f3350d714de8a94033bbbcf83b33，EV1316，45全融合样本连续43.302s，服务flags maxgap1.011311s/独立1.011047s/common_gaps=[]；静止配对1309/1316 max13.970ms reset全[21,7,6,16,2]。用户同窗XRCE RX3453/TX64471/sync true，历史1.021s max未增。支持零RX ping候选不证明唯一根因。50s期限TERM正常service0/reader关闭确认，timeout124为预定截止；EV现已停，不复用旧配对。无目标/运动/参数修改。下一步推进导航目标与唯一控制入口交接，不再重复同种只读。EV自启仍待本地sudo与冷启验证。

2026-10-01机载证据：1.16.2 hash54f0455ffc，uORB flags实例0/1各1Hz，XRCE串口connected TX64151/RX0、timesync true、历史cycle max1.021363s。按精确commit+子模块711aef核对：零RX触发每秒ping并最多阻塞等pong1000ms，是共同2.01s缺口的重要候选但非因果定论。旧机载发送逐条flush/固定10ms，无较新HEAD的按topic rate_limit/flush重试；纠正参考混用。921600 8N1容量92160B/s，payload69.6%不含协议开销。下一次EV并行flags时需读取XRCE RX及周期统计，不发假数据、不先刷固件/改门槛。本轮只读未启动EV/运动，详见总记录。

2026-10-01PX4/XRCE只读核对：实际flags唯一writer BEST_EFFORT/TRANSIENT_LOCAL，GID旧会话一致；Agent17755 /dev/ttyTHS1 921600无重启，6 UDP sockets drops0。主机UDP RcvbufErrors106259历史累计不能归因，UART统计权限不足。源码5368e5f1fd约1Hz/输出best-effort，dds_topics.yaml既存dirty，机载版本尚未确认。QGC连接正常未抢USB/peer。已异步问用户ver all、uxrce_dds_client status、uorb top estimator_status_flags看10秒；等原始频率/链路/timesync证据，不放宽阈值、不发水平控制，无新EV。

2026-10-01同窗双接收器：task_joint_timing_b9d3f0942bf745e3af7ca8261943f8ee（26.269s，两进程0），关联task_readonly_87eccc4479274efb9f262aaf686a8aaf（20.413s delivery=true）。无EV/目标/控制。两个接收器在同一源戳1790816313246902→1790816315260730均缺更新，task2.013121s/独立2.011968s，源间隔2.013828s；源年龄都<9ms。本次未复现源年龄拒绝，但共同2s缺口确有证据，不把观察exit0当时效通过。下一步PX4原始发布/共享传输核验，不只责怪task地图处理；不放宽门槛，不开启运动。诊断脚本默认describe，有界显式只读；2项分析测试+6工厂测试通过。

2026-10-01A*/APF真实输入推进：task_readonly_89de85db85494bca91c26080521b72ec，14.073秒源年龄拒绝flags而退出，非先前2秒停流。同次VIO/depth/tracking239各、map39/bounds76/H79，导航来源1但无目标所以未收命令；PX4 local1094/flags11。零真实发布/EV/目标，reader关闭确认。新增core.source_rejection保存首次拒绝stamp/age/prev以区分未来与陈旧，门槛不变；不要盲目重试或称无缺口即已就绪。真实task未初始化/未发控制，下一步同会话联合时序与控制接入，详见总记录。

2026-10-01flags排查：25.005秒独立只读，flags25/maxgap1.010775s，status49/maxgap.510978，local2468/maxgap.026914，无当前停流；EV未运行所以fusion false正常。PX4本地5368e5f1fd源码约1Hz发布，DDS5Hz为上限，未证实机载同版本。联合运行2秒缺口仍未归因，不能用此次低负载成功撤销，未放宽/重试EV。继续完成实际production任务工厂离线路由检查：真实core+模拟许可/ROS，三控制无EV、LAND接外部EV、close不影响证据、绑定丢失锁定；test_external_ev_routing共6通过.385秒。非实际DDS/实飞。下一步若联合复测需同次独立flags观察，不再重复单独只读。

2026-10-01融合配对新证据：resident_ev_service_795c4fb655eb4293897b8e8b83199b36，23.915秒因flags遥测>2秒退出，EV610，20全融合样本连续18.135秒；静止源戳配对603/610、max14.248ms，reset全[19,6,5,15,2]。不能外推退出后融合或把旧会话配对用于下一次；没有动态alignment/飞行授权。reader关闭确认正常，视觉maxgap约53ms。新resident_ev_observation记录真实flags/local/发送后配对，85项离线通过。EV自启service+shell已准备静态验证，sudo需本地密码尚未安装，命令见systemd/BOOT_STATUS.md，禁止--now与手动链重复启动。下一步查flags更新缺口，不放宽阈值掩盖，冷启动未验证。

2026-10-01EV配置修复真实复测：resident_ev_service_0b33034cad6a46d0b5f8e2a7dc3d1532，约54.44秒1503EV/fault=null，reader pose/tracking各1533/maxgap64.531/64.865ms。修正worker未显式采用VIO的16MiB SHM配置遗漏；在ROS init前固定profile/domain/RMW，不改域0或阈值。支持修正有效，未作唯一根因证明。55秒外部INT导致rclpy ExternalShutdown，报告exit2且迟到回执误判未关闭；后续同session回执晚约27ms证实关闭，进程均结束。新增SignalHandlerOptions.NO和客户端退出后0.5秒有界等回执，83项离线全通过，退出修正未真实复测。无重试/解锁/模式/运动；EV已停，无融合配对证据，EV自启服务尚未安装/冷启验证。详细原因和证据在总记录顶部。

2026-10-01首次真实resident服务授权已消耗：resident_ev_service_8946dc92c5554f5585d92f7810da5f03，4.017秒exit2，Tracking stale，EV0。reader关闭确认/ROS关闭true/client0，服务closed=true，无重试/运动/参数操作。20协议包不是20帧；队列驻留max0.856ms不足归因。guard此报错意味着已有tracking后接收间隔>0.3s，不是首帧发现超时；缺分段时序，不得猜测放宽或原样重测。下一步补入口时序证据并离线验证；融合/配对/实飞未通过。

2026-10-01本轮：新增test_resident_task_entry.py四项0.057秒通过，真实CLI＋模拟运行器/许可/锁，覆盖默认不运行、独立控制锁、确认/绑定复核/消费顺序、过期不运行和旧锁保持。不代表实际ROS工厂或降落交接已验证；无硬件操作。算法沿用A*＋APF，H对准后AUTO_LAND；剩余是常驻EV真实任务分支联合验证、持续融合/同会话配对与受控退出。另需审查首次试飞许可与试后h_alignment_and_landing证据的循环依赖，禁止伪造reviewed。详见总记录顶部。

2026-10-01任务常驻EV接线：task入口--resident-status绑定后选择/tmp/robocup_task_control.lock，保留原permit/本地确认；run/precheck传binding，只允许匹配唯一EV，核对共同VIO/PX4源；runtime真实外部EV仅专用task+live permit+匹配binding，3控制发布器、不建EV。初始缺失只等待不放行，冲突立即拒绝；运行发现窗口仍原2秒。resident75/external12离线通过；原task277通过task_regression_20261001_083342（21.387秒）。尚缺新绑定分支联合验证，不声称实飞已接通/通过；无新硬件动作。

2026-10-01任务服务绑定：ResidentStatusBinding读取单一status路径，500ms年龄、session/PID/startticks/boot/time namespace、EV及5源GID固定、计数不倒退；失败锁定；生成ExternalEvEvidence但不续期伪造收到EV。service状态已补来源/进程身份。5新测试后resident72项1.840秒通过，无真实操作。尚未接task_flight_runtime/live_flight_runtime/flight_runtime：旧锁与precheck禁所有fmu输入、真实external_ev拒绝均仍在。下一步连这三处并保持许可，不直接换锁。

2026-10-01常驻服务入口：resident_ev_service.py默认describe，显式--execute-real-ev连接owner/host0节点/176管道，单循环续期与处理，每200ms状态；EV发送不冒称融合。退出先关闭EV再停reader/收3秒回执/销毁ROS/释放锁，失败明确exit2，无自动重启。4新测试后resident67项1.754秒通过；只执行describe，无真实run/systemd安装。下一步任务外部EV会话绑定/独立控制锁，旧任务不能直接与该服务并行。

2026-10-01真实EV工厂绑定：ResidentEvOwner持原shadow排他锁+同session真实域176 port；resident_ev_node显式owner可路由host0唯一/fmu/in/vehicle_visual_odometry，无运动接口，审计/发送前核对owner，落地未解锁起始保护不变。owner局部名冲突首轮7错误已修，最终resident63项1.550秒通过（假ROS真实话题名，不是实机）。尚无服务入口；旧任务同锁被排斥，必须外部EV任务/控制专用锁正式接线后共存，不可盲目换锁。无新EV/硬件操作。

2026-10-01真实只读链代码接通：worker --lease-sensor-only/domain176→real_source，manager sensor_only=True→无台架deadline的持续子进程。默认describe，显式互斥模式；没有PX4/EV/控制接口。隔离25秒路径不变。新增3离线后resident59项1.690秒通过，均假进程/ROS，无新Docker或硬件运行。下一步主端常驻EV唯一出口/会话绑定及任务接线；真实EV工厂仍隔离禁用，不声称生产服务已运行。

2026-10-01真实源适配器：resident_real_source工厂限定176，订阅原始VIO+VisualSlamStatus，保留原stamp/state转管道String，逻辑tracking GID来自raw status，不建ROS发布器。TF按已确认[.196,.025,-.05]/零角核对，q/-q等价、NaN拒绝；未核验不发送、丢失/改变锁定。5新测试后resident56项1.716秒通过，模拟ROS无真实订阅。还未接生产worker/EV服务，无CLI、真实输出仍禁用；不要把上一轮隔离CDR证据当真实源验证。

2026-10-01跨容器整链已通过：resident_container_c73e76f495b348b0b1c20b6e00d862f7，24.305秒exit0，源183→实际CDR/Docker pipe→host182常驻节点→合成飞控，182数据/60EV发布/59接收、唯一EV、commands=[]、未解锁。reader协议包213/213发送、pending0，stop后ROS关闭/回执confirmed=true、client0、source0。非真实传感器/PX4/完整航段。不要再重复该隔离测试；下一步生产来源/TF/会话绑定及任务外部EV接线，真实输出仍未开放。

2026-10-01主端管理：ContainerResidentInputs已实现惰性构造/单次隔离start、evidence新目录、200ms主循环续期、1秒卡顿拒绝恢复、非阻塞stop/poll_close和会话回执确认，无自动重启/相机Agent启动/ROS发布。8新测试用假Popen，resident总51项1.488秒通过。未实际调用Docker；真实模式仍拒绝，隔离上限25秒。下一步整套跨容器/CDR/续期/关闭验证，然后生产来源外参与会话/任务绑定，勿宣称常驻服务已经部署。

2026-10-01读取器生命周期：resident_reader_lifecycle提供主循环原子续期、1秒失联清理、boot/time namespace+session/seq核验、故障不可恢复；resident_vio_worker默认describe，仅隔离显式入口<=25秒，退出写可验证回执，清理失败不冒称关闭。新增10测试后resident43项1.473秒全通过（模拟ROS，无新子进程/设备）。下一步主端Docker启动/所有权/退出回执汇合及生产服务绑定。不要把隔离25秒入口循环当常驻；真实路由仍未启用。

2026-10-01持续发送端：新增resident_vio_sender生产者+隔离ROS双订阅工厂，复用PacketWriter与receiver图身份校验；序号/CDR保留、图每100ms、发现10秒、积压500ms故障锁定并清本地未发队列。发送→匿名OS管道→接收及工厂模拟端点7项新增通过，resident总33项1.062秒。非跨Docker/实际CDR/实机。尚无进程生命周期/CLI/真实服务绑定，真实工厂仍拒绝；下一步补该层，不循环限时EV。无硬件动作。

2026-10-01持续传输接收：新增resident_vio_transport的ResidentVioPort/Pipe，原CDR/时间不改、会话/域/序号/来源校验、500ms图新鲜度、非阻塞32KiB有界读取、EOF锁定不重连，无50秒寿命限制。实际OS匿名管道+模拟ROS节点回调测试已通过，resident套件26项0.898秒。仅接收半段，无生产者/子进程管理/真实服务；不宣称已跨Docker或实机运行。下一步容器生产者和生命周期，真实模式保护仍保留，无新硬件授权消耗。

2026-10-01常驻双域接口：resident_ev_node新增可选ScopedVioInput，VIO/tracking订阅与审计走外部输入，主端仅PX4遥测及唯一EV；只允许不同隔离域，真实输出未开放。resident_vio_input仅2话题只读，无任务/H/OpenCV依赖；首版复用ScopedPerceptionNode触发OpenCV导入错误，已通过拆除不必要依赖修正，未改系统库。节点10+核心7离线通过；尚无本版真实双域运行证据。下一步持续reader/生产服务及任务外部EV绑定，勿循环限时入口冒充常驻。

2026-10-01最新核对：实际ROS隔离resident_ev_dds_b933aaa643fb4731b07a680f88e65b9b通过4.246秒，域187同进程三节点，发送80/任务接收50/合成飞控80，唯一EV发布器、任务EV出口0、停流锁定、commands=[]。不是跨容器/真机/完整导航；不要重复此测试。真实external_ev与resident入口仍禁用，生产持续双域服务/会话绑定/任务控制分离尚未交付。下一步只补这段，不重做标定/选算法。PX4官方main Offboard文档已核对，Offboard摇杆不直接控制本身正常，撤回笼统遥控器故障推断；切出后的受控退出仍未证实。新硬件测试未执行，旧一次性授权不复用。详见总记录顶部。

2026-10-01常驻端：新增resident_ev_node仅隔离模式/域180～187，唯一测试EV发布器，地面启动/源身份/遥测时间/质量/异常关闭，publish后才确认，不发控制。5项实际工厂与回调内存联测加7核心共12通过，external12也通过；包含resident→task外部回调、只有1EV、停流锁定。端点为模拟，无实际DDS/传感器，真实模式拒绝、未装服务。下一步实际隔离ROS消息联测及真实双域绑定，勿声称已部署持续EV。

2026-10-01外部EV隔离路由：flight_runtime绑定external_ev且isolated时只建3控制发布器，订阅EV；消息合同/重置/唯一图GID核验后续期。实际Humble执行器只传msg，不支持MessageInfo，已改单参数+端点查询，非逐消息认证。真实外部模式仍拒绝。12新测试模拟端点通过；277默认任务回归072510最终通过20.843秒，无硬件。下一步常驻ROS发布端/会话绑定/隔离联合，不可称已部署持续EV。

2026-10-01外部EV核心接入：ExternalEvEvidence绑定session/gid，实际新EV续期，重复/停流/源变化不恢复。FlightRuntimeCore绑定后pose不返回EV、不以候选刷新last_ev；Live与base健康统一读取外部证据，默认未绑定保持旧路径。实际ROS路由未完成，create_node显式拒绝外部绑定，不能拿它启动实飞。7新测试单独通过；277既有任务回归task_regression_20261001_072001全通过21.764秒。无硬件/服务启用。继续完成常驻ROS与任务出口接线，勿宣称已部署。

2026-10-01架构推进：新增resident_ev_core纯逻辑层复用FlightEvAdapter，候选与实际dispatched分离，地面启动/唯一身份/停流故障锁定，不随任务落地自动停定位。7项离线测试通过，含模拟>60秒。尚无ROS常驻服务/任务外部EV接入；现有flight_runtime仍自己发布EV，绝不可并行启动第二发布者。无实机操作，详细缺项见总记录。

2026-10-01最新只读实测：ev_readonly_timing_77a302a7804d4d8ea21c1ab34b9ae54d总长24.311秒exit0，授权30秒已使用、reader关闭确认；pose622/跟踪621，pose最大源年龄39.172ms、接收间隔53.802ms，源间隔33.355ms，跟踪仅状态1。分段audit最大19.548ms、spin45.123ms、stop_file27.826ms、pipe8.050ms，未复现361ms故障；不代表EV联合负载下稳定或融合配对通过。reader送1244/host收1243为结束边界，勿说完整无丢。无EV/额外图像/控制/重启/自动重试。应比较EV联合负载差异，不再原样只读循环。

2026-10-01续办：原disarmed_ev_observe已接分段耗时/延迟关联reader_delay_analysis；缺回执不报正常，重叠不当因果。修正非stereo模式缺pose/tracking也可能complete的问题。8项纯离线通过，未硬件执行。已请求新的<=30秒仅VIO/状态只读诊断，等明确答复；不新增图像负载、不注入EV、不重启/控制/重试。

2026-10-01自动续办：reader增加有界分段耗时证据（stop_file/graph/TF/audit/spin/pump），退出receipt保存；每段count/max、最近64慢调用>=20ms，嵌套不可求和。3新+66既有纯离线通过，无真实ROS采集/EV/重试。只能说补足诊断，未确认/修复361ms延迟根因。下一步需新授权有界仅订阅观察其分段记录，勿原样EV循环。

2026-10-01最新授权已消耗：disarmed_ev_a223eeacba6e4649930f64800f62c7ef总长5.702秒，error=null但VIO陈旧365.355ms退出，EV sent/output均0、配对0、4融合样本未全融合。reader内已陈旧360.335ms，管道约3.454ms；末次pose接收间隔361.156ms、源戳间隔33.339ms。双端ROS/system与单调补偿时基差很小（末包约0.004ms），本次不是跨端固定时钟差或nanoseconds异常。reader关闭确认且无残留，无自动重试/运动。下一步定位图像/VIO/读取器调度排队，勿放宽阈值；尚无真实配对/实飞通过。

2026-10-01最新：两端时钟snapshot已加入reader/host。新授权60秒EV配对disarmed_ev_2a3fe59211ab4427b439359551cf3130在4.893秒因dispatch误调用整数.nanoseconds()退出；首候选发布前异常，61trace/1候选、配对1尝试0匹配，原异常路径sent/packets缺失；reader关闭确认，无自动重试或运动。已修为.nanoseconds，finally保留sent/packets/paired/output_count，新增实际dispatch分支回归；66EV+2时钟离线通过。61trace年龄18.434～61.796ms，仅短窗、非两端完整对照。真实配对未通过、常驻EV/控制拆分未完成。不得再次无授权硬件重试。

2026-10-01最新时序观测：新授权20秒已完成并消耗，source_clock_2d542f5db7ea4091bcf6e20309eb94fc总长17.296秒exit0、无残留、无EV/控制/重启。容器内同节点infra1/infra2各458，VIO440；源年龄中位12.424/14.396/19.254ms，最大62.746/64.655/70.116ms，无未来>50ms；440/440 VIO与两路红外源戳精确匹配。历史host EV未来戳1.955秒本窗口未复现，不是已修复证明，亦非host/PX4配对验收。需要区分跨进程时钟与瞬态，不能因此直接飞；无自动重复试验。

2026-10-01时间证据补充：用户timesyncd日志最后同步06:01:40但无校正量；ps相机6778显示启动06:01:54（墙钟推算，非独立时间轨迹）。本轮<=12秒只读在线参数实得depth_module.global_time_enabled=true/use_sim_time=false，不能认定当前使用本地源码HARDWARE_CLOCK固定映射。无参数变更/重启/EV。未来戳1.955秒根因仍待同窗口红外与VIO源戳对照；不要按猜测偏移改时间。

2026-10-01自启注册确认：用户已sudo enable sensor/DDs两个boot unit；本轮show确认均enabled但inactive/dead，未启动或重启，现有手工链另算。BOOT_STATUS已更正，不能继续说未安装，也不能称冷启动成功。VIO未来时间戳1.955秒未解决；本地驱动HARDWARE_CLOCK映射仅为候选机制，需运行域/校时证据，sudo日志仍需本地密码。本轮无新硬件尝试、无EV/运动；常驻EV与任务出口拆分仍待实现。

2026-10-01最新复测/自启：新授权60秒已消耗，disarmed_ev_040307ef88be4794933f74389490774f在4.085秒退出；五个PX4来源唯一ready，local275/status5/flags3/land3/RC125，发现交接故障未复现。首VIO pose源时间超前1.955秒（非陈旧积压，容器→主机约5.3ms），触发时间保护；EV0/配对0，reader关闭确认，无重试。主机已报NTP同步，校时日志需管理员访问；不能减猜测偏移/放宽阈值。用户确认每次开机地面静止原点，授权唯一常驻EV+任务控制分离、故障不自动恢复；架构尚未实现。仅准备systemd/两份sensor与DDS自启候选及入口，5测试、shell语法与systemd验证通过；未安装/启用，现有运行链不变。sudo需密码/Linger=no，用户可本地执行安装；无--now。详见systemd/BOOT_STATUS.md。常驻EV、任务出口拆分、源时钟修复及实飞仍未完成。

2026-10-01最新软件修正：disarmed_ev_session用同一休眠Observer做预发现，ready后单次activate创建订阅/EV发布器，不再销毁probe并新建采集节点。保留原时限与所有数据/状态门控；6项新生命周期正反对照加入EV离线集，最终65项2.549秒全部通过，diff --check通过。首轮新测试使用3.11 enterContext与本机3.10不兼容，改ExitStack后通过，未弱化断言。仅软件/假DDS图验证，无实际ROS节点/EV/运动/硬件重试；真实发现恢复及同步配对仍须下一次新授权有界测试，不宣称根因已获实机确认。

2026-10-01最新：新授权<=60秒EV＋同步配对已执行一次，disarmed_ev_9856b8fb747e4badbf82a9aea143ccd7总长3.335秒失败；预发现ready但Observer五个PX4来源均0，2.084秒触发discovery_timeout。EV0/配对尝试0/融合样本0；reader exit0、关闭confirmed。无运动/参数/重启、无重试。会话入口仅增加复用closest_local的同步配对记录及五项reset，27项针对性离线通过。下一步修复预发现→实际Observer发现交接并隔离验证，不能把本次称为配对或融合精度失败；单次授权已使用。

2026-10-01后续更正：用户报告摇杆无法操纵，但能切模式/触发急停，不能继续假设人工接管已验证。配对matched=false还可能由XY/Z无效、未落地、reset/年龄等造成，不是独立时钟故障证据；06:24报告未记录各拒绝原因。EV已按06:19限时契约停止，随后XY失效仅为合理待证假设。正确配对需在持续EV、PX4位置有效的同会话未解锁窗口中完成。LiveFlightCore WAIT允许在任务对齐前按健康条件发送EV，无必然启动循环。此轮只读诊断并更新文档，无硬件测试或控制输出；先核对QGC四轴响应及实际模式，不通过真实起飞诊断摇杆故障。

06:24：修正正式task host遗留旧同进程双域设置，px4_runtime_transport统一域0/localhost0/fastdds_bridge；感知管道域176不变。277项离线通过062222，真实单次只读task_readonly_fabe85e2109f43d6b93dfc7de3a19ac4收齐PX4/VIO，20.477秒，无输出/未解锁。但当前50ms VIO/PX4配对matched=false，H误差/地图制动审核等未完成，不能立即发2.5m自主航段；勿改false来冒充通过。先明确现场受控动态/悬停观测方案，不重复静止EV。

06:19：新会话EV-only disarmed_ev_f62beb4a08944ec88a2563320d6125a8通过，总长49.780秒、1337EV、fault=null；46个位置/航向/高度融合样本覆盖44.308秒，末尾2000 local均xy/z有效且三项reset[15,4,3]稳定。仅静止融合，不是动态/飞行。EV和reader已停止/确认exit0，Agent17755保留；授权已消耗，勿后台续发或重复静止EV。下一步真实任务/H/放行剩余证据与单航程持续输出，而非另一次相同EV短测。

06:16：当前Agent PID17755已存在并保留，域0/localhost0/fastdds_bridge.xml。单次15秒只读preflight_readonly_20261001_061559收齐PX4位置/遥控/状态/估计器，未解锁、QGC与RC正常、无source fault。无EV发布者，EV融合/水平有效/preflight仍false；下一步新鲜配对EV-only恢复需授权，不再把DDS当缺失。reset counters[14,3,2,13,2]，旧对齐不可沿用。本轮未EV注入/解锁/参数变更。

05:54最新：完整双域task_full_pipe_20261001_055232已通过，63.494秒；3秒HOVER→NAVIGATE去返→ALIGN_H→原LAND→DONE，最高中心AGL1m、最远2.423433m、H对准后2.650cm模拟残差，2278/2278请求确认、pending0、源/reader退出0且关闭确认。此次授权已消耗，不重复。仍是理想导航/自由地图/同模型H/理想降落，与此前真实A*/APF单独证据不可拼成实飞证明。离线主线收尾，下一步受控实机输入/融合/地图与真实H落点条件确认；生产许可不变。

05:51：单次完整双域task_full_pipe_20261001_055011在21.107秒失败；已走到3秒HOVER后NAVIGATE，新增夹具把numpy.float32速度赋给PX4标量vx时报类型错误，未往返/降落。已转Python float并增加实际反馈消息赋值回归，275项全通过055059。源/reader退出0、关闭确认且无残留；此次授权已消耗、未重试。待新一次<=180秒完整双域纯合成复测，不借用旧授权。

05:49：完整入口tests/task_full_pipe_check.py已实现、尚未运行，默认只描述；--run-isolated才启动host182/container183，实际管道＋RuntimeNode＋混合合成飞控＋随位置更新的合成H，导航明确为理想趋近/自由地图，不冒充A*/APF。新增专用隔离150秒预算，不放宽普通25秒/真实许可限制。275项离线全通过task_regression_20261001_054830。下一步待新一次<=180秒纯合成运行授权，准备好后一次执行，不自动重试；无需再拆阶段做静止检查。

05:42：tests/task_mixed_plant.py新增MixedXY及懒加载MixedTaskPlant(domain182)，支持实际XY速度/Z位置消息及逆变换VIO反馈，原垂直夹具未动。269项纯离线通过task_regression_20261001_054131；没有ROS整链运行。尚缺动态容器输入（位置/地图/H）与专用隔离长时限入口，普通只读25秒不可直接放宽。该模型理想无惯性，不能证明PX4实机响应。

05:39补充：74.905秒通过报告使用旧TaskRig的1秒悬停，不是生产3秒完整时序。两项任务回放已显式hover=3，264项纯离线通过task_regression_20261001_053848，未重跑实际导航。旧双域runtime_pipe测试仅WAIT/READY，SyntheticPlant仅垂直，不能直接当全航程；下一项需独立混合XY速度/Z位置合成飞控与随位置更新的容器输入，走实际运行器唯一出口。已有算法通过证据保留，未实飞。

最新单次实际算法合成测试已通过：task_actual_navigation_20260930_213403，总长74.905秒，模拟60.3秒，A*＋APF双箱1.2m通道前进/返回→合成H对准→理想原LAND→DONE。1312输出/245路径，最高中心AGL1.00m、最远2.422227m（2.50m目标/8cm阈值）、最终XY残差2.710cm、contact=false。主要直穿对称通道而非外绕；非SITL/真机/实际H识别/新host管道整航程。此次授权已消耗、进程已退出、未重试。实飞许可仍false。下一步关注实际任务输入/唯一控制出口整段交接与H实机误差，详见总记录最新节。

最新：当前目标排程以“避障→H区降落→窄门→图像识别/投递”为准。导航合成复测仍待新授权；只增加子进程死亡快速报告、本机隔离前置检查、异常passed=false及策略头哈希。纯离线task_regression_20261001_053305为263项全通过，20.161秒；未重跑实际导航节点。

修复后最终离线结果：task_regression_20261001_053112，261项全部通过、21.251秒；隔离APF二进制编译成功，尚未再次启动实际导航节点。

05:33最新：一次<=180秒实际A*/APF整段纯合成授权已消耗，task_actual_navigation_20260930_212447在35.654秒启动超时，WAIT、路径/命令0、无运动。APF main遗漏domain186白名单导致exit64，已与构造函数统一为domain_policy.hpp；域186仍需显式isolated_task_test，保留严格输入检查。仅重建专用build/task_nav_isolated；新增纯离线策略回归，launch缺参默认值已修复。没有自动重试；再次实际导航合成测试需新授权，不要把旧授权重复使用。详见总记录05:33节。

05:20最新：当前配置整段理想回放task_configured_replay_20261001_051855通过，实际TaskFlightSupervisor/消息合约+旋转场地+合成H投影，0/45/90°三例均返回H对准后原LAND→DONE；模拟52.75秒，目标2.5m、8cm到达阈值使实际最远2.421m，非精确2.500m。仅理想目标趋近速度/自由地图/同模型H，未跑A*/APF、容器完整航程或实机，不是避障验收。259项离线通过051916。旧单程持续视觉下降入口不作为本次流程证据；下一步须当前任务实际导航链隔离接入，不重复此理想回放或静止EV。详见总记录05:20节。

05:16最新：前方已确认6.20m，四侧为前6.20/后1.80/左右2.50m，不再待答；任务前进仍2.50m。已实现并接通起飞点/航向绑定的精确旋转矩形，扣0.48m后中心范围前5.72/后1.32/左右2.02m，制动余量另检；不取旋转外包矩形。生产bounds_odom=null现在表示使用完整现场测量动态解析，不再是尺寸缺失。258项离线通过051603，无实机输出/许可变化，H误差和完整航程仍未验收。详见总记录05:16节。

场地最新补充：用户确认起点PX4中心到后边界1.80m、左/右各2.50m；已存roundtrip_task.yaml.site_measurement，前方仍null待答，不推定6.2m。bounds_odom仍null，测量尚未转换为可飞边界。注意起飞航向与odom轴可能不同，禁止用旋转矩形的外包矩形扩大可飞区域；精确旋转边界/包络处理尚待实现。无硬件动作。

05:12最新：已将下视偏移[.12,0,-.055]接入roundtrip_task配置，去掉投影隐藏常量；名义零倾角方向矩阵已填但axes_reviewed仍false，不是精确外参通过。黑环0.60m/电机对角0.40m且同心→0.10m名义余量，任务/许可统一按配置计算；总误差预算仍null。253项离线通过051117，无新硬件动作。已询问起点到场地前后左右四距以确定边界，待回答；不假定起点居中，不重复棋盘格。详情总记录05:12节。

05:08最新：正式task入口已切ContainerTaskInputs，不再旧PerceptionDomain；真实导航出口仅持已消费TaskFlightPermit可用，reader校验共享会话/哈希/截止/消费记录。只读25秒不变，许可绑定单航程输入200秒，原175秒运行窗口不变。未开启实飞或执行真实模式。发现启动构造导致来源记录过期，保留失败050634，已修为启动丢旧数据、激活前刷新、运行期时效不放宽；隔离050740通过（17.402秒，117测试EV、15/15请求、中断后无新输出且PX4合成监听继续）。最终244项离线通过050759。尚未验证200秒稳定性/完整航程；H正式投影、航段及落点预算仍待落实。详见总记录05:08节。

05:02最新：用户已同意最小必要范围继续。输入异常隔离已落地：prepare/pump失败锁存中止、清旧目标、停止新EV/导航，不销毁PX4监听；管道quarantine不等待子进程且不重放。235项离线通过（050150）；实际运行器隔离域182/183中断测试050109通过，17.444秒，READY后中断，PX4合成状态/位置各再收100条、EV/导航新增0，未解锁、无运动命令、读取器退出0。不是空中实测。正式入口仍旧PerceptionDomain、真实导航写入仍禁止、25秒限制仍在，下一步接正式许可约束下的单航程链。详见总记录05:02节，勿将该阶段称为可实飞。

04:55范围调整：用户要求直接推进避障往返/H降落，后置中断交接和持续运行扩展。已停止扩大耐久/复杂恢复/其它任务范围；但本次单航程持续输入与基本失效接管不能跳过。只读确认正式入口仍走旧PerceptionDomain，新管道仍只允许隔离域导航写入且25秒截止；正式H变换/航段边界/误差预算仍为空。未改代码/飞控参数/实飞许可，未新增硬件测试或解锁。最短必要工作和暂停直接起飞的依据见总记录04:55节；不要把用户赶时间或历史READY当作缺失配置已通过，也不要重复已完成真实只读输入测试。

04:50最新：实际任务运行器+新容器管道隔离接入通过。task_runtime_pipe_20261001_044749实际exercise=true进入READY等待、164条EV仅测试域、20/20导航/状态回执；合成Position始终未解锁，无运动命令。启动阶段显式丢弃旧包，完整非空回调+新鲜来源才激活。230项离线通过（044924）。未实飞，真实域写入仍拒绝；下一步故障/停止交接及生产生命周期整合，不能直接沿用25秒读取器执行175秒飞行。详见总记录04:50节。

04:43最新：导航目标/状态私有返回通道已实现并仅隔离域183开放（真实域强制拒绝）。task_pipe_20261001_044111确认目标/状态各1次、容器独立订阅者收到、99条正确目标ID合成回应；原只读回归044203也通过且零导航写入。227项离线通过。下一步为实际任务运行器的启动/停止及生产接线整合，不能把传输测试当实飞控制验证；生产入口仍未切换，未发真实目标，详见总记录04:43节。

04:37最新：TaskFlightCore新增有界本地位姿历史，TaskSceneInput初始化按源消息timestamp匹配≤50ms的同重置、新鲜且地面有效样本，真实使用其位置/航向。222项离线通过（task_regression_20261001_043653），未真实复测、不放宽阈值、不改review配置。不能把原只读pairing失败认定已实测解决；新initial_pose_pairing报告供后续交接验证。下一项是容器导航目标返回通道及生产任务接入，而非重复输入只读或立即起飞。

04:33最新：用户新30秒真实只读复测已完成，20.4905秒，task_readonly_9a6a2c3cbba54ade955b24bb1046e7b0，input_delivery_observed=true。真实定位/跟踪各288、深度298、地图78、H候选146及PX4收到；无EV/目标/控制，读取器退出确认，许可已消耗。用户要求通过后避障实飞，不能遗忘；但生产任务仍直接DDS未改、新只读端口不能发导航目标，需先软件交接与配置核验，flight_ready=false。详细时效/证据边界和剩余项见总记录04:33节，不重复已经通过的只读输入验收。

04:29最新：容器侧限定任务输入读取+宿主私有管道候选已实现，task_readonly_session用显式--container-inputs选择，仍无EV/目标/控制输出；生产飞行入口未切换。217项离线通过，task_pipe_20261001_042726隔离端到端七类各105条、读取器正常关闭，详细时效限制见总记录04:29节。下一步申请新一次总30秒真实只读复测，不重启设备、不注入EV、不自动重试；此前30秒许可已消耗。候选不可被当作已实飞接通。

04:23最新：隔离域181跨容器合成图像已复现发现成功却三路0/150；仅接收端UDP仍0，同容器两进程则三路150/150。未触及真实话题或改变运行感知链。209项离线回归通过（task_regression_20261001_042154）。下一步实现容器侧限定任务感知读取＋宿主私有管道接入，保留源戳/来源/退出监督；不能只搬VIO，地图/深度/H/导航均须接通。尚未实现此接入或申请新真实复验；详细证据及两次夹具启动失败见总记录04:23节。

04:17最新：新授权一次30秒真实任务只读检查已执行，20.113秒正常退出（task_readonly_a409cce6e9574c1e9183c2a5f03741a9），PX4输入收到、感知域定位/深度/地图均未收到，input_delivery_observed=false；不是接线通过。无EV/目标/控制，两个上下文已关闭，无进程残留；许可已消耗。原容器感知进程仍在，host网络/private IPC；需先离线诊断跨容器接入，不能将同进程合成双域通过外推。207项离线回归通过，详细证据见总记录04:17节。

04:14最新：专用task_readonly_session入口与真实运行器的只读分支已软件收尾；不创建EV/控制/导航目标/任务状态业务发布器，不执行状态机，ROS自动元数据不包含在该“零输出”声明内。修复缺失跟踪戳及异常成功状态，205项离线回归通过（task_regression_20261001_041237，hardware=false）。尚未执行真实只读观察，下一步需新一次最多30秒仅订阅授权；不恢复EV、不重启感知、不自动重试。生产实飞配置仍未放行。详情见总记录04:14节。

当前：代码接线与隔离DDS通过；真实传感器/飞控混合目标联调、实飞未验收。默认不允许飞行。

04:02最新：新许可EV-only已成功执行（`disarmed_ev_e553bed1a8fc41bd843d30b73b780085`），49.734秒、1341条EV、46个融合有效样本覆盖44.316939秒，位置/航向/高度均开启；无fault。末尾2000条本地位置xy/z有效、重置计数稳定，仅证明静止窗口，不是动态对齐/飞行。EV与容器读取器已停止，退出确认0.282秒，Agent21331/QGC1870/感知保留；不要后台续发测试或重复重启。下一步专用任务真实输入/控制交接、航段制动与覆盖及H落点条件；生产配置未review项与实飞许可false保持不变。详细指标/限制见总记录04:02节。下方EV=0失败为上一轮历史，不代表本次。

03:58最新：用户已批准并执行单次60秒DDS恢复＋EV-only，3.499秒因冷启动话题创建晚于2秒发现窗口退出，EV=0，许可已消耗。Agent21331和QGC1870/感知保留，读取器已确认退出；没有新会话融合证据。入口已加共享总预算内的无输出图就绪阶段（最迟原调用第10秒），之后原EV门控和50/55秒期限不变；128项离线回归通过，未再次硬件尝试。详见总记录03:58节。下方“许可仍待确认/Agent不存在”均属于旧阶段；现在等待的是新一次EV-only许可。

03:53补充：EV专用入口新增逐次有效融合遥测及纯离线采样区间统计；123项无硬件回归通过（`disarmed_ev_regression_20261001_035215`），原输出/时效合同不变。未启动Agent或真实EV，上一条新60秒EV-only许可仍待确认，自动目标继续不代替该确认。感知恢复状态见下节。

## 2026-10-01 03:49 最新会话：感知已恢复并短窗通过，PX4未接入

重启后用户授权一次90秒全部必要传感器/地图/导航候选恢复，47.171秒成功，已保留链，许可已消耗且没有重试。域176运行D435i视觉-only、深度、下视/H候选、RPLIDAR/TF、nvblox、A*＋路径引导APF；无目标发布者、无/fmu、无Agent或飞行运行器，QGC桥1870保留。不要重复启动这些节点，不要直接用下一节的旧停止状态。

初始地面z=-0.004792642313987017m，地图中心z=0.855207357686013m；外参XYZ(0.196,0.025,-0.05)、零角。最终12秒定位最大接收间隔43.74ms、地图222.15ms，来源/内参/高度带符合候选验收。证据`perception_reboot_20261001_034552/session.json`及`perception_transport_rollout_20260930_194555/report.json`。详细表格、当前PID和未解决告警见总记录03:49节。

实际航线、PX4融合、控制交接和H对准降落尚未通过；当前实飞许可false。下一步单独确认新会话EV-only与Agent恢复，不重新要求地面看完整H。新外层恢复入口默认仅描述，要求初始容器exited；不能在当前运行中的容器再次执行。停止/失败分支测试不代表重新取得硬件许可。

最新无硬件回归`task_regression_20261001_034824`，198项全通过，源码SHA已留存；首次缺nvblox_msgs测试路径的导入错误保留在`...034744`。复跑仅给测试进程指定已有host_task_build消息库路径，无安装或系统环境改动。

## 2026-10-01 03:32 当前状态与双域接入（优先于下方旧会话记录）

真实D435i/VIO、nvblox、task_navigation已在03:03失败退出时停止；本轮仅软件与隔离测试，没有恢复它们。下方00:13/23:54的PID、地面z与“运行中”描述属于旧会话，不能当作当前地图或对齐使用。

用户批准保留域176感知和域0PX4的限定双域软件接入。专用任务运行器现在以独立Context处理两域，主线程顺序调用；不转发整域或向域0复制图像/地图。感知侧只发布导航目标和任务状态，PX4实际输出仍在原唯一出口及TaskFlightPermit后。域布局如下：

| 部分 | 实际候选域 | 本轮测试域 |
|---|---:|---:|
| VIO/跟踪、深度、地图/高度边界、H、导航候选、任务导航目标 | 176 | 183 |
| PX4遥测、唯一EV/运动输出 | 0 | 182（仅隔离输出话题，无/fmu） |

`task_domain_io.py`为限定端口与Context；`flight_runtime.py`按输入侧查来源；`live_flight_runtime.py`仅task模式接入双域并使用本进程的16MiB传输候选。非任务入口保留单域默认。`task_navigation.launch.py`启用`nvblox_guard relay_tracking:=true`，提供原运行器需要的`/robocup/alignment/tracking`，保留cuVSLAM原时间戳/状态，不再为该接口启动第二套相机。此relay默认false，避免改变其他旧启动链。

本轮实际DDS验证是合成WAIT场景，确认收数、目标路由、故障锁存，不是完整往返或H降落验证；见总记录03:32节。191项回归通过。生产`roundtrip_task.yaml`未核的旋转/误差预算/边界保持缺失，实飞许可仍关闭，旧代码摘要对应许可不可直接复用。

只运行隔离接线测试（默认不执行；显式选项也只允许测试域182/183）：

```bash
env PYTHONPATH=/home/cfly/ros2_ws/robocup_nav/host_task_build/nvblox_msgs/rosidl_generator_py:$PYTHONPATH \
    LD_LIBRARY_PATH=/home/cfly/ros2_ws/robocup_nav/host_task_build/nvblox_msgs:$LD_LIBRARY_PATH \
    python3 /home/cfly/ros2_ws/robocup_nav/tests/task_dual_domain_check.py --run-isolated
```

当前宿主nvblox消息类型动态库在上述已存在路径；仅有Python包路径不足以创建DDS订阅。不要将隔离脚本的固定域/话题改为真实PX4。真实感知加载、双域接入、EV与飞行需分别按当前许可执行，本次未进行。

2026-10-01 00:13更新（替代下文旧会话运行状态）：已授权重启定位/地图/导航候选，域176仍仅真实感知，禁止同域合成测试。实际D435i TF为XYZ(0.196,0.025,-0.05)、单位四元数。新地面z=-0.013416571542620659，地图中心0.8465834284573793。证据`perception_recovery_20260930_161250`：12秒内VIO最大间隔169ms、地图217ms，地图高度一致；无/fmu、无导航目标，flight_ready=false。运行PID与工具会话见总记录顶部。新会话未接PX4；不要直接运行要求拆桨的flight_bench_check或将下视旋转缺失批量设为已核验。

23:54实际状态：Docker内感知、雷达、nvblox和task_navigation候选已在域176运行，无/fmu输入；不要重复启动或在同域发布合成传感器。`task_navigation.launch.py`已移除APF单独强制域0的设置，全部继承调用域；当前APF还需已有`/workspaces/ros2_ws/build/uav_task/ament_cmake_index`加入AMENT_PREFIX_PATH，以满足旧加载器在显式配置前的包查找。实际planner active、A*=true、unknown=false、tolerance=.05；地图高度带已核对。158项单元回归通过。被动观察仍见VIO约376ms、地图约528ms间隔，来源未定，不等于飞行时效通过；实际H米制/飞控水平交接仍未验收。当前无导航目标发布者，不会自动执行2.5m航线。进程/PID/证据见总记录顶部。

23:22更新：起飞条件与在线地图/H阶段已分离。`task_scene_input`不再要求地图/深度已出流才能初始化地面参考或维持人工核过的起飞柱；这些仍是水平导航条件，未知不清空。原悬停保持XYZ位置最多额外等待10秒，HOVER期发布规划目标，当前航段/实际地图包络及制动空间通过才交接NAVIGATE；等待失败走普通LAND，不宣称H降落。H只在返回后的对准阶段必需。配置审核/许可未自动填写，真实控制未运行。

新增复验：`python3 tests/task_handoff_dds_check.py --online-bootstrap`，域177理想DDS夹具在空中HOVER开始等待1秒后才提供地图/深度/H；检验地面无需完整H或先验地图、地图前无水平输出、往返/H/原LAND完整链。它不运行真实GPU建图/规划或飞控。最新单元回归157项见`evidence/task_regression_20260930_232045/report.json`；DDS结果以总记录最新节为准。

最终136项回归通过：`evidence/task_regression_20260930_222515/report.json`。
完整运行器理想DDS链通过：`evidence/task_handoff_dds_20260930_222022/report.json`。
实际APF与动作桥隔离测试通过：`evidence/task_apf_link_20260930_222217/report.json`；
此项路径服务为模拟直线，不是实际A*。这些测试均不是真实飞行验收。

## 唯一控制链

```
VIO + PX4遥测 + 深度/nvblox + 下视H候选
                  ↓ task_scene_input
原READY/AUX/ACK → 原竖直起飞/悬停 → TaskFlightSupervisor
                                      ↓ 目标/航段ID
                      Nav2 A* → 路径关联 → 旧APF路径引导
                                      ↓ 带源时间的水平候选
                     地图制动/H对准/坐标变换/独立输出检查
                                      ↓
                      原运行器的唯一PX4输出 → 原AUTO_LAND
```

PX4中心离地1m，相对起点爬升0.86m；沿捕获航向前进2.5m、返回、H对准再LAND。
保持原READY、遥控边沿、ACK、EV/遥测健康、起飞0.3m横移保护。
导航后不再套用竖直航线固定XY限制，而是独立检查混合目标、0.15m/s速度和任务范围。
先完成原稳定悬停才能交接，LAND后不能回到水平控制，接管后不得自动重入。

## 已部署文件

- `scripts/navigation_goal_link.py` / `navigation_goal_bridge.py`：ComputePathToPose动作、航段/路径关联，拒绝旧目标、过期/重复候选；不连接飞控。
- `legacy_apf_shadow/node.cpp`：保留旧比较输出，新增原子JSON候选；真实输入必须显式启用路径引导，到点容差0.05m小于任务0.08m。
- `scripts/task_scan_bridge.py`：实际TF转换/保留原时间和未知射线，XY偏移或非平面安装不支持时拒绝。
- `scripts/task_scene_input.py`：真实ROS输入/米制H投影/地图制动；配置中未核的数值保持缺失。
- `scripts/task_flight_controller.py` / `task_dispatch_contract.py`：交接与独立混合目标检查；不修改原起降核心。
- `scripts/task_flight_release.py` / `task_flight_runtime.py`：专用许可入口，默认仅检查；不启动Agent/传感器、不写PX4参数。
- `launch/task_navigation.launch.py`：仅A*、APF候选、雷达转换和yaw-only TF；不与旧Nav2启动文件同时启动，避免两个planner/TF源。

## 输出语义

起飞仍为有限XYZ位置目标。导航是 `OffboardControlMode.position=true`、
`position=[NaN,NaN,z]`、`velocity=[vx,vy,NaN]`、固定yaw。
必须有真实PX4版本和混合控制响应证据，不能仅凭消息合法认定实飞可用。
H对准后交回原LAND，不是下降全程持续视觉修正，也不保证误差为零。
失败分支沿用原中止/接管策略；停止发消息不是物理制动证据。

## 配置与现场未完成项

本次航线的地面启动组合为 `launch/ground_task_navigation.launch.py`，不要误用旧
`ground_navigation_shadow.launch.py` 的0.6m/0.15m演示默认值。新组合使用0.86m/0.14m，
必须显式给出该VIO会话地面初始 `initial_z`；这是启动传感器的入口，本轮未执行。
已有传感器时应按同一个 `task_map_profile.mapper_profile(initial_z)` 分别启动地图和导航，
不能重复启动相机/TF/规划器。

任务现在还订阅 `/nvblox_node/esdf_slice_bounds`，须与ESDF切片来自同一地图进程，
检查内部积分器上下界与切片中心、时效和完整高度覆盖。新组合已传入
`bounds_rate_hz=5.0`；仅起 `task_navigation.launch.py` 不会替既有地图启用该诊断。
参数更改不代表旧积分器已经重新初始化，应使用新地图会话。上下界一致性不等于真实空间已观测为空闲。

`config/roundtrip_task.yaml` 已保存获准内参及本次降落方式。区域边界、全高度地图覆盖、
制动和落点误差证据、精确光学到机体旋转矩阵未自动填写/认证。
这些决定是否放行真实任务，但不妨碍用明示合成数据验证软件。
不需要重做已确认的内参标定；缺少的运行配置和现场证据不能伪造。

真实规划启动配置依赖已运行的VIO、带正确高度区间的nvblox、雷达及TF。
宿主已构建消息类型、APF和nvblox Nav2 CPU插件；已完成实际A*新接口隔离测试，
不等于宿主已运行GPU建图或真实传感器链，原容器路径仍须核验。
APF宿主产物为 `host_task_build/apf/legacy_apf_shadow`。未启动该真实输入配置。

`task_flight_runtime.py --release <任务专用记录> --params /home/cfly/param.params.txt`
默认仅验证记录；缺少证据或实飞许可false会拒绝。不要为了运行而将未验收项批量改true。
带 `--authorize-real-flight` 的入口还要求本地交互式操作员确认、原同一把锁及一次性记录；
本次没有执行该入口。旧普通起降入口保留，不接受新水平任务的隐式授权。

## 复验（仅软件）

最新实际规划接口验证：`evidence/task_astar_apf_20260930_225045/report.json`。
`tests/task_astar_apf_check.py`复用`task_navigation.launch.py`的`planner_parameters()`，
域176、本机回环运行真实NavFn A*、nvblox Nav2插件、目标桥、C++ APF；
地图/雷达/定位为合成静态夹具，不模拟飞机运动、不启动GPU建图/传感器/PX4。
包括绕墙路径、返程旧航段拒绝、封路/未知区域拒绝和目标过期，55秒总时限另加有界清理。
先加载Humble与工作区，再分别加载
`host_task_install/share/nvblox_msgs/local_setup.bash`及
`host_task_install/share/nvblox_nav2/local_setup.bash`；私有install根目录没有总local_setup。
执行`python3 tests/task_astar_apf_check.py`，不要为离线复验启动真实任务launch。
本次插件仅从已有源码本地构建至`host_task_build/nvblox_nav2`/`host_task_install`，
无系统包安装；不加载该私有prefix即不使用本次插件，旧环境保留。

规则/阶段顺序见`config/competition_rules.yaml`及总记录最新节：先避障/H，再投递，再门。
比赛候选中心离地1.30m/12s与当前1.00m/3s调试分开，未启用；当前任务运行器并非全比赛监督器。

在ROS Humble/现有工作区环境加载后，再加载
`host_task_install/share/nvblox_msgs/local_setup.bash`。
在 `tests` 目录执行 `python3 -m unittest test_navigation_goal_link test_task_handoff test_task_scene_input -v`。
`task_input_dds_check.py` 为6秒WAIT输入检查；`task_handoff_dds_check.py` 为最长105秒完整理想DDS链，
域177、本机回环、固定隔离话题，无真实PX4输入。它们不能与其它域177测试同时运行。

## 关于旧精降代码

2025提交cd11857为视觉对准后普通LAND。未提交的 `src/uav_task/src/obstacle_course_mission.cpp`
具有精降构想，但其高度原点、12cm容差、875px焦距、丢目标超时普通落地和高度阈值完成判断
不能直接套到当前相机/地面参考/60cm环。没有改动这些用户文件。
“降一段→悬停再对准→慢降”要保留单一Offboard控制直到最终交给PX4，不能来回抢模式；
还需要确定可见H的中间高度、实际净空与慢降速度。本轮未启用该扩展。

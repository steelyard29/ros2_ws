# 起降已通过，后续比赛任务

## 2026-10-01 新增地面推动方向/比例分析器（尚未实测）

- 新增scripts/analyze_ground_motion_alignment.py（只读，不改任何reviewed）：读取resident_ev_service的report.json配对，按首个VIO位姿建LocalFRD，判定：前推>=0.30m为PX4 +x（横向<=35%）、随后左移>=0.30m为PX4 -y、水平比例PX4/VIO在±10%、航向变化>=20°且PX4与VIO航向差<=5°；重置代数变化即拒绝。tests/test_ground_motion_alignment.py 5项通过（一致通过、比例1.3拒、左右镜像拒、静止拒、重置变化拒）。
- 对静止会话resident_ev_service_8ceab4fe运行：forward 0.002m、yaw 0.08°，正确判为未通过。证明工具不会把静止当动态对齐。
- 配对只在未解锁、landed、xy/z valid时产生，所以只覆盖地面水平方向/比例/航向；高度方向、空中、下视轴向、落点误差、起降柱净空仍需各自实测，不由本项替代。

## 2026-10-01 EV＋独立flags同窗验证通过本观察窗口：1316EV、43.302秒融合、1309配对

- 用户明确授权一次<=60秒EV联合验证，保持未解锁地面静止。本轮新增resident_joint_check（默认describe），调用现有常驻服务＋独立flags订阅；服务50秒TERM期限/3秒退出兜底，独立观察54秒，总体54.303秒。未启动任务目标/控制/解锁/模式/舵机，未改参数、阈值、固件或串口配置，无自动重试。
- 总证据evidence/resident_joint_773e2f73a2fa4f77a7a8aafe24a30d30/report.json；服务证据resident_ev_service_5fe4f3350d714de8a94033bbbcf83b33/report.json，session=ce01982183ab45a496a644e1ed9f1151。service_exit=0、reason=operator requested stop、fault=null，服务自身49.511秒；外部timeout返回124表示按期发送TERM，不是服务失败。观察器0，管理器0，所有进程关闭、reader confirmed=true。
- EV实际1316条；47个融合状态样本中45个位置/航向/高度均true，连续采样跨度43.302374秒，maxgap1.011311秒、issues=[]。独立观察55条/maxgap1.011047秒，两端common_gaps=[]。不把预热段或最后样本至停止间隔算入持续融合时间，不将局部成功说成长期稳定。
- 同会话静止源戳配对1309/1316，最大时间差13.969545ms，所有配对重置计数[21,7,6,16,2]。不是动态方向/比例验证；本次服务结束，后续新EV会话必须重新配对，不能沿用此会话alignment。
- 读取器pose/tracking各1350，maxgap59.810/56.659ms，最大队列驻留9.142ms，would_block0。协议包enqueued3150/sent3149，停止清理丢弃末尾未发送包，不能声称全量无丢包；pending0/ROS关闭true/client0/worker0。
- 用户在本次EV活跃窗口查询PX4：XRCE running connected/serial，TX64471B/s、RX3453B/s，timesync=true；历史max cycle1021363us/interval1021364us未增加。与精确机载源码TX/RX均>0跳过零RX ping路径相符，支持零RX阻塞ping为历史缺口候选，但缺同窗函数跟踪/可控A-B，不能宣称唯一根因已证明或任何负载下均已修复。
- 本次保留配置：D435i/VIO既有域176＋显式perception_dds_shm16m.xml，主端PX4域0＋fastdds_bridge.xml，Telem1 /dev/ttyTHS1 921600，唯一常驻EV出口，原时效和故障锁定不变。EV已按授权期限停止，未保留后台运行；自启EV单元准备好但仍未获本地sudo安装结果，未冷启动验收。
- 下一步可推进真实导航目标/任务控制交接验证，不继续原样重复独立只读测试；须在新持续EV会话内取得新配对并使用唯一控制入口，不能凭本窗口直接自动解锁。避障、H落地、窄门、投递整赛目标尚未完成。

## 2026-10-01 机载版本确认与XRCE机制定位：零RX时1秒阻塞ping为重要候选，不冒称已证实唯一根因

- 用户QGC原始输出确认PX4_FMU_V6C/V6C002001，Release1.16.2 stable，git54f0455ffcd755534539a7cf33a09a20bf71d29d，构建Apr22 2026。XRCE running connected/serial，TX64151B/s、RX0B/s，timesync converged=true。cycle平均3370.76us、历史最大1021363us。uorb top显示estimator_status_flags实例0/1各约1Hz、queue1、size96；这是当前窗口原始发布频率，不证明历史缺口时每次都正常发布。
- 改用git show读取匹配机载hash的源码，不改checkout。更正先前参考较新本地HEAD的发送描述：54f0455的dds_topics.h.em对各发布topic使用固定10ms orb_set_interval，逐条序列化后flush；prepare失败没有前述较新版的flush后重试。该提交不读取本地YAML新增rate_limit字段，不能靠修改Jetson YAML给现有飞控限流；需要编译/刷固件的修改仍未授权执行。
- 已确认候选机制：该提交uxrce_dds_client.cpp checkConnectivity仅在TX>0且RX>0时跳过ping；RX=0时每>1秒调用uxr_ping_agent_session(timeout_ms=1000,attempts=1)。按该提交.gitmodules锁定PX4/Micro-XRCE-DDS-Client 711aef423edd1820347b866d1e4164832df35d04，在线核对src/c/util/ping.c，调用uxr_run_session_until_pong等待回应。因此1秒阻塞路径确实存在，与历史max1.021秒和约2.01秒状态间隔吻合，但尚未取得同窗调用轨迹，不能写成唯一根因已证实。
- 串口921600在8N1下理论每方向92160字节/秒；TX有效载荷64151约69.6%，尚不含XRCE/串口封装，不能简单宣称链路只有69.6%占用或一定饱和。RX0与当前没有EV/控制输入一致，不表示物理串口断开，也不能用虚构位姿/空运动命令制造RX。
- 当前结论：原始EKF发布现时正常，XRCE处于连接/同步状态，但零RX监测ping和较高发送负载为待验证因素。下一次应在真实合法EV输入期间并行观察flags并让用户读取XRCE RX/周期统计，比较是否避开零RX路径；不得用过去18秒融合或当前1Hz证明整段稳定。未改参数/波特率/固件/自启、未启动EV或导航运动。

## 2026-10-01 PX4原始发布/XRCE核对：BEST_EFFORT实测确认，等待飞控端只读状态

- 真实图只读核验：/fmu/out/estimator_status_flags唯一发布者_CREATED_BY_BARE_DDS_APP_，GID与此前resident记录一致，QoS reliability=BEST_EFFORT、durability=TRANSIENT_LOCAL。接收端现有sensor_data BEST_EFFORT匹配；不能改为RELIABLE订阅就保证可靠，BEST_EFFORT发布者不能满足RELIABLE订阅。此次未改QoS/参数/固件、未注入EV或发导航/控制。
- Jetson Agent PID17755保持运行，命令MicroXRCEAgent serial -D /dev/ttyTHS1 -b 921600 -v 4；进程环境使用域0/fastdds_bridge.xml。QGC USB转发独立PID2028，当前状态usb_connected=true，Jetson192.168.43.228→QGC192.168.43.89:14550，未占用USB或改变QGC peer。
- 本地PX4源码PublishStatusFlags约1Hz/状态变化立即发布；DDS生成发送代码使用best_effort_stream_id，输出缓冲不足会flush并重试一次，仍失败分支无重传保证。这是源码机制，不是已证实的本次根因。源码HEAD5368e5f1fd，但dds_topics.yaml有既存修改；本地rate_limit=5不能冒称机载固件的实际编译配置，须核对ver all。
- 只读系统计数Udp InErrors/RcvbufErrors各106259是历史全主机累计，不能归因PX4；Agent六个当前UDP套接字drops均0、收发队列均0。此观察不能排除UART/XRCE丢失、已销毁接收器套接字丢弃或飞控未发布。/proc/tty/driver权限不足，未使用sudo绕过、未开启被占用串口。
- 已向用户请求QGC MAVLink控制台依次ver all、uxrce_dds_client status、uorb top estimator_status_flags（看约10秒Ctrl+C），以核对机载版本、connected/payload/timesync、原始uORB频率。未拿到输出前不猜测“飞控没发”或“串口丢包”，不放宽源年龄/2秒门槛，不接水平运动。下一步根据飞控只读证据决定是否需更具体的同窗原始发布记录；无新EV试验。

## 2026-10-01 同窗双接收器定位：真实任务输入完成，但共同flags缺口约2.01秒，不能放行控制

- 新增task_joint_timing_check默认仅描述，显式一次<=30秒启动现有任务只读入口＋独立进程域0 flags只读订阅；没有EV、导航目标、运动控制或相机/Agent重启。新增任务flags_timing环形记录120条，包含接受和拒绝前的源戳/年龄/回调时刻；不改变保护规则。只按完全相同源戳对照，不靠近似到达时刻猜测。
- 实测evidence/task_joint_timing_b9d3f0942bf745e3af7ca8261943f8ee，26.269秒，两进程exit0。任务证据task_readonly_87eccc4479274efb9f262aaf686a8aaf，20.413秒、input_delivery_observed=true、source_rejection=null、无任务异常。这里exit0只说明观察正常结束，不表示全程满足飞行时效。
- 对照结果：任务记录flags17条、独立20条（两端发现及观察起止不同，不以计数差推断丢包）。共同源时间戳1790816313246902→1790816315260730，源间隔2.013828秒；同一对端点之间任务接收间隔2.013121秒、独立2.011968秒，双方都没有中间源戳。共同窗口内独立接收器没有额外的任务漏收消息。不能只归因任务处理地图太慢；问题位于共同发布/传输/主机中间件路径，尚不能区分PX4未发布、XRCE丢失或共同DDS缺口，也不能声称确有某一条消息丢包。
- 本次所有task flags源年龄最大8.720ms，独立最大8.376ms；15个同源戳配对的接收时刻差仅毫秒级。未复现上一轮>500ms或<-50ms的源年龄拒绝，但确实复现2秒时效缺口，保留两种故障区分。没有放宽阈值/回填消息/把最后融合状态延长为持续健康。
- 原始报告保留；事后分析新增shared_gaps函数供后续报告自动列共同端点缺口，2项纯离线对照测试通过，实际工厂路由6项也通过。新增函数没有重新执行硬件窗口。
- 下一步应核对PX4端estimator_status_flags原始发布与XRCE传输，不再重复独立低负载只读成功；当前真实导航输入可达，但任务仍无新会话EV配对，未发导航目标/水平控制。未解锁未飞行，比赛任务未完成。

## 2026-10-01 推进A*＋APF真实任务输入：感知跨域数据已到达，PX4源时间拒绝阻断检查

- 本轮执行现有task_readonly_session --observe-existing-inputs --container-inputs，一次只读真实输入检查。证据evidence/task_readonly_89de85db85494bca91c26080521b72ec/report.json，14.073秒exit1；no_controller_tick=true，真实发布器0、导航目标0、EV0，不启动相机/DDS/飞行。
- 实际容器→主机收到VIO/depth/tracking各239，地图39、地图高度边界76、H候选79；navigation_command来源1但本轮未收到命令（没有发目标，不将缺命令等同于APF故障）。PX4 local1094/status22/flags11/land12/RC505。读取器无传输故障、max_pipe_age92.592ms、关闭回执confirmed=true；说明感知与地图真实跨域链已接入，不等于所有几何/导航有效性通过。
- 检查因invalid/stale source: estimator_status_flags停止。这是收到消息时源年龄不满足[-50ms,500ms]等源时间条件，区别于前一轮超过2秒没有新回调。旧报告未记录被拒消息stamp/age，不能断言是过旧还是超前，不能认为“话题无缺口”就已消除所有时序问题。
- 修正诊断缺失而不放宽门槛：FlightRuntimeCore记录首次source_rejection的消息源戳、接收单调时间、源年龄、上次有效戳和阈值，写入runtime报告；不更新拒绝消息的新鲜度，后续拒绝不覆盖首次现场。新增离线测试通过。没有自动复测或用已有静止配对伪造新会话alignment。
- 当前仍无常驻EV，真实任务尚未初始化：缺本会话配对/有效地面PX4数据，以及现有配置的geometry/coverage审核项。未修改reviewed标志，未向PX4接通水平控制，不能声称真机避障已跑通。下一步应在同一联合会话记录两路flags观察和源年龄，解决时序后接真实导航目标，不重做A*/APF算法。

## 2026-10-01 flags只读排查：当前25秒无缺口；推进实际任务工厂离线交接

- 用户要求排查融合状态话题，若无缺口继续下一步。本轮只订阅域0 flags/status/local共25.005秒，创建ROS发布器0，不重启、不注入EV、不发控制。flags25条，最大接收间隔1.010775秒/源戳间隔1.007115秒；status49条/maxgap0.510978秒；local2468条/maxgap0.026914秒。flags最后源年龄1.062ms。本窗口未发现停流；因EV已停，视觉位置/航向/高度融合标志均false，不将其解释为融合故障。
- 本地PX4源码checkout 5368e5f1fd，EKF2.cpp PublishStatusFlags明确约1Hz或状态变化立即发布；uxrce_dds_client/dds_topics.yaml rate_limit=5是上限，不是5Hz保证。未重新核对机载固件与此checkout完全一致，源码作为解释参考，当前实测周期为直接证据。
- 上一轮联合运行的flags>2秒未更新仍是有效失败记录；本次独立低负载订阅未复现，不能称不存在间歇缺口，也没有证据区分飞控发布、XRCE传输或常驻节点调度。Agent现有info日志只有端点建立，无法据此定位数据包丢失。没有放宽2秒阈值或自动重新启用EV。必要的后续为同次EV联合负载下并行独立flags观察，不能再用独立只读成功替代。
- 已继续下一步安全的软件交接：test_external_ev_routing新增实际production工厂分支测试，使用真实TaskFlightCore/类型检查、模拟ROS端点，以及明确模拟的许可校验和临时目录许可标记；只证明接线，不验证真实飞行许可。确认真实话题路径仅创建3个控制发布器、无EV发布器；LAND阶段仍能接收外部EV；任务close不使外部EV证据失效；绑定来源丢失锁定。该文件6项全通过0.385秒，不是实际DDS/飞控或完整任务飞行验证。
- 本轮未启动硬件运动，开机EV服务安装仍需用户本地sudo且尚无冷启动验证。继续主线是联合负载遥测来源定位与任务真实输入接入，不重做相机标定/规划算法。

## 2026-10-01 PX4融合与同会话静止配对已有真实证据；flags停流导致服务退出，自启候选已准备

- 新增resident_ev_observation，仅记录既有融合标志并订阅VehicleLocalPosition；复用closest_local的源时间差<=50ms、年龄<=250ms、未解锁/落地/位置有效/重置计数一致条件。每次成功发送EV后尝试配对，不根据发送量推断融合，不初始化飞行任务。数据按会话保存，有界保留400融合样本/2000配对；不创建第二EV或运动出口。
- 85项resident离线测试通过2.010秒；shell语法与systemd-analyze verify通过（输出另有系统现存无关单元警告）。本次只执行一次真实EV，外部foreground 55秒TERM/3秒强制退出兜底，未重启相机/DDS、未改参数、未解锁/运动。
- 证据evidence/resident_ev_service_795c4fb655eb4293897b8e8b83199b36/report.json，session=654630d964eb48a08992385fb7ecf7d2。运行23.915秒exit2，EV610条。23个有效融合状态样本中20个position/yaw/height均true，最长连续采样跨度18.135032秒，期间最大flags接收间隔1.012176秒，无该区间内记录问题。ev_velocity=false/baro=true/range=false。不能把最后true外推到退出，更不能写成60秒持续融合通过。
- 同会话静止配对610次尝试/603次匹配，最大源戳差14.247773ms，配对的5项PX4重置计数均[19,6,5,15,2]。这证明该窗口同步静止位置数据可配对，不是动态方向/比例验收，不自动把SessionAlignment标为verified，服务退出后的旧配对不得用于新会话。
- 停止原因telemetry stale：最后flags回调10271.673009646，检查结束时年龄2.284秒；status/land/local均更近。读取器pose/tracking各644条，maxgap49.627/52.609ms，无旧视觉停流故障。原因目前定位到主端flags消息更新缺口；不能直接断言PX4停止融合（最后样本仍融合true），也不能只凭约1Hz正常周期就放宽阈值。下一步查清flags的发布/串口DDS/接收缺口。
- 退出修正这次正常工作：closed=true，reader owner_requested_stop/ros_closed=true/exit0，client0，confirmed=true；不是上次回执迟到误判。完整报告保留，服务控制台改为简要摘要避免日志输出上千配对。
- 新建scripts/boot_resident_ev.sh及systemd/robocup-ev-boot.service：依赖sensor/DDS服务，检查NTP同步、唯一EV所有者、地面未解锁启动；Restart=no，故障不自动恢复，KillMode=mixed使owner先处理停止。只有EV/只读观测，没有自动飞行。自启安装/回滚命令写入systemd/BOOT_STATUS.md。
- 当前sudo需要本地密码，无法代为安装；sensor/DDS已enabled但inactive，EV单元尚未安装。未冷启动、未启用实飞。可注册故障即停的自启候选，但不能称可靠开机融合已完成；用户本地enable不带--now，不在现有手动进程上重复启动。全目标仍未完成。

## 2026-10-01 EV重启修复：明确DDS配置遗漏，修正后1503条EV；退出回执竞态另已修复

- 用户授权重启EV查因修复。本轮只启动一次修正后的EV-only服务，外部55秒INT/3秒强制退出上限；没有解锁、运动/模式/舵机指令、改飞控参数或重启相机。证据evidence/resident_ev_service_0b33034cad6a46d0b5f8e2a7dc3d1532，session=113330929adb4b6091e9adbb57aa2d7e。
- 确认的配置缺陷：容器内运行中的VIO进程143明确使用config/perception_dds_shm16m.xml；resident_vio_worker原先仅设置ROS_LOCALHOST_ONLY，并未显式配置与感知链一致的16MiB SHM及两个FastDDS profile变量。container_env.sh也不设置这两个变量。新增configure_transport，在rclpy初始化前固定域/中间件/localhost/两个profile，文件缺失拒绝启动；不依赖登录shell环境，不修改PX4域0传输。这能避免此配置遗漏随开机再次出现，不保证所有停流不会发生。
- 修正后的实际窗口：服务started=10006.677429，最后tracking=10061.112451；发送1503条EV、warmup_count=1532、EV fault=null，约54.44秒内未复现Tracking stale。读取器pose/tracking各1533条，最大回调间隔64.531/64.865ms；3578个协议包入队且发送、pending0、max_queue_residence=9.200ms。源时间未重写，300ms停流保护未放宽。结果支持配置修正有效，但没有同负载A/B对照，不能严格证明它是历史361ms间断的唯一原因。
- 到外部55秒截止时rclpy抢先处理SIGINT，报告reason=ExternalShutdownException、exit2；Docker客户端先退出，report读取时回执尚未写入，closed=false/confirmed=false。后续只读核验同session回执：owner_requested_stop、ros_closed=true、exit0、closed_at=10061.167857，比服务报告结束晚约27.24ms；主机service和容器worker均无存活进程。保留原报告不篡改，不能把整个试验写成exit0。
- 退出问题已单独修正：rclpy使用SignalHandlerOptions.NO保留服务的显式停止处理；客户端结束后最多等待0.5秒原worker回执，不启动新进程、不提前缓存缺回执失败，服务循环识别awaiting_receipt。新增迟到回执和永久缺失的离线回归。最终83项resident测试1.718秒全部通过；本机SignalHandlerOptions.NO存在。退出修正尚未真实复测。
- 结论：已修复确定的启动配置遗漏，EV真实输出在本窗口恢复；未证明PX4持续融合、VIO↔PX4配对或可飞。此次按时退出，当前未保留常驻EV；未安装/启用新的EV systemd单元，已有sensor/DDS服务未改，冷启动整体验证仍未完成。后续先验证融合/配对及退出修正，再启用真正的EV开机常驻；不要把已有两个enabled服务当成常驻融合已上线。

## 2026-10-01 新常驻EV首次真实启动：4.017秒停止，Tracking stale，EV发送0

- 用户确认未解锁、地面H起点静止，并授权最多60秒验证、通过保留/失败停止不重试。本次执行resident_ev_service --execute-real-ev，外部55秒INT超时及3秒强制退出兜底；实际由内部故障约4.017秒退出2，未到超时。证据：evidence/resident_ev_service_8946dc92c5554f5585d92f7810da5f03/report.json。
- 会话a4691da288ec492fb40f078df08af25f；故障Tracking stale; new session required，sent_count=0。未注入任何EV，不能声称PX4融合/配对通过，也未解锁、发运动或模式指令、修改参数。此次一次授权已消耗，没有自动重试或保留服务运行。
- 读取器owner_requested_stop，ROS关闭true、exit0，主端client_exit_code=0、running=false、confirmed=true，服务closed=true/cleanup_errors=[]。传输共入队/发送20个协议包，pending0，最大队列驻留0.856ms；协议包包含图信息，不代表20帧VIO，也不能证明视觉输入连续。
- 对照源码：此错误由已收到过的tracking回调距离watchdog检查超过0.3秒触发，不是“尚未收到首帧”的启动超时。现有报告未记录源/读取器/主端分段接收时间，不能归因相机坏或纯启动发现竞态；不放宽阈值、不原样复测。下一步补本次入口的分段时序证据并离线验证，再申请一次针对性的真实复测。真实融合、坐标配对及避障/H降落实飞仍未通过。

## 2026-10-01 本轮收敛：算法已有，补任务入口交接验证；未执行实飞

- 再次核对PX4官方Offboard文档：规划运行在Jetson，通过TrajectorySetpoint/OffboardControlMode交给PX4，不需要把A*/APF刷入飞控；ROS 2心跳必须持续高于2Hz，切入前预发送至少1秒。Offboard内摇杆通常不直接控制，不能以此单独判定遥控器故障。官方main文档不替代本机版本和失联行为验证。
- 沿用已有NavFn A*＋路径引导APF、往返任务、H对准后AUTO_LAND；不重新标定，不改DWB/APF选型，不要求地面完整看见H，不把首次飞行要测的落点结果当成已通过。
- 新增tests/test_resident_task_entry.py，实际调用任务CLI但替换ROS运行器/许可/文件锁：4项全部通过（0.057秒）。覆盖默认不运行、常驻绑定选择控制专用锁并保持确认→复核→消费许可→运行的顺序、绑定过期不消费/不运行、旧入口保持EV排他锁。未创建真实许可/锁/ROS端点；不等于实际runtime工厂或真实降落交接验证。
- 明确剩余关键路径：新常驻EV任务分支的完整联合验证；实际持续EV期间PX4融合和同会话坐标配对；实际导航控制输入及受控退出确认。当前常驻服务尚未安装/执行，本轮不能声称数据链已稳定或已可起飞。
- 已发现试飞许可把h_alignment_and_landing等审核材料与普通任务许可绑定，需要区分首次受控试验的前提和试验后验收结果；本轮只记录此问题，没有伪造审核、删除保护或启用live_flight_enabled。硬件一次性授权未复用，未注入EV、解锁、运动、改参数或启动服务。

## 2026-10-01 任务入口接通常驻EV绑定与控制专用锁，离线回归通过

- task_flight_runtime增加可选--resident-status；先做原TaskFlightPermit校验，再固定常驻状态文件。显式实飞路径仍需原本地确认和一次性permit消费；选择绑定时用/tmp/robocup_task_control.lock，未选择仍保持旧shadow锁/行为。状态绑定不授予飞行许可，没有改变live_flight_enabled或审核证据。
- live_flight_runtime把绑定带入只读precheck和任务core/runtime：只允许已核对GID的唯一EV真实输入，其他/fmu/in发布者仍拒绝；比较任务VIO及PX4 status/flags/land来源与常驻服务身份。启动图未发现时等待但不ready，重复/错误身份立即拒绝。运行器只在前2秒原发现窗口允许缺失，不续期ownership，之后缺失锁定。
- flight_runtime真实external_ev仅接受匹配ResidentStatusBinding.evidence、专用任务core和既有live permit；bench/handshake/preview及任意未绑定real external路径仍拒绝。复用已有三控制发布器＋EV订阅结构，不创建EV发布器；每100ms审计同会话/端点，消息自身仍独立验证源时间/帧/质量。任务退出不调用常驻服务停止。
- 本轮resident75项通过1.626秒，external12项通过0.009秒；完整既有任务277项通过、0失败/错误/跳过，证据task_regression_20261001_083342，21.387秒。新增三图验证测试覆盖真实来源对应、启动缺失不放行、发现期重复仍拒绝。这些套件分开运行，不混写为同一个测试报告。
- 代码接线已改变真实任务入口，但新绑定实飞分支尚无完整联合运行证据，不能用原277项或旧隔离往返替代。没有启动真实服务/传感器/EV/控制，下一步对本分支做专用入口联合验证及处理实际启动条件，再进入有界实机融合/飞行确认。全比赛任务未完成。

## 2026-10-01 任务侧常驻服务身份绑定实现，任务运行入口尚待接入

- 新增resident_status_binding.ResidentStatusBinding：固定一个status.json，核对500ms状态年龄、运行/发送状态、host0/source176、唯一24字节EV GID和五路来源GID；固定session/PID/进程启动ticks/boot/time namespace及源身份，任何变化或计数回退锁定，不自动重绑。读取真实/proc身份排除PID复用/僵尸；仅本地来源归属核对，不宣称安全认证。
- service.status增加process_identity和source_gids，供任务绑定同一个实际定位服务。绑定创建ExternalEvEvidence但不更新其消息计数或新鲜度；必须另收真实EV消息，PX4融合仍独立检查，不因状态文件健康就放行。
- 5项新增离线覆盖正常更新但不伪造EV接收、状态过期且恢复也不解锁、session/GID/进程变化、停止/无效来源、计数回退。resident72项全通过1.840秒；无ROS订阅/发布或新进程启动。
- 已读当前任务入口：task_flight_runtime仍走原shadow锁，live_flight_runtime.precheck拒绝所有真实输入发布者，flight_runtime真实external_ev仍拒绝。这些尚未修改，因此本轮只是任务绑定实现，不是已接通任务。下一步同时接入预检查允许的唯一EV、真实runtime绑定审计和控制专用锁，保留原单次飞行许可/现场确认，不直接换锁绕过旧保护。完整目标未完成。

## 2026-10-01 常驻EV服务运行入口已接线，默认仅说明，未执行真实输出

- 新增resident_ev_service.py，默认describe；显式--execute-real-ev才会获取原输出排他锁、配置host域0、创建同session真实EV工厂、启动域176只读管道，并在同一服务循环处理PX4和视觉输入。入口不启动DDS/相机，不发解锁/模式/轨迹/参数/舵机；需要既有链及落地未解锁状态，未安装/启用systemd。
- 服务每200ms保存本会话状态（EV发送计数/最近发送、唯一出口图GID、session/PID/时间），明确fusion_verified=false/flight_ready=false，不能把已发送说成EKF融合。SIGINT/SIGTERM请求退出；故障不重启。停止先关闭EV再停止reader；最多3秒收集进程回执，分别销毁节点/ROS、释放锁，并写最终报告/失效状态。关闭未确认或清理异常exit2，不把未知说成已关闭。
- 新增4项服务循环离线测试，覆盖正常服务顺序、输入异常先禁用EV再清理、遥测回调故障不再pump输入、发送状态不提升为融合/可飞。最终resident67项全通过1.754秒；仅执行默认描述入口，无ROS、Docker、EV或硬件操作。完整真实run尚未运行，这不是常驻实机成功证据。
- 下一步是任务入口读取该服务会话/唯一EV身份、取消自身EV发布并使用控制专用锁；当前旧任务入口仍与服务争用原锁，不能直接并行。之后才是新明确范围下的真实持续融合/配对与受控避障/H落点测试。全比赛目标未完成。

## 2026-10-01 真实EV工厂接入独占锁与同会话输入绑定（未运行）

- 新增ResidentEvOwner，复用原/tmp/robocup_flight_runtime_shadow.lock作为EV/旧入口排他锁。非阻塞获取、同实例不可重获；要求ScopedVioInput内层为真实ResidentVioPort、域176、会话一致且无输入故障。它是合作进程锁，不替代DDS发布者唯一性审计，也不等于飞行许可。
- resident_ev_node真实工厂现在必须显式给绑定owner，宿主域0，视觉端176；唯一发布接口/fmu/in/vehicle_visual_odometry，三路PX4只读遥测status/land/flags，无Offboard/轨迹/VehicleCommand。持续审计和每次位姿发送前均验证锁/会话，丢失即关闭输出。仍要求未解锁落地起始、有效时间/跟踪/帧及唯一稳定源。
- 首轮离线新增参数owner与旧审计局部变量同名，引发7项UnboundLocalError；已改局部名source_node并保留原测试，修复后resident63项全部通过1.550秒。4项新增包括真实临时文件flock冲突/不能重获、真实同会话绑定/错误来源拒绝，以及模拟ROS下只创建EV真实话题端点/owner关闭撤销输出；没有真实ROS/PX4输出。
- 此改动仅工厂可接线，没有常驻服务执行入口或systemd启用。旧任务/bench入口共享上述锁会被常驻定位排斥，必须把新任务改为外部EV+独立控制锁并验证后才能共存；不能直接给旧脚本换锁绕开。下一步接服务运行循环与任务唯一控制出口。无新硬件授权消耗、无解锁/运动，整赛目标未完成。

## 2026-10-01 真实传感器持续读取模式已接入worker/主端，尚未执行

- resident_vio_worker新增显式--lease-sensor-only：固定域176，接入上一轮真实VIO/raw status/安装TF适配器，使用主服务续期控制寿命，无台架deadline；仅订阅和stdout传输，无PX4订阅、EV/控制发布、相机启动或参数写入。默认仍describe，隔离模式与真实只读模式互斥，真实模式必须显式给域176及控制/回执路径。
- ContainerResidentInputs新增显式sensor_only=True/domain176，构造仍无启动副作用；start()构造持续worker命令，不加25秒deadline或timeout。主循环续期、1秒失联退出、来源时效、停止/回执和单次不重试合同不变。原隔离模式保持<=25秒/timeout26，不把它改成真实发布捷径；常驻EV发布工厂仍只允许隔离。
- 3项新增测试覆盖真实只读命令构造、模拟超过60秒持续续期、混合模式/台架deadline拒绝、真实适配工厂选择及关闭回执。用假Popen/模拟ROS，不调用Docker或订阅硬件；resident59项全通过1.690秒。默认描述入口也执行，无ROS副作用。上轮真实跨容器证据仅覆盖当时隔离版本，不能冒充本模式已实测。
- 真实传感器持续输入的代码调用链现在已接通，仍未运行或装服务；下一步为主端PX4常驻EV唯一出口/会话绑定及任务控制分离。运行新真实订阅/EV联调须按现场状态及明确测试范围执行，不复用以前一次性硬件授权。无新硬件操作，避障/H降落及整赛任务未完成。

## 2026-10-01 真实cuVSLAM来源/安装TF适配器已实现，未启动硬件订阅

- 新增resident_real_source.create_sensor_node，限定实际domain176及非隔离sender；订阅原始/visual_slam/tracking/odometry与/visual_slam/status，不依赖单独tracking中继。原始状态源戳和vo_state映射到管道既有String格式，逻辑/robocup/alignment/tracking的来源GID记录原始status发布者；不创建该ROS中继话题，不改状态或更新时间戳。
- 发送前检查base_link←camera_link TF，要求已确认xyz[.196,.025,-.05]/零安装角；四元数q/-q等价，拒绝NaN/错误偏移/旋转。1e-6是配置一致性公差，不是物理安装精度。启动TF发现最多10秒，已验证后TF缺失或改变关闭输入并锁定sender；未验证不转发数据。沿用图来源唯一/稳定、接收端时效和帧检查，不移除原保护。
- 5项新增纯函数/模拟ROS工厂测试通过：几何与四元数等价、错误/非有限外参、原始时间戳和不健康状态保留、只订阅原始两话题及逻辑身份映射、TF丢失关闭和隔离sender拒绝。resident全部56项离线通过1.716秒。未启动真实ROS节点；新增工厂无CLI，生产worker/常驻EV出口仍未开放。
- 上轮跨容器成功证据仍只覆盖隔离合成链，不能自动证明本轮真实源工厂；下一步将该工厂接入生产worker的会话/生命周期与主端真实EV所有者，再处理任务唯一控制出口。没有新硬件测试/EV注入/解锁，比赛目标未完成。

## 2026-10-01 新常驻链实际跨容器/CDR/关闭联测通过（非硬件）

- 新增tests/resident_container_check.py，默认describe；显式隔离执行一次，总长24.305秒exit0、passed=true。证据 `evidence/resident_container_c73e76f495b348b0b1c20b6e00d862f7/report.json`。容器183独立合成VIO发布进程→实际resident_vio_worker序列化→Docker stdout管道→主端182反序列化→实际resident_ev_node→合成飞控。只使用测试命名空间，不订阅真实传感器/飞控，不改相机、参数或服务。
- 主端收到182条视觉/跟踪数据，resident实际发布60条测试EV，合成飞控收到59条（结束边界差1，不冒称全量接收）；EV发布者唯一，commands=[]、armed=false。读取器入队/发送213个协议包、pending0、peak1229字节、最大队列驻留6.564ms、would_block0。协议包包含图记录，不与182条数据混写成丢包率。
- 主端stop后读取器reason=owner_requested_stop、ROS关闭true、exit0，宿主docker客户端exit0且关闭回执confirmed=true；合成视觉源按自身截止退出0。没有自动重试。此次证实两端实际ROS CDR、域隔离、续期文件身份和进程关闭路径在该窗口可用；不是PX4 SITL、完整导航往返、实际VIO稳定性或生产连续运行验证。
- 本次是上一轮51项离线回归之后的新整链证据，不再停留在假Popen。下一步应做生产域176→0的真实来源/TF/会话绑定和任务控制出口对接，保留真实输出禁用直至接线与授权条件具备；不重复本隔离成功窗口。开机常驻融合、真机避障/H降落及后续窄门/投递仍未完成。

## 2026-10-01 主端容器读取器管理器完成，51项离线回归通过

- 新增resident_container_inputs.ContainerResidentInputs，构造不执行操作；显式start才构建单次Docker读取器命令。当前只允许隔离域/最多25秒窗口，外部timeout26秒作为隔离兜底；真实模式仍拒绝。目录必须位于本项目evidence下且新建，防旧控制文件/回执复用；不启动相机或Agent，不创建ROS发布器。
- 主服务处理循环每>=200ms续期；停顿>1秒拒绝续接。停止时立即禁用本地输入/停止续期，写停止请求，poll_close非阻塞排空但不分发旧数据。只有docker客户端退出码与当前会话关闭回执相符才confirmed；客户端仍运行、无回执/错误会话不冒称已关闭。没有自动重试、重启或杀整个容器。
- 新增8项假Popen/真实匿名管道测试：构造无副作用、一次启动、续期、非阻塞停止、缺失/匹配/旧会话回执、进程结束锁定、主循环停顿、路径/真实域拒绝及启动异常清理。最终resident51项离线全通过，1.488秒；没有实际Docker调用或ROS子进程运行。调用方仍须先禁止EV输出再stop，并持续服务PX4及核对关闭，不把本管理器当飞行停止动作。
- 现在发送器、接收器、读取器生命周期、主端管理器代码已具备隔离接线条件；仍无这一整套真实跨Docker/CDR/续期运行证据，也未完成生产外参/会话绑定和真实任务接入。下一步应一次验证整条隔离链，而非重复单元测试；真实服务未安装、无新硬件尝试、无解锁/运动。完整比赛目标未完成。

## 2026-10-01 持续读取器续期与退出回执机制已实现，未启动真实进程

- 新增resident_reader_lifecycle：主服务在实际处理循环写原子续期文件，读取器核对session、单调序号、boot_id及time namespace；1秒未续期或身份/时间异常锁定退出，不自动复活。1秒是父进程失联清理期限，不替代原EV/数据新鲜度限制。读取器在spin前后都检查，避免阻塞后继续发送旧队列。缺文件/损坏文件也终止。
- 新增resident_vio_worker：默认仅描述，不启动ROS；目前仅显式--run-isolated可运行，域180～187且最多25秒隔离预算。工作循环复用持续租约模型，25秒只是当前隔离执行上限，不通过循环台架入口实现常驻。真实路由仍未开放；退出时停止发送、销毁节点、关闭ROS并原子写回执，清理异常明确ros_closed=false/exit2。
- 7项租约测试与3项工作器测试新增通过；含模拟续期超过200秒、父进程停止、过期不可恢复、spin期间过期不pump、默认预算退出、正常/失败关闭回执。工作器测试用模拟ROS，未启动真实节点/子进程。最终resident43项离线通过1.473秒；默认描述入口已执行且无ROS访问。
- 本轮未启用Docker worker、systemd或EV，没有硬件尝试。还缺主端容器进程所有权/启动与回执收集、生产外参及来源会话校验、真实常驻服务与任务接线；不能据本轮声称开机融合或真机避障已完成。下一步完成主端进程管理，而非重复这些单元测试。

## 2026-10-01 持续VIO发送端和两端协议联测完成，尚未启动跨容器服务

- 新增 `resident_vio_sender.py`：持续生产者复用PacketWriter，保持原CDR，不改源戳；与接收端共用图/身份校验。未发现双源时不发数据，10秒发现超时；队列驻留超过500ms、来源更换/丢失、编码或写入异常即锁定并丢弃本地未发队列，不重新连接。已经写入OS管道的数据无法撤回，接收端原有时效/序号/源校验仍必需。
- 新增隔离ROS读取器工厂，仅订阅VIO与tracking，每100ms审计源图，不建ROS发布器；当前真实模式拒绝，工厂与发送器域必须一致且隔离。只提供工厂和协议组件，没有新CLI/服务启动，不借此绕过真实生命周期和会话绑定。
- 7项新增测试覆盖发送器→真实匿名OS管道→接收器的载荷/序号、来源替换后不发送待发数据、积压过期、超大载荷、发现超时，以及实际工厂在模拟ROS环境下只有两路订阅/非法域拒绝。601条数据/120秒为模拟时钟推进，非实际连续运行；编解码替身不证明ROS CDR版本兼容。最终resident相关33项离线全通过，1.062秒。
- 尚需服务级容器进程启动/关闭回执、同一会话/时间命名空间核对、真实输入外参与tracking源链核验、生产任务唯一EV绑定，以及有授权的真实联调。原实飞入口和自启服务未改，未运行Docker读取器、真实EV或运动。完整比赛目标未完成，下一步补进程生命周期而非重复已有匿名管道测试。

## 2026-10-01 常驻VIO管道接收端完成并接入节点回调测试

- 新增 `resident_vio_transport.py`：ResidentVioPort提供上一轮ScopedVioInput的底层接收实现，校验会话、域、连续序号、管道年龄、两话题白名单及唯一稳定来源；CDR在接收时反序列化，不重写视觉源时间。图信息超过500ms、来源消失/替换、协议错误锁定故障。这里的端点身份仍是周期图信息，不是消息级认证；宿主机/容器单调时间比较要求相同内核时间命名空间，实际部署仍待核对。
- ResidentVioPipe使用调用者提供的非阻塞文件描述符，32768字节缓存上限、每轮最多16次处理；支持分包、EOF/异常关闭且不自动重连。它不启动Docker/ROS/设备，不打开串口；生命周期由服务控制，不采用50秒台架倒计时，也不循环短测脚本。服务仍需负责文件描述符关闭和子进程清理。
- 8项新协议测试通过，含真实匿名OS管道分包/EOF/缓冲上限；模拟时钟推进120秒、601条数据证明逻辑无短测时限，不代表120秒真实运行。另新增receiver→实际resident节点回调联合测试，90条合成输入到达、唯一EV出口、图停流关闭；ROS端点和CDR解码仍是替身。最终resident全部26项离线通过，0.898秒，无硬件。
- 当前只完成持续传输的接收半段。容器端常驻生产者、双端退出管理、服务入口/会话唯一绑定及真实任务接入仍未交付；不把它写成开机持续EV已部署。无新EV实测、参数修改、传感器重启、解锁或运动。下一步补容器生产者与生命周期，再接生产服务；比赛全目标保持未完成。

## 2026-10-01 常驻EV输入可分域；新增只读双话题接口，未启用真实输出

- 上轮以状态整理为主；本轮修改 `resident_ev_node.create_node` 增加独立perception_node输入，视觉订阅和视觉来源图审计走输入端，status/land/flags及唯一EV发布仍走主端。先验证两个不同隔离域，真实输出保持拒绝；既有同域测试兼容。
- 新增 `resident_vio_input.ScopedVioInput`，只允许VIO和tracking的订阅/图查询，无发布方法、无导航目标、无PX4写接口。调用者拥有并驱动底层输入；没有创建第二个EV出口，也没有自动启动传感器。
- 首版尝试复用ScopedPerceptionNode时，2项新测试因其导入整套任务/H/OpenCV导致cv2.gapi.wip.draw.Text缺失而失败。未修改系统OpenCV或掩盖异常，改用定位专用轻量端口去掉不必要依赖。修改后节点10项、核心7项离线全部通过，分别0.178秒/0.732秒。新增覆盖分域订阅/来源身份、源替换、输入异常锁定且不自动恢复、非法/相同域拒绝及无发布/越界读取能力。
- 这是实际工厂与模拟端点的接口测试，不是跨进程DDS或硬件证据，不能继承上一版实际DDS测试为本版已实测。未运行新硬件试验、未改飞控参数、未启用服务或解锁。后续明确缺口：持续跨容器reader/生命周期、真实会话唯一所有者绑定、生产任务对常驻EV的接入；不通过重复短时EV脚本拼接常驻服务。

## 2026-10-01 实际ROS隔离EV交接已通过；收敛到实机接入缺口

- 已核对 `evidence/resident_ev_dds_b933aaa643fb4731b07a680f88e65b9b/report.json`：passed=true，4.246秒，隔离域187，resident发送80、任务接收50、合成飞控接收80；EV发布者唯一，任务EV发布器0，停止后任务锁定 external EV receipt stale，commands=[]、armed=false。这是同进程三节点实际ROS消息联测，不是跨容器、PX4 SITL、实际导航全链或真机飞行；不声称80/80到达任务。此前“尚无实际ROS消息证据”已由此结果更新，真实服务仍未部署。
- 本轮只读核对代码与报告，未新增硬件采集、EV注入、解锁、运动、参数修改或服务启动；未重复已完成隔离测试。生产 `flight_runtime.create_node` 仍明确拒绝真实external_ev模式，resident工厂也仍仅隔离可用。不能把这两处保护删除就称为部署完成。
- 已读取PX4官方main文档源 https://raw.githubusercontent.com/PX4/PX4-user_guide/main/en/flight_modes/offboard.md （对应 https://docs.px4.io/main/en/flight_modes/offboard.html ）。核对：ROS 2 OffboardControlMode心跳需持续高于2Hz，切入前至少1秒；TrajectorySetpoint给位置/速度等目标，使用NED约定；失联行为由COM_OF_LOSS_T/COM_OBL_RC_ACT等控制。main文档不替代本机固件版本/消息定义核验。
- 更正遥控器判断：官方文档说明Offboard下摇杆通常不直接控制、可切换模式，因此用户“摇杆不能操纵”不能单独证明接收机故障，也不应要求在Offboard用摇杆控制才能继续开发。现有信息只确认能切模式/急停，未确认切出后的具体模式与受控退出效果；急停不等于可控降落。参数文件快照COM_RC_IN_MODE=0、COM_OF_LOSS_T=1、COM_OBL_RC_ACT=4，仅为磁盘快照，不声称当前飞控值或已经验证失联动作。
- 固定本次主线：Jetson运行现有NavFn A*＋路径引导APF与H识别，通过现有任务控制器/TrajectorySetpoint交给PX4；不是把A*/APF刷入飞控。保留“中心AGL1.00m、悬停3秒、前进2.5m再返回、H对准后AUTO_LAND”，不新增持续视觉下降、重新标定或完整比赛任务。
- 未完成的工程交付明确为：域176真实VIO到域0唯一常驻EV服务的持续输入/会话绑定；真实任务只发送控制、不再拥有EV出口；任务退出和AUTO_LAND期间定位服务继续运行。之后才是同一新会话持续融合/同步对齐的实测确认。历史361ms延迟在后续只读窗口未复现，不能说当前必坏，也不能说已修复。不以“收到话题”替代真实融合和坐标对齐。
- H完整可见不是地面起飞条件；落点漂移/实际避障效果属于受控试飞要测的结果，不要求先完成飞行验收才允许首次测试。持续有效定位、正确控制坐标和独立受控退出仍是试飞前提。目前尚未完成实机接入，不宣称能立即飞或保证完成时间。

## 2026-10-01 常驻EV隔离ROS发布端已实现，内存接口联合测试通过

- 新增resident_ev_node.create_node，当前仅isolated且域180～187可创建，真实模式/域0在创建端点前拒绝。只发布测试命名空间的VehicleOdometry，不包含模式/解锁/运动/参数/舵机接口或服务启动CLI。复用ResidentEvCore与原消息转换。
- 发布端验证5路源唯一/身份稳定、EV出口唯一、遥测原始时间单调且年龄<=500ms、status/land/flags实际接收新鲜度、地面起始、跟踪/位姿时间与质量。实际publish成功后才dispatched；异常关闭输出。启动发现10秒、故障不重连/不自动重建会话。正常任务落地不自动关闭定位，满足定位与航段生命周期分离的方向。
- 新增5项工厂/回调级内存联合测试：resident→真实task回调→ExternalEvEvidence有实际消息对象传递、仅1个EV发布者/任务3控制出口；关闭EV后任务锁定；模拟解锁落地继续定位；发送异常不确认；重复EV与真实域拒绝。测试使用模拟ROS端点，无DDS网络、PX4或真实传感器，未运行完整航线控制状态机。
- resident组12项（7核心+5接口）通过0.812秒；external组12项通过0.011秒；diff --check通过。当前常驻节点文件尚无实际ROS跨进程运行证据，仍未部署到systemd，原入口/生产服务未改动，无实机测试。
- 下一步为真实ROS消息/隔离域联测，验证源发现、唯一出口和消息类型，再处理真实双域感知入口/服务会话绑定。开机持续融合、真实避障/H降落及窄门/投递尚未完成，不把内存测试说成DDS或飞行通过。

## 2026-10-01 外部EV隔离运行器接线完成；真实服务尚未启用

- flight_runtime在显式绑定外部EV且isolated时，不创建vehicle_visual_odometry发布器，只创建3个合成控制出口，改订阅外部EV输出话题；默认/真实原路径不变。真实外部模式、bench混用仍在ROS创建前拒绝，不接入生产许可。
- 消息接收复用ExternalEvEvidence，并检查FRD帧、有限位置/单位四元数、NaN速度合同及reset变化；图中必须有一个且符合绑定GID的EV端点。仅接收新有效消息更新last_ev_at，本地VIO候选不发布。
- 核对本机/opt/ros/humble的executors.py确认_take_subscription取msg_info[0]，_execute_subscription只传msg，不支持所假设的MessageInfo双参数回调。已在硬件运行前改为单参数回调内查询唯一图端点，不声称逐消息GID认证；新增查询耗时需后续测量，真实模式仍禁用。
- test_external_ev_routing调用实际工厂/回调但ROS端点是模拟对象，验证仅3控制发布者、外部订阅、有效接收、错误帧/来源拒绝、reset变化和真实入口拒绝；与7项证据测试合计12项离线通过。它不是实际DDS跨进程验证。
- 最终Humble兼容版本的277项原任务回归task_regression_20261001_072510通过（20.843060秒、0失败/错误/跳过）；早一版072312也通过，但不拿它证明后来兼容修正。12项新增测试单独运行，不在277报告内。无传感器/PX4访问、无新服务启用、无飞行操作。
- 剩余：常驻EV实际ROS发布端、唯一会话/来源绑定建立、隔离联合运行、再按明确授权接入实机；这不是已完成开机持续融合或真机避障。完整比赛目标未完成。

## 2026-10-01 任务核心接入外部EV接收证据，实际ROS分离仍待接线

- 新增external_ev_evidence.ExternalEvEvidence：绑定会话与预期发布者标识，检查实际EV消息发布/采样时间、质量及重复/重置/源间断；只有新消息续期，接收停流锁定，不自动重绑。该接收证据不等于PX4 EKF融合；融合标志仍由原控制逻辑独立要求。
- FlightRuntimeCore增加单次bind_external_ev和external_ev_received；绑定后本地VIO仍进行原适配器检查，但不返回可发布EV候选，也不据候选刷新last_ev_at。普通和LiveFlightCore的健康判断改用统一ev_receipt_recent；未绑定时保持原路径。
- 尚未完成实际ROS VehicleOdometry订阅、源身份/会话登记与发布出口拆分，所以flight_runtime.create_node在检测到外部EV绑定时明确拒绝创建节点，防止误起双发布者。这是开发阶段保护，不是生产已接通；默认实飞配置未改，未启动任何EV服务或硬件测试。
- 7项新离线测试通过，覆盖本地候选不续期、外部新消息续期、重复不续期/停流锁定、来源变化、未来戳/低质量、源间断、重绑定拒绝和未接通ROS入口拒绝。既有整套277项任务回归通过，0失败/0错误/0跳过、21.763733秒；证据task_regression_20261001_072001/report.json。此277套不包含单独运行的7项新增测试，不能把它混写成284项同一报告。
- 下一步为常驻定位服务ROS实现及任务控制唯一出口接线、离线联合验证；新核心不会自动消除历史时效故障或授予飞行权限。避障/H降落、窄门、投递全目标仍未完成。

## 2026-10-01 唯一常驻EV架构开始实现：纯逻辑核心通过7项离线测试，未接实机

- 上一轮状态答复未改变软件；本轮按用户已授权的唯一EV服务/任务控制分离推进。现有flight_runtime会创建EV发布器，FlightRuntimeCore.pose以候选生成刷新last_ev_at；拆分时不能把任务本地生成候选冒充外部EV实际送达，不直接加第二个发布器。
- 新增`scripts/resident_ev_core.py`，复用FlightEvAdapter的坐标/质量/姿态突变与停流保护，区分生成候选和调用方实际publish后的dispatched确认。只允许落地未解锁建立会话；解锁前需实际送达记录；记录唯一EV/source身份、来源变化锁定、250ms年龄上限和未来50ms限制、禁止未确认发送后继续生成、故障不自动恢复。无ROS导入、发布器、模式/运动/参数API，也未修改原60秒入口。
- 7项纯离线测试通过（0.723秒）：模拟超过60秒含地面→解锁→落地仍持续定位、空中启动拒绝、只有候选没有发送不得健康、来源改变锁定、停流不恢复、未来戳拒绝、重复EV所有者拒绝。模拟时钟推进不代表真实连续运行时长或飞行通过。
- 这是拆分工作的第一层，不是已部署常驻服务；真实订阅/身份获取/遥测源时间校验、发布与发送确认、任务侧外部EV证据接入及唯一出口合并验证仍待实现。现有实飞入口未改，service未加EV，未执行硬件测试。原未来戳/361ms历史异常仍未解释，不能依据新核心测试放行实飞。

## 2026-10-01 新授权30秒只读分段诊断完成：本窗口正常，361ms故障未复现

- 用户授权一次<=30秒仅VIO/跟踪订阅，已执行一次；证据`evidence/ev_readonly_timing_77a302a7804d4d8ea21c1ab34b9ae54d/report.json`及packets.json。总长24.311250秒，exit0、complete=true，reader exit0/关闭confirmed=true。无图像额外订阅、无像素保存、无EV/PX4/运动/参数/重启、无自动重试。
- 主机记录pose622/跟踪621；pose源跨度20.704938秒，最大源间隔33.355ms、最大reader接收间隔53.802ms、最大主机源年龄39.172ms、最大reader→host6.077ms。跟踪最大源年龄44.958ms、接收间隔56.197ms、状态仅1，无非递增戳/源间断>300ms。
- reader计数enqueued/sent1244（主机结束边界保留1243，不冒称逐包完整）；无pending/would_block，队列最长驻留8.103ms。分段最大stop_file27.826ms、spin_once45.123ms、pipe_pump8.050ms、graph_pose17.180ms、graph_tracking4.851ms、TF1.156ms、audit19.548ms；慢调用8次，无事件截断。各段可嵌套，不能累加当总延迟。
- 本窗口延迟事件列表为空，支持单独只读链在此窗口内正常，不能倒推此前361ms故障不存在，也不能确认EV联调负载下稳定。无证据支持删除图/TF安全检查或改阈值。历史异常仍保留待解释；配对与真实融合未在本轮验证。
- 下一步诊断需区分仅订阅与EV联调同时存在PX4遥测/主机图审计时的行为；不要再次原样重复本窗口。当前没有进行新EV授权外的动作或实飞，常驻EV/任务出口拆分及比赛飞行仍未完成。

## 2026-10-01 只读延迟诊断入口复用完成，未新增硬件尝试

- 复用现有`disarmed_ev_observe.py`而非新建重复采集链，把reader关闭回执中的分段耗时与pose接收间断/源年龄进行离线关联。新增reader_delay_analysis，列出时间段重叠的慢调用，但不把重叠当因果证明；无耗时记录明确标未知，64事件缓存可能截断的限制保留。
- 同时修正旧只读入口无stereo选项时即使未收到pose/tracking也可能complete=true的问题；现在两种模式都要求各自全部必需流至少有数据。没有放宽任何原EV/时限规则，也没有把“收到流”当实时性通过。
- 8项分段记录/关联纯离线测试通过，包括360ms延迟、慢graph重叠、无记录未知、不重叠不归因、无数据不得通过、正常连续数据；默认描述入口执行无ROS访问。既有EV回归另行执行，结果以工具输出为准。没有启动传感器或真实EV。
- 已向用户请求一次<=30秒仅现有VIO/状态订阅诊断（不增加图像负载、不重启/参数/EV/控制/自动重试），尚未执行；前次硬件授权不复用。下一步用分段证据决定修正位置，目标仍是完成真实避障/H降落及后续比赛任务，未宣称完成。

## 2026-10-01 自动续办：读取器分段耗时观测补齐，未新增硬件测试

- 上一目标轮属于有效进展：新实测将365ms延迟定位到容器reader收到数据之前/之时，管道约3.45ms，时基差很小。仍按避障→H降落→窄门→识别投递完整目标推进，不宣称子项通过即完成比赛。
- 源码确认reader单线程spin_once同时承担订阅、每0.1秒来源图检查及TF查询，每循环/回调还读取停止文件。没有证据判定其中任一就是361ms间断根因，未删除这些检查。
- 新增reader_timing_evidence.ReaderTiming并接入stop_file、graph_pose/tracking（可选图像）、extrinsic_lookup、audit、spin_once和pipe_pump。统计次数/最大耗时及最近64个>=20ms慢调用，包含单调时间起止；嵌套耗时不可相加。数据保存在reader关闭回执及主报告reader_cleanup中。仅诊断，不修改线程、QoS、源戳、停止语义、门槛或截止时间。
- 3项新纯离线正反对照测试覆盖360ms模拟慢调用、异常透传与记录、快调用/有界事件缓存；既有66项EV测试全部通过（2.592秒），共69项。未采集新真实数据、未启动EV、未重启、未发运动/改参数，前次一次性授权不复用。
- 下一步需要有界仅订阅证据把慢audit/调度与源数据延迟区分，不原样反复EV；若本地阶段正常但源年龄仍高，再定位上游VIO/图像。常驻EV/任务拆分及真机比赛任务仍未完成，目标保持未完成。

## 2026-10-01 新授权60秒EV配对复测：两端时钟记录有效，VIO陈旧365ms退出

- 用户重新授权一次<=60秒未解锁EV＋同步配对，已执行一次。证据`evidence/disarmed_ev_a223eeacba6e4649930f64800f62c7ef/report.json`及supervisor.json。总长5.702240秒exit1、passed=false、error=null；fault为invalid sensor packet: stale/future/invalid sensor source time。sent=0/output_count=0，配对尝试0/匹配0，融合标志4条无全融合区间。没有自动重试、解锁、运动或参数修改。
- 此次不是nanoseconds软件异常，也不是前次未来1.955秒。第23包pose源年龄host365.355465ms、reader360.334734ms，超过原250ms门槛；reader receipt到host receipt约3.454ms。首包相应年龄35.538/41.702ms、管道6.325ms。12条pose的最后一次reader接收间隔361.156ms，但两源戳间隔33.339ms，提示数据到reader回调时已滞后；尚不能区分上游图像/VIO计算与reader调度/DDS排队。
- 双端同步观测经单调时钟读数中点补偿后，首包ROS时基差约0.019ms、末包约0.004ms；各端ROS-system差绝对值均小于0.205ms。仅在本窗口支持两端时基相符、不支持把本次365ms归因于跨端固定时钟偏移，不解释历史未来戳根因。不改时间戳或阈值。
- reader退出码0、关闭回执confirmed=true；进程查询无本次EV入口/reader残留。新增两端时钟和异常证据保存功能本次有效。授权已使用，下一步应围绕源接收延迟/调度排队定位而非原样重试EV。真实配对/融合持续性、常驻EV与任务出口拆分、避障/H降落实飞仍未完成。

## 2026-10-01 两端时钟记录已补；新有界复测遇配对观测代码异常，已离线修正未重试

- 用户要求补读取器/主机时钟并有界配对复测；按此前<=60秒未解锁EV范围执行一次。新增ev_clock_evidence.snapshot记录ROS/system纳秒时钟及单调时间前后界、读数跨度；reader_clock/host_clock附原始packet，未修正源戳或放宽阈值。事前2项记录测试+65项EV离线通过，但没有覆盖实际dispatch首次配对路径，是测试覆盖缺口。
- 本次证据`evidence/disarmed_ev_2a3fe59211ab4427b439359551cf3130/report.json`及supervisor.json：总长4.892797秒exit1、passed=false，error为TypeError(int object is not callable)。实际dispatch配对分支误写`.nanoseconds()`而不是整数属性`.nanoseconds`，是此前新增观测代码缺陷，不是PX4/相机故障。首个EV候选进入此分支时、publish之前抛异常；trace61条仅1个EV候选，配对尝试1/匹配0。旧异常路径漏存sent及sensor_packets字段，不能把缺失字段当独立实测0或声称两端对照分析完成。
- 保留下来的61条vision_trace源年龄18.434～61.796ms，未复现未来1.955秒；仅约1秒短窗，不外推稳定性。PX4遥测status7/flags4/land3/rc159；4个融合标志观测无全融合区间。reader exit0/关闭confirmed=true，未自动重试、未解锁或发运动。
- 已修正属性调用；同时把sent/fault/sensor_packets/paired_px4/output_count保存放到finally，以免后续异常丢失原始时钟证据。新增实际Observer.dispatch假ROS回归，首次有效候选使用整数nanoseconds、成功记录同源戳配对且禁发输出。最终66项EV离线+2项时钟记录测试全部通过（66项2.561秒）；本轮没有再次硬件执行。
- 自启EV仍未实现，任务控制出口仍未拆分，当前未完成真实配对或避障/H降落测试。后续硬件重试须新的一次有界授权，不以修正软件自动重复消耗上次授权。

## 2026-10-01 单次20秒源时间戳检查完成：本窗口正常，历史未来戳未复现

- 用户明确授权一次<=20秒仅订阅现有左右红外/VIO。新增`tests/source_clock_observer.py`只创建三个订阅，无发布器、无像素保存，以共享单调时钟18秒截止/外层19秒预算运行；证据`evidence/source_clock_2d542f5db7ea4091bcf6e20309eb94fc/report.json`及supervisor.json。实际17.295638秒、exit0、completed=true，无stderr；收尾进程查询无观测器残留。无相机/Agent重启、无参数/EV/控制、无自动重试。
- infra1 458条：源年龄min/median/max为10.089/12.424/62.746ms，最长接收间隔81.159ms；infra2 458条：11.455/14.396/64.655ms，最长78.890ms；VIO 440条：15.173/19.254/70.116ms，最长64.608ms。三路future_over_50ms均0。
- 440/440条VIO源戳分别与左右红外源戳精确一致，支持当前窗口内VIO保留对应输入时间戳，不支持“当前VIO固定加了1.955秒”。三路统计来自同一个容器内ROS节点的当前时间，不能单独证明主机侧ROS时钟或PX4时间配对；观测增加了图像反序列化负载但不保存像素。
- 上次主机EV入口源年龄-1.955秒是历史实测，本次未复现，不能删除或称已修复。可能是瞬态时钟映射/校时或跨进程时间问题，尚未定位。未改源戳、未放宽阈值；未注入EV，故无新增融合/配对结果。下一步若复测EV需新的有界授权，并在主机与读取器同时记录ROS时间以区分时间基准，不能把本次短窗正常自动转为飞行放行。

## 2026-10-01 校时日志与相机实时参数核对（只读，无EV复测）

- 用户提供本机本次启动timesyncd日志：从未设置/回退时钟恢复至05:26:33；05:58:13首次同步111.230.189.174；06:01:39超时，06:01:40同步84.16.67.12。日志未给出各次调整量，不能断言发生了与VIO超前1.955秒等量的跳变。
- 当前ps显示相机PID6778启动时间06:01:54，晚于日志最后一次同步。该墙钟时间仅为系统当前推算显示，不是独立校时轨迹；不能继续声称当前相机必定在首次校时前已启动。
- 新的一次最多12秒只读参数查询成功：实际运行相机`depth_module.global_time_enabled=true`、`use_sim_time=false`。这不是每帧实际timestamp_domain的测量，但说明此前本地源码HARDWARE_CLOCK固定基准路径只是候选，不能认定它是当前运行路径。未改相机参数、未重启、未采集新图像、未EV注入。
- 已启用sensor boot unit的脚本有NTPSynchronized=yes检查，未同步会退出而非盲目启动；time-sync.target排序本身不保证等待同步。该条件只核验启动瞬间，不保证之后时钟不调整、帧戳正确或冷启动总能成功。此轮未修改已启用服务。
- VIO未来戳根因尚未确定；下一条有用证据是同一短窗口内对照左右红外原始header、VIO源戳和同节点ROS当前时间，区分相机源映射与VIO输出，而不是重复EV注入或强行减去1.955秒。

## 2026-10-01 用户已启用两项系统自启；仍未完成常驻EV或实飞

- 用户在本地sudo执行enable两个绝对路径service，反馈创建/etc/systemd/system及multi-user.target.wants链接，两项is-enabled均enabled。本轮只读systemctl show独立确认两项UnitFileState=enabled、ActiveState=inactive、SubState=dead；没有--now，没有启动/重启/冷启动验收。现有手工感知与Agent进程不能算作这两个服务已成功启动。
- 上一轮VIO未来戳1.955秒的阻断尚未解除。本地RealSense源码setBaseTime/frameSystemTimeSec在HARDWARE_CLOCK域使用初始_ros_time_base加硬件增量，提示需排查主机运行中校时/源时间映射；当前运行二进制、实际时间域与校时事件尚未确认，不能将此机制当作已测唯一根因。sudo -n读取timesyncd日志仍要求密码，无日志证据。
- 本轮仅只读诊断和文档更新，无新EV复测/传感器重启/飞行命令。开机常驻EV和任务唯一输出分离仍未实现，启用DDS不等于EKF融合。用户要求立即实飞未执行；当前源时钟故障及此前摇杆接管问题没有因自启安装而解决。

## 2026-10-01 新60秒复测：发现交接通过本次观测，VIO未来时间戳拒绝；自启仅完成候选文件

- 用户授权新的单次<=60秒未解锁EV＋同步配对，已执行，证据`evidence/disarmed_ev_040307ef88be4794933f74389490774f/report.json`及supervisor.json。总长4.084968秒、exit1、passed=false，EV0、配对尝试0、匹配0，无自动重试；reader exit0/关闭confirmed=true，无解锁、模式、运动或参数指令。
- `discovery_handoff=same_node_deferred_endpoints`，最后审计五个PX4来源均唯一且ready；实际收到local275、status5、flags3、land3、RC125。上次Observer来源全0的交接故障本轮未复现，不外推长期稳定性。
- 当前阻断为首个pose `source_age_s=-1.955134787`，即源时间比主机ROS时间超前约1.955秒，而非包积压陈旧。reader_receipt2863.011771154→host_receipt2863.017077987，约5.307ms。源戳1790808312937549316，报错`invalid sensor packet: stale/future/invalid sensor source time`。没有减常数、改消息时间戳或放宽时效来通过。3个融合观测均未形成全融合区间；不能称本次EV融合成功。
- 主机timedatectl显示NTP active、System clock synchronized yes；这不能证明相机映射无漂移。系统校时日志普通账户无权读取，sudo -n也要求密码；尚未定位是系统校时跳变、相机时间映射还是其它源时钟问题。不重启相机碰运气。
- 用户明确新架构：每次开机未解锁、静止在地面，开机位置作为任务原点，不空中重启Jetson；一个常驻服务统一提供EV，任务只负责运动，故障锁定且不自动恢复飞行。以上已记录，但常驻EV及任务出口拆分尚未实现/验证；现有飞行入口仍会发布EV，绝不能与另一个EV服务并行运行。
- 已新增`systemd/robocup-sensors-boot.service`、`robocup-dds-boot.service`及对应入口，**仅候选文件，未安装/enable/start**。前者复用有界90秒静止地面感知恢复并保留成功链（D435i视觉-only、下视H、雷达/TF、nvblox、A*/APF，无EV/目标）；后者前台独占Telem1 921600 Agent，无飞控指令。两者Restart=no。容器当前running/restart-policy=no，不重启现有链部署候选。
- 5项自启纯离线测试通过（0.103秒），bash -n通过，systemd-analyze verify退出0；输出包含系统既有snapd/NVIDIA/Jupyter单位警告，不是新增单位故障。感知服务是oneshot，active(exited)仅表示启动曾成功，不证明持续健康，停止单位不会停止Docker内已保留链；详见`systemd/BOOT_STATUS.md`。
- 系统服务安装需要本地管理员认证（无免密sudo；Linger=no，不能把用户登录自启当作开机自启）。用户确认可在本地终端执行安装命令。安装传感器/DDS两层仍不等于常驻EV、持续融合、任务控制交接或实飞完成。禁止使用--now重复启动当前Agent/感知链。本轮未完成自动避障/H降落实飞。

## 2026-10-01 发现交接软件修正完成：65项EV纯离线回归通过，未实机重试

- 按用户“修复并离线验证、不原样重复测试”执行。旧worker销毁临时预发现节点后新建Observer，前节点ready不能传递为后节点已发现；这构成可消除的节点生命周期缺口。上次实机故障是否完全由该缺口造成尚未实测确认，不写成已证实的唯一根因。
- `scripts/disarmed_ev_session.py`改为先创建无订阅、无EV发布器的休眠Observer，将同一节点借给wait_px4_graph；发现成功后activate(gate)才创建5项订阅和唯一EV发布器。预发现函数不销毁借入节点，最终仍由worker统一清理；旧独立探测调用仍自行创建/销毁。激活单次有效，关闭后不得重复激活。
- 原预发现预算、活动50秒/监督55秒/授权60秒、后续2秒来源发现保护、状态/消息时效、唯一发布者及未解锁约束不变。未修改DDS配置或PX4参数；预发现ready仍不等于已收到有效遥测，实际输出仍受原门控限制。
- 新增`tests/test_disarmed_ev_handoff.py`6项测试，调用真实Observer工厂但替换ROS为节点局部假图：同节点保留发现；旧重建序列丢失发现的负对照；超时无端点且由所有者清理；竞争发布者不激活；单次激活/关闭不重开；旧立即创建接口兼容。它验证生命周期逻辑，不是Fast DDS网络仿真或真实恢复证据。
- 首轮65项中新增6项因测试夹具使用Python3.11的enterContext在本机Python3.10报错；改为ExitStack，不改变断言或生产判定。最终`python3 -m unittest discover -s /home/cfly/ros2_ws/robocup_nav/tests -p 'test_disarmed_ev*.py'`运行65项、2.549秒、全部通过；git diff --check通过。
- 本轮无真实ROS节点、无EV注入、无相机/Agent重启、无解锁或运动、无硬件重试。下次若获新的单次授权，只需验证修正入口的遥测持续接收与EV期间同步配对；当前不能称配对已通过或飞行已获放行。

## 2026-10-01 新授权60秒EV＋同步配对检查：启动发现失败，已结束

- 用户明确授权一次<=60秒未解锁EV输入＋同步配对，不发运动、不改参数、不自动重试。执行证据：`evidence/disarmed_ev_9856b8fb747e4badbf82a9aea143ccd7/report.json`与supervisor.json；实际总长3.335秒、exit1、passed=false。EV发送0，配对尝试0，融合样本0；不是已观察到配对偏差超限或融合失败。
- 事前仅为disarmed_ev_session增加同步观测记录：沿用closest_local原判定，在可生成EV样本时记录VIO/PX4近邻、双方位姿、时间差和完整五项reset；原三项reset字段保留兼容。无新增控制发布者、不改输出资格。4项配对和23项EV入口/会话离线测试通过，不能替代本次硬件结果。
- 预发现节点报告ready，但之后实际Observer在2.084秒时发现五个PX4源全部为0，触发`discovery_timeout: required source not discovered within 2 seconds`；无重复/竞争发布者记录。真实数据未进入观测节点。此为发现阶段前后不一致，尚未定位是节点生命周期/发现时序还是链路变化，不归因于相机、位置精度或特定根因。
- EV发布器关闭，容器读取器exit0且关闭回执confirmed=true，未重启相机/Agent、未解锁、未发运动/舵机、未改参数或放行标志；没有自动重试。当前授权的一次尝试已使用。后续应先修复并隔离验证发现交接问题，不再重复相同入口碰运气，也不放宽时间限制来获取通过。

## 2026-10-01 配对诊断更正与遥控接管条件变化（本轮未启动硬件测试）

- 用户最新澄清：遥控器不能用摇杆操纵飞行，但可以切模式或触发急停。保留这项用户报告；此前“可接管”不能继续当作已验证条件。能收到RC/切模式不证明四轴摇杆映射、模式切换被飞控接受或人工飞行接管有效；空中断电急停也不等于可控降落。需在未解锁状态核对QGC四轴输入及实际模式，不能要求用起飞试错来诊断。
- 更正配对结论：`task_pose_pairing.closest_local`同时要求源时间年龄<=250ms、接收年龄<=250ms、XY/Z有效、未解锁且落地、reset一致及VIO/PX4时间差<=50ms。`matched=false`不是单独的时钟失同步证据。06:24只读报告未提供各条件的拒绝原因，不能确定具体根因。
- 06:19有界EV成功后已停止输出；之后无EV的只读检查中水平定位可能失效并拒绝配对，这是待证假设，不是已测结论。不要因泛化报错重复宣称D435i损坏或时钟同步失败。
- 配对应在同一新会话、持续EV输入且PX4位置有效的未解锁落地期间完成：按原始消息时间找近邻，记录时间差、全部五项reset及ENU/NED和机体外参变换，生成本会话对齐；重启/重置后旧对齐不可复用。`LiveFlightCore.ev_allowed`的WAIT路径不以任务alignment或XY有效为发送前提（仍有状态、模式、所有权等健康条件），故源码上不存在必然的“先配对才可开始EV”依赖。
- 本轮只读源码与既有报告，未重启传感器/DDS、未注入EV、未解锁、未发运动、未改参数或放行标志。原有离线与静止融合证据保留；落点误差可在受控试飞中测量，不冒充已经验收。当前不能承诺仅配对成功即可直接完成2.5m自主往返。

## 2026-10-01 06:24 正式任务输入通信修正并真实收数确认，未执行起飞

- 用户要求开始真机测试。审查发现live_flight_runtime的task分支仍残留旧同进程双域配置（host域0但localhost1/感知SHM配置），而现在感知已由独立容器管道承担。新增px4_runtime_transport.configure统一host为域0、localhost0、fastdds_bridge.xml，与成功EV-only/只读飞控链一致；容器感知仍域176、localhost1、SHM16MiB。仅新进程配置，不重启Agent或修改飞控参数。此前无DDS输入时Agent也未运行，不能把该历史故障单独归因于通信配置。
- 正式运行器及container-inputs只读路径使用该配置；277项纯离线回归task_regression_20261001_062222全部通过，22.566秒。合成全链使用域182/183的旧结果不直接证明这个真实域配置，故单次<=30秒只读核对。
- 真实证据`evidence/task_readonly_fabe85e2109f43d6b93dfc7de3a19ac4/report.json`：20.477秒、input_delivery_observed=true、无error/task_fault、关闭正常。收到local1651、RC748、派生开关743、status32、land17、flags17、VIO450；输出{}、armed_on_real_input=false。未EV注入、解锁、发目标或运动，无自动重试。
- 仍不放行自主往返：该检查的当前同reset VIO/PX4 50ms配对matched=false，alignment未审核；真实全高度地图/制动、H外参审核及总落点误差仍未完成。已有几何/场地确认不等于飞行期间动态误差证明。不能把合成reviewed=true迁移到生产配置；live_flight_enabled仍false。
- 更正此前“离线只差一次完整链”可能造成的预期：离线交接已通过，但完整自主实飞还存在上述实体条件，不能承诺只需再授权就起飞。不继续重复静止EV/相同离线链；下一现场阶段需明确受控动态/悬停观测与接管范围，而不是直接发2.5m自主航段。

## 2026-10-01 06:19 新会话EV-only通过：静止视觉融合样本覆盖44.308秒

- 用户授权一次总长<=60秒新会话EV-only，已执行并消耗；证据`evidence/disarmed_ev_f62beb4a08944ec88a2563320d6125a8/report.json`及supervisor.json。总长49.780秒、exit0、passed=true（仅EV送达范围），flight_ready仍false。保持装桨未解锁/静止，无模式、解锁、运动、舵机或参数指令，无自动重试。
- 向PX4发送1337条外部视觉位姿，fault/telemetry_receipt_fault均null。融合起初关闭，随后位置/航向/高度均开启；50个有效标志样本中46个同时融合，连续采样跨度44.308123秒、最大样本接收间隔1.014099秒、无issues。统计不把最后一个样本外推到退出，不宣称覆盖飞行动态。
- 报告保留的末尾2000条PX4本地位置样本xy_valid/z_valid全部true，记录内的三项reset值稳定[15,4,3]；这是启动融合后的当前记录，不可用此前未融合的[14,3,2,...]构造旧会话对齐。未完成移动方向/尺度与完整五项reset配对的实飞审查，不把静止数据冒充动态验收。
- EV专用节点和容器读取器已停止，reader_exit0、关闭回执confirmed=true；进程检查无本次EV会话残留。DDS Agent PID17755继续运行，感知链未重启。停止EV是本次有界授权的正常收尾，不能称后台仍保持融合；不要未经授权续发或循环重试。
- 同样的静止EV验证无需再重复。下一步围绕真实任务当前输入/地图覆盖、H方向与落点总误差、单航程持续EV/控制交接及本地操作员放行处理；已确认的几何和内参沿用，生产未审核项不能由本次静止融合置真。真机避障与H区降落尚未验收，窄门/识别投递仍未完成。

## 2026-10-01 06:16 DDS只读确认完成：通信正常，当前无EV输入/水平定位未有效

- 用户授权恢复/dev/ttyTHS1、921600，并一次<=30秒只读确认、成功保留DDS。执行前发现MicroXRCEAgent PID17755已经运行，域0、ROS_LOCALHOST_ONLY=0、fastdds_bridge.xml；未重复启动/重启，也未声称该既有进程由本轮启动。
- 复用Agent执行一次preflight_observer --duration 15，外层29秒超时且最多额外1秒退出；证据`evidence/preflight_readonly_20261001_061559/report.json`，观测15.000秒。收到local1484、rc668、status29、land15、flags15，source_fault=null、duplicate0；飞机始终未解锁且landed=true，QGC连接true、rc_valid=true、failsafe=false。DDS收数确认成功；进程保留运行。
- 飞行就绪判定仍false：input_publishers={}，没有当前EV发布者；ev_position/height/yaw均false，xy_valid/v_xy_valid=false，z_valid=true、baro_height=true、range_height=false，preflight_ok=false。此结果不是DDS掉线，也不是本次验证证明D435i损坏；本轮按范围完全未注入EV。
- 当前PX4 reset counters为[14,3,2,13,2]，不能复用旧会话对齐。下一步需经授权使用新鲜配对恢复EV-only并确认持续融合；不修改参数、不解锁、不发送运动指令。真实任务H/地图/制动的未审证据未被这次通信成功覆盖，实飞许可仍关闭。

## 2026-10-01 06:04 实机入口只读检查：感知有数据，重启后DDS Agent未运行

- 用户确认当前装桨、未解锁、未挂货、遥控接管人员在场、QGC遥测正常、区域无人；H起点、双箱净距/前距1.2m及前6.2/后1.8/左右2.5m边界不变。本次仍中心AGL1.00m、悬停3秒、前进2.5m返回，H对准后原LAND（不持续视觉下降纠偏）。
- 执行一次<=30秒现有真实输入只读检查，证据`evidence/task_readonly_3b8cca7aa7f44cc4ba0d021182d3ae09/report.json`：20.587秒完成、contexts_closed=true，但input_delivery_observed=false、flight_ready=false。定位/深度/跟踪各403条、H候选134条、地图62条；PX4状态/位置/估计器/遥控输入均未收到，未创建EV/控制/目标发布器。
- 随后只读进程/服务检查：主机uptime约6分钟，当前没有MicroXRCEAgent进程。早前进程快照中的Agent PID21331不能当作重启后的当前状态。/dev/ttyTHS1及PX4 USB设备仍存在；未打开串口或重启服务。QGC的MAVLink遥测与ROS DDS是两条链，不能据QGC在线推断DDS可用。
- 直接阻断项为恢复当前PX4 DDS输入并核对状态；地面下视相机看不到完整H不是本轮阻断。未解锁、未发真实运动、未注入EV、未改飞控参数或实飞许可。生产H审核/总落点误差及地图/制动现场证据仍未被合成通过替代；恢复DDS不自动等于允许起飞。

## 2026-10-01 05:54 完整双域纯合成控制交接通过，离线主线收尾

- 用户重新授权一次<=180秒完整双域纯合成复测，已执行并消耗；证据`evidence/task_full_pipe_20261001_055232/report.json`。总耗时63.494秒、passed=true，host182/container183，无真实传感器/PX4连接、无真实运动，不是PX4 SITL。未自动重试。
- 实际ContainerTaskInputs→TaskSceneInput→TaskFlightCore→RuntimeNode唯一控制出口→MixedTaskPlant形成反馈。轨迹WAIT→STREAM→OFFBOARD→ARM→TAKEOFF→HOVER→NAVIGATE→ALIGN_H→LAND→DONE；报告主trace明确包含ALIGN_H（core内部states列表未单列该阶段，以主trace/控制输出为准）。HOVER17.381秒、NAVIGATE20.384秒，悬停约3.003秒。
- 最高PX4中心模拟离地1.000000m，2.50m目标最远2.423433m（既有8cm到达阈值）。57.840秒进入ALIGN_H时距起点7.680cm，61.322秒LAND交接时2.650cm，62.757秒DONE，模拟z回0且上锁。该残差来自理想模型，不是真实精降精度；下降由理想原LAND承担，无持续视觉修正。
- 1340条合成EV、2483条心跳/设定值，VIO/深度/地图/H/导航/跟踪各1406条、bounds2812条；导航管道2278请求/2278确认、pending0、fault为空。最大记录管道年龄0.17225秒（包含启动阶段），读取器最大排队0.02293秒。周期图审计保持唯一来源，不声称逐消息身份认证。
- LAND期间按现有原控制策略停止EV，DONE快照的preflight/EV融合标志因此为false，不能把该合成终态误说成真实PX4融合掉线，也不能用它证明真实停止EV后的降落性能。真实机上的降落定位与漂移仍须现场验收。
- 源与读取器退出码均0，关闭回执confirmed=true，检查无专用进程残留。此前numpy.float32→标量反馈错误未复现。生产reviewed/实飞许可未改，读取器普通25秒限制不变。
- 两类证据分开保留：此前74.905秒实际A*/APF通过的是双箱通道算法回放（旧1秒悬停）；本次63.494秒通过的是实际双域/控制交接（3秒悬停、理想导航、自由地图、同模型H）。不得合并宣称实际A*/APF＋真实传感器＋真实PX4的端到端实飞通过。离线主线不再重复；下一步集中于当前实机会话输入/融合、地图与真实H方向/落点误差/下降漂移的受控现场确认，随后才是避障和H区降落验收。窄门/识别投递仍未完成。

## 2026-10-01 05:51 完整双域单次测试在水平反馈类型错误处退出；仅夹具已修复

- 用户授权的一次<=180秒完整双域纯合成测试已执行并消耗。证据`evidence/task_full_pipe_20261001_055011/report.json`；21.107秒退出、passed=false，没有真实传感器/PX4输入或真实运动。不得把合成ARM状态写成实机解锁。
- 真实运行器经双域输入达到READY，合成开关触发STREAM→OFFBOARD→ARM→TAKEOFF→HOVER→NAVIGATE；HOVER始于17.304秒，NAVIGATE始于20.311秒，3秒悬停已实际走到。首次水平反馈时触发`AssertionError: The 'vx' field must be of type 'float'`，未完成去返或H/LAND。
- 根因：新增MixedXY直接保存TrajectorySetpoint数组读取出的numpy.float32值，传给VehicleLocalPosition标量setter时不接受；原纯数学单测未覆盖实际反馈消息赋值。现已在夹具接收边界把XY目标速度和Z目标转为Python float，并将真实VehicleLocalPosition赋值加入已有回归，不改生产运行器或放宽字段校验。
- 退出前353条合成EV，384组VIO/深度/地图/H/导航/跟踪输入（bounds768），管道fault为空；source.log记录388份源快照/151个goal回调。主端182个请求/180个ACK、退出时2个待收，读取器收尾回执为182次已发布；这是停止时未读完ACK，不宣称全程零未决。
- 读取器与动态源退出码均0，关闭回执confirmed=true，专用进程检查无残留（仅检查命令自身匹配）。本次没有自动重跑。修复后task_regression_20261001_055059为275项全部通过、0失败/错误/跳过、20.831秒，不能替代修复后的整段隔离复验。
- 下一动作仅申请一次修复后<=180秒完整双域纯合成复测，不新增其它阶段；A*/APF既有通过证据保留，完整输入/控制/H降落链仍未通过，实飞许可保持关闭。

## 2026-10-01 05:49 完整双域任务隔离入口已接线，待一次有界执行

- 新tests/task_full_pipe_check.py默认只描述；显式--run-isolated才连接合成节点。host域182为MixedTaskPlant＋真实RuntimeNode/TaskFlightCore，container域183为随运动快照更新的VIO、深度元信息、地图高度界、H像素及明确标记的理想目标趋近导航，经过实际ContainerTaskInputs往返。无真实传感器/PX4，未运行入口。
- 新tests/task_dynamic_sources.py使用同一快照的原时间戳和位置/速度，拒绝错会话/非有限数据；超过150ms的快照不再发布，不换新时间戳伪装新鲜。H中心离开视野时不可用，返程进入后才投影；与生产投影同模型，只测接口，不证实真实标定精度。地图为自由地图、导航为理想趋近，明确不是A*/APF重复验收或障碍成绩；实际算法证据仍是此前74.905秒报告。
- 完整入口通过原READY持续判定后才生成合成遥控开关边沿，不直接跳过状态机。3秒悬停、2.5m前进/返回、H对准后LAND均走同一个真实运行器输出接口，要求DONE、RETURN、路径距离/高度范围、返回误差、模拟落地/上锁后才判通过。reviewed字段只在内存合成配置置真，不写回生产配置。
- 为覆盖全航程，新增显式isolated_sortie预算150秒，仅isolated＋navigation_writes＋无live permit组合可用，reader仍固定域183、允许话题不变；普通只读/旧隔离短测仍25秒，真实许可仍按原200秒规则，不能用新开关进入真实域。外层任务140秒、内部153秒中断、结果上限160秒，正式执行时再加<=180秒外层超时，不自动重试。
- 最终纯离线回归task_regression_20261001_054830：275项全部通过，0失败/错误/跳过，20.748秒；含新入口默认不执行、非法worker参数拒绝、长时限组合拒绝、动态源坐标/H可见性和原回归。先前task_regression_20261001_054439的273项只覆盖动态源，不代表后续长时限/入口通过；以054830为最新软件证据。
- 待用户授权一次<=180秒完整双域纯合成运行；此前实际A*/APF测试授权已消耗，不借用。未启动任何新ROS测试或实机链。此入口通过后也不替代真实H落点误差、现场地图时效和实机控制/制动验收。后续目标仍为避障→H区降落→窄门→识别投递。

## 2026-10-01 05:42 水平/垂直混合控制的隔离反馈夹具已实现（未运行DDS整链）

- 新增tests/task_mixed_plant.py，默认导入不创建ROS上下文/网络。MixedXY接收真实build_task_messages生成的PX4消息字段：原固定XY起飞/悬停或NaN XY位置＋有限XY速度＋固定巡航Z位置；保持0.15m/s、0.86m爬升及固定航向的当前调试范围，拒绝旧时间戳、非法混合字段/高度/航向。此限制仅测试夹具，不改生产参数。
- 懒加载MixedTaskPlant复用旧SyntheticPlant的垂直、ACK、模式、落地逻辑，新增local_position水平反馈及逆变换的VIO位置/速度。创建节点前要求当前ROS上下文域182、本机通信；旧垂直夹具未替换。仅理想运动学、LAND时水平速度立即归零，无惯性/真实制动/风扰，不是PX4 SITL或飞行精度证明。
- 新增5项无ROS单元测试：实际任务序列化消息保持前进方向、独立标量坐标反变换、未解锁/非Offboard/过期心跳或指令无位移、非法输入不污染已接收目标、原垂直目标兼容。task_regression_20261001_054131：269项全部通过、0失败/错误/跳过、20.541秒。
- 本轮只实现并验证反馈模型，未实例化ROS混合飞控、未运行新的整段测试或连接任何硬件。下一步仍需把随模拟位置更新的容器VIO/地图/H接入，并配置专用隔离整段时限；不能把当前固定位置task_pipe_check当作动态源，也不放宽普通只读短测25秒限制。准备后再请求有界运行授权。

## 2026-10-01 05:39 完整交接覆盖核查及悬停夹具纠正（仅软件）

- 当前代码的真实任务运行器已选择ContainerTaskInputs，TaskSceneInput→TaskFlightCore→RuntimeNode.tick→build_task_messages共用原PX4发布出口；这属于代码接线成立，不是整段实机运行证明。旧task_runtime_pipe_check只到WAIT/READY及地面输入中断，未解锁合成飞机；其SyntheticPlant.setpoint明确只允许固定XY位置和全NaN速度，不能直接延长此短测并宣称覆盖水平导航。
- 新发现并纠正证据范围：TaskRig原默认TaskFlightSupervisor(1.)，刚才74.905秒实际A*/APF合成报告中HOVER20步即1秒；当前调试要求3秒。该次去返/H交接通过保持为历史事实，但不能称完整生产时序通过。tests/task_actual_navigation_replay.py和task_configured_sortie_replay.py现在显式TaskRig(hover=3.)，实际算法回放报告将记录hover_s。保留原单元夹具默认1秒，避免改变其它测试场景的预算；新增3秒悬停后才进入导航的断言。
- task_regression_20261001_053848：264项纯离线全部通过，0失败/错误/跳过，20.086秒；本轮没有重新运行实际A*/APF节点，也没有任何真实传感器/飞控输出。旧74.905秒报告未改写，不将它归给修正后的源码。
- 下一项实现应是支持混合XY速度/Z位置的独立合成飞控夹具，以及跨容器任务输入随模拟位置更新，贯穿实际运行器到H对准/原LAND的单出口整段验证。不能复用固定位置输入冒充运动反馈，也不能把纯TaskRig回放当成管道验收。准备完整后再请求一次有界隔离运行，不重复无新增覆盖的静止EV/WAIT检查。真实H外参/落点总误差、控制/制动响应仍另需现场证据。

## 2026-10-01 实际A*＋APF双箱通道往返＋合成H对准/原LAND交接通过（单次74.905秒）

- 用户新授权一次总长最多180秒纯合成测试，本次已执行并消耗，不自动重试。证据：`evidence/task_actual_navigation_20260930_213403/report.json`、`samples.json`、`navigation.log`（目录UTC）；总时间74.9046秒，未超限。domain186、本机隔离，无真实传感器/PX4连接、无真实EV或运动/舵机输出。
- 使用真实NavFn A*、nvblox Nav2代价地图插件、目标桥和路径引导的旧APF C++；合成地图/雷达、理想运动响应/定位、合成H误差和理想LAND/ACK。没有运行真实GPU建图、PX4 SITL，也没有覆盖新的host-container任务管道整段飞行。
- 导航输出1312条、路径245条，模拟1206步/60.3秒。实际任务状态STREAM→OFFBOARD→ARM→TAKEOFF→HOVER→NAVIGATE→ALIGN_H→LAND→DONE；ARM/OFFBOARD/LAND仅为内存合成命令。去程678步、返程528步；最高PX4中心离地1.00m，目标2.50m，最远2.422227m（既有8cm到达阈值），最终水平残差2.71045cm、模拟z约0.01m并完成合成落地/上锁。不能把阈值终点写成精确2.500m或把模拟落地写成物理接触。
- 合成地图碰撞检查contact=false；横向绝对位移最大1.408cm，表明本例主要穿过对称双箱1.2m净通道，不是阻挡中心线后的外侧绕行测试。NAVIGATE共898步中2步导航不新鲜，夹具将其速度置零，不据此宣称连续时效无丢失。该模型不证明真实制动距离、控制延迟、立体净空或动态障碍响应。
- 修正后的APF启动成功，上一轮exit64缺陷未复现。结束时launch返回0，APF/规划器正常退出，三个Python子进程在统一SIGINT退出中记录KeyboardInterrupt/-2；进程检查没有发现本次测试残留（检索命中的仅检查命令自身）。没有重新执行第二次。
- 当前实际算法＋任务的软件隔离衔接证据已从启动失败推进到通过；production_axes_reviewed仍false、flight_ready=false、实飞许可未改变。下一步应集中补齐新双域任务输入管道到唯一飞控出口的整段交接、真实定位/地图有效性及H落点误差/降落漂移证据，不再把静止EV或理想自由地图回放当作新的避障成果。真实动态绕障、H区实机降落、窄门、识别投递与全比赛仍未完成。

## 2026-10-01 下一次合成测试前的故障快速退出检查（未重跑导航）

- 当前持续目标顺序按用户最新目标：避障→起降区H降落→穿越窄门→图像识别及快递投放，五天内完成竞赛部署；此前“先投递再门”的顺序记录不再作为当前排程。此处只改变排程记录，不宣称后三项已实现。
- 待新的单次合成授权期间，仅修改测试诊断：实际导航launch仍活着但必需子进程死亡时，每0.25秒检查日志并报告原始退出原因，不再空等35秒。对已保存exit64日志做正例，对正常启动日志做负例；检查未引入自动重启。所有异常将passed明确置false。
- 合成入口在导入ROS/创建节点前要求ROS_DOMAIN_ID=186且ROS_LOCALHOST_ONLY=1，缺失或错误直接拒绝；域策略头文件纳入该次运行源码哈希。没有启动实际导航节点、真实传感器或PX4链。
- 离线回归`task_regression_20261001_053305`：263项全部通过，0失败/错误/跳过，20.161秒。此为单元/离线证据，不是修正后的整段导航通过。新的<=180秒纯合成测试仍待授权；此前一次授权已消耗，不自动重试。

## 2026-10-01 05:33 实际A*＋APF整段隔离测试启动失败，入口已修正，未自动重试

- 用户授权的一次最多180秒纯合成测试已消耗。证据：`evidence/task_actual_navigation_20260930_212447/report.json`（目录使用容器UTC）；总耗时35.654秒，domain186，未连接真实传感器/PX4。模拟任务停留WAIT，路径0、导航命令0、模拟运动时间0；未起飞，不能算避障或H降落通过。
- 根因已由源码和navigation.log交叉确认：APF构造函数已支持显式isolated_task_test/domain186，但main入口仍仅接受176/0，提前退出64。NavFn规划器虽激活，目标桥等待唯一APF候选发布者，因此没有路径/导航输出。报告actual_astar_apf=true仅代表所选测试模式，不代表APF实际运行成功。
- 修正为共享domain_policy.hpp：入口域白名单176/0/186；参数验证仍要求域0显式real_sensor_inputs、域186显式isolated_task_test，隔离标志不能用于其它域。real_sensor_inputs在此同时启用严格路径时效/路径引导要求，合成生产链保留这些检查，不将其关闭。launch默认isolated_task_test=false，未更改生产许可或飞控参数。
- 新增无ROS的C++域策略组合回归及合成入口默认只描述检查；仅重建build/task_nav_isolated专用二进制，不替换运行中生产程序。第一次离线回归task_regression_20261001_052913为261项中1错误（launch单测上下文缺新参数默认值），已补false默认值，保留失败记录。
- 本次未自动重跑实际导航测试，未启动EV、解锁、真实目标、运动或舵机。下一次整段A*/APF合成运行需新的一次有界授权；完整实飞控制/落点误差等仍不能用此软件修复替代验收。
- 最终离线回归`task_regression_20261001_053112`：261项全部通过，0失败/错误/跳过，总耗时21.251秒；专用APF二进制重新编译成功。此结果只验证源码/策略和既有离线测试，修正后的实际导航节点尚未重新运行。

## 2026-10-01 05:20 当前配置驱动的完整往返/H原LAND离线回放通过，尚非避障实测

- 复核旧offline-ready-sortie-h：其底层导航目标是旧单程路线，采用持续视觉下降，不等于当前2.5m往返→H对准→原PX4 LAND方案；本轮没有调用该旧容器入口，也未用其旧结果替代当前任务验收。
- 新tests/task_configured_sortie_replay.py复用TaskRig、实际TaskFlightSupervisor/TaskDispatchContract/RoundTripMission，读取当前roundtrip_task.yaml的完整场地和相机几何。理想目标趋近速度替代导航算法，合成H像素由同一名义相机模型生成后交给实际target_on_ground投影；不初始化ROS、不创建发布器。默认只描述，显式--run-offline执行。
- 初版task_configured_replay_20261001_051827保留；发现旋转场地后夹具的VIO↔PX4合成航向仍固定0，已修正为对应航向，不以初版证明旋转对齐。最终task_configured_replay_20261001_051855在0°/45°/90°三个固定起飞航向均通过，同一任务顺序STREAM/OFFBOARD/ARM/TAKEOFF/HOVER/NAVIGATE/ALIGN_H/LAND/DONE；这些命令均仅为内存消息，无真实输出。
- 每例模拟52.75秒，最高PX4中心离地1.00m，目标2.50m，实际最远2.42143m（进入既有8cm到达阈值后驻留并返回，不能说精确达到2.500m）；返回水平残差约2.73cm、合成落地/上锁成立。沿用夹具1cm落地阈值，最终z约0.01m，不宣称物理接触实测。H中心在视野内280帧/视野外775帧，只建模中心像素，不证明完整H检测率；未让起点地面完整H成为起飞条件。
- 该回放为理想定位/动态/自由地图/理想LAND响应、同模型合成H，不含真实A*或APF、不含纸箱、不含PX4 SITL、不含新容器管道的完整航程，也不证明真实相机精度或飞行误差界。极小的投影回算残差仅是同模型数值一致性；生产axes_reviewed仍false，实飞许可和误差预算未改。
- 最终task_regression_20261001_051916：259项纯离线测试通过，0失败/错误/跳过，19.058秒，新增完整回放回归及源码SHA。下一步仍需实际A*/路径引导APF与当前往返/新通道的整段隔离接入验证，以及现场H/制动/控制交接证据；不能宣布避障或H实飞已完成。完整竞赛目标继续保持。

## 2026-10-01 05:16 四侧边界确认并接入精确旋转边界；前进目标仍2.50m

- 用户补充前方边界6.20m，并明确只需前进2.50m测试避障。四侧物理边界现为前6.20/后1.80/左2.50/右2.50m；配置已更新，前方不再待确认。本次不扩展到6.2m，原前进2.5m→返回→H对准接PX4降落流程未改。
- 新task_site_bounds.py将现场矩形绑定到本会话起点与航向，用逆旋转进行精确包含检查，不采用会扩大可飞区域的odom外包矩形。机体半径0.43m+位置不确定度0.05m只扣一次，静态机体中心范围为起飞系前5.72/后1.32/左2.02/右2.02m；当前/候选速度的额外制动距离仍需在边界内。
- 已接入TaskSceneInput初始化、任务许可几何校验、RoundTripMission起终点、MissionShadow运行期边界及旧离线下降链边界；制动检查同时要求原地图自由空间与场地边界内的制动余量。显式旧bounds_odom仍按已扣包络的绝对边界处理，不重复旋转或扣机体。本配置bounds_odom保留null是采用本会话相对矩形，而非再缺场地尺寸。
- task_regression_20261001_051603：258项纯离线测试通过、0失败/错误/跳过、16.290秒。新增任意起点/航向和2.5m目标不变、外包矩形内但实际场外点拒绝、制动圆/接触边界拒绝、缺项/错误测量拒绝及旧odom边界兼容。原回归全部保留，源码SHA已保存。
- 边界成立不等于纸箱/树等障碍已避开，不等于高度柱净空或制动参数已经实测。实飞许可false，H外参/误差预算及原余下验收状态不变；本轮没有真实采集、EV、目标、解锁或运动。后续只推进既定2.5m避障往返与H降落，不重复询问已确认四侧距离。

## 2026-10-01 场地边界补充：后1.80m、左/右各2.50m，前方待确认

- 用户对“起点PX4中心到场地边界，以机头为前”的问题回答：后距离180cm、左右各2.5m。已将三项写入roundtrip_task.yaml的site_measurement，参考系为起飞点PX4中心与起飞机头方向；不是障碍箱位置，也不是直接可用的odom绝对边界。
- 前方距离仍未知，保留null。左右合计5m与原测试场宽一致，但不能据旧8×5m尺寸就默认为前方6.2m，需用户确认。bounds_odom仍null、converted_to_reviewed_flight_bounds=false、实飞许可不变。
- 只读边界代码核查发现现有MissionShadow/RoundTripMission按odom轴对齐矩形检查；若现场边界随起飞航向旋转，不能直接取旋转矩形的外包矩形，否则可能纳入场外区域。后续应将现场矩形保留在起飞参考系内做点/制动区域检查或使用等价的精确旋转边界，扣除机体包络和不确定度；本轮尚未实现这项转换，不把新记录冒充已接通边界。
- 本轮只补充配置和记录，没有新实机采集/EV/导航目标/解锁/运动，也未新增飞行放行条件。

## 2026-10-01 05:12 H投影已知几何接入配置，未将名义方向当成精度放行

- 本轮读取landing_candidate.yaml、原12.5mm纠正后的board_endpoint_20260930_210050/report.json及实际TaskSceneInput投影代码，未采集新图、未要求重标定。复用用户确认的下视相机前0.12m/左0m/下0.055m；此为物理镜头最低点测量的名义投影偏移，不宣称光心已测绘。
- roundtrip_task.yaml新增camera.offset_flu_m和来源，TaskSceneInput不再隐藏硬编码[.12,0,-.055]。根据“图像上→机头、图像右→机体右、光轴向下”的既有符号证据，body_from_optical填入名义零倾角矩阵[[0,-1,0],[-1,0,0],[0,0,-1]]，明确rotation_status=nominal_signed_axes_only_not_flight_reviewed，axes_reviewed仍false。它不是由本次新测得的精确外参，真实H目标仍被未审核门控拒绝。
- 降落几何采用已确认黑环内径0.60m、电机对角线0.40m、圆心与PX4中心重合，名义径向余量(0.60-0.40)/2=0.10m；landing_geometry显式记录来源。任务落点判定与专用许可验证统一从几何计算余量，不再独立硬编码0.1m；总误差预算仍null，保留原最多0.05m预算限制并扣除对准阈值所需空间，没有把10cm几何余量当成实测定位精度。
- 最终纯离线task_regression_20261001_051117：253项通过、0失败/错误/跳过、16.178秒。纳入原投影测试，新增名义轴符号/未审核仍拒绝、偏移来自配置而非隐藏常量、同心条件与无效尺寸拒绝。测试中的axes_reviewed=true只在明确合成夹具中使用，生产配置没有置true。本轮没有ROS节点、真实目标/EV/解锁或运动。
- **待现场回答：**已询问当前起点PX4中心至前/后/左/右场地边界四距（机头为前，纸箱不算边界），现有8×5m尺寸不能唯一确定起点位置。收到后可定义相对起点的可飞边界，并在新会话对齐时变换为odom；不能直接将起点假定场地中央。
- 仍未完成：飞行高度下H方向/倾角与投影总误差审核、对准后原PX4下降漂移和落点预算、航段覆盖/制动条件及完整单航程验收。无需重做本次已确认的棋盘格格长与地面位移。正式通道代码状态见05:08，实飞许可仍false；避障/H/窄门/识别投递完整目标未完成。

## 2026-10-01 05:08 正式任务改接容器管道；许可绑定单航程期限已实现，未启用实机

- live_flight_runtime的task分支已不再构造旧PerceptionDomain，改用ContainerTaskInputs：先仅发现并丢弃启动数据→原PX4只读预检→创建实际TaskFlightCore/运行器回调→刷新来源并激活。非任务的原竖直起降入口不变。故障沿05:02的隔离路径进入原中止/接管逻辑，清理失败也写入失败报告，不掩盖读进程退出问题。
- 新task_pipe_authority约束真实导航出口：必须是TaskFlightPermit，调用原before_arm并核对已消费的会话记录；reader再核对共享会话、许可哈希、截止时刻、过期时刻及消费记录。只允许既有navigation_goal/readiness两个String话题，不能发布/fmu。此为本机启动协议，不宣称抵御可写工作区的恶意本地进程；没有创建真实许可或消费记录，没有开启live_flight_enabled。
- 只读/原隔离短测仍最多25秒；持专用许可的单航程输入上限200秒，外层timeout201秒。原飞行执行窗口仍150秒触发中止、最多175秒观察，未放宽飞行任务时长。启动后若不足175秒+3秒余量，在激活/解锁前拒绝；许可有效期必须覆盖整个输入窗口。该改动修复“短测读取器25秒必停”的代码限制，**尚不构成200秒实测稳定性证据**。
- 最初新版本隔离试验task_runtime_pipe_20261001_050634失败：创建运行器后旧发现记录超过0.5秒，activate_inputs拒绝，EV/目标均0，读取器退出0，合成fixture退出1。失败保留。修正启动发现阶段可有界消化旧队列但不向任务交付旧数据；激活前最多0.5秒刷新来源，激活后的管道0.5秒、VIO/地图等消费端原源时效仍不变。新单测验证旧数据只在启动阶段丢弃、运行期仍拒绝。
- 修复后task_runtime_pipe_20261001_050740通过：域182/183、17.402秒、READY等待，117条EV仅测试话题、15/15导航/状态回执；七类输入各149条、42条启动丢弃。按计划中断后无新EV/导航，继续收到合成PX4遥测；无ARM/运动命令，reader/fixture退出0。该模式未走真实许可模式，未执行A*实景航程/实飞或200秒耐久。报告前后快照已分离。
- 最终task_regression_20261001_050759：244项通过，0失败/错误/跳过，16.625秒。新增许可类型/消费记录、会话/哈希/截止/过期、错误域、无授权reader拒绝、启动命令时限、正式入口构造及预检失败清理、启动积压处理测试。ROS与进程替身仅用于相关启动单测，不冒充真实模式成功。
- **剩余任务：**本次航段/地图覆盖与制动条件、正式H方向变换/落点误差预算仍需落实；单航程完整链和实际混合控制响应/接管仍未验收。生产roundtrip_task.yaml及platform.yaml未改，真实输出未启动。下一步使用已有标定/现场证据补齐配置并验证完整任务，不重复静止EV/已通过只读检查。完整避障、H降落、窄门、识别投递目标仍未完成。

## 2026-10-01 05:02 最小输入故障隔离已修复并离线验证；正式通道接线仍待完成

- 用户已同意按最小必要范围继续。保留避障→H起降区降落→穿窄门→识别投递的完整目标；本轮只修复一次航程的基本输入故障交接，不扩展自动恢复、不重做相机标定、不执行实机输出。
- flight_runtime新增锁存task_transport_failed：清除旧导航命令/H目标、停止后续prepare，不再因prepare或导航发布异常跳过原飞行状态机；调用既有request_abort。live_flight_runtime对感知pump异常单独隔离，不立即销毁独立PX4监听上下文。遥控接管/急停仍优先，原LAND请求是否接受仍需飞控ACK/遥测，不能宣称失效必然安全降落。
- ContainerTaskInputs新增quarantine：取消写入口、丢弃未发队列、请求读取器停止，不在PX4循环等待子进程退出；停流后不恢复或回放。停止文件写入失败记录stop_request_error，不通过该异常中断PX4后续处理。读进程的退出回执仍在最终清理检查。
- 纯离线回归task_regression_20261001_050016为235项全通过；修正隔离测试报告可变列表快照后，最终task_regression_20261001_050150仍235项通过、0失败/错误/跳过、16.150秒，保留源码SHA。新增5项覆盖prepare异常后仍tick、EOF只处理一次、原任务空中ABORT/LAND与接管/急停、队列不重放、停止文件写失败；不建立ROS上下文或真实控制输出。
- 隔离实际运行器证据task_runtime_pipe_20261001_050109：域182/183，总17.444秒。合成Position未切Offboard，READY等待通过；118条EV只发测试话题，16/16导航/状态请求确认。显式中断任务输入后模拟PX4状态/位置各继续收到100条，新增EV=0、导航请求=0、plant_commands=[]；原地未解锁分支ABORT→STOPPED，core.closed=false（监听保留）。读取器exit0、ros_closed=true、pending_bytes0，fixture退出0。该试验不是实飞/实际A*/PX4 SITL，也不证明空中飞控接受LAND。
- 上述隔离报告顶层runtime为中断前采样，但其states/control_trace原先共享列表，落盘包含后续状态；interruption.runtime是中断后快照。已修测试保存深拷贝供以后运行，不篡改原证据、不重复硬件。纯离线空中LAND/接管覆盖与本次隔离未解锁接收证据分开陈述。
- **尚未完成：**正式入口仍为旧PerceptionDomain，新通道真实域写入仍拒绝，25秒期限未扩展；当前只补好了接线前需要的故障处理。下一步在现有专用任务许可约束下完成正式输入/导航目标接线及覆盖单航程的有界生命周期，再落实既有H投影/航段配置。实飞许可false、飞控参数、生产任务review项均未改，没有新实机采集/EV/解锁/运动。

## 2026-10-01 04:55 范围收敛：只推进本次避障往返与H对准降落，尚不能直接起飞

- 用户要求后置新通道中断交接与完整飞行持续运行支持，直接进行避障和起降区降落。记录该优先级：停止扩展长时间耐久、复杂自动恢复及其他比赛任务；本次目标仍为PX4中心离地1.00m（相对爬升0.86m）、前进2.5m、返回、H对准后交原PX4降落，不改成全程视觉下降。
- 本轮仅检查源代码/配置与历史证据；没有新传感器采集、EV注入、导航目标、解锁、运动、飞控参数修改或设备重启。没有执行新的飞行测试，不将已有静止EV或隔离READY结果提升为实飞通过。
- 实际必要缺口再次由文件确认：live_flight_runtime.py的task分支仍构造PerceptionDomain，即此前真实观察未收到感知的直接DDS路径；新task_perception_pipe.py与task_perception_reader.py两端仍拒绝真实域导航写入，且读取器期限最多25秒、外层timeout为26秒。正式运行器最终观察期限为175秒。短测读取器不能直接替代一次完整航程的输入链；支持本次单次任务直到降落/接管退出，不属于可省略的长期耐久功能。
- 正式roundtrip_task.yaml仍有bounds_odom=null、landing_error_budget_m=null、camera.body_from_optical=null、axes_reviewed=false及覆盖/对齐未review项。已有相机标定/安装和地面H不可完整可见的确认不撤销，也不要求重新标定；应从已有证据落实配置，不允许仅为解锁而将空值或review标志改成通过。
- 本轮不按“跳过基本失效处置”方式开启实飞。复杂自动恢复可以后置，但最小的输入中断停止旧导航输出及明确PX4/操作者接管责任仍需成立；不能承诺一旦断流就一定自动安全降落。live_flight_enabled保持false，原现场交互确认与遥控READY切换不绕过。
- 最短剩余工作范围：接通正式任务的容器输入和真实导航目标出口；将生命周期限定为本次任务并验证已有退出/接管行为；用既有依据补齐本次航段及H投影/落点配置。此前真实输入只读通过不重复采集。上述尚未完成，因此本次没有起飞，且不承诺未经核验的完成时刻。

## 2026-10-01 04:50 实际任务运行器接入新管道隔离验证：READY等待通过，仍无实机控制

- 新增tests/task_runtime_pipe_check.py，实际create_node/TaskFlightCore/TaskSceneInput与容器管道组合；宿主域182为现成SyntheticPlant，容器域183为合成感知与导航回应，去掉宿主重复VIO/跟踪源。无/fmu话题，合成遥控始终Position，不切Offboard、不解锁；不是PX4 SITL、实际A*或飞行物理仿真。
- ContainerTaskInputs新增显式discovery_only启动阶段：先接收来源查询，启动期消息计数为startup_discarded而不作为任务证据，禁止导航写入。待来源新鲜且所有非空任务回调注册完整才activate_inputs；早于激活时刻的管道包不回灌为新任务数据。关闭入口先取消宿主写入能力，再按原停止/退出回执收尾。
- 首轮task_runtime_pipe_20261001_044636：17.433秒，实际任务上下文建立，配对差11.421ms，162条测试域EV由合成飞控接收，导航请求/回执20/20，无错误。该轮exercise=false，不声明就绪逻辑验证。
- 正式就绪逻辑对照task_runtime_pipe_20261001_044749：17.417秒、exercise=true，ready_at=4242.200268，状态仍WAIT且operator_edge_at=null、arm_requested=false；164条测试域EV，合成plant_commands=[]，未产生心跳/轨迹/解锁/降落命令。上下文已初始化，历史配对差13.102ms，七类输入195～196条，目标由实际TaskSceneInput端口送出，关联导航回应由其实际回调接受；20/20导航/状态请求确认、pending0。
- 此轮丢弃52条启动期数据、宿主共1540包，max_pipe_age360.784ms含初始化阶段，不宣称活动位姿全程满足该数值或扩大源门限。退出回执reader/client0、ros_closed=true、pending0、停止→确认约0.368秒；合成源退出0。生产配置未修改，测试仅在内存设置明确合成的alignment/bounds/takeoff-column条件，不能用作实机场地证据；地图全高度/H配置仍未review，因此不宣称可实飞。
- 离线回归task_regression_20261001_044924：230项通过，0失败/错误/跳过，15.638秒，包括启动数据不回灌、缺/空回调禁止激活、默认不执行；保留源代码SHA。实际运行器报告start_conditions使用last_tick作参考，可能早于后续回调时刻而出现false；本节不将其当独立实时飞控故障，READY结论来自隔离状态机ready_at而非那些字段。
- **下一项仍是新通道故障/停止与实际飞行控制交接的隔离验证，再做生产入口集成。** 不能直接把最长25秒的读取候选塞入最长175秒的飞行运行器；必须明确整个任务生命周期与异常时原PX4降落/接管行为。生产入口当前仍未切换，真实域导航写入仍拒绝，未再用消耗过的真实观察许可。完整避障/H/窄门/投递目标未完成。

## 2026-10-01 04:43 导航目标私有返回通道已在隔离域往返通过；生产任务尚未切换

- 新增task_navigation_pipe.py，扩展已有容器任务管道的显式隔离导航写入模式。仅允许navigation_goal/readiness两个String话题，拒绝/fmu或任意其它输出、错会话/跳号/重放/未来/超过250ms请求、超过64KiB载荷、同目标ID改坐标；逐条回执，500ms未确认即失败、不自动重发。源时间与原导航目标关联校验保留。回执表示容器执行publish，不单独作为订阅者收到或规划成功证明。
- 当前host端与reader端都强制导航写入仅用于isolated domain183，非隔离模式显式拒绝；真实只读入口继续不发布目标/状态/控制/EV。生产task_flight_runtime未切换，真实导航目标仍未发送。下一步需要把该通道接到实际任务运行器的启动/停止/控制交接流程并隔离验证，不能把传输往返冒充飞行状态机通过。
- evidence/task_pipe_20261001_044111隔离端到端：16.385秒，宿主发送1个目标+1个状态，容器读取器确认2/2，独立容器ROS订阅者也确认各收到1次；收到99条带pipe:OUTBOUND目标ID的合成导航回应。七类输入各138～140条，1045包，fault=null，最大管道年龄194.407ms。未运行实际A*、APF或PX4；响应是明确合成夹具，不是绕障路径。关闭回执ros_closed=true、reader/client0、待发0、无回压，停止→确认约0.397秒；fixture0。
- evidence/task_pipe_20261001_044203只读回归：16.436秒，七类137～138条，宿主1035包（reader发1036，主动停止未消费尾包不作丢包结论），导航requests/acks/pending均0、两目标话题发布者0，fault=null；退出确认约0.396秒，双方exit0。证明新隔离写入开关未使默认只读模式生成业务发布。
- 离线回归task_regression_20261001_044034：227项通过，0失败/错误/跳过，16.369秒；包含真实域写入拒绝、非法目标/会话/时效/重放拒绝。补正首拒绝诊断last_accepted_seq为失败前序号，保留首错、不以之后恢复覆盖。上述均无真实采集/控制/参数改动，不重复04:33已通过的实机输入检查。

## 2026-10-01 04:37 任务初始位姿改为源时间历史配对；222项离线通过，未新增硬件测试

- 针对04:33只读末态的VIO/PX4 pairing >50ms，代码核对发现原TaskSceneInput仅把最新VIO与最新PX4消息比较，无法使用已经收到的同时间历史PX4。该机制能因不同传输延迟而误阻断，但原实测未保存完整配对历史，因此不宣称已经证明那次超限的唯一原因或实际故障已消失。
- 新增task_pose_pairing.py；仅TaskFlightCore保存最多128条已由现有receive源时间规则接受的本地位姿快照。初始化选择与VIO源戳最近、差值仍≤50ms的样本；PX4源年龄及接收年龄均≤250ms、重置计数与当前一致、xy/z有效、历史样本未解锁且落地。当前PX4新鲜度≤100ms及当前未解锁/落地要求保留；VIO原250ms规则不变，不插值/外推、不修改时间戳或放宽50ms。
- 对齐构造真正使用匹配历史样本的XYZ/heading，而非仅过配对检查后仍用最新位置。报告新增initial_pose_pairing，记录是否匹配、双方源戳、时间差和字段名。沿用VehicleLocalPosition.timestamp（DDS调整后源消息时刻，微秒转纳秒）；源码另有timestamp_sample，但未核实时钟转换，所以没有将二者混用，也不称为已核实原始IMU采样时间对齐。
- 离线回归evidence/task_regression_20261001_043653：222项全部通过，0失败/错误/跳过，15.367秒；覆盖VIO晚到100ms时选择旧对应位置、错误重置/过期/未来/超过50ms/无历史拒绝、无效或空中历史拒绝、真实TaskSceneInput对齐原点采用匹配历史值。普通起降核心未修改，生产配置未review标记保持，不把单元测试当新会话对齐实测。
- 本轮没有新ROS真机观察、EV/运动/目标发布、设备重启或参数修改；04:33的输入接通结果不撤回，也不重复该验收。下一软件工作仍是导航目标返回通道与生产任务控制接入；原始时间配对的真实效果须随之后获准的交接验证观察，不能据此直接实飞。

## 2026-10-01 04:33 新容器接入真实只读复测通过输入投递；尚未控制或起飞

- 用户明确新授权一次总长最多30秒，并要求通过后推进避障真机测试。已执行本次只读许可、不自动重试：evidence/task_readonly_9a6a2c3cbba54ade955b24bb1046e7b0，总20.4905秒、worker20.4152秒、exit0、observation_completed=true、input_delivery_observed=true、flight_ready=false。范围是实际任务输入投递与解析，不是航路/控制/实飞验收。
- 当前任务运行器实际收到VIO288、跟踪288、深度298、地图78、地图上下边界154、H候选146；PX4本地位置1849、遥控824、状态37、融合状态18、落地状态18。导航命令发布者1，但本轮无目标，未收到导航命令是待目标状态，不伪造导航数据或宣称已完成规划。H候选接收不等于地面能看到完整H或已完成米制落点。
- 最终TaskSceneInput已接受定位/深度/地图及上下边界，navigation_blockers=[]；末段报告源年龄VIO103.7ms、深度137.0ms、地图217.5ms、边界207.5ms。来源查询各输入1、目标和任务readiness发布者0，无fault/首拒绝。只保存聚合计数/末态，不将这些末态年龄当作全窗口最大值，也不声称VIO全程30Hz。
- 私有通道宿主保存1412包，读取器发1414包、pending0；主动退出尾部未消费2包不作为传输丢帧结论。最大管道年龄103.956ms、最大队列驻留25.081ms，would_block63为非阻塞回压次数，不是丢包；未触发队列耗尽。来源证明为周期性DDS图查询，并非逐消息发布者身份核验（当前安装版rclpy回调无该接口）。
- ev_generated=0、published={}、output_topics=[]、control_trace=[]，无控制器tick/解锁/运动/舵机/参数修改/设备重启。保持未解锁、GCS连接；两个上下文清理无错误。读取器stop→退出确认约0.366秒，ros_closed=true、client0；进程检查无本轮观察器/读取器残留，Agent21331保留。EV未启动，不声称当前仍视觉融合。
- 前置诊断增强回归task_regression_20261001_043015：217项通过，0失败/错误/跳过，14.763秒。增加首拒绝包/来源计数诊断；04:29合成证据保留。
- **输入接通这一关已推进通过，不再重复该只读测试。** 用户的后续避障测试要求保留，但现有生产task_flight_runtime/live_flight_runtime仍走直接PerceptionDomain，未切换这条新容器输入；新端口明确拒绝任何发布，导航目标返回通道尚未实现，不能把只读修复直接称为可飞。任务仍报告alignment_reviewed/bounds未设、VIO/PX4配对>50ms，以及H外参/落点/覆盖等配置未review。只读运行器start_conditions包含缓存判据，不将全部false当作独立实时故障。
- 下一步为真实任务生产接入的离线控制/导航目标交接验证、按原始采样时间配对诊断，以及当前航段/落点条件落实；达到受控试飞条件后再推进PX4中心离地1m、前进2.5m、返回H对准接原PX4降落。不得直接打开实飞许可或解锁来绕过上述尚未实现的软件链路。

## 2026-10-01 04:29 容器任务输入只读接入候选已实现；七类合成消息端到端通过，等待新真实只读复测

- 新增scripts/task_perception_reader.py与task_perception_pipe.py。限定感知话题在容器订阅，通过私有有界非阻塞管道送宿主：定位/地图/边界/H/导航/跟踪以原ROS序列化内容传送，保留源时间戳；深度只传当前TaskSceneInput检查所需的header、尺寸、真实数据字节数，宿主仅有长度证据，不伪造像素或宣称转发了整幅深度图。
- 端口拒绝任何发布请求，因此仍只读，不是生产导航目标出口。包括会话/顺序/域号/管道年龄、来源唯一性、来源丢失更换、竞争目标发布者、固定消息类型/话题、缓冲上限检查。消费端原有VIO250ms、地图/深度500ms等源时效规则不放宽；管道500ms是额外传输诊断上限，不代替消息自身时效。
- task_readonly_session新增显式--container-inputs候选开关；默认描述无硬件，原直接DDS模式保留供比较。候选仍只接task_preview，不改变task_flight_runtime生产接线、不启用实飞、不创建导航目标/控制/EV业务发布器。原总30秒窗口：观察20秒，容器截止23秒、worker闹钟25秒、监督28秒；停止控制文件/回执复用EV链机制，不自动重试。
- 纯离线回归evidence/task_regression_20261001_042809：217项通过，0失败/错误/跳过，14.959秒；新增源消息逐字段保留、缺/错会话与源替换/丢失拒绝、深度长度边界、未知订阅/任何发布拒绝、真实只读运行器接受该端口且无业务输出等测试。
- 隔离端到端evidence/task_pipe_20261001_042726：domain183，无PX4上下文/真实传感器/导航目标/控制器tick；容器合成发送→容器读取→私有管道→宿主真实TaskSceneInput解析，七类各105条，789包，scene定位/深度/地图已接受、地图与边界来源一致，fault=null。H/导航使用无效候选负例，只证明消息传输，不证明H投影/导航路径有效；原配置未review条件保持。
- 该隔离会话总16.422秒，读取器发送789包、pending0、would_block0、最大队列驻留5.496ms，最大管道包年龄253.050ms（包含启动，不将它写成全程满足VIO250ms）；尾部已接受输入源年龄约16.8ms。结束回执ros_closed=true、client退出0，停止到确认0.529秒，合成发送端退出0；后续进程检查无本轮读取器/夹具残留。
- 本轮未进行新的真实读取/EV/运动，也未修改platform.yaml、roundtrip_task.yaml、param.params.txt，SHA分别保持4137c1…/c3d697…/3d3f5b…。需要新一次总长最多30秒真实只读授权才能验证现场链；不能复用04:17已消耗许可。新方案实际数据投递、持续时效、H落点与唯一飞行控制/目标交接仍未验收；避障→H降落→窄门→识别投递完整目标未完成。

## 2026-10-01 04:23 无硬件跨容器传输对照：发现成功但跨边界0帧，同容器150/150；任务接入尚未修复

- 没有复用已消耗的30秒真实观察许可。本轮仅域181、随机测试话题的合成ROS图像，无真实相机/地图/PX4订阅、无实机目标或控制；不重启在用进程、不改系统参数/容器IPC或真实节点配置。扩展现有tests/image_transport_probe.py支持发送/接收位于容器及独立接收端传输配置，默认仍仅描述，容器内timeout限定自有子进程。
- 初两次夹具image_transport_20261001_041841_shm16m_59c23e与041905_shm16m_00b7df未加载容器ROS环境，ModuleNotFoundError:rclpy，无发送端报告；不能用于传输结论，失败保留。随后固定加载/opt/ros/humble/setup.bash。
- 有效对照（evidence/目录）：041933_shm16m_793830，容器发→宿主收，两端16MiB配置，发现到唯一订阅者后测量发送150帧/路，IR1/IR2/深度均0帧，双方正常退出，8.019秒；041953_shm16m_6ec3ac仅宿主接收端改UDP XML，结果仍三路0/150，8.068秒。UDP配置未修复，不部署到真实节点；实际进程仍有默认SHM映射，不声称该实验已经完全消除所有内部SHM路径。
- 正对照042017_shm16m_71aebb，容器发→同容器收，两个独立进程，均16MiB配置：三路各150/150，另有28个暖机帧，测量帧无缺失；最大源间隔38.615ms，最大年龄10.98/13.14/15.13ms，8.118秒，两子进程退出0，无XML错误。640x480双mono8+16UC1深度、30Hz，保留原30帧暖机及150帧测量口径。
- 证据支持当前宿主/容器边界存在“发现成功、数据不达”的可复现问题，同容器读取可在该合成窗口正常工作；不据此唯一归因于private IPC，不把合成图像结果冒充实际地图/VIO传输验收。下一步优先把限定感知订阅放回容器，并复用已成功EV链的有界私有管道/源时间戳/来源审计/退出回执机制交给宿主任务；还需实现和验证地图、深度有效性、H候选及导航数据，不得只转VIO却宣称任务接通。
- 最终离线回归task_regression_20261001_042154：209项通过，0失败/错误/跳过，15.721秒；包含夹具默认不执行与UDP配置检查及源码SHA。生产任务入口/实飞配置本轮未切换，跨容器任务修复尚未实现，真实避障/H降落仍未验收。

## 2026-10-01 04:17 一次真实任务输入只读观察结束：PX4收到，感知全缺失，未通过接线验收

- 用户明确批准一次总长最多30秒；执行且消耗该许可，未自动重试。evidence/task_readonly_a409cce6e9574c1e9183c2a5f03741a9：总20.113秒、worker20.076秒、exit0、observation_completed=true，但input_delivery_observed=false、flight_ready=false。exit0只表示观察正常完成，不是输入验收通过。
- 实际任务运行器收到PX4本地位置1595、遥控746、状态33、融合状态17、落地状态17条；derived_aux741、failsafe31。未观察到解锁，GCS已连接。VIO观察计数缺失，TaskSceneInput所有接受源戳为空，无定位/深度/地图输入证据。因此不能进行避障或H降落控制交接，不能把只读运行器报告的派生start_blockers全部当成独立实机故障。
- ev_generated=0、published={}、output_topics=[]、control_trace=[]、无控制器tick，fault=null；cleanup_errors=[]、contexts_closed=true。结束进程检查无本次入口残留，Agent21331仍在。没有EV注入、导航目标、运动/解锁/舵机、参数写入或设备重启。
- 后续仅进程/文件检查：容器running，原感知7530及VIO子进程7593、地图11653/11688、导航11654仍存在；存在不等于流仍有效。实际容器VIO/地图域176、localhost_only=1、FastDDS；VIO加载16MiB XML，地图未加载该XML。容器host网络但private IPC。宿主直接双域观察无法收到容器感知，需核对跨容器发现/传输与读取方式；尚不能将private IPC认定为唯一根因。先前隔离双域测试为同进程合成输入，不覆盖此宿主/容器边界；不声称其已证明跨容器可用。
- 观察前新增缺失任一路输入/跟踪戳及意外解锁无业务输出测试，evidence/task_regression_20261001_041408：207项通过，0失败/错误/跳过，14.447秒，hardware=false。
- 下一步先离线修正/验证跨容器任务感知接入与诊断（优先复用已成功EV读取链的容器侧订阅经验），不改现有感知进程或飞控；再另行授权真实复验。不重复本次许可、不扩大为起飞，完整目标顺序不变。

## 2026-10-01 04:14 真实任务输入只读入口完成软件收尾；205项离线通过，尚未执行硬件观察

- 推进方向保持避障→H起降区降落→窄门→识别投递；04:02的静止EV融合通过仍有效作为历史证据，不重复重启或重复静止EV测试，也不外推为当前持续融合。
- 修正原task_preview仍创建影子控制/EV、导航目标与任务状态发布器并执行控制器的问题：现在运行器与TaskSceneInput的只读分支不创建这些业务发布器，不生成EV、不执行飞行状态机，仅接收域0的PX4与域176的感知输入，保留来源竞争检查和未review配置。正常任务分支默认行为不变。
- 新专用入口scripts/task_readonly_session.py默认仅描述，显式观察模式不启动传感器或Agent、不发布EV/导航目标/控制、不改参数；目标观察从调用起20秒，25秒工作闹钟、28秒监督退出，拟定一次授权总长最多30秒。退出清理两域上下文，运行时未解锁条件仍检查。
- 本轮修复缺失tracking_stamp的None比较、异常后observation_completed误留true及退出状态判定；预览解锁告警不再错误称为拆桨。默认报告改称task_output_publishers_created=0：宿主rclpy会自动创建parameter_events，故不宣称无任何DDS/ROS元数据发布。
- 离线证据evidence/task_regression_20261001_041237/report.json：205项通过，0失败/错误/跳过，14.530秒，hardware=false。实际运行器构造/回调由惰性Node替身验证无业务输出，异常/清理失败/超时返回失败有测试；不是实际双域数据投递、ROS上下文退出或飞行验收。
- 本轮没有启动真实只读入口、没有新EV注入、未改生产任务配置/飞控参数/实飞许可。先前EV单次许可已经消耗；下一步申请一次最多30秒真实任务输入只读观察，保留既有感知/Agent，不重试，成功只推进输入接线验收，不直接放行起飞。
- 距离真实避障/H降落仍缺：真实任务输入和唯一持续EV/水平控制交接；当前航段、地图高度覆盖与动态制动/接管验证；生产H投影配置、落点误差预算及H对准后交原PX4降落实测。地面完整H不可见不作为起飞前必须补齐的条件，不再重复询问已确认的棋盘格/安装尺寸。

## 2026-10-01 04:02 新会话EV-only复测通过：1341条EV，融合有效样本覆盖44.32秒；已停止注入

- 用户新授权一次最多60秒EV-only复测。实际复用Agent21331和既有感知，不传恢复Agent选项、不重启任何设备、不改参数、不解锁、不发运动/舵机。**此单次许可已执行并消耗，没有自动重试。** 新入口先无输出核验DDS图就绪，再进入原EV门控；本次图探针第1次检查即ready，原冷启动失败仍保留、不追改。
- 真实证据`evidence/disarmed_ev_e553bed1a8fc41bd843d30b73b780085/`：监督总长**49.734秒**、worker48.805秒，exit0，passed=true（范围为EV投递），flight_ready=false，sent=**1341**，fault/error均null。报告/reader回执/监督记录均已落盘；不是只凭发送计数判断融合。
- PX4逐次有效融合遥测共50次，其中46次位置/航向/高度融合均true，构成单一区间`1373.951121126～1418.268060358`（单调秒），跨度**44.316939秒**，最大接收间隔1.010250秒，无分析异常/截断。此前4次尚未开启；后续有效样本未见关闭。视觉速度融合false、气压高度融合true、测距高度融合false。这支持本次静止测试窗口内持续收到融合开启确认，不外推到最后样本以后的状态。
- VIO输入1371个pose，最大源间隔**33.402ms**、源年龄**47.067ms**、宿主接收间隔**52.175ms**；容器回调至宿主接收最大**9.739ms**。首个pose未被guard应用且fault=null，其后没有源间隔超限导致锁存；不把首包启动状态当运行期丢流。位置值全段相同、姿态有微小变化，符合本次无运动输入范围；**本次没有独立位移，不能据此证明动态定位响应或绝对精度**。
- 实际保存的PX4本地位置是上限2000条环形尾部，覆盖最后**20.00046秒**；2000/2000的xy_valid/z_valid均true，重置计数始终`[3,2,1]`。不能据该尾部声称整个49秒没有启动融合重置。此尾部X/Y/Z峰峰值约**7.21/11.42/4.18mm**，航向峰峰约0.0501°，仅静止估计波动，不是实测定位精度或飞行误差界；未完成带运动的坐标/尺度/方向验收。
- status/flags/land/RC最大接收间隔分别0.51210/1.01022/1.01116/0.04949秒，源年龄最大20.18/47.04/48.27/32.30ms，均在现有静止合同内；无遥测时效故障。末次所有权检查5类PX4输出来源各1，无重复/更换/竞争输入，只存在本入口EV发布者。
- 结束按约先销毁EV发布器，再通知容器读取器；stop_requested=1418.467661，ros_closed=1418.503024，确认回执与客户端退出=1418.749365，确认耗时约**0.282秒**。读取器exit0、pending_bytes0、would_block0。传输共2744包发送，宿主保存2742包；结束时未消费的尾包不解释为传输丢帧，也不延长测试追读。随后只读进程检查无读取器，Agent21331、QGC1870、感知/地图/导航启动器仍在。
- **当前EV已停发**，本报告不声称现在仍保持视觉融合；QGC随后若提示外部视觉失效，可能是本次按约停止输入的预期后果，需要另行状态证据，不能把短测入口当长期飞行定位服务。下一次运动任务必须由经过验收的唯一任务出口持续提供EV，不自动后台续发本测试。
- param.params.txt、platform.yaml、roundtrip_task.yaml的SHA与前轮相同，live_flight_enabled=false。当前生产任务配置仍含bounds_odom=null、alignment/full-height/takeoff-column/landing-column未review、landing_error_budget=null、下视body_from_optical=null；本次静态融合通过没有把这些未核验条件设成true。
- **阶段结论：新会话静止EV输入与持续融合这一项获得正向真实证据。下一步转向专用往返任务的真实输入/唯一控制交接、现场航段/制动/覆盖和H投影/落点条件核对，不再重复重启相机或同一静止融合测试。** 避障往返、H对准接原PX4降落、窄门与识别投递仍未实飞验收；真实导航目标/运动需单独明确范围。

## 2026-10-01 03:58 DDS已恢复；单次EV联调因冷启动话题尚未创建退出，EV=0；启动顺序已离线修正

- 用户明确允许恢复DDS并注入EV，沿用上一条单次总长≤60秒、未解锁静止、不发运动/舵机、不改参数、不重试、结束停EV而保留DDS/感知的范围。**已执行且许可已消耗**，不是等待同一许可，也没有在失败后重试。
- 证据`evidence/disarmed_ev_d634836ad841461e9b5e4da93a808f8b/`：Agent恢复0.668秒，总监督3.499秒，worker2.293秒；passed=false，sent=0，fault=`discovery_timeout: required source not discovered within 2 seconds`。5类PX4输出（status/flags/land/RC/local）在整个发现窗口内计数均0，无重复或竞争输出；仅本入口EV发布器被发现，但没有发布任何位姿。没有可用于本会话融合/对齐判断的数据，不能说“PX4拒绝融合”或“相机又坏了”。
- 读取器退出回执匹配、ros_closed=true、exit0，无排队/背压；停止请求1084.012924、确认1084.269241，约0.256秒。容器读取器与宿主EV进程已退出。Agent宿主PID21331（Telem1 /dev/ttyTHS1,921600）按许可保留，QGC桥1870、既有感知/地图/导航候选保留；未重启它们。仅进程存在性已复核，不冒充退出后另一次ROS话题健康测量。
- 启动原因有日志依据：`xrce_serial_agent.log`本次段从03:55:18开始，03:55:19.224850建立会话、19.270773创建participant，**03:55:25.073459才开始创建话题**，输出datawriter在25.284977等时刻建立；EV失败报告在03:55:21.655落盘。`ensure()`只等进程存活0.4秒，不能代表DDS遥测就绪。Agent和worker使用相同fastdds_bridge.xml，ROS_LOCALHOST_ONLY=0；此次缺失发生在冷启动话题创建之前，不通过扩大视觉时效门限处理。
- 修正专用入口：先建立**无发布器、无数据订阅**的域0图发现探针，原调用总时间前10秒内等待5类输出来源均唯一；任意/fmu/in竞争发布者或重复来源立即退出，截止时刻优先于就绪，退出销毁探针。10秒是此次约7秒冷建图加发现余量的有界启动诊断预算，不是传感器/飞行时效阈值。就绪仅说明话题来源存在，之后原遥测源年龄/接收周期、未解锁/RC/模式/高度融合等检查仍必须通过，才可能输出EV。
- EV对象及容器读取器仅在上述图就绪后创建。随后原2秒所有权发现、源更换/竞争保护、VIO门限不变；原调用start+50秒活动截止、start+55秒父监督、60秒授权上限不重置。Agent恢复耗时包含在前10秒内，不是额外增加10秒。当前运行中的Agent不需要再次启动或重启。
- 离线新增5项实际探针函数的模拟图/时钟测试：7秒才出现源、热Agent无固定等待、缺失/恰好截止、重复源、无源时也发现竞争输出；禁止网络，节点退出已核对。`evidence/disarmed_ev_regression_20261001_035734/report.json`：**128项全部通过，0失败/错误/跳过，5.691秒**。新启动顺序未真实复测；旧失败结果保留。参数及实飞许可未改。
- 下一步需要**新一次最多60秒EV-only许可**；复用已运行Agent和感知，先无输出确认DDS图就绪，再注入并统计逐次有效融合证据，结束停止EV，不解锁/不发运动/不改参数/不重试。当前仍不具备宣布实机自主避障或H降落通过的证据。

## 2026-10-01 03:53 EV持续融合证据补齐；仅软件推进，未执行新硬件测试

- 上一轮47.17秒感知恢复属于实质进展；自动目标继续不构成新的Agent/EV授权。本轮只读确认容器running、QGC桥1870保留、未见Agent；没有重新订阅传感器、重启节点、注入EV、改参数或发控制。目标顺序仍是避障→H降落→窄门→识别投递，未缩减为感知恢复。
- 核对现有入口：容器读取器已从域176经专用管道给宿主域0；已有`--start-and-keep-agent`把Agent恢复纳入同一总预算（启动8秒截止、EV活动截至总50秒、监督截至55秒、授权上限60秒）。无需另起Agent后重新计时。实际执行仍待上一轮范围确认。
- 发现证据缺口：原`fusion_flags`只记状态变化，旧真实失败报告只有“关闭→开启”两条，不能单凭它精确量化随后每次有效融合确认及停流边界。旧报告仍passed=false、sent166、VIO stale，不追改结论。
- `disarmed_ev_session.py`新增`fusion_samples`，在原遥测源时间审核接受后逐次记录位置/航向/高度标志、源时间、源年龄及接收时间；最多400条并显式标记截断。保留原变化日志、输出门控、期限、唯一VehicleOdometry出口、退出顺序和实飞许可。
- 新纯离线`disarmed_ev_fusion_evidence.py`统计三个所需融合标志均true的采样证据区间，给出首末真实样本/数量/最大间隔/最长跨度，不外推最后true到进程退出。false、重复/倒退、无效样本或超过原flags接收2秒合同的间隔拆分区间。缺逐样本记录的旧报告为unknown，不用EV发送计数代替融合。分析不产生飞行或融合放行结论，也不替代坐标配对、重置计数与控制交接。
- 8项新增正负例涵盖21个1Hz真值样本仅支持20秒、单样本0秒、缺记录未知、任一融合退出、停流、重复/倒序/非有限/过期/类型错误、2秒边界与截断、旧变化日志不可替代逐样本。最终无硬件回归`evidence/disarmed_ev_regression_20261001_035215/report.json`：**123项全部通过，0失败/错误/跳过，5.721秒**；语法检查通过，源码/参数SHA已保存。新记录逻辑尚未在真实PX4上运行。
- 仍待上一条提出的单次最多60秒EV-only范围确认：恢复DDS、结束停止EV而保留DDS/感知、不解锁/不发运动/不改参数/不重试。收到后才取得本会话真实持续融合证据；实机避障、H降落、穿门、投递均未完成。

## 2026-10-01 03:49 重启后的单次传感器恢复通过，47.17秒；保留新链，未接PX4

### 授权、执行与当前状态（优先于下方旧PID/停止记录）

- 用户在更换电池并重启PX4/Jetson后，批准一次总长最多90秒恢复D435i/VIO、地图和导航候选；随后明确把下视相机、二维雷达及其固定TF纳入同一次恢复。装桨、未解锁、静止、未挂货；没有要求重新拆桨或地面看到完整H。**该单次许可已执行并消耗，没有自动重试。**
- 重启前的地面参考、对齐、地图与PID全部作废。执行前容器`isaac_ros_dev`为exited、restart=no，入口只source环境后运行sleep infinity；下视设备为LRCP by-id，雷达`/dev/rplidar -> ttyUSB0`。D435i的udev序列号915323051253与SDK配置912112073953属于已记录的同机双编号，未误改序列号。
- 新增`perception_reboot_session.py`，只接管启动前已停止且入口/工作区挂载匹配的精确容器ID；容器启动也计入同一个monotonic 90秒预算。内部恢复入口显式`--restore-support-sensors --expect-stopped --session-deadline`，所有新启动器独立进程组，失败清理、成功保留；无导航目标、无PX4输出API。没有调用会发布目标的旧task_sensor_probe。数值验收门限未放宽。
- 实际执行始于本地03:45:52，外层总耗时**47.171秒**，内部43.405秒；两阶段均在启动就绪后完整测量12秒，均通过。外层证据`evidence/perception_reboot_20261001_034552/session.json`；内部`evidence/perception_transport_rollout_20260930_194555/report.json`。容器目录使用UTC、宿主目录使用北京时间，二者是同一次运行，不是跨日旧证据。
- 成功保留D435i视觉-only cuVSLAM、深度、下视/H候选、RPLIDAR/TF、nvblox、A*＋路径引导APF导航候选，全部感知域176。03:48进程只读复核：宿主启动器PID分别7530、7527、7528、7529、11653、11654；容器内分别80、77、78、79、352、353。QGC桥宿主PID1870一直保留；未见DDS Agent或真实飞行运行器。PID只适用于本次会话，不能在重启后复用。

### 新会话实测结果

最终连续12秒测量证据：`evidence/perception_recovery_20260930_194621/report.json`；前一阶段为`...194558`。启动期原始数据与暂时无效时间戳保留于各自stream_timing.json，不将启动期当成稳定期，也不修改此前失败记录。

| 输入 | 测量频率 | 最大接收间隔 | 最大源年龄 |
|---|---:|---:|---:|
| VIO | 29.98 Hz | 43.74 ms | 32.37 ms |
| 深度 | 30.00 Hz | 41.29 ms | 27.69 ms |
| 下视压缩图像 | 9.50 Hz | 116.26 ms | 29.56 ms |
| 雷达scan | 11.93 Hz | 98.23 ms | 99.16 ms |
| nvBlox地图切片 | 4.87 Hz | 222.15 ms | 228.13 ms |

- VIO最大源间隔33.381ms；跟踪状态均1。初末VIO/depth/status来源GID一致、各必需输入来源唯一；无/fmu话题，导航目标发布者0、导航候选命令发布者1。**没有发送2.5m航线或规划请求，也没有验证真实路线可通过。**
- 实际相机TF为`[0.196,0.025,-0.05,0,0,0,1]`，与用户最后确认的平移及零安装角一致；下视授权内参匹配。未重新标定、未补写尚未核验的下视光学旋转。
- 建图采用初始静止参考XYZ=`[0.0036191924,-0.0062896812,-0.0047926423]`m（odom），60样本最大离散1.83mm。最终参考=`[0.0103165479,-0.0038769064,-0.0070253666]`m，局部离散1.35mm；初末相距约7.46mm，小于既定静止会话5cm核对量。这不是绝对定位精度或PX4对齐验收。
- 对应相对上升0.86m的候选地图中心z=`0.855207357686013`m，实际切片z=`0.8552073836`m；高度带below=`0.3786726824`、above=`0.3436726824`m，实际mapper与边界一致。末帧3207个已知格、431个障碍格；**不等于起飞柱/全航段/下降柱覆盖通过，未知区域没有清空**。
- 地面完整H不作为恢复条件。下视/H候选节点运行，最终阶段157条H消息、稳定H为0；未据此宣称H识别/米制对准/精降通过，也不因地面近距看不全H阻断恢复。
- 相机启动仍报`Motion Module failure`，同时启动检查收到404个连续有限组合IMU帧、约2秒；最终窗口IMU约199.83Hz。当前cuVSLAM是`imu_fusion:=false`，该告警没有消失，不能说IMU硬件故障/标定已修好。

### 边界与下一步

- 本次没有启动DDS Agent、注入EV、发解锁/运动/舵机指令、改飞控参数。`param.params.txt`、`platform.yaml`、`roundtrip_task.yaml`的SHA与03:32记录一致，`live_flight_enabled:false`，未把未核验项批量设成true。
- 软件准备先通过34项相关离线测试；实际执行后进一步补充外层最终状态读取失败也清理的分支，仅离线验证，没有再次启动硬件。完整回归首次因宿主缺少nvblox_msgs测试路径产生1个导入错误，记录`task_regression_20261001_034744`保留；这是测试环境错误，不改写本次真实恢复通过结论。
- 为测试进程补入已有host_task_build/nvblox_msgs的Python/动态库路径（未安装或修改系统环境）后，完整回归`evidence/task_regression_20261001_034824/report.json`：**198项全部通过，0失败/错误/跳过，14.353秒**，源码SHA随报告保存。仅无硬件回归，不是第二次传感器尝试。
- **下一项是本新会话VIO↔PX4持续融合验收**，不是再次重启相机或重新做棋盘格。本次明确禁止启动Agent/EV，因此需要另外确认新一次有界EV-only范围及Agent启动/退出处置；仍不解锁、不发运动、不改参数。通过后还要真实双域输入/控制交接、航段制动与覆盖、H米制投影/落点误差预算核对，才能安排受控避障往返→H对准→原PX4降落。感知短窗通过不等于这些任务已完成。

## 2026-10-01 03:32 限定双域接入已实现并通过隔离DDS验证；真实接入未执行

- 本轮先核对实际任务接线，确认旧`live_flight_runtime.run`强制域0，`flight_runtime.create_node`又把VIO/跟踪/任务场景订阅全部建在同一节点；现有真实感知候选则在域176。不是“感知复验通过即可直接用原入口飞行”。用户随后明确批准**保留两域、限定话题的软件修改与隔离验证**，真实接入/飞行另行确认。本轮没有使用这项软件许可启动域0/176节点或设备。
- 新增`task_domain_io.py`：同一任务进程内使用独立ROS Context，感知176、PX4接口0；主线程顺序处理两侧回调，每次感知pump最多16轮/约4ms调度预算（不是单个回调最坏执行时间保证），避免并发修改控制状态。仅订阅VIO、跟踪、深度、地图/高度边界、H候选、导航候选；感知侧只允许发布任务导航目标/状态，禁止/fmu以及未列入白名单的话题。不搬相机、不把传感器数据重发到域0、不做整域桥接。原PX4消息发布/许可/接管检查仍由域0的唯一运行器负责。
- 修改`flight_runtime.py`，将VIO/跟踪、TaskSceneInput及其来源查询放到明确的感知端口，PX4遥测与输出审计保留在PX4端；隔离与实际域布局分别校验，报告记录两域。非任务旧入口默认仍单域。`live_flight_runtime.py`仅在专用task模式创建感知Context并在退出时关闭；前置只读检查也到正确域查找VIO/跟踪。该专用task进程候选使用已有16MiB XML和localhost_only=1，同时作用于它自己新建的两个Context，不修改Agent/PX4或其它在用进程；尚未验证与真实Agent的这组配置。
- 补齐第二个接线缺口：原`/robocup/alignment/tracking`由会启动相机的旧alignment_camera监督器提供，当前单独感知链没有它。为避免重复启动相机，给现有`nvblox_guard`增加默认false的`relay_tracking`，仅task_navigation显式启用。直接将cuVSLAM状态及原源时间戳转成原接口String，不改写状态、不重发旧帧；原始状态来源重复/替换锁存停止转发。原始类型仍来自容器已安装的VisualSlamStatus；本轮真实容器relay未启动，其编码/锁存经过离线单测。
- 新增`tests/task_dual_domain_check.py`，默认只描述，显式运行仅用**182/183**两个测试域。实际rclpy Context、DDS、生成消息、任务输入及原运行器代码；PX4、定位、地图、导航均为合成夹具，深度1像素、地图100×100，不是全尺寸相机吞吐测试，不是A*/APF实规划、真实H检测或PX4 SITL。保持WAIT，模拟飞机也不解锁、不执行航线；只向隔离输出话题产生EV候选。单轮约8秒，10秒活动alarm、15秒外层进程截止；未在真实0/176域发布合成数据。
- 首次隔离执行`task_dual_domain_20261001_032630`因nvblox类型支持动态库未加入搜索路径而失败（0.551秒），记录保留。库已在`host_task_build/nvblox_msgs`，仅为后续测试进程补PYTHONPATH和LD_LIBRARY_PATH；没有安装依赖或修改系统库。后续结果：

| 证据目录 | 结果与范围 |
|---|---|
| `task_dual_domain_20261001_032757` | 正例通过，8.027秒，任务初始化/导航候选/合成已知空闲地图均收到；120条隔离EV候选。测试导航目标在感知域收到1条、PX4域0条；VIO没有泄漏到PX4域，PX4状态没有泄漏到感知域；两域均无/fmu话题。 |
| `task_dual_domain_20261001_032841` / `...032912` | 首轮停流/重复来源负例通过；分别STOPPED和HANDOVER，未发ARM/运动。 |
| `task_dual_domain_20261001_033004` | 扩展负例：4秒停止VIO、6秒恢复，8.027秒结束，仍STOPPED，EV计数保持40，没有自动续发。 |
| `task_dual_domain_20261001_033043` | 扩展负例：增加重复VIO来源、6秒移除，8.025秒结束，仍HANDOVER，EV计数保持41，没有自动恢复。 |

- 最终整体回归`evidence/task_regression_20261001_033118/report.json`：**191项全部通过，0失败/错误/跳过，13.519秒**；相关源文件编译通过，SHA随报告保存。新增白名单、隔离域拒绝真实域、跟踪原时间戳/失败状态保留及来源锁存测试。实际DDS各轮均关闭Context并退出，随后主机进程检查未见本次测试或真实perception/nvblox/task_navigation启动器。
- 参数导出、platform.yaml与roundtrip_task.yaml SHA均未改变；实飞许可仍false；没有把alignment_reviewed、下视光学旋转、边界/误差预算等未完成项填写成通过。新增代码会改变原代码摘要，旧实飞许可/旧代码验收不能不经复核直接套用。
- **当前下一步**：先获得并执行此前待确认的新一次90秒感知候选加载/就绪后连续验收，重建新地面参考；之后分别确认真实双域只读接入、新会话EV持续融合与控制交接。H真实米制投影/降落误差预算、现场制动与覆盖仍未核验；自主避障、H降落、窄门、识别投递均未完成。本轮只交付双域接线及其隔离验证，不扩大为实飞授权。

## 2026-10-01 03:19 观察器到加载入口的无硬件整链补测：发现并修复报告核验缺口，186项通过

- 上一目标轮属于实质进展，本轮继续避障/H降落前置感知恢复的软件验证；没有新的硬件授权，未重启设备、建立真实ROS节点、注入EV、发目标或控制。目标仍为五天内按避障→H降落→窄门→识别投递开发并完成比赛，未以本地测试替代实际任务。
- 新增`test_perception_observer_offline.py`：运行实际观察器main，但ROS API、时钟、图、消息与TF完全为内存替身，socket和子进程创建明确禁用。不是ROS/DDS实运行，也不是硬件L4证明。正例包含完整测量且地面H缺失不阻断；启动延迟/零戳原始记录保留，测量段独立通过；九类测量期负例（来源替换/重复、出现目标源、TF/内参错误、跟踪丢失、非有限位姿、地图停流、VIO停流）均触发失败并关闭替身节点。
- 新增`test_perception_trial_offline.py`：以模拟子进程/时钟运行实际加载入口，验证只在成功时保留三条自有链，初始失败不继续加载地图，冲突不启动，最终超时不重试，拒不正常退出的自有组升级SIGINT→TERM→KILL并在预算内退出。所有信号都发给内存替身，不是实际系统进程；不能据此宣称实机进程退出性能已验证。另将实际观察器代码生成的模拟报告接入实际入口，核对两者格式及正向路径。
- 新负例确实发现旧入口问题：只信`passed=true`，没有独立检查完整12秒证据；地面离散度及TF的NaN可绕过原比较；观察器超时的部分stdout/stderr未保留。修复前完整失败证据保留：`task_regression_20261001_031637`，184项、5个失败、0错误/跳过；不删除负例、不追改结果。
- 修正`perception_transport_trial.py`：强制核对域176、被动范围、非飞行报告、PASSED状态、无故障及完整12秒起止；地面参考/外参必须为有限数值并满足原范围。跨初始与最终观察核对VIO/depth/status endpoint GID相同，最终地面参考仍稳定且与初始相距≤5cm（沿用已有静止5cm容许量，非绝对定位精度），避免对不同定位会话沿用初始地图基准。每次观察独立落盘stdout/stderr，超时保留部分输出及已解析的失败报告。
- 最终证据`evidence/task_regression_20261001_031828/report.json`：**186项全通过，0失败/错误/跳过，13.844秒**，本轮新增10个测试方法（含多组故障子用例），代码编译通过，源码SHA随报告保存。未修改相机XML、数值时效门限、飞控参数或实飞许可。
- 当前硬件状态没有新采集证据，沿用03:12只读检查与03:03停止记录，不能声称感知链已经恢复。等待上一轮提出的新一次最多90秒、仅感知/地图/导航候选加载授权；收到后重新核实状态并只执行一次。感知通过后仍需另行新会话EV持续融合及导航/控制交接验收，实际避障、H降落、穿门和投递尚未完成。

## 2026-10-01 03:13 启动／连续验收分段已实现，176项离线回归通过；未再次启动硬件

- 承接03:03节，原一次90秒真实加载授权已经消耗，`perception_transport_rollout_20260930_190143`仍为失败，不追改结论。本轮只编辑、离线测试、读取已有证据及主机进程；没有重启传感器、创建真实ROS订阅、注入EV、改PX4参数、解锁或运动。03:12主机进程检查未见perception/nvblox/task_navigation启动器，Telem1 Agent PID71084仍在；其余保留链本轮未重验话题健康。先前地面参考/对齐不得当成运行中的当前会话。
- 新增纯Python、无ROS依赖的`perception_acceptance.py`，观察器显式`--qualified-window`启用：每阶段最多18秒等待；来源唯一、外参、已授权内参、定位坐标系/跟踪、必需输入等就绪连续至少1秒且累计30个新VIO样本后，才启动独立完整12秒测量窗口。导航阶段额外核对实际地图高度带、同一mapper、候选命令源唯一且导航目标发布者为0。来源检查保存DDS endpoint GID，测量阶段来源改变不放行。全程仍无控制输出。
- 原数值门限未放宽：VIO源/接收间隔≤0.3秒、源年龄≤0.25秒；深度源年龄≤0.5秒；地图接收间隔/源年龄≤0.5秒。新增明确的离线验收防漏项：深度停止接收0.5秒拒绝，核心话题零戳/重复/倒序拒绝，位姿非有限值拒绝；其余必需话题末次接收≤0.5秒作为本候选输入存在性标准，实际地图边界仍用既有MapBandEvidence标准。这些是短静态候选验收规则，不是PX4厂商门限或飞行安全证明。图/几何先决条件每0.1秒检查，非连续硬件监控。
- 启动期不就绪留在WAIT_INPUTS并保存原因；开始MEASURE以后任何被观察到的超时、停流或先决条件失效均FAILED锁存，不自动重计时或恢复。18秒到期优先失败，不能因同一时刻刚达到就绪而越过截止。原始`stream_timing.json`保留包括启动期的全部事件；测量统计单独标明起止、启动样本数量和规则，不能靠事后剪去前三秒使旧记录通过。
- `perception_transport_trial.py`新增显式`--expect-stopped`，与旧PID替换模式互斥：授权执行时先核查冲突进程及2秒ROS发现后的相关发布者，存在冲突即退出，不擅自停止未知程序。默认仍只描述计划。两轮观察各按“最多18秒就绪+12秒完整测量”，共享原90秒总预算，75秒活动截止预留清理；失败对本次自有进程组分级退出并记录剩余组，不重启容器、不重试。此模式尚未执行真实加载。
- 离线证据：`evidence/task_regression_20261001_031246/report.json`，176项全部通过，0失败/错误/跳过，12.459秒；包括本次14项时序／入口测试。覆盖启动零戳/延迟、完整12秒、缺输入、截止竞态、重复倒序、原300.058350ms间断、真停流、故障后恢复不清锁、默认不执行、冲突进程不发信号。观察器/入口编译检查通过；没有把纯Python测试称为真实ROS链复验。报告含本次源码SHA。
- PX4导出参数SHA仍为`3d3f5ba479409c0c7dc5127fd25ffcacb4356256d67f4c054545d0af65a3eb5a`；platform.yaml仍为`4137c1e9fe5792ff0e2bba6623c27c225fb490450c281899ee1a799fed3e7133`，实飞许可未修改。
- **下一步需要新的单次授权**：从已停止状态加载16MiB候选，最多90秒仅传感器＋地图／导航候选验证，重建地面参考；保留下视/雷达/PX4/Agent/QGC，不注入EV、不发送目标或运动、不改飞控参数。成功保留新链；失败停止本次新链、不自动重试。通过仅能推进至另行授权的新会话EV持续融合复验，不能直接起飞或宣布绕障/H降落通过。

## 2026-10-01 03:03 图像传输修正已形成候选；一次真实加载因启动期时效未通过，三条新链已停止

**当前运行状态必须注意：D435i/VIO、nvblox、task_navigation均已按本轮失败处理约定停止，未自动恢复或重试。下视相机、RPLIDAR、QGC桥PID1837、Telem1 Agent PID71084保留。没有PX4 EV/运动/解锁/舵机输出；不能起飞。**

### 隔离复现与候选修正

- 新增`tests/image_transport_probe.py`，默认只描述；显式模式仅在域181启动两个自有进程，发送640×480 mono8左右IR（各307200字节）及16UC1深度（614400字节），30Hz。发布RELIABLE/depth10，订阅BEST_EFFORT/depth5；这是明确的合成QoS夹具，不冒充实时相机QoS读回。无相机、NITROS、GPU、PX4、导航目标。每次约6～7.5秒，自有进程有12秒工作预算和退出清理，不清理全局共享内存、不修改内核参数。
- 默认传输初测`image_transport_20260930_185416_builtin_b11d49`，各发送150帧，仅收到133/127/117，最大源间隔133/133/167ms；16MiB初测`...185433_shm16m_a1b42a`各收到148，仅缺启动前2帧。为分离发现期，后续固定保留30帧暖机（负序号，仍落盘）并另测150帧，不用删掉丢失样本伪造通过。第一次暖机夹具`...185521_shm512k_2584a1`因随机会话名以数字开头违反ROS命名而未运行成功，已改为`s_`前缀；失败记录保留。
- 同一脚本的对照，除XML共享内存segment_size外其余XML语义一致（单元测试核对）：

| 隔离证据（目录UTC） | 测量段收到 IR1/IR2/深度（各发150） | 最大源间隔 IR1/IR2/深度 |
|---|---|---|
| `image_transport_20260930_185604_shm512k_1f6a5c` | 124 / 108 / 97 | 166.7 / 333.5 / 400.2 ms |
| `image_transport_20260930_185612_shm16m_4c9c7c` | 150 / 150 / 150 | 三路均35.4 ms |
| `image_transport_20260930_185630_builtin_83d6d7` | 145 / 135 / 127 | 100.0 / 100.8 / 133.4 ms |
| `image_transport_20260930_185838_shm16m_424fe4`（最终候选XML） | 150 / 150 / 150 | 三路均37.2 ms |

- 实际进程映射证据：默认/512KiB段约549408字节；16MiB对照确有16802336字节段，含运行时管理开销，不只是XML落盘。最终候选三路最大源年龄约10.3/13.7/17.4ms。结论为**该合成满尺寸传输场景可复现默认方案的丢帧，16MiB方案在两次有暖机的短窗中消除了测量段丢帧**；不是已证明真实cuVSLAM所有间断唯一根因。
- 新增`config/perception_dds_shm16m.xml`及`perception.launch.py image_transport_profile:=shm16m`显式选项。默认`unchanged`完全保留原环境；只为相机和VIO容器进程设置XML，不改PX4/Agent、其它ROS节点、系统默认配置或ROS域。4项启动/作用域/XML对照测试通过；任务回归`task_regression_20261001_025858`162项全部通过，0失败/错误/跳过，13.391秒。参数文件SHA及platform.yaml SHA与02:50节相同，实飞许可false。

### 用户授权的一次真实候选加载（已消耗，不得自动重试）

- 用户明确同意一次最多90秒：重启D435i/VIO及地图/导航候选、重建地面参考；保留下视/雷达/PX4/Agent/QGC；不注入EV、不发运动、不改飞控参数；成功保留新链，失败停止新链。执行`perception_transport_trial.py`一次。该入口默认仅描述，核对旧启动器精确PID/路径与后代进程启动时刻后SIGINT，旧树全部退出才启动新链；不调用会关闭整个容器的旧task_sensor_session入口。
- 证据`evidence/perception_transport_rollout_20260930_190143/report.json`：总34.513秒，小于90秒；passed=false、flight_ready=false、timing_passed=false，retained_new_pids=[]。新感知/地图/导航启动器均按约停止，后续进程复查及相机/cuVSLAM/nvblox退出日志一致；未重试，未回滚重启旧会话。当前没有可沿用的运行中VIO地面参考或地图。
- 初始感知观察`perception_recovery_20260930_190149`：VIO155条，约30.59Hz，最大源间隔34.258ms、接收间隔45.807ms、源年龄119.965ms（包含初始化）；深度158条约30.02Hz、最大源年龄20.815ms。跟踪全1，实际TF `[0.196,0.025,-0.05,0,0,0,1]`。60个末段新鲜地面样本中位数`[-0.0005101854,-0.0010350590,-0.0026614879]`m，最大相对偏差1.804mm；这是短窗重复性，不是绝对精度。由此生成地图中心z=0.8573385121m及原上下包络。
- 加载地图/导航后的观察`perception_recovery_20260930_190202`：输入/来源/高度带检查通过，地图已知3084格、障碍376格，候选输出发布者1、导航目标0、域176无/fmu；地面完整H为0不作为起飞阻断。**时效整体检查失败**：VIO最大接收间隔383.339ms、源间隔200.007ms、源年龄529.223ms；第一条地图戳为0。脚本对全观察窗口检查原门限，未按结果放宽，故停止新链。这不是PX4融合/控制或避障/H降落测试。
- 随后仅离线分析已保存stream_timing：所有VIO源年龄>250ms的事件都在窗口最初0.605～2.296秒；唯一零戳地图在2.565秒。以起点后3秒/5秒分别描述末段（**事后分析，不重判原试验**）：后3秒起剩270条VIO，最大源间隔33.364ms、接收间隔43.398ms、源年龄30.149ms；深度270条最大源间隔33.364ms，地图43条最大接收间隔226.792ms、有效源年龄231.806ms。说明剩余失败集中于新地图/候选初始化阶段，短暂稳定段确有明显改善；尚无足够独立新试验支持持续验收通过。
- 诊断流程需改正的下一步：启动阶段应保持WAIT_INPUTS/禁止控制，待新会话来源、TF、非零新鲜地图和定位连续就绪后，再开始完整、有界的稳态验收窗口；窗口内任何过期照常拒绝，不延长原数值门限，不直接用事后裁掉前3秒来冒充通过。该启动/稳态分段逻辑**本节记录时尚未实现及独立复验**，先离线补齐正反例，再申请新的有界加载验证；不得继续复用本轮已消耗的90秒许可。

本轮未改变完整目标：仍按避障→H落区→窄门→识别投递的开发顺序。隔离传输修复和短静态感知片段不等于真实导航、PX4持续融合、动态接管或比赛验收；这些后续工作仍未完成。

## 2026-10-01 02:50 图像输入与VIO对照：间断已缩小到视觉处理链，未重试EV

- 本轮在02:34试验后继续软件诊断。先读既有cuVSLAM日志，发现与当次拒绝包完全对应的厂商告警：容器`/root/.ros/log/component_container_17677_1790784604592.log:37924`，日志时间1790793201.763636264，`Delta between current and previous frame [300.058350 ms]`。因此不能再将该次间断仅归因于自建位姿读取器或宿主管道；更上游已出现相同的被处理图像时间戳空洞，仍不能直接判定相机硬件损坏。
- 版本溯源：运行中进程映射`/opt/ros/humble/lib/libvisual_slam_node.so`，dpkg/package.xml为Isaac ROS Visual SLAM **3.2.6**；宿主`src/third_party/isaac_ros_visual_slam`源码则为**4.4.0**，不是运行库源码。已实际获取NVIDIA官方`release-3.2`分支的package.xml（3.2.6）、visual_slam_node.cpp和visual_slam_impl.cpp，并读取安装版同步器/排序器头文件；未替换库。官方该版本UpdatePose中的告警比较本批图像最大源时间戳与上次处理图像时间戳，发生于调用CUVSLAM_Track之前；visual-only时不创建IMU订阅，不能无证据归因于D435i IMU排序。分支源码与软件包版本吻合，不宣称逐字节证明发行二进制构建来源。
- 修正独立观察器诊断：保存首次拒绝的话题、源时间戳、接收时刻、源年龄、既有门限和上次接受时刻；报告标记遥测冻结，另外保存每话题最后收到的**未验证**原始状态及时间。后续新鲜原始数据不更新已接受状态、不清除故障锁存、不使ready通过。各周期快照含source_fault/telemetry_frozen。该修正不放宽任何门限，不将原失败报告追改为通过；本轮未对修改后的PX4观察器做新真机复验。
- 新增显式双目输入时序选项：`disarmed_ev_observe.py --observe-existing-vio --include-stereo-inputs`通过同一有界读取器只订阅左右IR图像、VIO和跟踪状态。图像只保存header时间戳/宽高，不保存或转发像素；额外图像反序列化负载仍存在，不能当作零开销。默认入口仅描述不启动；真实EV入口未启用此选项。`vio_input_timing.py`为纯离线精确时间戳比较，100ms只是选取诊断样例的筛选条件，不是更改飞行门限。
- 离线回归：最终`evidence/disarmed_ev_regression_20261001_024819/report.json` **115项全通过**，0失败/错误/跳过，5.803秒；覆盖首错保留、故障后原始数据恢复不放行、重复数据不续期、非有限值诊断JSON、双目间隔正反例。域180合成双目/TF/位姿集成`disarmed_ev_reader_20260930_184835_stereo`通过：44包，退出0/回执确认，无/fmu；该目录UTC。没有用该合成结果冒充真实双目性能。
- 用户随后明确授权**一次总长最多30秒、仅订阅**。该次已执行且已消耗：`evidence/ev_readonly_timing_073d2dbe9de945ada45e4f1edad05eda/`，总24.352秒，complete=true，读取器退出0、ROS关闭及客户端退出均确认，停止到关闭约30ms、到回执约320ms。没有重启传感器、Agent或相机容器，没有PX4 EV/控制发布器、解锁、参数更改或自动重试。结束进程检索没有本次读取器/观察器残留。
- 真实结果：pose504、tracking504，源窗口20.702秒，跟踪全1；最大VIO源间隔166.679ms，最大源年龄46.573ms，位姿管道最大7.282ms。左IR510、右IR571，最大源间隔分别200.030ms、133.343ms。本短窗没有>300ms VIO间断，**不撤回旧失败、不证明长期连续、更不是PX4融合复验**。读取器入队/发送2090、待发0、EAGAIN=0；宿主保存2089包，主动截止时有1包未收入观察窗口，不宣称全部已发送包均落盘。
- 最关键证据：24个>100ms位姿间隔中，16个间隔内独立订阅器见到至少一对源时间戳完全相同的左右图像。最大166.678711ms间隔起止源戳1790794155802910889→1790794155969589600，内部3对完整双目均在终点位姿前到达；cuVSLAM日志第42652行在1790794155.982495717同样报告166.678711ms。这说明**至少该间断期间相机已向ROS产出配对图像，而VIO处理没有使用全部这些时间戳**；应重点定位图像传输到VIO订阅、同步器/队列与执行调度，而不是直接更换相机或继续修下游stdout。独立观察器不是VIO内部接收探针，尚不能唯一分离DDS丢失与执行器/同步丢弃。
- DDS只读状态：两进程domain176、localhost_only=1、rmw_fastrtps_cpp，未设置FASTRTPS/FASTDDS profile环境变量；内核rmem/wmem默认和上限均212992字节，/dev/shm总3.8GiB、使用约8.9MiB，存在默认大小约549408字节的FastDDS段。**这些值不是已证实的故障原因。** 下一软件实验应在隔离域用实际尺寸双目/深度消息复现传输，比较默认与有依据的缓冲方案，再决定是否需要经过授权更改新感知会话；不直接改内核参数、清理共享内存或重启在用定位。
- 开发顺序已按用户最新目标同步为**避障/H降落→窄门→识别投递→全赛程**，只改`competition_rules.yaml`的development_order和对应测试，不改比赛单架次规则。任务回归首轮`task_regression_20261001_024437`因未加入现有nvblox_msgs Python路径而报1项导入错误，失败保留；加入`host_task_build/nvblox_msgs/rosidl_generator_py`后`task_regression_20261001_024544` **158项通过**，0失败/错误/跳过，12.859秒，没有安装新依赖。当前参数文件SHA仍3d3f5ba479409c0c7dc5127fd25ffcacb4356256d67f4c054545d0af65a3eb5a，platform.yaml SHA仍4137c1e9fe5792ff0e2bba6623c27c225fb490450c281899ee1a799fed3e7133。
- 完整比赛目标仍未完成：持续融合、新会话动态控制交接、真实绕障/H落区闭环未验；窄门与投递按上述顺序推进，不能因本轮诊断/软件测试通过而启用实飞。上一轮许可仅用于只订阅实验，不能复用为EV重试、传感器重启或起飞许可。

本轮实际核对的官方来源：
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-3.2/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-3.2/isaac_ros_visual_slam/src/visual_slam_node.cpp
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-3.2/isaac_ros_visual_slam/package.xml

## 2026-10-01 本轮结果复核：观察器末态证据修正

- 已核对02:34单次试验原始报告和退出回执，确认本次授权已执行，不重复EV注入。166条EV、视觉位置/航向/高度融合开启、约7.70秒水平位置有效，以及源间隔0.300058350秒触发停止等结论保留；持续融合验收仍未通过。
- **撤回下节“结束时视觉融合均false、xy_valid=false”的实时性表述。** 代码复核确认独立观察器的SourceGuard首次源时间故障后锁存，拒绝所有后续遥测更新；report.latest中的遥测字段是各话题最后被接受的记录，不保证代表退出瞬间。最后接受记录确为融合关闭/水平位置无效，但不能据冻结快照认定当前实时状态。结束时无EV发布者来自独立图审计，不受此锁存影响。
- 后续应为观察器补充首个拒绝包的源年龄、时刻及冻结标记，再区分cuVSLAM产出、DDS接收与读取器调度造成的间隔。当前证据没有证明D435i硬件故障，也没有证明读取管道拥堵。此次仅复核文件与代码并纠正文档，没有新硬件采集、重启、EV注入、参数修改或飞行操作；原始失败报告保留不改写。

## 2026-10-01 02:34 修复后单次真实EV复测：166条，已融合但源间隔越限后停止

- 用户确认QGC重新连接后，只读检查恢复`gcs_connection_lost=false`，未解锁、Position、已落地、遥控有效、无/fmu/in发布者，遂使用此前尚未消耗的单次最多60秒授权。一次前置只读命令在权限审核超时、尚未执行后重试成功，不是重试真实EV。随后真实入口仅执行一次，没有自动重试、没有改参数/阈值、没有解锁或运动/舵机输出，也没有重启现有感知或Agent。
- 真机证据`evidence/disarmed_ev_2bc402792ab142169c37b317441672d7/`：sent=166，passed=false、flight_ready=false，活动11.857秒，worker12.128秒，入口至退出12.821秒。退出原因`VIO stale; new session required`。独立仅订阅观察器`evidence/preflight_readonly_20261001_023404/`运行54.002秒；两进程从共同启动至均退出55.089秒，小于60秒。
- **本次确实融合：**入口`fusion_flags`由视觉位置/航向/高度均false变为均true，视觉速度保持false、气压高度保持true、测距高度false；1119条配对本地位置中757条xy_valid=true，有效窗口从11012.030570至11019.730074单调秒，约7.6995秒。初次融合重置从(7,4,3)变为(8,5,4)，融合后至入口退出保持(8,5,4)。本地EV候选跨度7.6257秒；这是静止窗口，不是位移/转角或飞行控制验收。
- 新诊断定位最后拒绝包seq393：相邻被接受pose源间隔0.300058350秒，容器回调接收间隔0.303620151秒；该包容器至宿主7.912ms，到达/处理源年龄27.483ms，`accepted_by_guard=false`、`ev_candidate=false`、其它gate blockers为空。相邻源间隔比门限多0.05835ms，不能称作纯主机旧缓存延迟，也不能由此直接判定相机硬件故障。间断已存在于容器订阅接收侧或上游，仍需区分cuVSLAM产出、DDS丢包/队列与读取器调度。
- 读取器入队394、发出394、待发0、EAGAIN=0、峰值缓存429字节、最大排队2.703ms；全会话容器到宿主最大13.438ms。故本次没有复现被修复的管道背压问题，不能把背压修复误称为已消除所有间断。PX4分话题遥测检查未触发，`telemetry_receipt_fault=null`。
- 观察器保存样本中视觉位置/航向/高度各39个true快照，观察器相对时间4.060～11.812秒；xy_valid有59个true快照，4.060～15.905秒。结束时视觉融合均false、xy_valid=false，外部视觉发布者为空。`ready_continuous_s_max=6.5293`，但观察器最终passed=false且`source_fault=invalid/stale source: flags`，这个额外源时间问题不能忽略，也没有逐条原始flags拒绝年龄可倒推，需后续单独核查；不把局部ready窗口当作整体通过。
- 收尾会话匹配、ros_closed=true、reader/client退出0，停止至ROS关闭25.872ms、至回执/退出确认260.053ms。进程检查仅保留原Agent PID71084，本次入口、容器读取器和观察器均已退出。此次真实授权已消耗。
- 结论：PX4接受与融合能力再次得到实时证明，但持续EV/融合验收仍未通过。下一步优先记录cuVSLAM实际产出与读取器订阅时序/执行器调度，评估源间隔规则的实际意义及异常处理，不用旧位姿重发、不在没有证据时增大阈值；旧普通起降成功不被否定，但不能直接进入新增自主绕障/H降落。

## 2026-10-01 02:25 新授权EV-only复测：QGC失联，前置阻断，尚未注入

- 用户明确新授权一次最多60秒EV-only＋融合观测。只读前置检查发现PX4 `gcs_connection_lost=true`；arming_state=1、Position、已落地、RC有效、failsafe=false，/fmu/in无发布者。代码/参数与`disarmed_ev_regression_20261001_021954`摘要一致。没有执行真实EV入口，该次授权尚未消耗，不记成已试验失败，也不自动扩大为重启/修改网络。
- QGC转发服务active，PID1837；实时状态`/home/cfly/.local/state/robocup-qgc/status.json`显示USB已连接，Jetson地址192.168.43.228，qgc_peer=null。与PX4地面站失联一致。原Telem1 Agent PID71084仍运行。日志查询权限不足，未擅自改权限；已有服务状态/最新状态文件足以确认当前没有有效QGC对端，不能断定电脑端具体原因。
- 已请用户打开QGC、确认同Wi-Fi及电脑是否仍192.168.43.89；恢复后重新只读核验再执行本次已授权的有界测试。保持未解锁，不改PX4参数，不跳过GCS保护。
- 长期目标保持完整：按用户最新顺序部署避障→起降区H降落→穿窄门→图像识别投递，五天内完成竞赛任务。此处是开发优先顺序，不擅自改写比赛单架次执行顺序；尚未完成，不以EV局部验证代替全任务验收。

## 2026-10-01 02:20 视觉读取管道修复、GitHub依据及单次真实只读验证

- 用户要求修复并参考GitHub，随后明确允许一次最多30秒、只订阅现有cuVSLAM的验证。本轮未启动/停止相机、VIO、地图或Agent，未创建PX4输入，不解锁、不改参数；真实只读仅执行一次，未重试。未将短时联调入口改成无限期自动重启/驻留服务。
- 确认并修复一个独立的软件风险：原读取器在ROS回调内直接阻塞写stdout，宿主不及时读取时会拖住ROS回调和停止处理。新增`disarmed_ev_transport.py`：回调只入有界64KiB队列，主循环非阻塞、每轮最多8次写入；短写/EAGAIN保留未发送字节，不悄悄丢行、重排或重写源时间，队列耗尽显式失败。主动停止时不重放待发缓存，回执记录未发送字节。此修复替代01:50节的“改回阻塞stdout”方案，但尚不能证明旧0.30006958秒源间隔的唯一根因就是背压。
- 诊断补齐：每包容器收到/入队、宿主收到/进入门控的单调时间、源年龄、相对上次接受样本的源间隔、guard接受结果、候选输出及阻断原因；报告保存最多4096条`vision_trace`和退出时guard时刻。读取器回执另记录入队/发送计数、峰值缓存、EAGAIN次数、已发送包最大排队时间。没有放宽现有视觉源间隔0.3秒、到达年龄0.25秒、分话题遥测门限或故障锁存；当前专用地面入口此前已采用1秒空闲判断，本轮未修改，原实飞0.3秒空闲不变。
- GitHub实际获取成功：PX4/PX4-Autopilot `v1.16.2` 的`src/modules/ekf2/EKF/aid_sources/external_vision/ev_control.cpp`和`src/modules/ekf2/EKF/common.h`。源码`EV_MAX_INTERVAL=200e3`微秒；启动条件检查质量、相邻源间隔和最新样本是否新鲜，没有新数据时按`2*EV_MAX_INTERVAL`超时停止EV位置/速度/航向/高度融合。这是官方源码语义，不是当前实机超时测量；不能将0.3秒本地门限称为PX4同值要求，也不能靠重复旧位姿维持融合。查询PX4 ROS示例旧路径和NVIDIA RealSense launch旧路径返回404，没有宣称阅读成功或导入第三方配置。
- 最终97项无硬件回归全部通过，0失败/错误/跳过：`evidence/disarmed_ev_regression_20261001_021954/report.json`。新增真实OS满管道、分段写/EAGAIN、缓存溢出、断管、非有限数值、单轮工作有界以及原0.30006958秒源空洞仍拒绝且不恢复等测试。参数/平台摘要与此前一致，实飞许可false。
- 容器隔离域180合成测试通过：`disarmed_ev_reader_20260930_181501_stop`、`...181503_duplicate`、`...181505_before-start`；目录为UTC。宿主到容器关闭握手`disarmed_ev_container_close_20261001_021506`通过。背压负例最初两次夹具分别未填满大管道、源未就绪，因此未通过记录保留于`...181601_backpressure`及`...181742_backpressure`，未伪报通过。夹具改为4096字节自有管道且先确认有输出，再停止读取；最终`...181910_backpressure`通过：239次EAGAIN，入队114/发送16，主动停止时剩余23524字节明确记录；不排空管道也在0.269秒确认退出。证明回调/停止不被管道阻塞，不是丢数据后假称全发送。
- 新增默认只描述的`disarmed_ev_observe.py`，显式只订阅选项才读取现有域176，经同一容器读取器/管道记录时间，不导入PX4接口或创建ROS发布者。用户授权的唯一真实观测证据：`evidence/ev_readonly_timing_780fbccacab3405b801e9d9c97a429ef/`，总24.292秒，完整结束并确认回执。
- 实测468条pose＋468条tracking，源时间跨度21.571286秒，全部tracking=1，序号连续、无源时间倒退、无超过0.3秒源间隔；最大源间隔0.200029785秒。位姿最大宿主源年龄40.061ms、跟踪47.466ms；位姿容器回调到宿主接收最大10.013ms。读取器入队/发送均936、峰值缓存428字节、EAGAIN=0、已发送包最大排队2.881ms，退出无待发缓存。观测期间另有无硬件回归运行，非独占性能基准；该短窗口不证明最坏工况或长时间稳定。
- 同批真实视觉包以原主机接收时序离线回放至会话门控，PX4遥测为明确的合成健康夹具：936包生成438个内存EV候选，fault=null；未发布ROS数据。说明该次录制数据可通过修复后的门控，不是PX4融合验收，也不代表在真实DDS/飞控负载下相同表现。
- 当前结论：管道阻塞风险已修复并有正负例；本次真实视觉只读窗口连续。旧源间隔超限根因未唯一定位，持续PX4融合仍未复验。下一步可在新授权后做有界EV-only＋同步融合观察，不能直接转起飞、绕障或H降落，不能用假时间戳/旧位姿重发维持融合。

官方参考：
- https://github.com/PX4/PX4-Autopilot/blob/v1.16.2/src/modules/ekf2/EKF/aid_sources/external_vision/ev_control.cpp
- https://github.com/PX4/PX4-Autopilot/blob/v1.16.2/src/modules/ekf2/EKF/common.h

## 2026-10-01 用户操作融合后的当前状态验收（仅订阅）

- 用户表示已执行融合，要求验收。本轮仅订阅现有PX4输出及EV输入进行12.003843秒观测；结束Unix时间1790790918.3880694。不启动/停止用户程序、不创建EV或控制发布器、不解锁、不改参数，不沿用旧失败结论代替实时检查。
- 收到新鲜、递增时间戳的估计器标志12条：cs_ev_pos/cs_ev_yaw/cs_ev_hgt/cs_ev_vel全为false，cs_baro_hgt全为true，cs_rng_hgt全为false；最大接收间隔1.010584秒。视觉位置/航向/高度当前持续融合验收不通过。
- 本地位置1181条：xy_valid、v_xy_valid、heading_good_for_control全为false；z_valid和v_z_valid全为true。最大接收间隔0.025192秒、最大源年龄0.007914秒，重置计数全程(7,4,3)。有XYZ数值不等于水平位置有效。
- 本窗口EV输入接收0条；结束时ROS域0图中没有/fmu/in发布者，估计器标志和本地位置各一个PX4 DDS来源。此观测说明当前ROS EV链没有持续供数；不据此排除其他接口历史输入，也不把它解释为PX4拒绝正在持续发送的有效视觉。
- status24条均未解锁、Position、failsafe=false、QGC未失联、pre_flight_checks_pass=false；land11条均已落地。本轮没有观察到飞行动作。
- 结论：保留01:50“曾出现融合”的历史事实，但本次当前视觉融合未保持，不能认定为持续融合验收通过。EV融合依赖持续的新鲜外部视觉输入，不是执行一次便永久保持。下一步需查清/恢复用户当前外部视觉发送程序的持续运行，再同步观察融合标志和有效位置；本轮不擅自重启或重新注入，不放行飞行。

## 2026-10-01 01:50 未解锁EV：融合保持到源间隔越限，发布已停止

- 先确认五个 PX4 输出话题各只有一个发布者，飞机未解锁、Position、已落地、遥控有效，再注入。没有加长 2 秒发现门限，没有改飞控参数，没有解锁或发送速度。
- 容器内单独读取定位时，4 秒内位姿 24.5 Hz、最大间隔 0.165 秒，跟踪状态均为 1。同脚本单独运行 6.3 秒时最大源间隔 0.1 秒。因此把未解锁会话的空闲无包判断从 0.3 秒改为 1 秒，用来识别读取进程停住；样本到手后的源间隔仍须 ≤0.3 秒。实飞路径的 0.3 秒空闲判断未改。读取端标准输出改回阻塞，避免管道满时在回调里丢行。
- 证据 `evidence/disarmed_ev_ba2e51c3d86047ce86e9fae1438015a6/`：sent=109，passed=false。融合从全关变为视觉位置/航向/高度开、速度关；1159 条本地位置中 564 条水平有效，结束时仍有效，重置计数 (6,4,3)。随后一条源间隔 0.30007 秒触发 `VIO stale`，会话在 12.2 秒停止，没有跑满 50 秒。结束后外部视觉发布者为 0，飞机仍未解锁、Position。
- 这证明新会话可以被 PX4 融合。0.3 秒源间隔门限没有放宽。轴向对齐仍要在不转桨时搬动机体比较位移，避障速度仍不接入。

## 2026-10-01 01:30 未解锁EV：融合标志曾出现，连续发送未保持，避障速度未接入

- 主机在读取下一条源时间连续的位姿之前就用空闲 0.3 秒门限锁存，会把调度间隙当成视觉停流。`EvSessionGuard.pose`改为只按源时间间隔拒绝超过 0.3 秒的空洞；没有新样本时的空闲门限不变。安全遥测、未解锁、已落地和 Position 检查仍在发布前执行。所有权审计改到采样时钟之前，避免新审计把 0.5 秒独占窗口判成失败。相关单测通过。没有放宽门限，没有改飞控参数，没有解锁。
- 第一次注入`evidence/disarmed_ev_f7039438540e4856bb6e1174987659ba/`：sent=21后因独占输出窗口锁存，passed=false。融合边沿从视觉位置/航向/高度全假变为全真，速度融合保持假；705条本地位置里82条`xy_valid`，重置计数到(4,3,2)。这是飞控接受外部视觉并重置本地原点的证据，不是搬动机体后的轴向对齐。发布者随后已关闭。
- 修正采样时钟后的第二次注入`evidence/disarmed_ev_1301457b74f64202b7b0751533b952a5/`在2秒内未发现PX4输入，sent=0，没有新的融合样本。不再自动重试。结束后`/fmu/in/vehicle_visual_odometry`发布者为0，飞机仍未解锁、Position、已落地。
- 避障速度仍在域176的`/robocup/legacy_apf/cmd_vel_odom`。任务出口只有在解锁、Offboard且悬停交接后才允许水平速度，当前没有向`/fmu/in/trajectory_setpoint`发布。装桨未解锁状态下不把该速度接上飞控。

## 2026-10-01 01:16 新授权单次EV-only复测：视觉时效保护退出，未产生EV，未融合

- 用户授权新的有界EV-only真机复测以确认持续输入和PX4融合。本轮仅执行一次；不解锁、不切模式、不发运动/舵机指令、不改参数、不重启感知或Agent，保持既有装桨未解锁边界。试验前3秒只读检查：arming_state=1、nav_state=2、landed=true、RC有效、QGC未失联、failsafe=false、/fmu/in发布者为空；源年龄约12～15ms。代码/参数与87项回归报告摘要逐一一致。
- 真实入口证据：`evidence/disarmed_ev_a6bbb5cc4c1d4b12b8f31b527204cbb7/`。`report.json`为passed=false、flight_ready=false、sent=0，故障`VIO stale; new session required`；活动4.192秒，worker4.566秒，入口至退出5.263秒。没有任何EV实际发布，不是“发了但PX4未融合”的测试。
- 同时运行既有仅订阅观察器，证据`evidence/preflight_readonly_20261001_011618/`，观测54.006秒；从两路进程启动至均退出合计55.049秒，小于本次60秒总预算。EV入口提前退出后只读观察继续，没有重新注入。该次尝试已消耗，不自动重试。
- 新分话题遥测规则在本次未触发：status最大接收间隔0.512秒、flags1.011秒、land1.007秒、RC0.061秒；故障退出时接收年龄分别0.094/0.885/0.466/0.009秒，`telemetry_receipt_fault=null`。不能再将本次失败解释为上轮统一0.5秒遥测门限问题。
- 记录pose19条、tracking19条，跟踪状态均1；源时间跨度0.800754秒，已记录相邻源时间最大间隔0.133457秒。位姿在静止时XYZ不变，不能单凭此判定算法冻结。门限代码表明`VIO stale`指最后被guard接受的位姿超过0.3秒未更新；当前报告没有每包容器/宿主接收时间和每次门控阻断原因，不能从源时间间隔倒推出确切间断位置，也不能断定为相机硬件故障。warmup要求30帧，本次包数不足且在输出前锁存。
- PX4配对位置366条，xy_valid为0/366，重置计数始终(2,1,0)。只读观察器接收local5322、RC2383、flags54、status105、land52条；267个状态快照中视觉位置/高度/航向/速度融合及xy_valid均为0，ever_armed=false，source_fault=null，最终已落地/Position/RC有效/QGC在线，preflight_ok=false。观察器passed=false，不是飞行放行。因本次sent=0，不把未融合当作PX4拒绝有效EV的证据。
- 收尾确认：读取器会话匹配、ros_closed=true、退出码0、Docker客户端退出0；停止请求至ROS关闭0.097秒、至退出回执确认0.365秒。末端只读图中/fmu/in发布者为空；进程核查仅保留原Agent PID71084，没有本次入口/读取器/只读观察器残留。
- 下一步应先做无PX4输出的视觉时序诊断：同一时钟记录容器接收、管道发出/宿主接收及guard接受或拒绝原因，区分源停流、传输/调度停顿与门控未更新。保持视觉0.3秒时效门限，不盲目延长、不重启硬件试错。查明并验证后再安排新的有界EV尝试；本次未完成持续输入或融合验收，更不能接自主起飞/避障/H降落。

## 2026-10-01 01:11 分话题时效修正：离线回归及隔离DDS通过，未重试真机

- 本轮仅按用户要求修改软件和离线验证；使用ROS工程技能的门限溯源/正负例要求。没有运行真实EV入口、启动/停止Agent或感知、解锁、发运动指令、修改参数。`platform.yaml`及PX4参数导出摘要与此前一致，实飞许可仍false。此前发布1条后失败的真实会话保持失败，不追改结论。
- 撤回专用入口“所有安全遥测接收间隔均≤0.5秒”的适用性：此前4.002秒被动采样显示status最大间隔0.509秒，flags/land各1.008秒，RC0.031秒。消息源年龄仅毫秒级，发布周期不等于传输延迟。该样本很短，不能代表最坏实时性能；本轮未声称重新验证PX4源码发布合同。
- 接收门限采用现有`BenchEvGate`的分项合同：status 1.5秒、flags 2.0秒、land 2.0秒、rc 0.5秒，边界包含等号。对应约3个status周期、约2个flags/land周期，并保留较严RC门限；仅适用于未解锁静止EV联调，不是空中控制超时认证。上层新规则不再比原门限错误地收紧；原共享飞行/起降代码未改。
- 消息到达时源年龄仍须在[-0.05,0.5]秒，时间戳必须递增；重复时间戳不更新接收时刻。视觉源年龄≤0.25秒、视觉停流0.3秒、首次输出等待10秒、50/55秒期限均未放宽。首次输出前缺失/过期阻止输出；输出后超时永久锁存，恢复数据不自动续发。local仍是配对分析数据，不是本轮新增的安全接收门限；其源时间及来源发现检查保持原样，不能将此联调视为水平定位已有效。
- `disarmed_ev_contract.py`新增按话题统计和故障瞬间快照；`disarmed_ev_session.py`报告保存`telemetry_timing`及`telemetry_receipt_fault`，包括触发话题、未更新年龄、门限、各话题接收计数/最大间隔/最大源年龄。重复数据不计入有效新接收。原时间戳错误仍由原源时间故障记录拒绝。
- 最终87项回归通过，0失败/错误/跳过，5.536秒：`evidence/disarmed_ev_regression_20261001_011058/report.json`。新增6个测试方法内含多组分话题子用例：15秒虚拟时间混合周期/错相/抖动、四话题分别真实停流及重复时间戳冻结、恢复不续发、启动缺失、过旧/未来源时间、门限边界及首次输出前接收缺口。测试时钟10ms，停流检测均在对应门限后一个测试tick内；这是软件调度夹具，不是硬实时保证。测试脚本摘要也已加入证据。
- 实际专用ROS节点仅在隔离域179、合成命名空间运行五例，无`/fmu/`话题，无Agent/真实传感器/PX4连接：
  - `disarmed_ev_dds_20261001_011019_mixed`：7秒窗口中发送/接收123条；status最大接收间隔0.539秒，flags1.044秒，land1.027秒，RC0.051秒；正常未误停，随后合成解锁触发停发且不恢复。
  - `...011027_stop-status`：仅停status，其余继续，未更新1.507729秒时拒绝。
  - `...011034_stop-flags`：仅停flags，其余继续，未更新2.036579秒时拒绝。
  - `...011040_stop-land`：仅停land，其余继续，未更新2.000532秒时拒绝。
  - `...011047_stop-rc`：仅停RC，其余继续，未更新0.525037秒时拒绝。
  - 四种停流都保持发布者存在，仅停止发送新消息，故不是依靠DDS发现丢失触发；恢复发送后均不续发EV。各例`report.json`均passed=true，负例的fault是预期拒绝，不代表软件回归失败。
- 下一步：这处误停问题的软件修复已具备新一次有界真实EV-only验证条件，但本轮没有授权/执行新的真实注入。仍需真实新会话验证持续输入、PX4融合和对齐；不得由离线通过直接升级为实飞、避障或H降落验收。

## 2026-10-01 本轮EV-only联调及分析：约6.02秒退出，发布1条，统一遥测时限不匹配

- 用户新授权执行一次EV-only并分析。本轮复用Agent PID71084，不重启Agent/感知，不解锁、不切模式、不发运动/舵机指令、不改参数；执行一次后停止，未自动重试。证据`evidence/disarmed_ev_4a9ec6699b89402287413a193795b3c9/`。
- `report.json`：passed=false、flight_ready=false，sent=1，故障`safety telemetry receipt stale`。约0.521秒已完整发现五种PX4输入，之后45次审核为ready，没有重复/竞争来源。活动约5.106秒，worker约5.370秒，`supervisor.json`从入口启动到退出约6.015秒，小于60秒。sent=1仅说明本地发布过1条，不证明PX4收到或采用。
- 同步收尾本次真实路径通过：`reader_closed.json`会话匹配、ros_closed=true、exit_code=0，Docker客户端退出0，主进程确认confirmed=true。停止请求至ROS关闭约29ms，至回执与退出确认约256ms。不是仅凭Docker客户端退出作判断。测试后的纯只读图检查/fmu/in发布者为空，Agent仍PID71084。
- 已保存视觉pose32条、tracking32条，vo_state均1，视觉源时间跨度1.667秒；PX4本地位置461条、跨度4.657秒，xy_valid为0/461，重置计数始终(2,1,0)。不同坐标原点不能直接相减当作对齐误差；单条EV及静止样本不足以验证新会话融合、方向、尺度或水平控制。
- 为分析原因另做4.002秒**只订阅现有遥测**的检查，不重新创建EV输入：status8条、最大接收间隔0.509秒/源间隔0.506秒；flags4条、最大接收间隔1.008秒/源间隔1.007秒；land4条、最大接收间隔1.008秒/源间隔1.007秒。三者消息最大源年龄分别约6.1ms/1.6ms/2.2ms；RC最大间隔31ms，local23ms。最后状态未解锁、Position、已落地、RC有效、QGC未失联；视觉位置/航向/高度/速度融合标志均false，气压高度true，xy_valid=false、z_valid=true。
- **明确的软件问题：**新`DisarmedEvSession.watchdog`把status/flags/land/rc一律按0.5秒接收时限检查，与实测约1Hz的flags/land正常周期不兼容，约2Hz的status也会触边。不能将正常发布周期混为通信延迟，也不能把此前20Hz合成遥测测试通过当作覆盖真实频率。本次故障时各话题接收年龄未逐项保存，不能事后断定唯一触发话题；后续被动观察足以证明该统一门限设计存在不匹配。
- 分析存入同目录`analysis.json`。本轮**未修改门限、未重新注入**。下一步应核对同版本PX4发布合同，按status/flags/land/rc分别设计接收时限，并加入2Hz/1Hz/高频RC等混合频率正例和真正停流负例，同时保留源时间、未解锁、Position、唯一来源和故障锁存保护；修订通过离线验证后再申请新的单次实机试验。当前不能进入真机自主飞行。

## 2026-10-01 00:54 启动发现诊断与读取器同步退出已修正，离线/隔离验证通过

- 用户本轮只授权修正两处软件问题并离线验证。没有再次启动真实EV入口、没有重启Agent/相机/地图/导航，没有修改飞控参数或操作飞机，原失败硬件尝试未自动重试。
- 启动来源检查不再将所有失败统称ownership conflict。新增`disarmed_ev_lifecycle.py`分类：等待发现、2秒发现超时、重复输入/竞争输出、已发现来源丢失、来源代次变化；报告保存逐话题计数、发布节点/GID和失败列表。原2秒时限和不满足条件不输出保持不变；未以放宽阈值获得通过，也不能用新诊断倒推上次失败原因。
- 读取器增加会话绑定停止文件及关闭回执：主进程先关闭EV，再请求停止；读取器关闭ROS后原子写入PID、会话、开始/关闭单调时间、原因、退出码。主进程须同时取得匹配回执和子进程退出才确认收尾。stdout非阻塞，避免无人读取时阻塞收尾；旧独立期限保留。回执缺失/错误会话/清理未完成或退出码不匹配均拒绝。
- 81项回归全部通过，0失败/错误/跳过，约4.286秒，`evidence/disarmed_ev_regression_20261001_005125/report.json`。其中覆盖发现超时与真冲突区别、来源丢失/代次改变、错误/缺失退出回执；原EV/控制边界回归保留。
- 实际ROS节点、隔离域179：`disarmed_ev_dds_20261001_005152`正常53条合成位姿发送/接收，解锁停发不恢复；`...005318_missing`缺少local话题，约2.044秒记录明确discovery_timeout且EV=0；`...005321_duplicate`status两个来源，约0.222秒明确conflict且EV=0。均无真实PX4输入，不能称新会话对齐通过。
- 容器隔离域180读取器三例通过：`disarmed_ev_reader_20260930_165144_stop`（主动停止）、`...165146_before-start`（启动前停止、0包）、`...165147_duplicate`（重复源退出1且清理回执确认）。UTC目录对应本地00:51。
- 宿主通过真实docker exec启动**隔离读取器**再经共享目录请求停止的传输级测试通过：`evidence/disarmed_ev_container_close_20261001_005315/report.json`。请求到ROS关闭约0.319秒，请求到回执及客户端退出均确认约0.556秒，总1.559秒；退出0，ros_closed=true。本例无传感器发布源，不订阅域176、不连接PX4，验证的是上次遗留进程问题涉及的完整关闭路径。
- 原硬件失败报告和退出时刻缺口保留，未追改为passed。下一步可安排新的有界EV-only真机尝试；需新授权，不自动重试，也不因此解锁/进入水平导航。详见`DISARMED_EV_SESSION.md`新增节。

## 2026-10-01 00:44 单次真实EV-only尝试：DDS已恢复，EV发送0，来源检查未通过

- 用户授权启动PX4 DDS Agent、测试后保留它，并开始一次含启动/退出最多60秒的EV-only联调。本轮执行一次，没有自动重试；不解锁、不切模式、不发运动/舵机指令、不改参数。该次尝试已使用，不能继续当作尚未消耗的许可。
- 为将Agent恢复纳入原总时限，专用入口增加显式`--start-and-keep-agent`：共享入口起始单调时钟，Agent启动阶段8秒超时，EV活动仍截至同一起点+50秒、监督器+55秒；不重新计时。执行前参数合同检查失败项为空，70项离线回归通过，证据`evidence/disarmed_ev_regression_20261001_004321/report.json`。原许可false、参数未改。
- 真实证据目录`evidence/disarmed_ev_887652b10855448ba2ca2904b7d940b1/`。`agent.json`：Agent PID71084，Telem1 `/dev/ttyTHS1` 921600，本次新启动约0.671秒，按授权保留。新Agent日志出现client_key=1及数据写者创建，说明建立了本次PX4 DDS会话，不是引用旧日志。
- `report.json`为passed=false/flight_ready=false，sent=0，paired_px4=[]、sensor_packets=[]；故障`publisher ownership conflict`，活动约2.082秒后终止，worker约4.097秒，入口监督器约5.351秒退出1。没有成功注入位姿，也没有新会话对齐/融合/水平控制通过证据。
- **不得仅凭故障标签认定真实重复发布者。** 当前代码会将启动2秒后仍未全部发现的输入也报为ownership conflict，失败时未保存逐话题计数。后续2秒纯只读图检查显示status/flags/land/RC/local五个输入均为单一DDS来源，所有/fmu/in发布者为0；这支持排查启动发现时序，但不能据后验图证明失败瞬间原因已确定。
- **收尾记录缺口保留：**报告reader_cleanup_unconfirmed=true、docker客户端退出0，但首次只读复查仍发现容器读取器PID45209，说明客户端退出不等于读取器退出。随后针对该精确会话检查返回reader_present=false，未向任何其他进程发信号，最终pgrep未见该读取器。读取器有共享起点+50秒独立截止，但实际容器退出时刻未落盘，不能把5.351秒说成整个采集已全部退出，也不能把全链60秒退出验收写为已证明。EV主进程已结束、输出源0、Agent仍运行。
- 下一步先修正启动发现诊断和容器读取器立即退出/退出回执，保持未就绪不输出、真重复源立即拒绝；离线验证后再安排新的有界真实尝试。本轮不擅自延长发现阈值、放宽门控、重启Agent或PX4。用户要求不确定先询问，真实重试需另行明确同意。

## 2026-10-01 00:37 装桨未解锁EV专用入口：软件实现及隔离验证完成，未进行真机联调

- 用户明确选择“保持装桨，先完成专用入口的软件验证”。新增`disarmed_ev_contract.py`、`disarmed_ev_sensor_reader.py`、`disarmed_ev_session.py`，没有改旧拆桨入口或冒用prop-off标志。说明见`DISARMED_EV_SESSION.md`。
- 专用节点仅一个VehicleOdometry输出接口，无解锁/模式/位置目标/舵机/参数API，不调用起飞状态机。沿用原BenchEvGate的RC、Position、已落地、高度融合和所有权约束，增加源时间、短时限及会话序号检查。故障锁存后不自动恢复。默认命令只显示说明，不访问ROS/设备；真实选项本轮未执行。
- 采用域176容器只读定位/跟踪→私有管道→宿主EV候选，不转发运动候选、不重启当前VIO/地图。读取器核验唯一来源、来源代次和实际安装TF；不在域176投喂合成数据。真实管道与PX4端到端尚未实测。
- 70项无硬件回归全部通过，0失败/错误/跳过，4.108秒：`evidence/disarmed_ev_regression_20261001_003651/report.json`。包含超时后终止自有卡死子进程、解锁/模式/RC/位姿跳变/时间/序号拒绝与故障不恢复。旧EV转换和会话回归纳入复验。
- 域179实际专用ROS节点＋合成输入：`evidence/disarmed_ev_dds_20261001_003335/report.json`通过，52条位姿发送/接收，模拟解锁后停发且再未解锁不恢复，无/fmu话题，约4.279秒。域180容器实际只读工作进程＋合成TF/传感器：`evidence/disarmed_ev_reader_20260930_163614/report.json`通过，47包，重复源触发预期拒绝并退出1，约1.556秒；该目录UTC对应本地00:36。两者均非真实传感器/PX4试验。
- 50秒活动截止、55秒外层自有进程兜底，先停EV再清理；容器读取器有独立同一截止。进程超时用缩短的合成卡死子进程测试，不宣称进行了真实60秒联调或硬实时认证。必须把Agent恢复也计入以后明确的总预算，不能随意叠加计时。
- 本轮未启动Agent、未改变真实感知链、未创建真实/fmu输入、未操作飞机。之前60秒真实联调仍未执行，许可次数未消耗；本地实飞false、PX4参数SHA仍`3d3f5ba479409c0c7dc5127fd25ffcacb4356256d67f4c054545d0af65a3eb5a`。旧两个入口摘要与本轮前一致。
- **下一步是真实EV-only联调，不是起飞：**先在有界安排中恢复单一Telem1 Agent并核实新鲜未解锁/RC/Position遥测，再用新入口采集时间配对位姿；分析PX4融合/对齐后才讨论水平控制交接。本入口passed只证明数据输出窗口，flight_ready永远false，不能据此接通解锁、避障往返或H降落。H安装旋转/落点误差预算等缺口仍保留，不重标内参。

## 2026-10-01 本次60秒新会话EV联调：启动前检查未放行，尚未执行

- 用户明确授权一次总长最多60秒（含启动/退出）的新会话VIO↔PX4外部位姿联调。不解锁、不发运动/模式指令、不改参数、不操作舵机、不自动重试；当前现场最后确认仍为装桨、未解锁。本轮只进行了文件/进程/串口占用的只读检查，未启动Agent、未创建PX4输入发布者，测试未开始，授权次数未消耗。
- 当前进程检索未发现MicroXRCEAgent或飞行运行器；`/dev/ttyTHS1`存在且fuser未报占用。参数导出显示UXRCE_DDS_CFG=101、DOM_ID=0、SER_TEL1_BAUD=921600；这是导出文件，不是新鲜飞控参数读回。PX4 USB稳定设备链接仍指向ttyACM0，雷达为ttyUSB0，不能把雷达端口误作PX4 Agent端口或抢占QGC USB。
- 代码核对：`flight_bench_check.py`强制`--prop-off`，只复用已运行Agent，并会启动另一套domain0相机；`flight_runtime.py`的bench入口和独立`ev_bridge.py`也要求拆桨。当前感知仍domain176，飞控预期domain0，尚无经核验的限定话题跨域接入。不能直接调用现有入口、伪传拆桨标志、直接实例化类绕过入口或热迁移相机造成会话失效。
- **纠正上一轮建议：**尚未核实装桨状态下可用入口就提出“下一步直接60秒位姿联调”不够严谨。用户已授权，不是缺少重复同意；真正缺少的是与当前现场匹配且经验证的执行入口。现有软件边界保留，实飞许可false未改。
- 下一决策：①现场拆桨后适配现有EV-only流程以复用新感知会话；或②保持装桨，先完成专用未解锁EV-only入口/限定跨域输入/总时限/停流及解锁即停止等离线验证，再安排该单次硬件测试。两种都不得把旧入口伪装为可直接运行；本轮未新增采集或复试，未获得新PX4对齐或控制交接通过证据。
- 工具首次只读检查因权限审核超时未执行，按工具允许重试一次成功；这是检查工具重试，不是硬件联调重试。

## 2026-10-01 00:10～00:13 新零角度定位／地图／导航候选会话已建立

- 按用户本轮明确授权，核对容器进程后向旧定位、nvblox、task_navigation启动器发送SIGINT，确认各自子进程退出。保留下视相机、雷达和雷达静态TF，不重启容器或QGC转发。随后同在ROS_DOMAIN_ID=176、ROS_LOCALHOST_ONLY=1重启上述三链；未运行Agent、EV桥、任务飞行运行器或旧投递脚本。
- 实际TF订阅读回`base_link→camera_link`：XYZ=`[0.196,0.025,-0.05]`m，四元数=`[0,0,0,1]`。这次是运行时证据，不只是YAML值。新cuVSLAM仍visual-only。首轮12秒证据`evidence/perception_recovery_20260930_161017/report.json`（目录UTC）通过输入检查；VIO26.92Hz、最大间隔132ms。
- 另一次5秒被动地面参考观察收到63个源年龄<250ms的定位样本，取中位数`[-0.008213249035179615,-0.035110462456941605,-0.013416571542620659]`m，样本相对中位数最大3D偏差0.0015010583m。此为短窗重复性，不是绝对定位误差或PX4对齐验收。TF/参考结果见本轮工具原始输出；不沿用旧会话z。
- 新地图中心=`0.8465834284573793`m，即新地面z+0.86m；上下扩展分别0.37867268238138374/0.3436726823813836m，bounds 5Hz。nvblox与task_navigation使用同一中心，地图内存由新进程建立，未加载旧地图。目标仍是PX4中心离地1m的调试任务，不启用比赛1.3m候选。
- 整链12秒证据`evidence/perception_recovery_20260930_161250/report.json`及`stream_timing.json`：VIO22.82Hz，最大间隔169ms、源年龄28ms；地图4.87Hz，最大间隔217ms、源年龄223ms；下视9.53Hz，最大间隔150ms、源年龄50ms；雷达11.84Hz，最大间隔95ms。跟踪状态均1，各输入发布者唯一，地图/高度边界来自同一mapper，实际高度带匹配；已知格3043、障碍格372。本窗口未超过原250ms定位/500ms地图限制；不因此宣布历史时延原因根治、长期稳定或实飞通过。
- 地面稳定H仍0，不作起飞阻碍，也不据此要求架高或重标内参。候选输出发布者1，导航目标发布者0，域176无/fmu话题；因此没有请求往返航线、没有PX4位姿/运动/模式/解锁/舵机输出。`flight_ready=false`保持真实，实飞开关false未改。
- 当前容器内PID：定位launch17636/相机17671/静态TF17675/cuVSLAM17677；地图launch17908/node17947；导航launch17931/planner17985/goal_bridge17990/scan_bridge17992/guard17994/APF17996。持续工具会话定位68085、地图67478、导航38782。下轮先核实再操作，不重复启动。
- **接通后续真实任务仍未完成：**现有往返→H对准→原LAND软件交接代码保留，但新会话VIO↔PX4配对对齐、水平混合控制响应/接管没有现场验收；`roundtrip_task.yaml`中的camera.body_from_optical仍null、axes_reviewed=false，落点误差预算未填，不能因D435i零安装角而清零下视旋转。现有`flight_bench_check.py`要求`--prop-off`，当前用户报告装桨，故没有伪传拆桨标志调用。下一步须采用与现场状态相符的有界联调方案；保持不解锁、不发运动指令，不宣称避障/H降落已经完成。

## 2026-10-01 安装外参最新确认：仅D435i安装角清零，平移保留

- 用户明确确认：“无安装偏移”仅指安装角度为零；D435i相对机体仍为前19.6cm、左2.5cm、下5cm。按ROS FLU记录，`base_link → camera_link`的XYZ为`[0.196,0.025,-0.05] m`，RPY为`[0,0,0] rad`。本次不是相机和PX4空间原点重合。
- 已修改`config/platform.yaml`的`camera_xyz_rpy`，替代旧继承角度`[-0.013094,-0.052326,0] rad`；`launch/perception.launch.py`从此配置读取安装变换。驱动内部camera_link到双目/深度光学坐标系的变换不清零，不将此平移重复写入PX4 EV杆臂参数。
- 下视相机仍在PX4前12cm、下5.5cm、左右零；其内参和已有局部位移核验结论保留。用户此次确认不等于下视相机光学坐标与机体坐标旋转为零，不改写下视安装旋转或伪造外参验收。
- **部署状态：仅落盘修正，尚未重启加载。** 本轮不热改运行中的TF、不启动新感知采集、不发PX4外部位姿/控制、不改飞控参数、不解锁或操作舵机；实飞许可仍为false。原配置中的历史verification标志不是本次新外参验证证据。
- 下次加载新角度须创建新的VIO/地图会话，重新采集地面参考并计算目标高度带；旧会话地面z、地图及VIO↔PX4对齐结论不能直接当作新会话通过。保留旧证据，不删除历史文件。此次修正本身不证明真实避障或H着陆完成。
- 本次离线检查通过：YAML可解析、XYZ符号/数值及零RPY正确、启动器从同一配置绑定RPY、实飞许可保持false；下视配置仍为`camera_offset_flu_m: [0.12,0,-0.055]`。未启动ROS节点，未将静态检查计作运行时外参验收。

## 2026-09-30 23:49～23:54 在线地图＋A*/APF候选已运行；未接PX4，时效仍需处理

- 沿用户“相机等恢复、cuSLAM在Docker、不重复拆桨台架”的当前方向推进纯感知/候选链。未重启已运行相机；刚收到的地面VIO位置为(-0.00311585,-0.00113307,-0.00125251)m，与前次静止记录相同。明确采用该会话地面z=-0.001252509537152946，mapper_profile生成中心0.858747490462847、下包络0.37867268238138374、上包络0.3436726823813836，边界诊断5Hz；不是默认把地面z设0。
- 修复`task_navigation.launch.py`中APF子进程硬写ROS_DOMAIN_ID=0的问题：六个候选进程现在继承同一启动域。新增启动环境回归；9项地图/启动检查通过，完整`evidence/task_regression_20260930_235222/report.json`为158项全部通过、0失败/错误/跳过、13.537秒。旧APF源码/二进制、飞控参数/许可未改。
- 实际开启nvblox、原二维RPLidar（/dev/rplidar→ttyUSB0、115200、health OK）、沿用已确认base_link→laser=(0,0,0.1m)/零旋转，以及原`task_navigation`的NavFn A*、地图插件、目标桥、扫描桥、yaw-only TF、路径引导APF。旧APF加载依赖用已有`build/uav_task/ament_cmake_index`，成功读指定legacy_apf_shadow.yaml，不运行旧飞行/投递节点。
- 真实规划器已读回active[3]、use_astar=true、allow_unknown=false、tolerance=0.05。宿主读取root进程/proc权限不足未算通过，随后从容器内核对APF实际ROS_DOMAIN_ID=176/ROS_LOCALHOST_ONLY=1。启动初期base_footprint尚未发现，随后TF到达、规划器正常激活；没有因此自动重启。
- 12秒被动证据`perception_recovery_20260930_155151/report.json`及停止并行回归后的`...155404/report.json`（UTC目录对应本地23:51/23:54）：实际地图中心/上下界匹配任务且来自同一nvblox；最新已知格3390、障碍492；VIO约22.93Hz、深度19.60Hz、雷达11.61Hz、地图4.85Hz、下视9.52Hz，跟踪状态均1。扫描转换/yaw里程计持续出流；guard在本次运行中未见之前的取消息异常，但未关闭/复现退出过程，不能宣布旧异常根治。
- **时效未通过完整验收：**第一次观察VIO最大接收间隔约345ms，后一次仍约376ms/最大源年龄361ms，地图最大间隔528ms/源年龄531ms；第二次已无并行回归，不能认定延迟由回归独自导致。当前仅有聚合数据，无法区分发现初期积压与稳定运行事件；后续观察器已增加stream_timing.json保存原始时序，本轮不再重复采集。未放宽250ms定位/500ms地图限制。报告passed只覆盖接通/身份/高度，flight_ready始终false。
- 域176无/fmu话题；导航候选发布者1、导航目标发布者0，**本轮未请求航线、未得到往返实飞验收，也未接通PX4 EV/控制/解锁/舵机**。地面完整H及完整地图仍非起飞前提，不清空未知格、不重复棋盘格。正常起飞后获取地图的阶段逻辑保持上一轮修改。
- 当前保持运行：原相机两组不变；新增宿主nvblox launch13560/node13624、雷达13608、雷达TF13728、navigation launch14122/planner14205/goal bridge14209/scan bridge14212/guard14214/APF14216。工具会话map31843、radar64582、laserTF17118、navigation23648仍对应活进程；observer和回归已结束。后续先查这些进程，不因会话观察超时就重启。相机/地图/导航日志仍在`evidence/perception_restart_20260930_2341/`。本地实飞开关false，未恢复或创建飞行许可。

## 2026-09-30 23:40～23:42 已恢复Docker内相机/定位；保持运行，无PX4输入

- 用户明确“相机等重新开启就行，不需要拆桨进行新增控制链的台架接口核验”，并提醒cuSLAM在Docker中。本轮执行感知恢复要求，不再要求为开启相机而拆桨；现场仍按用户报告为已装桨、未解锁、未挂货、操作者在场/QGC有遥测。不将这些视为水平控制或H着陆已验收，不运行声明`--prop-off`的旧台架入口。
- 核对容器入口和相机无占用后启动原`isaac_ros_dev`，cuVSLAM继续在原Docker运行：RealSense ROS4.58.2/librealsense2.58.2、cuVSLAM12.6、接口包3.2.5，不使用宿主4.4.0源码。D435i双目＋深度、visual-only（IMU融合false）、下视相机/H像素预览已恢复，**域176/localhost，未接PX4 EV/模式/运动/解锁/舵机**。本轮没有启动雷达、nvblox或导航控制。
- 下视固定by-id当前指向video0而非旧video6。启动有相机名不匹配/不支持控件警告，但CameraInfo实际K与获准文件一致；未改标定。地面完整H不是恢复或起飞条件。
- 12秒纯订阅核对`evidence/perception_recovery_20260930_154212/report.json`通过（容器UTC命名，对应本地23:42）：VIO28.99Hz、深度25.93Hz、下视9.51Hz、IMU200.07Hz；跟踪状态均1，最大接收间隔分别约69/134/116/13ms。输入发布者唯一，域176无`/fmu/`话题，获准K确认加载。H状态106条/稳定H0只记录地面视野，不视为失败，不要求架高/重标。passed仅为出流/来源/内参，不是实飞验收。
- 启动仍报Motion Module failure，但连续IMU启动检查及本轮IMU出流均有数据；未声称告警根治，定位仍不融合D435i IMU。PX4参数、原起降核心和实飞开关false不变。
- **相机保持运行**，不是90秒采集后停止。宿主PID：perception launch7898、RealSense7933、cuVSLAM7939；downward launch11981、camera12042、H预览12044。工具会话8474/87796对应两组感知进程；后续先查真实进程，不因观察调用结束而重启。日志`evidence/perception_restart_20260930_2341/`目录标签非精确时间。禁止在当前真实感知域176启动合成传感器夹具。
- 护航异常：复制停止容器3.2.5消息产物至独立host_guard_runtime_325后，宿主复现仍因typesupport缺依赖失败（guard_transport_20260930_233835），未安装系统包。随后原容器独立域178合成测试`guard_transport_20260930_154045/report.json`通过：5.850秒，80条里程计/88条状态，停流后7条零候选，SIGINT退出0，测试子进程已退出。该报告scope的host字样是初版标签错误，实际在容器运行；脚本已修正标签并加入runtime字段。未使用真实传感器/GPU地图，**未复现原异常，不宣称已修复**，生产guard未改。

## 2026-09-30 23:32续接：已装桨现场状态与联调边界

- 用户最新确认：**桨叶已装回，未解锁、未挂货，遥控器操作者在场，QGC遥测正常**。取代之前“仍拆桨”的现场描述；无人机在地面H正上方。保留用户要求进行1m高度/2.5m往返/H降落的任务意图，不把现场确认等同于新增链路已验收。
- 本轮只读核查：`isaac_ros_dev`为exited/Pid=0，所查任务运行器、飞控通信Agent和测试进程未发现遗留；本地`live_flight_enabled=false`。没有启动容器、传感器、飞控输入或舵机，没有解锁/起飞。当前不得沿用旧拆桨授权或给已装桨飞机传入`--prop-off`。
- 针对上次guard取消息异常新增`tests/guard_transport_check.py`，拟检查宿主隔离域178的5类合成输入、停流抑制和正常SIGINT退出。**尚未运行到节点：**首次工具临时路径失效未建文件；纠正后脚本在导入`isaac_ros_visual_slam_interfaces`时失败，未创建ROS节点/子进程，无通过报告。宿主已有接口源码package.xml为4.4.0，而真实传感器依赖容器安装版本；未擅自构建/替换不同版本。后续复现应使用核对过的运行版本，不能用缺依赖或未复现声称异常修复。
- 本轮保留上轮157项回归、在线启动DDS完整链和错误高度负例结论；未重复同样地面采集、内参/棋盘格或完整H检查。真正下一步仍是新控制链的有界接口验收，再到受控空中覆盖/水平响应/H对准验收。已有`flight_bench_check.py`明确是拆桨EV联调入口；它不会自动启动Agent，也不证明水平混合控制或实飞通过。装桨状态下不运行该入口，不自动恢复实飞许可。

## 2026-09-30 23:22 按用户要求修正：地面无完整H可起飞，空中在线建图后交接导航

- 用户再次确认当前飞机在地面H正上方，近地看不到完整H属于正常视野限制；不再以地面完整H为起飞前提，也不要求先创建完整先验地图。本方案是cuVSLAM连续定位＋nvblox在线建图，观测随飞行更新，不是必须先人工建图再飞。
- **纠正此前说明不够完整之处：**`TaskFlightSupervisor.ready()`确实未直接要求起点二维包络free/H，但`TaskSceneInput.prepare()`把地图/深度出流、mapper身份与高度带检查混在起飞柱有效条件中，形成了额外地面依赖。本轮已分开`blockers`（起飞/定位/唯一输入/净空）和`navigation_blockers`（地图/深度/高度带），无地图/H可初始化地面参考；起飞仍需原PX4/VIO条件及明确核过的竖直净空，不能取消原保护。
- 原稳定悬停完成后不立即切换到零XY速度模式。新逻辑保持原XYZ位置目标，悬停期开始向规划器提供本航段目标，等待实际高度带、当前包络、鲜活路径候选及制动空间通过；成功才交接水平往返。无需H参与去程放行，也不把未知区域清成free。等待上限10秒仅为本次软件调试候选；到期走原PX4 LAND，不宣称H对准成功，不无限等待。原起降程序本体/60秒保护/0.3m横移保护未改。
- 用户所需正常链：地面定位与起飞净空 → 原竖直爬升0.86m（PX4中心离地1m）→ 稳定悬停/在线地图与路径就绪 → 前进2.5m/返回 → H米制对准 → 原AUTO_LAND。不保证起飞后前视相机自然覆盖全部近场；若悬停后仍未知，禁止水平前进，按上项超时退出。
- 针对性22项通过；集中回归`evidence/task_regression_20260930_232045/report.json`为**157项全部通过**，0失败/错误/跳过，11.935秒。包括地面无地图/H仍起飞、空中等待位置保持、地图恢复后无H仍交接、错误航段/过期地图拒绝、10秒超时普通LAND、接管/横移保护。
- 首次新增在线启动DDS夹具`evidence/task_handoff_dds_20260930_232124/report.json`实际到DONE、先HOVER才有首张地图且地图前水平输出0；但报告把落地后1条H算进起飞前，最终断言失败，**保留failed不改写**。已将起飞前/落地后H分开计数并复验，结果另行追加。此夹具仍是理想机体、合成地图/H/目标速度，不是实际A*/APF或PX4 SITL，不证明实飞精度。
- **23:22:50复验完成：**`evidence/task_handoff_dds_20260930_232250/report.json`通过，56.853秒，DONE。首张地图出现在HOVER，地图前水平输出0、起飞前H样本0；之后完成前进/返回/H对准/原LAND，最终落地未解锁。这证明修改后的实际消息适配/唯一运行器交接支持“先起飞后获得地图/H”的合成场景，不证明实机已飞过。进程已退出。原起降两文件、platform.yaml（false）和PX4参数快照SHA与本轮前一致。
- 23:24错误高度DDS负例`evidence/task_input_dds_20260930_232430/report.json`通过，6.006秒：实际适配器已初始化地面参考、起飞输入blockers为空，但错误高度仍记录在navigation_blockers，footprint_known_free为false。夹具始终WAIT/无ARM/真实控制发布器0；这是隔离输入拒绝证据，不是实机空中制动测试。所有本轮测试均已退出。
- 本轮只做本地代码/隔离域177软件测试，不启动相机、雷达、Agent或真实PX4输出，不消耗/延续上次已用完的90秒采集许可；本地实飞开关false、PX4参数不变。下视K/D继续沿用获准标定，不重做棋盘格。**仍未完成真实自主避障/H降落**：精确下视旋转/误差预算接入、实际水平跟踪与接管/制动验收、空中真实近场覆盖仍缺；上次guard取消息异常也未被本轮修复或忽略。接下来只推进这些接口与有界现场交接，不扩展投递/穿门。

## 2026-09-30 23:03～23:09 本次90秒授权结果：真实高度带通过，起点地图仍未知

- 单次检查已完成，证据`evidence/task_sensor_20260930_230223/session.json`及`report.json`。从启动到容器停止共**73.803秒**，小于90秒；容器恢复exited/Pid=0，video6/ttyUSB0无占用，本次授权已用完，**没有自动重试、没有PX4 EV/控制或舵机输出**。原参数/起降程序未改变。
- 20帧有效静止VIO采得地面初始参考(0,0,0)，不是使用默认零值；规划候选为沿本次航向前2.5m。nvblox实际中心0.86m、积分器上下界与任务高度带一致，地图/边界来自同一nvblox节点。已知格3300、障碍格403。此证据关闭“正确高度带是否真正载入”的疑问，但**切片已知不等于每列全部高度体素均被观测**，不认证整个升降空间。
- 基本出流约：VIO20.87Hz、深度20.28Hz、雷达11.19Hz、下视9.52Hz、ESDF4.85Hz；VIO跟踪状态均1，静止相对首帧最大位移约8.02mm。报告passed只表示采集完成/基本输入收到，**不是导航或降落通过**。
- `snapshot_analysis.json`进一步分离启动段：VIO起流10秒后的最大接收间隔分别为VIO205ms、下视120ms、雷达99ms、地图220ms；深度有一次268ms。地图初始1条零时间戳导致旧总体年龄异常大，不能当作实际数十年延迟；已保存原始时间记录并在后处理区分。未放宽生产时效阈值。后续采集脚本已单列零时间戳计数，仅语法检查，未新增采集。
- **路径拒绝原因已具体化：**实景起点格未知，前向0/0.25/0.5/0.75m采样未知，1～2.5m中心线采样为空闲；起点0.48m包络的图内166格全部未知，且后半包络超出当前地图边界。目标点和约(1.475,0.025)到目标这一段通过保守静态包络快照审计，但不说明飞机能安全穿过起点缺口。实际NavFn没有非空路径，关联APF导航命令0；未清空未知格、未缩小足迹、未把雷达单平面当全高度净空。
- H链605条状态、稳定H=0；已人工查看保存的`downward.png`，是近距离裁切黑白图案、无完整H。不能把这次地面视野当作H正例或据此重标内参。内参沿用，不要求重复棋盘格、不要求将比赛地面起点架高。正常离地高度下的完整H获取/米制对准仍需实机证据。
- 子进程均退出，guard退出码1，日志为rclpy取消息时Python转换异常；本次活动循环未报child-exited，但旧日志未带该异常时刻，不能武断称其发生于正常运行或仅为退出竞态。已给后续脚本增加停止前子进程状态记录；未重跑硬件，也不通过吞掉所有异常伪造退出成功。此项尚待针对性核查。
- 下一步**不重复同样地面采集、不换规划器、不做投递/穿门**。在真实飞控输入/现场起飞柱条件复核满足后，按已验证原起降先竖直到目标高度、悬停获取近场地图/完整H；未建立包络可行路径或H条件就不得进入水平/H下降任务。起飞后可能改善视野但不能预先保证覆盖；若后方/机下仍未知，需有来源的局部净空方案或感知补盲，不能自动将未知设free。本次传感器许可不扩展为解锁/飞行许可。

## 2026-09-30 新授权：一次90秒仅传感器/地图/导航候选检查

- 用户重新确认仍拆桨、未解锁、未挂货，明确允许一次总长最多90秒（含启动和退出），要求只推进避障＋H降落，不跑其它任务。此次不输入PX4 EV/控制、不改参数、不操作舵机、不自动重试。
- 已只读确认容器处于exited、入口仅加载ROS后exec sleep infinity，相机video6/雷达ttyUSB0无占用；未运行Agent。新增`task_sensor_session.py`只拥有原先停止的这一个容器，外层预算包括启动/退出，70秒截止活动采集，81秒兜底关停，恢复容器停止状态；`task_sensor_probe.py`仅域176真实传感器/地图/候选。
- 先从20帧有效稳定VIO捕获地面初始位置，再按0.86m升高/0.14m地面中心设置地图高度带；不把初始z默认0。下视用原标定和压缩链，雷达沿用用户确认旧安装朝向/当前0,0,0.1m配置；不启动旧`sensors_extra`中的自动重启或其它节点。旧APF包索引采用已有容器可见build索引，不修改旧源文件。
- 启动前2项容器所有权/拒绝回归与语法检查通过；接下来提交单次硬件调用，结果另行追加。此记录本身不代表采集成功或实飞放行。

## 2026-09-30 规则复核与实施顺序（最新要求，历史记录保留）

- 用户明确研发验收顺序：**①避障＋识别H对准降落 → ②自主搜索与三件快递投递 → ③穿门 → ④串联整场比赛**。取代历史“先窄门后投递”的安排，不表示比赛先降落再投递。整场拟为起飞/稳定→搜索与在线避障→投递→返航或穿门→H降落；返航得分歧义仍需裁判解释，不自动记满分。
- 本轮重新全文读取本文和两份PDF文字，继续采用第8.3节用户裁定：解析的10×8×4m场地、1～3m障碍；2026的随机红十字/权重、零碰撞避障、电机圆降落判分。旧T265/无TFmini配置只借鉴算法与任务接口，不导入旧定位/高度参数、固定投递航点或启动时舵机动作。
- **用户已确认：调试保持PX4中心离地1.00m、相对升高0.86m、悬停3s；另记比赛候选中心离地1.30m、稳定12s，不启用实飞。** 规则要求离地超过1m、稳定10秒以上；候选按未载货下包络0.15m估算最低点约1.15m，未含姿态/定位误差，载货后须重算。顶部约1.415m，超过此前仅确认到1.3m的起飞净空，因此不是当前场地放行值；穿门1.5m顶网须另算顶部/倾斜/误差，不能照搬候选高度。
- 当前优先cuVSLAM连续里程计＋nvblox可观测高度带地图＋NavFn A*＋路径引导APF侧向避让＋独立制动约束，依据已有同场景对照，不宣称APF天然更稳定。DWB保留对照，纯APF曾在双箱入口停滞。未知格不清空；固定高度二维切片不等于1～3m障碍、升降全空间验收。
- 阶段①已有真实输入适配/唯一控制出口代码与原起降→往返→H对准→原PX4 LAND隔离DDS交接；**真实自主避障和H着陆未完成**。低空二次对齐/持续视觉慢降未启用，不能保证普通LAND完全落圈。下一验证重点是新目标接口下实际A*/APF、真实地图/H输入、PX4混合目标响应，不重复已认可内参标定。
- `config/competition_rules.yaml`补齐无预给靶坐标、在线水平零碰撞避障、单次≤600s、剩余<180s禁止起飞、裁判指令后30s不起飞本次0分、申请起飞后不得再电脑操控、紧急接管终止且不自动恢复。它仍是**规格记录，尚未作为全比赛运行监督器执行**。
- 投递下一阶段需自主搜索/去重、目标高度、三货位单次释放及反馈；100g±5%是2026规则，14.5×8.5×10cm仅解析资料，实物和挂载外廓待确认。穿门需现场门位置感知、门前对准/固定航向、全机含罩/载货扫掠和顶网净空，不只验A*中心线。
- 原四小时窗口未按时完成，不重置期限。本轮不启动传感器/Agent、不写飞控参数、不解锁或操作舵机；预计超过20分钟的安装/搜索先征询用户。

### 22:34记录的测试收尾更正

- 下节所说“完整交接仍在运行”已结束：`evidence/task_handoff_dds_20260930_223509/report.json`通过，55.83秒；理想机体/合成地图/H，不是PX4 SITL或实飞。
- `evidence/task_input_dds_20260930_223630/report.json`错误0.6m地图高度负例按预期拒绝：覆盖失败、保持WAIT、未ARM、真实输出0。另有fresh字段false，不能宣称高度是该次所有前置失败的唯一原因。后续须用匹配正负对照区分输入时序和高度拒绝。

### 22:50新增：新目标接口下实际A*＋APF隔离验证通过

- 本地从已有源码构建nvblox Nav2 CPU插件至私有`host_task_build/nvblox_nav2`、`host_task_install`，未安装系统依赖或启动容器。`task_navigation.launch.py`仅提取`planner_parameters()`供测试原样复用，原配置值未改变，真实入口未运行。
- 新`tests/task_astar_apf_check.py`运行实际NavFn A*、nvblox Nav2地图插件、目标动作桥和C++ APF，核验活动参数`use_astar=true / allow_unknown=false / tolerance=.05`；绕墙路径、返程旧航段拒绝、封路及未知区域拒绝、目标过期停发均通过。证据`evidence/task_astar_apf_20260930_225045/report.json`：约10.04秒、10条非空路径、49条关联候选；域176本机回环、无`/fmu/in`、全部子进程退出，源码/二进制摘要运行前后不变。
- **范围：**地图/雷达/位姿为合成静态场景，去返程以两个静态观察点验证，不是动态绕飞，不是GPU建图、PX4 SITL或H识别着陆。它补上此前新接口仅用模拟直线规划服务的缺口，不能替代真实覆盖/制动/H外参和混合控制响应验收。
- 第一次调用因误用不存在的私有install根`local_setup.bash`导致消息导入失败，未启动测试节点；改用各包`share/.../local_setup.bash`后上述测试通过。保留此失败事实，不将其计为一次通过。7项规则规格回归通过，复验与回退见`TASK_HANDOFF.md`。
- 22:52集中回归**152项通过、0失败/错误/跳过**，证据`evidence/task_regression_20260930_225212/report.json`，约11.42秒。原`flight_supervisor.py`、`live_flight_core.py`、PX4参数导出和实飞许可false配置的SHA256与本轮前一致；测试子进程均已退出。
- 下一步拟做一次总长最多90秒（含启动/退出）的仅传感器＋真实地图导航候选检查，不发PX4 EV/控制、不改参数、不操作舵机、不自动重试；实施前核对当前拆桨/未解锁/未挂货状态并取得新有界采集授权。重点是地面初始化后的实际起点/通道覆盖，不把地面H裁切图反复当正例要求，不重复棋盘格标定。若启动或数据条件不满足即停止，保留明确缺口。

## 2026-09-30 22:34续接：修正本次航线的地面初始化/地图高度一致性

- 复核发现旧 `ground_navigation_shadow.launch.py` 仍是0.6m爬升/0.15m地面参考的演示配置；它不适用于当前“PX4中心离地1m”的航线。保留旧入口与历史试验，新增 `ground_task_navigation.launch.py` 和 `task_map_profile.py`，统一按相对升高0.86m、地面中心0.14m生成地图和导航配置。`initial_z`必须显式提供，不能默认把里程计零点当地面。
- 不再只凭 `full_height_slice_reviewed=true` 接受地图。当前nvblox源码中 `DistanceMapSlice.origin.z` 来自实际ESDF切片高度，`esdf_slice_bounds` 的上下界Marker来自内部积分器min/max；新任务输入同时检查中心、上下界对、本机全高度带覆盖、源时间和地图/边界发布者一致性。发布端身份改变即使名字未变也要求新会话；未知栅格仍不视为空闲。单纯高度一致不证明障碍观测覆盖或制动安全。
- 新地图入口将上下界各保守扩大半个0.05m体素（2.5cm），仅用于吸收启动参考的小偏差；运行时仍须完整覆盖任务所需高度带，不能用中心相近替代覆盖。这是几何模型余量，不是实测定位精度。新入口以5Hz发布积分器边界诊断；旧nvblox入口默认0Hz不变，未热改任何运行进程/飞控参数。
- 上下界为两个DDS消息，已避免“先收到新下界、尚未收到新上界”时误判失效：仅原子更新同时间戳的一对，旧完整对必须仍在有效期内；错误帧/删除/非平面边界及过期仍拒绝。
- 22:33新回归 **145项通过，0失败/错误/跳过**，证据 `evidence/task_regression_20260930_223353/report.json`。6秒带边界证据的WAIT输入DDS复测通过：`evidence/task_input_dds_20260930_223411/report.json`。完整交接复测已结束并通过，见顶部收尾更正及`evidence/task_handoff_dds_20260930_223509/report.json`；这些仍为合成地图/飞机数据，不是实际GPU建图或实飞验收。
- 没有重新索取或修改已认可的相机内参，没有启动相机/雷达/Agent或真实飞控输入。真实地图覆盖、H运行几何和实机混合控制响应仍未验收；窗口已超时，未将目标重置或报告为实飞完成。

## 2026-09-30 本轮最新：真实任务接口已实现，完整隔离 DDS 交接通过；未实飞

- **22:25最终集中回归：136项通过，0失败/错误/跳过**，证据 `evidence/task_regression_20260930_222515/report.json` 与 `unittest.log`，包含本轮源码SHA。真实C++ APF＋实际规划动作桥另已完成隔离DDS验证：`evidence/task_apf_link_20260930_222217/report.json`，20次路径请求、105条关联候选，去/返程方向与旧航段拒绝、目标过期停发通过，测试进程已退出。此项使用模拟直线路径动作服务，**实际APF运行了，但实际A*与真实障碍地图未在本轮该测试运行**。宿主APF二进制SHA256为 `67e8a369b0b094f4b529ff895a631d91c470fb39e5efc221dba18ab890ef61d2`。部署/回退与隔离复验说明见 `TASK_HANDOFF.md`。
- 本轮请求为“完成真实导航输入、水平控制输出与原起降程序交接”。新增 `task_scene_input.py`、`task_flight_controller.py`、`task_dispatch_contract.py`、`task_flight_release.py`、`task_flight_runtime.py`；修改通用运行器为显式任务类型接入。真实输入使用已有 VIO、PX4 遥测、深度、nvblox ESDF 切片、H 像素候选和带航段 ID 的导航命令，不再只接受外部手工拼装的 Scene。水平与原起降使用同一组四个 PX4 发布器，不另启动旧任务/舵机通讯器。
- 原 `flight_supervisor.py` 与 `live_flight_core.py` 源码未改。新独立子类仅在原稳定悬停完成时交接：PX4中心离地1.00m＝相对升高0.86m → 去程2.5m → 停稳 → 返回 → H对准 → 原PX4 AUTO_LAND。起飞阶段0.3m横移保护保留，导航使用固定yaw、XY速度≤0.15m/s与Z位置混合目标。输出端额外禁止未悬停进入导航、超速、过期/重复批次、LAND后重新接入导航及接管后恢复。原竖直 `FlightDispatchContract` 未放宽。
- 用户本轮明确同意“先对准再交原程序降落”。随后提出“下降一定高度后二次对齐再慢降”，已检查 `src/uav_task`：2025提交 `cd11857` 的 `waypoint_flight_test.cpp` 是视觉对准/悬停后 `land()`，不是分段精降。当前未提交的 `obstacle_course_mission.cpp` 另有视觉修正下降到0.35m后LAND，但使用 `-position.z` 当离地高度、12cm对准容差、丢目标超时普通降落及高度阈值完成判定；不直接复用。其圆环投影焦距875px也不同于当前获准内参。保留旧文件，不运行或覆盖。低空二次对齐高度和速度尚未确定/启用；降低高度可能裁切H，不宣称能保证完全落圈。
- APF接口新增携带路径时间、定位时间、扫描时间的原子候选；`navigation_goal_bridge.py` 用 Nav2 `ComputePathToPose/GridBased` 将当前目标、路径、候选关联，换航段拒绝旧命令。新增 `task_scan_bridge.py` 按实际TF转换二维雷达，要求XY原点共点/平面安装，不猜倒装方向、不把未知射线改为空闲；新增仅规划/候选的 `task_navigation.launch.py`，不启动DWB、飞控、舵机或相机。
- 首轮81项原起降/许可/新交接回归通过；新增真实消息适配测试中倒装雷达夹具曾错误保留Quaternion默认w=1，修正夹具为有效180°旋转后22项针对性测试通过，没有放宽运行时四元数检查。APF已在宿主独立目录 `host_task_build/apf` 编译；旧root所有的build目录写入失败，未改权限/删文件，改用新目录。已有nvblox消息定义编译至 `host_task_install`，未安装系统包、升级环境或启动容器。
- `evidence/task_input_dds_20260930_221734/report.json`：6秒真实消息类型/DDS输入测试通过，WAIT态，0个真实PX4输入发布器。`evidence/task_handoff_dds_20260930_222022/report.json`：完整新运行器DDS交接通过，55.83秒，真实消息适配/READY/AUX/ACK/混合输出/原LAND均走实际代码；**飞机、定位、空地图、H中心和目标跟随速度均为合成夹具，本轮这份报告未使用真实A*/APF规划，不是PX4固件SITL，更不是实飞**。
- 新任务实飞入口必须有单独 `TaskFlightPermit`，旧竖直起降许可不能放行水平任务；绑定本次配置、APF源码/二进制与已审阅的任务证据。`config/roundtrip_task.yaml` 中未审阅的区域、地图高度覆盖、制动/落点误差等不伪造通过；当前下视标定继续使用获准原K/D，没有要求重复标定，也没有凭粗略方向填入缺失的旋转矩阵。本地 `platform.yaml` 实飞许可仍false，未改PX4参数、未创建真实控制发布器、未解锁/飞行/操作舵机。
- **剩余边界：**完整真实传感器下的地图覆盖与H米制输入、实际规划/APF到唯一出口联调、PX4混合目标响应/接管与实飞验收仍未通过。当前是“接口代码＋隔离交接通过”，不是“今晚避障/H实飞已完成”。原四小时截止未完成的事实不变。后续以真实链路有界核验为重点，不重做已解决的棋盘格/内参工作。

## 四小时推进窗口（2026-09-30，最新工作区）

### 当前总览：用户要求停止重复标定，转向避障/H降落实机接入

#### 新增往返任务代码；用户已明确PX4中心离地1m

- 用户再次要求编写飞行测试代码并执行“前进2.5m、返回起点降落”，随后明确高度定义为**PX4中心距地面1m**。未载货落地中心离地0.14m，因此本条航线相对爬升量为**0.86m**，不是1.0m。新任务同时检查起点相对高度和同一地面平面的绝对高度；不会把里程计原点当作地面，也不使用近地TFmini读数替代。
- 新增`scripts/roundtrip_flight_test.py`的`RoundTripMission`，复用现有导航/H任务接口：按起飞航向生成2.5m前方目标，终点连续停稳2秒后切换起点目标，再停稳后才进入H对准/原降落流程。导航目标带会话/航段ID；返航时旧去程候选不能继续使用。保留未知地图/制动空间、过期输入、遥控接管和H缺失检查。`Scene`仅追加`navigation_goal_id`字段，原实飞控制核心和竖直起飞0.3m横移保护不改。
- 新增`tests/test_roundtrip_flight_test.py`、`tests/test_roundtrip_sortie.py`及`tests/roundtrip_sortie_replay.py`；现有导航、H下降、起飞/任务交接等8模块合计**47项回归通过**。新任务及高度修正相关12项包含任意里程计Z原点、错误地面/错误爬升拒绝、航向换算、去程不能提前降落、旧目标命令拒绝、未知地图、H缺失和接管锁定。
- 正确高度完整理想回放证据：`evidence/roundtrip_replay_20260930_214528/report.json`。DONE，模拟时间约61.95秒，PX4中心最高约1.000m；2.5m目标按8cm到点容差在前方2.421m停稳后返航，最终H对准、落地及未解锁状态完成。报告带源码摘要。所有这些均为理想机体/定位、合成H与空地图；速度输入是目标跟随夹具，**本轮未运行真实NavFn/APF节点，也不是PX4固件SITL或真实飞行**。模拟毫米级终点误差不是实机精度。早前`...214209`按相对升1m，保留作历史，不作为本次已澄清高度的验收证据。
- 用法见`ROUNDTRIP_FLIGHT_TEST.md`，描述入口和回放均不创建ROS发布器。当前交付是可嵌入既有链路的任务代码与离线验证，**真实导航目标/候选适配、唯一PX4任务出口和实机避障/H降落仍未完成**；不把这次部分进展写成用户要求已完成。未启动容器/Agent/相机，未发解锁或运动指令，未修改PX4参数；本地实飞许可仍为false。

#### 最新请求：离地1m、向前2.5m、返回起点降落；未执行实飞

- 用户明确要求上述真实飞行避障与降落。记录为本次任务意图，不把技术未就绪误报为用户未授权，不扩展为投递、舵机操作、改参数或其他航线。
- 中断后重新只读核查：相机容器为exited/Running=false/Pid=0，精确查询无MicroXRCEAgent进程；QGC用户服务active。Docker报告ExitCode=255、OOMKilled=false；停止原因未确定，不仅凭时间字段推断原因。没有本次运行中的测试句柄。
- 真实`live_flight_runtime.py`仍仅支持起飞/悬停/普通降落，`flight_dispatch_contract.py`仍强制竖直包络；`sortie_input_shadow.py`和`sortie_shadow.py`仍为固定域176/隔离路由。尚无已验收的真实避障/H任务出口，因此未发解锁、起飞、水平运动或降落指令。不能通过删除保护或改名隔离话题来代替真实接入。
- 必须补齐真实定位/PX4链、真实任务输入/唯一控制输出和接管边界验证，并核对本次航线与装桨现场条件。用户停止重复标定的决定继续有效；当前软件接入缺口不归因于标定资料。

#### 本地许可已按授权关闭；单次联调调用中止，未验证成功

- 用户已允许临时关闭本地实飞许可。仅将`config/platform.yaml`的`live_flight_enabled`从true改为false；旧SHA256为`43a24f84189771277947e6a8d8c213800172592264893c6217ce0b0c908b9761`，现为`c8d6f37e267e0079d4b6d1e98d8bf40713c1984c4f7a66f816c213b3eed75e8c`。未自动恢复true。PX4参数导出SHA256仍为`3d3f5ba479409c0c7dc5127fd25ffcacb4356256d67f4c054545d0af65a3eb5a`，未写飞控参数。
- 新增可选`bench_session_deadline.py`及联调/相机监督器绝对单调时钟期限，预留最终样本和退出时间。宿主/容器time namespace相同；5项时限、8项观察顺序、9项输出隔离回归共22项通过，语法和静态参数契约检查通过。此为软件证据，不是实机时限/定位验收，原实飞控制核心未修改。
- 提交了一次55秒安排、60秒兜底的相机定位+EV联调调用，工具返回aborted且无执行结果。中断后未发现运行中的Agent/联调/相机、当天新flight_bench_check证据或Agent日志更新；不能报告通过，也不能仅凭aborted保证绝无短暂执行。遵守不自动重试，未再次提交该调用。
- 原四小时21:20目标未按时完成，不因追加授权重置期限。真实自主避障与H降落仍未完成。

#### 历史：60秒实机联调授权与启动前检查（最新状态以上文为准）

- 用户已明确确认拆桨、未解锁、未挂货、遥控可接管、QGC有遥测，并允许一次最多60秒相机定位+PX4外部视觉位姿联调：不解锁、不发运动指令、不改参数、不操作舵机、不自动重试。无需重复索取这些确认。
- 尚未启动Agent、相机或任何PX4发布者，本次尝试未消耗。启动前发现原`config/platform.yaml`实际为`live_flight_enabled: true`（此前已飞普通起降的本地许可）；`flight_bench_check.py`、`flight_runtime.py`的bench入口和`ev_bridge.py`均明确拒绝此状态。这是Jetson本地软件许可，不是PX4参数；没有修改它，也没有通过直接实例化节点或改路由绕过检查。
- 需向用户明确说明并确认是否允许在本次拆桨联调期间临时将这一个本地实飞许可置false，保留原值及文件摘要作为回退依据；不改变任何PX4参数、标定、几何或检测阈值。未获准前不静默改动已成功实飞的配置。
- 同时，旧工具的观察时长不等于包括预热/退出的会话总长；执行前仍须落实总时限监督，不能直接用`--duration 60`充当60秒总限制。未执行的方案不记为验证通过。
- 21:19只读基线：`platform.yaml` SHA256为`43a24f84189771277947e6a8d8c213800172592264893c6217ce0b0c908b9761`，仍为实飞许可true；参数导出SHA256仍为`3d3f5ba479409c0c7dc5127fd25ffcacb4356256d67f4c054545d0af65a3eb5a`。精确查询无Agent进程。本轮未改以上文件、未消耗物理尝试。
- 时限复核：旧工具另有2秒发现、最多35秒等待准备、5秒最终样本、最多20秒相机退出等阶段；相机自身退出包含8秒SIGINT等待及5秒SIGTERM等待。即便把主观察设为20秒，也不能保证整次会话60秒，因此不可直接运行后再把超时归为现场原因。

#### 21:13调整推进重点（覆盖下面21:10的暂停安排）

- 用户明确：昨天已做支撑条件下的测量，确认原标定文件准确，不再进行这项重复验证；应把时间用于避障和降落真机操作。按此执行：保留既有标定文件、内参和12.5mm纠正记录，不再索取支架或重复棋盘格/飞行高度标定。用户确认作为现场来源记录，不冒充本轮新测量，也不把自动下降开关直接置true。
- 重新阅读真实入口后确认主要软件缺口：`scripts/live_flight_runtime.py`只执行起飞/悬停/普通降落；`scripts/flight_dispatch_contract.py`明确限制竖直位置目标与固定XY/yaw。新`scripts/sortie_input_shadow.py`固定域176和隔离输入/输出，仅服务合成试验。当前没有已验收的真实A*＋APF/H任务输出入口；不能仅将话题改到`/fmu/in`，也不能把缺口推给标定资料。
- 实机推进顺序改为：核对现有真实定位/PX4/RC链路→完成独立任务输入/输出适配与接管边界（保留原普通起降入口）→拆桨核验真实输入下的任务候选与唯一输出责任→另行确认小范围避障和H对准/下降实飞。基础定位联调不是新增避障/H任务验收。
- 已询问当前拆桨/未解锁/未挂货、遥控接管和QGC状态，并请求一次最多60秒的相机定位+PX4外部视觉位姿联调授权；不发解锁/模式/运动指令、不写参数、不动舵机、不自动重试。收到答复前不执行。旧`flight_bench_check --duration`仅限制主观察期，还存在相机预热和清理时间，不能将其直接当作60秒总时限；若使用须另有明确总时限监督，不能悄悄超出许可。
- 原四小时21:20期限不重置，已用时与未完成事实保留。不承诺剩余分钟内能把尚未完成的生产控制接入和实飞验收全部补齐；本轮不为赶时限删除原竖直起降保护。

#### 21:14～21:16实机通信接入检查（只读）

- 精确进程查询`pgrep -ax MicroXRCEAgent`未找到进程，`fuser /dev/ttyTHS1`未找到占用；串口节点存在，Agent程序位于`/usr/local/bin/MicroXRCEAgent`。现状不是正在运行的DDS联调会话，不能只看旧日志中的session established就当作连接正常。
- QGC用户转发服务active，PX4 USB稳定路径仍指向ttyACM0；这是独立的USB/MAVLink通道，不代表Telem1串口DDS链可用。无需停止QGC、抢占USB或改飞控参数。
- 下一次经确认的拆桨联调先在既有Telem1链路`/dev/ttyTHS1`、921600恢复单一Agent，验证当前新鲜PX4遥测；连接未建立则停止，不自动重启飞控。现有`flight_bench_check`仅复用已运行Agent，直接调用会因Agent缺失退出；本轮没有执行启动。
- 相机/EV主观察阶段与预热、最终样本和退出时间必须区分；未把旧工具`--duration 60`误称为完整会话60秒。实机动作仍等待已发出的当前现场确认与一次有界联调授权，不重复索取标定资料。

#### 21:10历史暂停点（已被上述用户新指示替代）

- 连续三轮未取得改变下一步的现场答复，自动推进标为受阻，不以重复状态核查或重复名义场景测试充当进展。原目标保持不变，真实自主避障与识别H降落尚未完成。
- 21:09只读核查：容器仅有原docker-init和sleep infinity，下视/dev/video6无程序占用，QGC用户服务active；没有后台测试需要继续等待。配置单格边长0.0125m、本地位移尺度一致为true，完整外参/米制精度/落点边界/自动下降仍为false。
- 待用户确认：能否在拆桨、未解锁、不改相机安装条件下，使用稳固支撑让下视相机观察完整H，并独立量出镜头至标志平面的距离。先确认条件，不要求手持悬空，不自动安排架高；这是静态测量布置，不改变比赛地面起飞要求。此前有界采集次数已用完，确定布置与具体采集范围后须取得新授权。
- 恢复后先补真实高度下的投影与方向误差证据；后续仍需实际地图/起降柱覆盖、定位下降漂移界、新任务生产控制接口与失效接管验收。单次H观测通过不等于以上条件全部完成，也不直接放行装桨起飞。
- 21:20截止与功能冻结保持；本次只追加交接记录，不启动传感器、不写PX4参数、不输入控制、不操作电机/舵机。

#### 棋盘格最终纠正：12.5mm，近期图像尺度矛盾已解决

- 用户明确：单格**边长12.5mm，不是对角线**；此前17mm为读数错误。本次更新计量元数据和离线诊断的显式输入，不新增控制功能、不改内参、不采集、不启用实机。
- 对19:49起点/19:56终点的同一格四角，按12.5mm重算：板面距离约71.81～74.88mm、平均73.497mm；加约10mm板厚得到离地83.497mm，与安装尺寸推算85mm相差约1.503mm。由格长得到的平均尺度换算位移为前35.329mm、右10.493mm，与用户尺测前36mm、报告右11mm分别相差约0.671/0.507mm。**当前地面局部尺度与方向一致，原约25mm矛盾关闭，不再要求重复同一34/36mm移动测试或追问17mm。** 尺测/板厚/相机光心的不确定度未量化，不能将这些数值差当作相机精度，也不外推为飞行高度误差界。
- 证据：`evidence/board_endpoint_20260930_200950/square_size_correction_125mm.json`为纠正与历史失效说明；`evidence/board_endpoint_20260930_210050/report.json`为用原图实际重算。离线脚本改为必须显式传`--square-mm 12.5 --operator-forward-mm 36`，不再硬编码错误17mm/旧估计34mm。f字母误匹配仍拒绝，不平均；结论使用人工核对的同一物理方格与亚像素四角。
- **撤回旧结论：**`chessboard_plane_20260930_185523`早前单图的“约80.06mm离地、仅差约5mm”依赖错误17mm格长，不能继续作为当前安装证据。按12.5/17比例，该旧场景板距约51.51mm、加板厚约61.51mm；不假定其与后来起终点场景相同，也不据其覆盖实测安装值。保留原报告，元数据将该旧估计标为失效；用户曾接受5mm的历史答复保留，但不是对失效计算的重新认可。
- 旧标定JSON中19mm是旧标定资料的历史输入，**没有据本次现场格长改写**；K/D、1280×720分辨率和安装偏移`[0.12,0,-0.055]`不变。当前板格改为0.0125m，只将本地位移尺度一致记为true；完整`metric_accuracy_verified / optical_axes_verified / landing_boundary_verified / descent_enabled`仍为false，因为整机外参、飞行高度误差、漂移和实机下降条件尚未验收。
- 已重跑3项诊断正负对照通过。后续只核对复算数值与内参/冻结控制源码哈希，继续遵守21:20截止，不重开传感器或飞控。

#### 冻结版本验收与下一步

- 20:52集中回归：18个相关测试模块，**80项全部通过**，耗时5.34秒。覆盖地面/巡航高度带、任意原点与FLU/FRD、消息边界、未知地图/制动、H多底色及像素坐标还原、投影、降落边界、目标锁定/漂移预算、下降、READY/ACK/接管和诊断正负对照。它们是软件单元证据，不是实飞验收。
- 20:54逐文件复核`nvblox_closed_loop_20260930_124457/report.json`记录的源码哈希：全部与冻结工作区一致。原实飞`flight_supervisor.py`哈希`67a98532b8490022e929b09d6bdd0d97fcfd82ffa9a6368eccceefd58dc96e51`，`live_flight_core.py`哈希`09a435d18c86cca171a95a3b4385d6dc8a1cbca9dbfc0a03c5feb8853532f17e`，均与本日较早证据一致。没有放宽原0.3m起降横移保护或更改实飞默认规划器。
- 当前`/home/cfly/param.params.txt`快照SHA256：`3d3f5ba479409c0c7dc5127fd25ffcacb4356256d67f4c054545d0af65a3eb5a`；本轮没有写参数。该哈希仅标识磁盘导出文件，不证明飞控RAM当前参数与文件逐项一致。
- 容器仅原`docker-init`/`sleep infinity`，`/dev/video6`无占用。`systemctl --user is-active robocup-qgc-bridge.service`返回active；该服务不是系统级unit，系统级查询not-found/inactive不可用于断言QGC故障。没有重启QGC、传感器、Agent、飞控或舵机。

| 最终需求 | 当前证据/结果 | 未完成、下一步及所需边界 |
|---|---|---|
| 地面初始化与升高后地图带 | 高度带单元及地面起步完整链通过；起飞/爬升不输出水平导航 | 真实起飞柱与全机高度范围的地图覆盖仍需仅传感器测量，不能用理想合成空间认证 |
| A*＋APF避障与独立制动 | 两箱/绕墙旧组合、新READY组合均各有完整通过；实际C++坏输入28段已有证据 | 未验证真实飞行制动、动态障碍、未知区域与2D雷达盲区覆盖；新输入并非实飞调试通过 |
| H中心识别与米制投影 | 已有多底色/旋转/负样本；12.5mm纠正后本地前35.33mm/右10.49mm与尺测36mm/报告11mm一致，原尺度矛盾关闭 | 单点不是全高度标定；整机外参与飞行高度误差预算仍需验证，不重复索取本次已纠正格长 |
| READY/AUX/EV到起飞/任务/触地 | 新隔离组合完整DDS通过；单一候选输出，正常与拒绝回归通过 | 不是生产实机运行器接入，没有独立任务飞行许可或真实PX4固件SITL；不能把隔离话题改名到`/fmu/in` |
| H下降与落点边界 | 合成目标锁定、失效/预算、触地/上锁2秒确认通过 | 真实下降漂移界、误差总预算、下降柱和失效接管未通过；1mm/s仍是假设，禁止用于放行 |
| 连续可靠完成 | 冻结版本两箱/绕墙有通过证据，80项回归通过 | H对准等待有3～34秒波动；新诊断证明存在迟到，但未完整复现旧慢轮，不宣称根治 |
| 实机自主避障＋识别H降落 | **尚未完成，不放行** | 需要先解决上述工程与测量项，再另行确认拆桨接口验证/有界实机测试范围，不能仅补一个“允许起飞”便执行 |

复现：按`APF_H_OFFLINE_README.md`，新入口为`bash /home/cfly/ros2_ws/robocup_nav/run.sh offline-ready-sortie-h --controller path-apf --scene two-boxes --scan-beams 720`，绕墙改`--scene wall`。只使用已运行容器、域176合成数据，不会启动硬件；不同域176测试必须串行。回退：不调用新`offline-ready-sortie-h`即可；原实飞入口与参数未替换，无需回刷参数/固件或重建容器。新模块为隔离候选，不是比赛实机自动执行入口。

本次21:20截止不变；20:50之后已冻结功能，只做回归/核对/记录。尚未达到“比赛任务实机完成”，不把软件交付改写成原目标全部完成。当前无后台测试等待；未申请或消耗新的传感器/实机动作授权。

**20:50功能冻结：不再新增功能或改变算法/判据，进入回归、证据核对与交接。**

- 诊断轮`evidence/nvblox_closed_loop_20260930_124457`绕墙完整DONE，通过；1463组模式/目标接收，3932条影子EV发布/3931条接收，最后一条未被夹具消费不代表完整投递保障。模拟误差4.986mm，输出开始至DONE约73.15秒，H对准约3.046秒，触地持续确认达到2秒，无fault/包络接触/实际飞控输入；源码运行期间不变。新增每周期H年龄、稳定帧数、误差、速度、驻留起点、地图条件记录。
- 该轮ALIGN_H的61个周期中2次H无效：图像年龄282.0/275.5ms、连续检测帧数13/16，超过250ms；地图有效。另20周期误差未入5cm、8周期速度未低于5cm/s；真正开始稳定计时后没有重置。证据说明合成图像结果存在迟到，但**未完整复现旧34秒等待，不能宣布根因已全部解决**；旧失败轮没有逐帧诊断，保留其未通过结论。
- 20:48尝试仅降低合成渲染像素量，等价性测试在(x=5.05,y=2.5)时原始sampling=1正对照未检出H，故整项验证不成立；`set -e`阻止后续全链启动。未把它当优化通过，也没有改检测门限。已撤回本轮新增渲染缩放代码和临时等价性测试脚本，恢复已通过的全分辨率代码；试验失败过程保留在此。没有删除用户文件、历史录包或证据目录。
- 当前剩余H时序波动未完全解释；新组合两箱和绕墙各有完整通过证据，但不证明每轮可靠或实机可飞。真实米制全局/下降开关保持false，原PX4参数、起降入口与QGC不变。

**20:40实机条件判定：不放行装桨自主避障/H降落。距21:20剩40分钟；20:50停止功能扩展。**

- 20:41收尾检查：`test_sortie*.py`共17项回归全部通过（含多个故障子场景），`run.sh`语法检查通过。容器仅剩原`docker-init`与`sleep infinity`，下视`/dev/video6`无占用；本轮离线导航/测试进程已退出，未启动硬件。时序诊断下一阶段另起隔离测试，不重复消耗传感器许可。

- 新组合两箱全链`nvblox_closed_loop_20260930_123542`通过：READY/AUX/EV预热→真实ROS格式的合成遥测/ACK→起飞→唯一交接→避障→H→末段→持续触地/上锁→DONE。1246组模式/目标、3384条影子EV实际接收，输出开始至DONE约62.30秒，模拟水平误差4.998mm，无控制fault/包络接触/真实输入，源码哈希运行期间不变。此成功实际也在旧85秒窗口内，**不能把成功归因于扩大观察窗口**。
- 随后只修正报告读取不应刷新所有权时间戳，并做新组合绕墙`nvblox_closed_loop_20260930_123750`：已到模拟地面，状态TOUCHDOWN；观察结束时触地确认约1.4秒，尚不足原2秒，**结果false**，不按“差一点”算通过。2077组模式/目标、5461条影子EV实际接收，无fault/包络接触；H对准等待约34.35秒，而通过的两箱轮约2.95秒。现需定位这一连续性波动，不再增加观察窗口或降低触地/视觉新鲜度要求。旧绕墙合成测试通过不替代本次新接口完整测试。
- 本阶段已补齐**隔离软件**的READY/AUX/EV策略与任务交接，但没有生产实机任务入口/独立许可，真实输入适配与实际失效接管仍未完成；地图与升降柱覆盖、飞行高度投影误差、定位下降漂移界仍缺真实证据。传感器采集许可已耗尽，没有重启相机/雷达或输入PX4。20:40结论不是“只差用户同意起飞”。
- 下一步限于现有合成链的H时序诊断、回归及交接；禁止把理想地图/1mm/s假设或本次36mm地面局部尺测扩大为实机放行。原起降入口/参数/QGC服务保留。

**20:34接口阶段进展（距21:20约46分钟）：**

- 20:35第二轮`nvblox_closed_loop_20260930_123326`已完成READY/ACK/地面起飞→唯一交接→避障→H对准→末段下降，1577组模式/目标与4211条影子EV实际DDS接收，最终水平误差5.918mm（仅合成）、距模拟触地还约47.5mm；85秒观察结束，**完整结果false**，没有fault或包络接触。已按新增预热/READY阶段调整**新夹具观察窗口**为110秒，原夹具默认85秒不变；SortieShadow仍保持启动后120秒总任务限制，所有250/300ms时效及落点/触地条件不改。这是延长离线观察，不是放宽控制安全条件；保留本轮失败证据，正进行独立重跑。

- 新增`scripts/sortie_ready_shadow.py`：以组合方式复用原`LiveFlightCore`的READY5秒、AUX边沿/去抖、EV预热和命令ACK逻辑，原`live_flight_core.py / flight_runtime.py / flight_supervisor.py`不改；新组合不是生产`LiveFlightCore`，没有真实路由或飞行许可。起飞仍经原竖直监督/发布检查，悬停成功才交接任务；任务使用既有唯一候选出口和混合目标检查。
- 6项单元回归已通过：完整地面至触地、过早切开关/起飞柱未知、缺模式ACK、导航中丢VIO/接管/kill锁定、空中意外上锁、关闭后不重放。首轮正常链曾因触地自动上锁消息先于执行周期而误锁定；已仅在既有末段状态、鲜活PX4触地与上下两路近地高度一致时保留观测，仍不接受空中意外上锁，不恢复既有EV故障；此为隔离软件行为，不证明PX4真实着陆响应。
- 新增`scripts/sortie_input_shadow.py`和`tests/sortie_ready_h_closed_loop.py`，固定域176及`/robocup/sortie_shadow/input/*`，实际PX4 ROS消息类型和订阅回调承载**合成**遥测/ACK/AUX/VIO；EV仅发隔离影子话题，不连接真实PX4，不使用串口或传感器。地图/H/高度/漂移和机体动力学仍为合成。
- 首轮`evidence/nvblox_closed_loop_20260930_123120`失败且无任何控制候选：执行器先取时间后更新所有权，造成新观测相对该周期看似“未来”，READY一直不成立。已修正夹具时间顺序，未放宽新鲜度阈值；正在重跑完整DDS链。该旧报告`production_ready_aux_runtime_used=true`命名不准确：实际复用策略，不是生产运行器；后续报告已区分`production_ready_aux_policy_used`与`dedicated_isolated_input_adapter_used`。
- 可复现入口新增`run.sh offline-ready-sortie-h`，只供隔离测试。真实下降/米制全局开关继续关闭；20:40仍进行实机条件判定，不因接口软件通过就跳过真实输入、地图覆盖、误差预算或授权。

**20:16最新补充：**已用约2小时56分钟，距离21:20剩约64分钟。图像分析已收口，当前无新增传感器采集或实机动作。

- 用户事后重新尺测，确认**前移36mm**，替代此前“估计34mm”作为本次前向比较真值；并确认确实有约11mm右移、飞机/镜头/板面垫高无变化。模型计算前36.051mm、右10.708mm：本次地面板面局部位移与现场测量一致，用户接受此项核验。新增`evidence/board_endpoint_20260930_200950/operator_followup.json`，保留原始报告和先前估计，不改写历史。数值差前0.051mm、右0.292mm**不是测量精度**；尺测不确定度未量化，右移量测法未独立说明。前向独立尺测已取得，不再将缺少尺测列为此项阻碍；完整外参/飞行高度精度/板距尺度矛盾仍未核验。仅新增范围明确的记录，不开启全局米制或下降开关。
- `tests/test_board_endpoint_audit.py`三项正负对照通过：正确且独特的匹配可给出诊断估计，冲突/弱/歧义/单个匹配不再平均成结果。
- `evidence/live_flight_check_20260930_121430_f32635/report.json`：固定隔离域177、生产READY/AUX/EV回调与序列化、合成PX4消息的两项DDS回归通过。1.1m普通起降场景DONE；缺模式ACK场景STOPPED且未进入解锁。没有真实发布者、没有真实PX4固件SITL；这是原链保留证据，不是新避障任务接入完成。
- `tests/test_sortie_legacy_boundary.py`三项新旧接口拒绝回归通过：原竖直发布检查拒绝NAVIGATE状态、原序列化拒绝NaN水平位置的混合导航输入、SortieShadow无法取得原真实许可入口。明确保留这些限制；后续需独立的任务交接与混合目标发布检查，不扩宽原起降保护来凑通。

**20:13增量（图像阶段提前收口）：**当前仍未完成真实自主避障/H降落。已核对下述旧图差异并进入接口检查，未重开相机；21:20截止、20:50停止功能扩展不变。

- 新证据 `evidence/board_endpoint_20260930_200950/report.json`：按打印字母g识别同一物理方格，人工对应四角后亚像素细化，平移为右111.5～112.5px、上373.9～377.2px。仅按暂用75mm板距及暂用光学轴，投影均值前36.05mm、右10.71mm。相互矛盾的f/g匹配不再求中位数；旧`...195900`的前23.29mm是误匹配平均，不可采用。四角一致性并非完整外参/米制独立验收。
- 用户新确认：前34mm是**估计**，确实同时右移约11mm；起终点之间飞机、镜头和板垫高均无变化。因此移动方向与用户描述相符；右移不能再当作“相机轴必然错误”的证据。不能把前34mm当精确独立尺测真值，也不推断右11mm的测量方法。
- 同一17mm方格的去畸变边长，在平行板假设下对应约97.7～101.8mm板距，与原75mm名义板距仍不一致。此为**待解释的尺度矛盾**，不是新外参测量结果；不擅改格长、安装偏移、焦距或高度，不将用户接受5mm差异扩大为接受约25mm尺度差。无需继续重复同一采集；下一次几何验收应使用独立尺测位移/光学中心到板面距离并保留测量误差。
- `optical_axes_verified / metric_accuracy_verified / landing_boundary_verified / descent_enabled`保持false；全部相机采集授权已耗尽。原实飞起降、PX4参数、QGC服务均不变。
- 接口审阅已确认：现有生产运行器的`FlightOutput`/独立发布检查只接收竖直起降状态及XYZ位置；新导航使用XY速度/Z位置，不能直接塞入旧入口，不能把旧检查删去后宣称集成完成。后续隔离检查围绕READY/AUX/EV与任务交接，原接口保护保留。

以下为**20:00历史快照**，不代表当前剩余时间或最新验收；后续修正以文档顶部20:55冻结交接为准。本轮17:20开始、21:20截止，20:00时已用约2小时40分钟、剩约1小时20分钟。

| 项目 | 已完成及证据范围 | 仍缺什么 |
|---|---|---|
| 原实飞起降 | 用户此前完成约1.1m起飞、短悬停、普通降落；原入口/参数/0.3m起降横移保护未改 | 不覆盖新增避障/H落点任务 |
| 地面初始化/地图高度带 | 已有地面初始高度＋巡航升高量的高度带；地面/爬升不执行水平导航；新执行器拒绝起飞高度与地图带不一致 | 实际起点自由空间、起飞/升降柱仍未取得充分传感器证据 |
| A*＋APF组合 | 原APF通道局部极小已定位；路径引导＋侧向APF＋独立地图制动在两箱与绕墙场景通过，另有到点2秒驻留 | 制动/滞后参数仍为简化模型，不是实机动力学验收 |
| H完整软件链 | 实际H检测、去畸变投影、稳定对准、锁定目标/误差预算、下降与触地确认已离线串通 | 精确实机米制/漂移界/下降柱仍未通过；假设1mm/s不准带入实飞 |
| 地面起步单一ROS候选出口 | `...114013`两箱、`...114621`绕墙全阶段通过；唯一输出、源码哈希不变；后者1452组模式/目标接收，模拟终点误差4.446mm | 输入遥测/ACK/触地仍为合成；未接入真实READY/AUX/EV运行器和独立实飞许可，非PX4固件SITL |
| 异常保护 | APF实际节点28段正常/异常对照通过；`sortie_writer_dds_20260930_114857`三个实际DDS场景（重复写者、发布异常、旧批次重放）均锁定且不新增输出；新执行器8项单元回归通过 | 发布停止不是实机停止；真实失效接管与PX4响应待验收 |
| 下视验证 | 压缩传输/标定加载有实测证据；本轮已保存固定起点和终点，设备现已释放 | 34mm米制核验尚未通过，不能把相机出流passed或图像有移动当作准确度通过 |

**当前仍未完成真实自主避障与识别H降落。** 不以毫米级合成终点误差冒充实机精度。`optical_axes_verified / metric_accuracy_verified / landing_boundary_verified / descent_enabled`仍全为false；没有新增实际PX4输出、参数修改、解锁或舵机动作。

剩余时间安排（硬截止，不自动启动硬件）：

1. **20:00–20:15：**仅集中处理已录起终点图的标记匹配/方向/距离差异，完成既定几何问题定位；不再重复60秒等待动作的采集。必要现场差异一次问清；新增采集仍单独批准。
2. **20:15–20:40：**针对真实输入适配、READY/AUX/EV与任务出口的剩余接口做隔离检查/修正；不再新增规划器、不装大依赖、不重建容器，不以重复跑相同名义场景代替接口工作。
3. **20:40：**做是否具备有界实机验证前提的明确检查；任何核心软件接口、定位/净空或现场授权欠缺，都不装桨启动自主任务，也不承诺最后几十分钟一定能补齐。
4. **20:50–21:20：**停止功能扩展，核对结果、进程、回退路径和未完成清单，交付当前真实进度；若不满足实机条件，明确交付阻碍，不改写四小时目标为已完成。

- 本次开始：北京时间17:20（工具UTC 09:20），截止21:20；原9月29日一个半小时窗口已失败，不重写历史。预留最后30分钟整理证据与剩余阻碍；预计超过20分钟的安装/构建/大范围搜索先询问用户。
- 首阶段：已在 `src/uav_task` 找到 `src/velocity_apf_controller.cpp`、`src/obstacle_course_mission.cpp`、`src/visual_servo_controller.cpp`、`scripts/circle_landing_target_node.py` 及舵机/起飞脚本，无需扩大搜索。该目录已有用户修改，不覆盖、不启动旧任务。旧任务含自动arm/Offboard/land/disarm与目标丢失后普通降落，圆靶检测偏蓝色，并非已验收的任意底色H降落。已询问用户旧APF是否实飞通过及当时入口，答复前不假定可靠性、不替换现用方案。
- 优先完成实际运动闭环，再合并H对准/下降，不以分模块通过数代替任务。当前只修复已证实的0.2角速度浮点边界误拒绝：容忍1e-12舍入并夹紧到原上限，真实超限仍锁定；不改飞控参数与默认规划器。
- 容器入口已只读核实为加载ROS后exec sleep infinity，并已恢复空闲容器用于隔离域176合成测试；没有启动相机或飞控链。第一次补丁因跨日临时工具路径失效未执行，随后4项测试是旧代码测试，不能计为修复回归；已找到当前工具重试。本日实机验证须核对新现场状态及授权，不自动延续昨日许可。
- 用户再次要求“不确定先询问”：几何/现场/旧实飞经历/方案切换先询问；已确定范围内的只读检查、明确缺陷修复和隔离离线验证继续。

### 17:50阶段更新（耗时约30分钟，剩余约3小时30分钟）

- 用户确认旧APF/投递任务去年实际成功飞过，使用机身上方二维雷达、基本固定机头；今日仍拆桨未解锁、设备全部连接。用户同意优先复用旧APF并与DWB对照，保留A*与地图制动约束，通过后再改部署默认。没有新增实机运动授权。
- 找到准确的历史入口 `src/uav_task/launch/waypoint_flight_test.launch.py`，仓库2025-08-17的`cd11857`提交中存在；节点为`waypoint_flight_test`，原航点12触发`heliport`视觉伺服，另外含三次舵机投放。旧启动日志描述的方形路径与源码实际13航点不一致，不能按日志直接飞。原节点构造时即发布舵机初始角度，故未启动它。该入口身份有源码/提交证据，但用户不记得去年实际命令，不能宣称已还原整套历史启动流程。
- 0.2角速度舍入修复已落盘，5项回归通过。隔离运行`evidence/nvblox_closed_loop_20260930_092413/`前进约12cm后又因0.15000000000000002的速度舍入中止；新增统一`navigation_limits.py`，仅容忍1e-12舍入并夹紧，真实超限仍拒绝。之后导航相关27项回归通过（含360个旋转端点数值检查）。
- 修复后的DWB StandardTrajectoryGenerator对照 `evidence/nvblox_closed_loop_20260930_092531/`：最终(1.9620,3.5223)，距离目标3.2053m，无碰撞/候选fault，但进度超时失败；停止处最佳轨迹为静止，总分24.3。原DWB默认配置未替换，不能宣称算法本身普遍不稳定，只证明当前配置/模型场景未通过。
- 新增 `legacy_apf_shadow/` 独立CMake包装，直接编译**现有**`uav_task/src/velocity_apf_controller.cpp`（非Python重写），不编译PX4Communicator或舵机代码，强制ROS域176；仅订阅合成里程计/雷达/A*路径并发布隔离候选速度。添加的路径前视、0.15m/s输出限制与坐标适配属于新代码，不继承旧实飞验证。容器编译成功，未改旧包，正在同一闭环场景比较。
- 为H地面投影已询问今日正常落地时PX4参考中心离地高度及带货最低点变化；未答复前保持真实米制/下降权限关闭，不把旧15cm推测写为实测。

### 18:00阶段更新：旧新硬件不可混同，几何与配置来源修正

- **用户明确补充：去年旧配置使用T265、没有TFmini；现在是D435i，另装TFmini。** 旧APF使用二维雷达且基本固定机头。旧实飞成功只作为旧平台的历史证据，不证明新D435i定位/延迟/外参/EV链或TFmini融合可直接沿用；不启动T265入口、不导入旧高度参数。当前D435i visual-only与已成功起降基线保留，TFmini不得因旧任务代码而擅自参与高度控制。
- 今日实测且用户再次确认：未挂货，PX4参考中心离地0.14m，最高点离地0.25m（相对PX4上方0.11m）。用户同意：地面投影采用今日未载货几何；避障继续保留旧的上0.115m/下0.15m较大包络。原几何配置不缩小；在landing_candidate增加测量来源记录。载货后几何、精确相机光学轴及米制误差仍未验证。
- 原APF首轮`evidence/nvblox_closed_loop_20260930_094956/`绕至目标附近，终点(4.8696,2.6430)、误差0.1935m，无碰撞/飞控输入，85秒未到达0.10m目标标准。**修正其配置解释：**后续强制加载检查揭露容器中install索引软链接指向不可见的/home/cfly，首次控制器实为源码默认参数（每轴容差0.2），不是原waypoint YAML（每轴0.15）；两者都与0.10m径向到点标准不一致，不能将首次结果称为去年配置复现。
- `...095520/`因新增profile核验拒绝默认回退而退出，是配置加载失败、不是算法运动失败。现在改用容器可见的真实build/uav_task/ament_cmake_index，加独立试验配置：保留原waypoint增益，仅到点每轴容差收紧至0.05m，适配器最终输出仍0.15m/s；旧源文件及原配置未改。正在复测，尚不宣称通过。
- H链已新增`landing_sequence_shadow.py`：把导航、H稳定对准、下降、末段冻结落点、触地/未解锁确认串到同一内存候选消息路径；真实几何缺失时仅保持巡航，不下降。几何/下降相关14项离线回归通过，包括完整合成序列及H丢失、会话重置、地图/下降柱失效、人工接管。属于L1合成验证，不是完整相机输入到实机着陆；既有live-flight没有修改。

### 18:15阶段更新：APF合成绕障到点通过，H尺寸不用于测距

- `evidence/nvblox_closed_loop_20260930_100057/report.json`：实际NavFn A*＋原C++ APF隔离包装完成合成绕障，终点(4.933136,2.571295)，目标误差0.097743m，805组内存候选消息，固定机头，无包络碰撞/故障，无`/fmu/in`话题。实际加载独立profile已核验。属于L3简化动力学，不是PX4 SITL；本次仅以进入0.10m范围结束，尚未证明到点后稳定驻留。DWB同场景此前未通过，故继续APF优先验证，未修改实飞默认入口。
- 用户补充当前H字母约60×40cm，实际比赛尺寸不确定；确认H处于降落区中心。该尺寸仅作测试图案记录，不作为比赛常量、不用于按字母大小估距。米制中心投影仍使用相机内参/畸变、经核验的安装方向、位姿与地面平面；H识别范围会随字母大小变化，不能把“尺寸不用于测距”误写为“尺寸不影响可见性”。
- 用户已同意先离线验证“完整H可见时对准并锁定地面目标，出视野后仅在定位误差及下降净空预算满足时继续”。新增可选anchor模式，默认关闭；合成测试假设1mm/s漂移界，**不是D435i实测值，禁止据此启用实机下降**。正在运行完整APF→渲染H图像→实际识别/地面投影→对准→下降候选链，并补充失效回归。
- 本日仅传感器30～60秒验证授权已收到，尚未执行；不启动旧任务、不输入EV/控制、不操作电机舵机、不更改PX4参数或QGC桥。

### 18:30阶段更新：一次真实联合出流验证完成，完整下降仍在调试

- 已按授权执行一次60秒仅传感器采集：`evidence/sensor_only_20260930_102322/`，隔离域179，启动D435i视觉-only＋深度/IMU、RPLidar、下视相机/H预览；未启动PX4桥/控制/舵机，结束后容器内只剩原sleep进程。SDK确认D435i仍为912112073953；udev路径中的915323051253与SDK序列号不同，未据此更换相机配置。QGC桥未动。
- 收到VIO 1373条（约26.39Hz，vo_state均1）、深度约19.41Hz、IMU约199.44Hz、雷达约11.21Hz；静止VIO最大相对首帧位移约8.86mm。雷达720束，末帧有效比例60.14%、最近有效距离0.509m；这不是雷达外参/方向/全机高度覆盖验收。日志仍有Motion Module failure警告但本轮IMU确实连续出流，不能将警告等同于无IMU，也不能声称硬件错误已根治。
- 下视标定K/D及1280×720尺寸一致，CameraInfo约9.51Hz；图像接收约5.26Hz、最长间隔1.578s。VIO最大接收间隔0.272s、最大时间戳年龄0.406s；这些超限需分析启动期/持续段，不作为飞行时序通过。H稳定消息0。报告passed仅表示基本出流和标定加载，**不代表时序、完整H可见性、米制精度或降落准备通过**。采集结束且无自动重试；已申请软件优化后最多两次60秒传感器复测，未获新答复前不启动。
- 完整合成图像链前两轮`...101346`和`...101712`因执行超时中止。时间记录显示导航输入间隔分别0.347/0.370s，而模拟位姿最大间隔0.197/0.155s；此前状态机错误依赖导航回调触发。已分离独立20Hz执行周期与导航消息时效，不提高0.3s执行截止和0.25s消息时效；图像另用单独工作线程，不阻塞控制。终止时清除合成执行器待处理旧命令，但这不等于实机PX4停止响应已验证。
- `...102103`改正后绕障并到达H附近，最终误差0.004917m，无碰撞/故障，但ALIGN_H阶段888个周期中只有589个满足视觉时效，稳定驻留条件不断重置，因此85秒内未下降，**完整测试仍失败**。新增640像素最大边长识别预览，结果还原为原图像素、仍用原标定，不按H尺寸估距；黑白/蓝底、旋转、坐标还原与负样本7项回归通过，正在重跑完整链。
- 本阶段回归：导航27项、降落14项、下降9项均通过，均是离线证据；所有真实米制/下降开关保持关闭。

### 18:35阶段更新：下视连续性改善已获一次实传感器证据

- 用户新授权最多两轮、每轮60秒仅传感器复测；第1/2轮已完成：`evidence/sensor_only_20260930_103021/`，均已停止，没有PX4输入。使用可选`h_transport:=compressed`，保留1280×720和原K/D，H算法仅内部缩小计算并还原原图坐标；没有改变相机EEPROM、飞控参数或旧起降入口。
- 对照初始采集：下视接收5.26→9.51Hz，最长间隔1.578→0.130s，图像年龄最大0.050s；H预览节点自身523帧，最大接收间隔0.128s、处理耗时最大0.0746s。VIO26.30Hz、最大接收间隔0.134s、最大时间戳年龄0.069s，静止位移约9.33mm。说明该组合下的下视传输/处理连续性明显改善，不证明实机运动或米制对准。
- H稳定消息仍0；飞机维持地面静止，未将无检测等同于算法不可用或完成H实景验收。独立域180回放已录完整H图像＋压缩传输，`h_preview_dds_20260930_103248`通过：完整H能稳定识别，停流/白图撤销候选，始终禁止下降/无飞控输入。H识别/压缩坐标8项单元回归通过。实际启动模式与标定加载已实测；精确外参与真实米制误差仍未通过。
- 完整合成图像链`...102721`仍因生成图像＋识别总延迟导致H新鲜度间歇不足而未下降（终点误差5.31mm，不计完整通过）。剖析发现主要耗时在模拟图像的两次重采样，而非H检测本身；合并透视与畸变成一次采样后，单帧合成＋识别平均约94→48ms，H检测约19ms。未改250ms视觉时效要求，正在再验证。

### 18:38里程碑：完整APF→图像H→对准→下降合成闭环首次通过

- `evidence/nvblox_closed_loop_20260930_103409/report.json`：完整测试passed=true，实际NavFn A*与原C++ APF隔离包装，H渲染→检测→去畸变地面投影→稳定对准→下降→末段下降→模拟触地/未解锁持续确认→DONE。1179组内存候选消息，最终(4.999507,2.504610)、径向误差0.004636m，z=-1.1m到达合成地面参考，无保守包络碰撞/故障，无飞控输入话题。
- 此次H锁定后确实出视野，使用**假设的1mm/s漂移界**和冻结目标完成下降；不是实测D435i误差模型。138帧合成图像，最大处理耗时0.1011s。摄像机使用真实K/D但理想光学外参，场景使用理想VIO/地图与简化速度响应/模拟触地，因此仅L3软件闭环，不是PX4 SITL、实机自主避障或H降落通过。原起降基线不变，实机下降仍关闭。
- 后续重点：将已通过的软件链接入有时效/会话证据的真实传感器候选输入，补充故障/驻留/不同起点场景验证，核实唯一控制出口、真实外参/米制误差与下降净空。禁止将当前毫米级合成误差写成实际降落精度。剩余仅传感器复测授权为第2/2轮60秒；实机运动尚未授权。

### 18:45现场核查与新确认

- 直接查看`evidence/sensor_only_20260930_103021/downward.png`：画面为电源线、插头及近处结构，没有H/地面标志。已询问遮挡或朝向变化。用户答复“原本Microdia下面什么都没有，现在放置每格2cm棋盘格”。不能把之前H稳定数0简单解释为正常地面H裁切；当前是否正确朝下、棋盘格可见性须以新图核验。不擅自移动镜头、不自动沿用精确外参。
- 用户确认PX4参考中心正处于四电机中心矩形中央；结合对角400mm，名义电机圆中心偏置为0、半径0.20m，名义黑环径向余量仍0.10m。已记录中心确认来源；单电机坐标和测量误差未量，真实定位/投影/下降误差仍需扣除，不能直接把0.10m当允许对准误差。
- 用户有棋盘格、称每格2cm；旧JSON记录19mm，已询问今天是实测20mm还是19mm近似，同时询问板是否水平、格线是否沿机头。此差别不自动废弃既有内参，但本次PnP米制平移必须用真实格长。未答复前只采图，不把估计标为已验证外参。
- 新授权第2/2轮60秒仅传感器复测已启动，用压缩图像路径；结束后此次两轮许可用尽，不自动追加启动。新增`run.sh offline-apf-h`和`offline-navigation`仅启动隔离合成软件，不访问传感器/飞控；两箱1.2m间隙场景正在补充验证。

### 18:55更新：通道对照失败已定位，棋盘格实际尺寸纠正

- 原APF两箱场景`nvblox_closed_loop_20260930_104055`未通过：停在(1.6847,2.4965)，未入1.2m通道、无碰撞/保护故障。A*确有159点通路，y约2.449～2.500；后段原始前向速度在约-0.077～+0.076m/s反复，符合入口吸引/斥力抵消停滞。单障碍通过不能推导原APF通道通过，也不称其普遍优于DWB。
- 用户明确同意先离线对照“ A*路径引导前进＋APF侧向避让＋独立地图制动检查”。新增仅隔离包装中的`path_guided`，原`uav_task`源文件不改；配置显式加载核验保留，额外要求有效雷达数据/360°覆盖，避免把旧APF因坏雷达给出的零速度重新变成前進命令。前进候选仍≤0.15m/s，侧向候选限0.08m/s，均是软件调试上限，非实测飞行性能。
- 新组合首轮`...104959`进入通道后在(2.6464,2.4779)被原制动圆盘保护阻止，未碰撞。0.15m/s与原包络/不确定度/假设制动产生半径0.58125m，大于此处到箱面约0.57795m；没有缩小包络或放宽余量。已增加在同一制动判据下选择更小速度的候选步骤，当前真实速度也参与检查，不能仅降低命令便假称已可停。5项制动/速度选择回归通过，正在复测。
- 最后一轮已授权传感器采集`evidence/sensor_only_20260930_104350`已结束，现无采集进程；本次两轮追加许可已用尽。下视9.51Hz、全程最大间隔0.141s；启动10秒后最大间隔0.118s/最大年龄0.030s。VIO26.09Hz、稳定段最大间隔0.168s/最大年龄0.023s；深度最大间隔0.364s。保存`stream_timing.json`供复核。静止场景VIO相对首帧最大变化38.44mm，不能据这次短测认证1mm/s下降漂移界；是否摆板期间机体/场景变化未独立核实。
- 最新图像已经看到棋盘格，成功提取4×3局部内角点。用户再次纠正：**实际格长17mm，板厚约10mm**；此前20mm是待确认说法，旧标定文件仍记19mm。此次使用17mm，不修改既有内参；均匀改变标定格长主要缩放棋盘格外参平移，不是直接按17/19缩放焦距。
- `chessboard_plane_20260930_185033`只报告相机相对板面候选，不写光学外参。初始IPPE最优解RMS约5px、法线距离68.9mm；标准LM优化后RMS约2.03px、法线距离约70.1mm（加10mm板厚约80.1mm），与测量配置85mm有约5mm差。网格相对机头朝向/机体水平度尚未核验，单图平面拟合不是完整机体外参标定，也不能把残差小当米制独立验收。
- 两次真实复测都改善下视连续性后，图像预览launch默认改为`h_transport:=compressed`，仍可显式`h_transport:=raw`回退；原始图像话题仍保留，分辨率/K/D不变，实飞起降入口与PX4参数不动。

### 19:15更新：两箱组合闭环通过，5mm安装差异保留记录

- 已复核 `evidence/nvblox_closed_loop_20260930_105354/report.json` 与 `...105735/report.json`：A*路径引导＋APF侧向避让＋独立地图制动约束，均完成两箱1.2m通道→H图像识别→对准→合成下降→DONE。第二轮使用720束合成雷达、起点横向偏置-5cm，最终合成误差5.502mm，924组内存候选消息；速度选择918周期保持原值、7周期降到0.75倍，无包络接触、无候选故障、无飞控输入。第一轮误差4.358mm，未用到降速，不能把该轮成功归因于降速。仍仅简化模型L3，非真实PX4 SITL、非实机落点精度。
- 用户确认棋盘格格线沿机体前后/左右摆正；实际格长17mm、板厚约10mm。5.5cm量的是PX4到**镜头最低点**，并接受图像估计与实测相差约5mm。保留原相机偏移 `[0.12,0,-0.055]`，不以单图估计覆盖实测值。镜头最低点与光学投影中心未独立测量；差异记入待核投影误差预算，不写成零误差或经证明的误差上界。无方向标签的格线也不能单独消除90°/180°轴方向歧义；先前纸片核验方向仍为粗验，不自动放行完整光学外参。
- 当前剩余工作：新组合绕墙回归、到点驻留及异常输入保护；随后仍需唯一控制出口隔离联调、真实米制误差与下降漂移/净空验证。真实下降开关保持关闭；仅传感器两次追加授权已耗尽，不再次采集。5mm差异本身不是停止离线工作的理由。

### 19:22更新：绕墙、驻留及实际APF节点异常输入验证完成

- `evidence/nvblox_closed_loop_20260930_111423/report.json`：新组合在720束雷达绕墙场景完成整条合成H降落链，DONE，模拟终点误差5.121mm，1128组候选消息，无包络接触/候选fault/飞控输入；45周期降至0.75倍、3周期因制动空间检查抑制候选。该抑制只证明软件约束被执行，不证明真实停止距离，不能把模拟毫米误差当实机精度。
- `...111622/report.json`：两箱、720束、起点横向+5cm的**单独导航**通过：终点误差63.136mm，末速度约0.0000177m/s，满足误差<0.10m、速度<0.03m/s连续2秒；598组候选、4周期0.75倍速度，无接触/故障/飞控输入。这补上此前只进入目标半径、不验证驻留的缺口。
- 新增 `tests/path_apf_fault_check.py`，实际启动独立C++包装，以合成DDS输入验证，不仅测Python函数。`evidence/path_apf_fault_20260930_111923/report.json`：28段全部通过（14段正常运动对照、14段拒绝输入）。覆盖启动无雷达、全NaN/Inf、有效率不足、半圈扫描、角度不一致、范围上下限无效、错误坐标系、过期/未来时间戳、雷达停流、里程计过期、无效四元数及空路径。异常稳定观察窗中均为零候选，正常对照为0.15m/s；不代表实机已停、也不是异常后自动恢复飞行许可。
- 导航27项、降落14项、下降9项、制动5项L1回归本阶段重跑全通过；`run.sh`语法检查通过。新增 `APF_H_OFFLINE_README.md` 及 `run.sh offline-apf-faults` 复现入口，均仅隔离域176。所有离线进程正常退出；没有追加传感器采集、改动PX4参数/原起降控制保护或重启QGC。
- **剩余关键路径仍是工程集成，不是5mm尺寸差异：**当前导航/H/下降是一条内存候选消息链；真实 `flight_runtime.create_node` 仍只允许已验证授权的 `LiveFlightCore`，尚无独立导航任务实机出口。下阶段先做起降→导航→H降落的唯一出口隔离联调，再核验实际地图/米制/漂移条件。不能直接把APF速度接入PX4、不能删除原0.3m起降横移保护。21:20截止及20:50开始整理交接计划不变。

### 19:40更新：地面起步到H降落的单一候选ROS出口首次通过

- 新增 `scripts/sortie_shadow.py`，复用原 `FlightSupervisor` 到稳定悬停完成，仅在私有离线子类将正常LAND转为显式MISSION_HANDOFF；原文件、原live-flight和原0.3m起降横移保护均未修改。起飞前要求起飞柱证据有效且地面初始高度＋升高量与巡航地图高度带一致；交接后检查PX4/VIO/RC/控制权/重置及既有地图制动和H下降约束。新执行器不是`LiveFlightCore`，没有真实飞行许可或输出路径。
- `tests/sortie_h_closed_loop.py` 从合成地面开始，实际A*/APF与H图像算法经**唯一ROS候选输出** `/robocup/sortie_shadow/output/*` 驱动简化机体，已不再仅靠返回的内存消息驱动。`evidence/nvblox_closed_loop_20260930_113437/report.json` 首轮通过：STREAM→OFFBOARD→ARM→TAKEOFF→HOVER→MISSION_HANDOFF→NAVIGATE→ALIGN_H→DESCEND→TOUCHDOWN→DONE，1236组模式/目标实际DDS接收，两个合成命令（切模式与解锁），无强制上锁或实际飞控输入，终点模拟误差4.325mm。完整阶段耗时约61.8秒，起飞交接约15.9秒。
- 首轮单位测试暴露“触地与自动上锁同时到达被误判为空中上锁”，已仅允许已进入下降末段、地面距离≤3cm且触地观察一致的情况，其余意外上锁仍拒绝。测试夹具的真值生成原先错误共用了控制器对齐失效锁，亦已分开。新增8项回归通过，覆盖完整序列、起飞柱未知/地图高度不一致、原横移保护、健康/接管锁定、地图过期及输出重复/冲突/异常/非法速度/导航解锁命令。
- 首次DDS闭环运行时尚未包含后来补充的独立消息边界检查；不把它记为最新代码全部通过。已补充固定路由、消息完整性/时间戳/阶段/模式/速度/高度检查及源码哈希记录，正复测最新版本。
- **仍不是实机联调完成：**当前PX4遥测/ACK/触地自动上锁、起飞柱、地图/VIO与下降漂移界都是合成条件；尚未接入真实READY/AUX/EV运行器，未运行真实PX4固件SITL，未创建`/fmu/in`话题。原源码PX4构建目录不存在，未为此升级依赖或启动耗时构建。真实下降开关继续关闭。
- 已另行申请一次最多60秒、仅下视相机的独立34mm棋盘格位移验证；此前两轮传感器授权已耗尽，未获新答复前不启动。

### 19:46更新：增强出口检查复测通过；下视现场核验没有取得棋盘格

- `evidence/nvblox_closed_loop_20260930_114013/report.json`：最新独立出口检查版本的地面起步→两箱→H降落完整DDS链通过，DONE；1226组模式/目标接收、唯一候选写者、仅两条合成模式/解锁命令、无实际飞控输入，模拟终点误差4.631mm。12项相关源文件哈希在运行前后相同，`sortie_validation_passed=true`。该版本已拒绝重复时间戳、不完整模式/目标批次、错误模式/高度/速度及导航阶段的解锁指令；仍没有真实READY/AUX/EV运行器与PX4固件仿真证据。
- 用户新许可一次最多60秒仅下视相机验证，已执行并**用尽**：`evidence/downward_metric_20260930_114329/`。保存551帧原始压缩图，最大接收间隔0.121757秒，K/D与1280×720一致；没有启动D435i、雷达、飞控/舵机。开始后50帧基线与结束帧均显示线缆/近处结构，没有棋盘格，因此没有要求用户执行34mm位移，也没有得到米制核验结论。报告中的`passed=true`仅指采集出流，不是34mm测试通过。
- 已询问用户棋盘格是否在Microdia镜头正下方且格面朝镜头，并要求先不移动镜头。不能根据这两张图就断言镜头装反或擅自调整外参。相机在上限到达后停止，超时清理出现`process_exit=-9`（非正常优雅退出）；随后`fuser /dev/video6`无占用且无采集/相机进程，设备已释放，没有重试。
- 现场原因待答复期间继续最新地面起步绕墙离线回归；不把这次缺棋盘格采集算作新的几何证据，真实下降权限不变。

### 20:00现场测量补充：34mm动作已确认，图像配对尚有歧义

- 用户确认“现在有棋盘格”后，另行批准的一次60秒下视采集 `downward_metric_20260930_114933` 已完成并用尽，保存537帧。起点/末帧均有棋盘；16个原始特征在487帧连续跟踪无丢失，但净位移近零，说明记录截止19:50:33时没有采到完整34mm动作。随后用户确认已向机头移动34mm，保留为操作员事实，不能用缺失的过程影像否定其操作。
- 改为静态终点短拍，用户另行明确允许15秒，`downward_metric_20260930_115615` 已完成并用尽：109帧，最大接收间隔0.118422秒，K/D一致，设备随后无占用；没有再要求移动纸板。终点图确认纸板位置与起点不同。该次超时清理退出码-9，设备已释放，不自动重启。
- `board_motion_20260930_195244`仅证明已记录片段基本静止，不是34mm验收。`board_endpoint_20260930_195900`试用不同字母匹配，g标签相关度0.968、候选位移约(+114,-376)像素；f标签低相关且匹配位移与g不一致，故**该报告的两点中位数不能作为有效米制结果**，整体几何验收保持false。下一步先消除标记匹配歧义、核对是否有侧移，不通过重复黑白格匹配强行得出成功。
- 新图7×5、6×4内角点的两次自动平面拟合都未找到完整角点阵列，没有产出新外参或改动旧标定。不扩大搜索、不把拟合失败转换成允许飞行。

## 0. 历史交接（2026-09-29，已被上方9月30日阶段结果部分更新）

**结论：既有起飞/短悬停/普通降落基线已通过；自主绕障并到达目标、识别H并对准下降着陆尚未完成，且离线动态绕障也未通过。原“一个半小时”目标未达成。不能因通信、传感器或单元测试通过就放行自主飞行。** 用户本次要求直接记录进度，本轮不再启动测试或实施新修复，只核对文件、已有证据和进程并更新本文。

### 0.1 已完成到什么程度

| 工作项 | 已有成果及证据范围 | 尚不能据此声称 |
|---|---|---|
| 原有起降 | 用户已完成约1.1m起飞、约5秒悬停、普通降落；保留第1节基线 | 自动绕箱或H视觉精准降落通过 |
| QGC无线连接 | USB独占转发与自动发现已部署；用户确认电脑显示飞机及遥测，见第6节 | 当前所有飞行控制链均已联调；本次未重新测试通信 |
| 比赛规格 | 第8.3节及competition_rules.yaml保存用户裁定：新场地障碍、原红十字投递、两处80cm H标志/60cm环内径 | 全比赛搜索、投递、窄门任务已实现 |
| 地面初始化与地图高度带 | 已实现面向起飞后巡航层的地面建图配置，地面禁止横移；实传感器出流及地图快照有证据（nvblox_live_20260929_120756） | 实际起点已被观测为自由空间；该次起点仍未知，仅前方局部快照路径可行 |
| Nav2软件链 | 真实nvblox代价地图插件、NavFn A*、DWB、数据过期保护到内存中的PX4候选消息已隔离联通（nvblox_navigation_20260929_141500） | 飞机能沿路径运动；该静态夹具的“非零”包含纯转向 |
| 下视相机 | 用户确认旧棋盘格标定来源；1280×720模式加载CameraInfo并实测出流通过（downward_20260929_131847、132256） | 精确外参和真实米制定位误差已通过 |
| H识别与下降软件 | 有完整H样本离线识别/回放、几何投影、落点包络、任务与下降预览模块 | 端到端H对准/下降已接入实机；地面裁切图无法验证完整H，不要求用户架高飞机来冒充起飞后实景 |

按ROS2验证范围划分：已有单元测试（L1）、隔离ROS运行联通（L3）和部分仅传感器硬件验证（L4）；**完整导航运动闭环的L3验收失败，新增自主避障/H降落没有L5/L6通过证据**。原起降成功不自动覆盖新增功能。这里是验证范围说明，不是安全认证。

### 0.2 最新失败、已定位问题和中断状态

1. `evidence/nvblox_closed_loop_20260929_143811/`：取得DWB完整评分。起步候选最大前进速度约0.01333m/s，2秒预测位移约0.02667m，小于0.05m地图格；目标评分仍55，原地转向总分27.5，前进候选最佳约27.8。证实该场景存在起步评分停滞机制，但不是证明所有后续停滞均为同一原因。
2. `...144223/`：运行中设置4秒虽返回成功，轨迹时间仍为2秒；此对照**未实际生效**，不得作为4秒效果证据。测试入口随后改为启动时注入诊断参数。
3. `...144451/`：启动时4秒预测真正生效，只前进约0.103m后仍报未取得进展；终点约(1.1032,2.5033)，距目标3.8968m，最高前进速度约0.02558m/s，无包络接触。**仅延长预测不能解决完整绕障，未采用为默认配置。**
4. `...144653/`：隔离对照DWB StandardTrajectoryGenerator，原始受保护前进指令最高约0.12273m/s，但候选链因角速度0.20000000000000007被严格“>0.2”误拒绝，锁定`invalid yaw rate`，飞机未移动。未证明换生成器即可完成绕障，默认仍未替换。
5. 最后计划修正浮点边界、加回归并重跑的命令因权限自动审核超时而未执行。本轮直接读取 `scripts/navigation_shadow_pipeline.py` 确认仍是严格`abs(yaw_rate_odom)>.2`：**浮点问题已定位，但修复未落盘、回归未运行。** 前条“正在修正”只代表计划，不能读成已修好。其余已写入的诊断功能为评分采集、原始指令日志、测试启动参数和生成器对照选项。
6. 本轮进程核查未发现运行中的`nvblox_closed_loop.py`。没有继续/重启测试；未写PX4参数、未解锁、未操作电机、未改QGC服务。已有闭环报告均明确`flight_validation=false`、`px4_sitl=false`、`no_flight_inputs=true`。

### 0.3 为什么一个晚上还未实现

- **实施顺序与预估不准确：**前期把较多时间用于传感器、标定、通信和独立模块，完整运动闭环接入过晚，才暴露原地停滞和接口浮点误拒绝。应更早建立“从起点运动到目标”的最小端到端验收，而不是累计分模块通过数。对此由实施方承担规划与进度说明责任，不归因于用户操作不及时。
- **实际仍缺工程集成，而不只是调几个参数：**既有live-flight只负责起飞/悬停/降落，横移超过0.3m会中止；导航候选链与下降预览尚未合成唯一飞控控制出口。不能直接接速度或删除原保护来宣称完成。
- **定位/几何证据尚不足：**实景起点未知、H精确外参与米制误差、近地目标出视野后的落点保持及载货下降净空尚未验收。标定文件成功加载不等于这些都通过。
- **调试也存在无效步骤：**容器消息路径、评分话题名和动态参数未实际生效消耗了时间；最后权限审核超时只解释那次修复未执行，不能解释或掩盖整晚未交付。
- 目前没有证据支持“再调一个参数即可今晚实飞”的保证，也没有证据表明直接改人工势场法就会更稳。不要在未定位闭环问题时承诺切换算法能按时完成。

### 0.4 未完成与后续优先顺序（均未放行实飞）

1. **修复候选链边界误拒绝并回归：**只容忍浮点舍入并夹紧到精确±0.2，真正超限/NaN/Inf仍拒绝并锁定；再比较轨迹生成器。此项当前未实施。
2. **完成真实Nav2动态离线闭环：**必须实际前进、绕过障碍并到达，不是转向非零；保留足迹/制动/未知空间/数据过期/进度保护。记录延迟、最小净空和停止响应，再做故障回归。简化XY模型不是PX4 SITL，仍缺实际飞控动力学/控制接入验证。
3. **唯一控制出口联调：**把导航、H对准、下降和原起降链的控制权/接管/超时/失定位/重置处理衔接；不能绕过原0.3m保护冒充集成。下降预览模块目前未接到候选消息链。
4. **地图起点与高度变化验收：**完成从地面到巡航层的起点/扫掠空间覆盖，不凭人工净空声明自动清空未知格；全比赛1～3m障碍需高度层切换与三维净空检查，当前固定层不等于全场能力。
5. **H落点与下降验收：**核验光学轴精确方向/倾斜、米制偏差、电机圆相对PX4位置与误差预算；完成H丢失、近地出视野、下降柱净空、触地/未解锁判定的全链测试。`landing_candidate.yaml`仍为`metric_accuracy_verified=false`、`optical_axes_verified=false`、`landing_boundary_verified=false`、`descent_enabled=false`。
6. **实机测试最后单独确认：**软件及离线验收满足后，再给操作员明确的低速/高度/路径、停止条件、接管方式与现场授权检查；当前不能据本文装桨启动自主避障/H降落。比赛搜索、红十字识别投递、三件载荷和窄门仍是后续未完成任务。

**交接要点：下一位执行者先读本节及最新失败报告，不重复索要用户已确认的尺寸/规则；先修复并验证端到端软件闭环，不重复相同地面裁切H采集，不重置/升级既有环境，也不以时间压力跳过现场授权。**

> **当前采用用户逐项确认的合并规则（2026-09-29）：**场地/障碍采用《无人机识别与快递运输实物赛项规则解析.pdf》；投递保留《2026比赛规则.pdf》的红十字任务；两处80×80cm H标志、黑环内径60cm及四电机中心圆降落判分已确认。第4～5节原场地等叙述为历史，不得整份照搬；以第8.3节和 `config/competition_rules.yaml` 为当前规格记录。其余冲突继续先询问。规格文件不自动启用飞行或覆盖PX4参数。

更新：2026-09-29。通信与控制话题全部在 **ROS 2 Humble**（`ROS_DOMAIN_ID=0`）。  
**已能稳定完成约 1 m 的起飞、短悬停和降落。** 本次不是比赛计分动作：稳定悬停仍短于 10 s。

## 1. 已成功的飞行配置

- 机载：Jetson + D435i 双目 IR → Isaac ROS cuVSLAM（`imu_fusion:=false`）→ `/visual_slam/tracking/odometry`（`odom`→`base_link`）。
- 飞控：PX4 1.16.2。EV 经 `/fmu/in/vehicle_visual_odometry` 进 EKF。`EKF2_EV_CTRL=11`（XY+偏航+高度，不融速度），`EKF2_HGT_REF=3`，`EKF2_EV_DELAY=0`，`EKF2_EV_POS_*=0`，`EKF2_EV_QMIN=50`，`EKF2_MAG_TYPE=5`，`EKF2_BARO_CTRL=1`，`EKF2_RNG_CTRL=0`。
- 杆臂只在 `camera_xyz_rpy: [0.196, 0.025, -0.05, -0.013094, -0.052326, 0]`。不要把同一杆臂再写入 `EKF2_EV_POS`。
- 链路：Telem1的DDS使用 `/dev/ttyTHS1` @ 921600，恢复时须核查MicroXRCEAgent和飞控重连状态。QGC独立走PX4 USB；2026-09-29已新增用户服务 `robocup-qgc-bridge.service`，替代此前需手动启动的MAVProxy转发。稳定设备路径 `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00`（当前ttyACM0），经Jetson Wi-Fi向QGC UDP14550自动发现；见第6节。不要同时运行第二个USB读取器或旧恢复脚本。
- 入口：`run.sh live-flight`。相对高度 0.2–1.3 m；已飞约 1.1 m、悬停 5 s。看到终端 `READY` 后再从 Position 拨到 Offboard。拨档后飞控会先进入 Offboard，AUX 消抖完成前仍继续发 EV，不要提前停视觉。
- 禁用旧桥 `src/px4_interface/src/vslam_odom_bridge.cpp`。低电仅警告（`COM_LOW_BAT_ACT=0`）。急停优先，接管后不抢回。

## 2. 避免 D435i IMU 无信号

同一时刻打开 infra/depth 和 Motion Module 会丢 `/camera/camera/imu`（`Motion Module failure`，组合 IMU 计数为 0）。告警本身不是硬件损坏，缺帧才是。

1. 全机只留一个相机进程。`scripts/realsense_imu_then_ir.py` 的实际顺序是 **IR → IMU**。
2. 先 `enable_infra1/2=true`，`enable_accel/gyro=false`，等到 `/camera/camera/infra1/image_rect_raw` 有帧。
3. 再开 gyro/accel（`gyro_fps=200`，`accel_fps=250`，`unite_imu_method=2`）。
4. 只有组合话题 `/camera/camera/imu` 大约 200 Hz 才算 IMU 可用。cuVSLAM 保持 `imu_fusion:=false`，飞控用自己的 IMU。
5. 不要刷 EEPROM，不要为此杀掉 MicroXRCEAgent。IR 没有帧时不要开 IMU。

## 3. 已有识别与投放代码

均在 ROS 2 Humble，尚未接入上述起降链。

| 位置 | 作用 | 话题 |
|---|---|---|
| `yolo_ws/src/yolo_detection` | YOLOv11 场景检测。权重 `weights/yolo11m_yihe.pt`、`yolov11m_yihe_0808.pt`。默认按 0.45 m 目标估距，输入是 `/image_raw`，不是 D435i | `/yolo/scene/name`、`/id`、`/centre`、`/confidence` |
| `ros2_ws/src/uav_task/scripts/servo_control_node.py` | 单路舵机，GPIO 物理引脚 32，50 Hz | 订阅 `/servo/set_angle`（`std_msgs/Float32`，0–180°） |
| `ros2_ws/src/uav_task/scripts/circle_landing_target_node.py` | 已知尺寸圆靶检测原型 | 图像输入，输出圆心 |

当前飞行相机是双目 IR，RGB 未开。投递和降落识别需要单独的彩色图像源，并改到 Humble 的实际话题上。

## 4. 后续任务

2026-09-29 重新提取核对 `/home/cfly/2026比赛规则.pdf` 的规则文字。2026 年变更：**不再给出靶标大致坐标**；**避障分要求无碰撞**。场地 10×10×4 m，飞行不得高于 4 m，单次不超过 10 分钟。**裁判下达起飞指令后 30 秒内**必须起飞（不是从桨叶转动开始算 30 秒）；桨叶启动记一次机会。自主任务期间不得用遥控或电脑操控，紧急接管终止任务。墙/网接触与门内碰撞计分存在不同条款，按具体区域及裁判解释执行，不将门内碰撞概括为必然全场结束。

1. **起飞计分（10）。** 离地超过 1 m，并稳定飞行不少于 10 s。配置沿用第 1 节，把悬停从已飞过的 5 s 加到不少于 10 s。先在净空场地做。
2. **避障（10）与两道窄门（最多 30）。** 高度保持 0.5–0.8 m，水平穿过投放区的树和 80 cm 纸箱，以及障碍区 1.5 m 宽走廊里两扇约 0.8 m、左右可调的门。从障碍上方翻过，或全程高于障碍物，避障分为 0。碰到树可以继续飞，但不得避障分；碰到墙或网则本场结束。无碰撞过一门 15 分，有碰撞通过 10 分。
   - 定位用 cuVSLAM 的 `odom`。障碍用 **RPLidar A2M8** 的 `/scan` 加 D435i 前视深度，写入同一占据栅格，未知格不可飞。雷达串口和 TF 尚未在飞行链验收。
   - 全局用 Nav2 **NavFn A\***。已知的是起飞区、走廊和降落区这些场地结构，**不是靶标坐标**。`launch/nvblox_navigation.launch.py` 里已有影子 A\*，未接 PX4。
   - 局部沿全局路径用 **DWB** 作为低速候选，高度独立控制；**不在窄门切换人工势场**。两者均无现成无人机闭环稳定性保证，人工势场还有局部极小和窄通道振荡问题。含防护外廓暂按 0.55 m 方形，必须检查姿态扫掠、制动与定位误差；仅“更慢”不足以保证过门。
3. **快递投递（最多 72.5）。** 规则要求携带三个快递盒，并在投放区**自主搜索**四个 1×1 m 标准靶（帐篷、地堡、桥梁、装甲车、坦克）和一个 0.35 m 红十字。规则没规定用广度还是深度；投放区用覆盖搜索，避免漏靶。舵机已调通：YOLO 认出靶标后下降，对准 `/yolo/scene/centre`，再发 `/servo/set_angle`。先做单靶、单次释放。权重：帐篷 1、地堡 1.5、桥梁 2、装甲车 2.5、红十字 10。操作员可另传靶标图片，用来核对或重训权重。
4. **降落（10）。** 规则没有写明起飞区和降落区是不是同一块地。场地只分成投放区和障碍区。返航条文允许两种做法，得分不同：回到起飞点降落，或穿越障碍区后再降落；一旦选择穿越，中途炸机即结束。计分写的是落在降落区内 10 分、压边 5 分、不在降落区 0 分，没有定义起飞点是否算降落区。实景图上的地面标识是 **H**。识别用操作员提供的降落区图片，把 H 修到机体中心后再降落。`circle_landing_target_node.py` 只认圆，不能代替 H。

顺序调整：已验证起降基线 → 宽裕空间单障碍影子导航 + 下视 H 感知独立验证 → 分别通过后接入低速导航/视觉对准降落 → 窄门 → 自主搜索与三件投放。今晚优先单障碍及 H 降落分项，不以完整比赛闭环为已完成目标。实际飞行每段确认现场条件及授权。

## 5. 2026-09-29 技术审查：替代原设想中的未验证假设

### 5.1 建图与规划

- **需要在线可通行空间表示，不强制完整三维重建或预先地图。** A* 和自主搜索需要障碍/未知区域记忆，只有反应式避障不足以完成未知靶标搜索和绕行。场地布局先验不能覆盖裁判移动后的障碍。
- 保留已安装 **nvblox 深度 TSDF → 覆盖机体高度带的二维 ESDF → Nav2**，不增加第二套稠密建图。关闭 mesh/color 集成，限制深度频率并实测 Jetson 联合负载；若影响 VIO 连续性，再评估更轻的深度体素/占据栅格方案，而非今晚盲换。
- nvblox 不是 SLAM，也不是自动提供全场已知地图。使用连续 odom 建图；VIO 重置使当前地图会话失效。当前静态 TSDF 也不能直接宣称能处理移动障碍。
- RPLidar 扫描平面不能代表全机体高度。机体中心飞到 0.8 m、雷达在上方 0.1 m 时，水平雷达可能从 0.8 m 纸箱上方扫过，而机体仍碰箱子；必须由深度高度带补充。深度对细枝/网也可能漏检。
- 当前 ESDF 带是 `center_z±0.4 m`，是保守影子设置；低高度可能把地面包含成障碍。需按实际 odom 高度、上/下外廓、倾斜和误差设置，不能机械固定在起点高度。升降期间停止二维导航。
- **A*+DWB 保留为候选，不保证稳定**：DWB 是速度采样局部规划器，不是 PX4 姿态控制器；需要测量实际跟踪滞后、制动、速度/加速度限制和抖动。RPP 简化路径跟随但不替代在线重规划；MPPI/MPC 可能改善约束处理，但有模型/算力/调参成本，今晚不直接换入飞行链。
- 未知格保持不可通行；搜索采用已观测安全空间内的局部目标/边界探索，不能把所有未知格清成 free。不要全方位开放侧移、倒退，直到对应感知覆盖验收。
- 0.55 m 方形在 45° 偏航的横向投影约 **0.778 m**；加每侧 3 cm 余量即超过 0.8 m 门。过门应有门前对准、受限偏航、直穿和出门判定；A* 栅格点路径并不证明整机可过。

### 5.2 H 降落、识别与任务

- 用户明确今晚要求 **识别 H 对准降落**，不能以定点降落替代后宣称完成。
- 检出 USB `0c45:6366 Microdia Webcam Vitade AF`，设备名 `LRCP H-720P`；本次容器内捕获设备为 video6，对应稳定 by-id `usb-LRCP_H-720P_LRCP_H-720P_SN0001-video-index0`。不要用易变化的 video0 以免占用 D435i。
- 已有 `sensors_extra.launch.py` 可启相机，但同时包含圆靶节点和默认 TF；不能将蓝圈/圆检测当 H 身份验证，也不能把未核相机 TF 直接用于飞控。
- 新增图像专用 `launch/downward_camera_preview.launch.py`，输出 `/robocup/downward/image_raw` 与 camera_info；不发布 TF、不控制飞机。启动前检查设备占用。
- H 方案：下视图像 → H 身份与外环/边界确认 → 连续帧一致性 → 相机内参/畸变与机体外参 → 地面相对位置 → 限速水平对准 → 满足中心/速度/置信度条件才下降。丢靶停止继续下降；高度阶段性变化和近地靶标出视野需专门处理。
- 图片中心不等于机体中心；只有像素中心输出不能直接给米制控制。需要实际 H 样图、尺寸、下视安装方向/偏移与标定。规则按四电机中心形成的圆与黑色环关系判降落分，不是“检测到 H 即成功”。
- YOLO 权重存在不代表类别/精度通过；标准靶搜索与红十字需实景核验。检测到目标不得直接触发舵机；需稳定对准、目标去重、货位选择、单次释放和释放反馈。已有单路舵机不等于三件独立投放已实现。
- 起飞计分应按 **超过 1 m、稳定超过 10 s** 留裕量（例如候选 12 s），仍先验证，不能用已飞 5 s 认定计分完成。规则对返航起点的降落计分仍需裁判明确。

### 5.3 已部署与未部署的边界

- 新增 `navigation_trial_profile.py`，影子 launch 可加 `trial_profile:=true`：前进上限 0.15 m/s、偏航 0.20 rad/s、规划加减速度 0.20 m/s²；禁止侧移。仅调整影子配置，已验证的 live-flight 起降入口不变。
- 示例（相机空闲且场地准备后才启动）：`run.sh nvblox trial_profile:=true center_z:=<实测odom高度>`，仍在独立 ROS 域 176，只能做影子测试。禁止直接 relay 输出到 PX4。
- 新增几何/制动预算测试；制动距离按 `v×延迟+v²/(2a)+误差余量` 评估，a 必须是实测可达减速度，配置值不能当实测证明。
- 真实导航接入必须由现有唯一飞行控制权节点接收，不另建第二个指令发布器；需 ROS FLU→会话本地 FRD 与 PX4 原点/航向对齐，cmd_vel 超时、地图过期、接管、越界和定位失效处理。
- 目前未完成 H 检测/视觉伺服闭环、导航飞行适配和真实障碍验收。今晚测试能否放行取决于现场和上述检查，不能保证截止时间替代测试结果。

### 5.4 本轮实测与当前停点

- 操作员确认卸桨、未解锁（暂无定位/地面站连接）；场地 **8×5×3 m**，含保护罩最大外廓 **不超过 600 mm**。影子 trial profile 使用两轴至少 0.60 m 外廓，覆盖旧 0.55 m 估计；不改既有起降配置。0.60 m 方形转 45° 的投影约 0.849 m，不能通过 0.8 m 门。
- 默认配置实插件测试 `evidence/nvblox_navigation_20260929_111137/` 通过；
  新低速/0.60 m 外廓配置 `evidence/nvblox_navigation_20260929_111353/` 通过。
  覆盖 A* 绕障、DWB 受监督输出、深度/地图/里程计过期抑制、封路拒绝、无飞控输入。
  场景为合成 ESDF，不是新配置的真实深度建图或动力学飞行验收。
- 新增 `tests/test_navigation_trial_profile.py` 三项通过，验证转角扫掠、制动预算与限速。
- 下视相机在隔离域 178 完成有界出流，`evidence/downward_20260929_111440/`：
  238 帧，640×480，约 **28.42 Hz**；有效内参未加载。`image.png` 画面模糊，
  无可辨认 H；不能判断是近距离、焦点或遮挡所致，需现场检查。
- 新增 `scripts/downward_snapshot.py` 只读截图工具。此次未启动飞控控制/EV、
  未改变 PX4 参数。仅启动既有容器、隔离影子测试与下视相机有界预览。
- 下一步需把实际 H 标志放到相机可清晰成像的视野内，确认标志/外环尺寸、
  下视相机内参与安装朝向；提供可用障碍物类型和摆位，再进行真实深度联合负载与影子避障。
  不要因合成场景通过就装桨验证导航。

### 5.5 真实传感器避障准备检查（2026-09-29）

后续双纸箱布局已确认：左右各一箱、总共两箱，从中间穿行；内侧净宽1.2 m，机头到最近箱面1.2 m，下方保留降落标志。按0.60 m机体正对通道每侧理论余量0.30 m，未含误差/制动/姿态。
新采集 `evidence/nvblox_live_20260929_115937/`（60秒，无导航目标/飞控输入）：深度27.02 Hz、VIO28.38 Hz、IMU199.98 Hz、ESDF4.86 Hz，1497跟踪状态全为1；出流检查通过，不是穿行通过。
离线检查最终ESDF：odom原点格未知（unknown=1000），沿y=0在x=0.6/0.9/1.2/1.5 m多个格非正，未建立可验证的中央自由通道。此为地图坐标抽样，不是纸箱物理定位验收。当时切片center_z=0、上下各0.4 m，近地初始化可能将地面并入障碍。后续已按操作员要求改为地面初始化、巡航高度带建图，见5.7；不再要求把飞机放支架上。未知格不得强行清空。

`evidence/nvblox_live_20260929_115030/report.json`：60秒隔离联合测试通过。
深度24.39 Hz、IMU199.67 Hz、VIO26.39 Hz、ESDF4.87 Hz；定位1371帧均state1。
定位最大源间隔166.69 ms、深度204.47 ms、ESDF212.92 ms；ESDF启动有1条零时间戳。
地图出现5333已知格、1142障碍格（各自最大计数），三项Nav2生命周期均active，
全局/局部代价地图39/92条。未发送导航目标、没有飞控输入话题，保护因缺少指令
持续抑制输出。出现Motion Module failure告警但IMU连续出流，不能仅凭告警判损坏。
这是默认配置真实数据出流/建图检查，非低速trial配置的动态验证，也非真实路径绕行、
停止距离或飞行验收。相机和测试已按有界时长结束。
此前待确认的摆位已由双纸箱1.2米中间通道替代；
保持卸桨，先验证真实地图上的路径与障碍边界，之后再评估影子跟踪。

### 5.6 模板、标定和现场信息补充（2026-09-29）

最新补充：操作员指出降落H底色/配色可能变化，可能黑、白或蓝底，因此不固定颜色阈值作为身份条件。再次调整后，`evidence/downward_20260929_113856/image.png` 中已能人工辨认完整H，为蓝白背景、黑描边，存在高光，外圈上下仍裁切。相机284帧约28.50 Hz，仍无有效内参。新增 `scripts/h_target_geometry.py` 离线多阈值/双极性轮廓原型，只输出像素候选；当前真实描边反光样本未检出，不能宣称H识别通过，尚未接入ROS控制或下降。需进一步处理描边/反光并验证多底色正负样本与时序稳定性；H候选、完整降落边界和米制位置必须分别验收。

- 高清图在 `/home/cfly/photo_UAV/`：降落标志、十字、帐篷、地堡、桥梁、装甲车、坦克，共七张 JPG。已查看降落模板，为白底黑 H 加黑圆环，1080×1080 像素；不是原节点检测的蓝圈。像素大小不能推定实际直径。
- 用户确认镜头正常、H 已摆好，标靶及降落区按规则尺寸制作，纸箱和树可用且按规则制作。纸箱边长 0.80 m；树实际外形、降落黑环直径未独立尺量，不把标准投放靶 1×1 m 当成降落环尺寸。
- 找到棋盘格标定 `/home/cfly/uav_circle_distance_orin/configs/drone_calibration.json`：1280×720，7×10 内角点、格长19 mm，17张采用/4张剔除，文件报告RMS 0.263 px；fx=775.616、fy=777.313、cx=605.328、cy=483.817。来源只记录video0，尚未证明对应当前相机/朝向/裁剪和焦点；不能直接认定标定已通过。
- 同目录另有 `drone_calibration_fov_measured.json`（仅找到路径），以及已读取的 `drone_calibration_fov_stream1_corrected.json`：视野尺量内参加单张2.14 m距离、假定60 cm靶径的经验修正，备注要求多距离验证。这个60 cm不是当前比赛降落环已确认尺寸。旧圆靶节点引用此经验文件。
- 当前640×480与历史1280×720宽高比不同，须核查模式缩放/裁剪或重新标定，不直接照搬K或等比例缩放。
- 新采集 `evidence/downward_20260929_113157/`：182帧，约28.49 Hz，640×480，未加载有效内参。实际截图仅见被画面边界截断的黑线，没有完整H及圆环；与操作员描述存在差异，需共同确认摆位/距离，不记为完整靶通过。
- 相机隔离域178有界采集已结束；未向PX4发输入、未改飞控参数。下一步让截图包含完整黑环并留边，再核验候选标定和外参，部署H+环检测及对准预览；尚未实现或放行H视觉下降闭环。

### 5.7 地面起步软件修正与验收更新（2026-09-29，本节覆盖前述历史停点）

**状态：软件候选及部分隔离验证通过，真实避障和H对准降落尚未通过，禁止把本节当作装桨/解锁许可。既有成功起降链未修改。**

1. 地面初始化不要求抬高飞机。新增 `scripts/cruise_band.py` 和 `launch/ground_navigation_shadow.launch.py`，在同一odom会话内预设巡航中心为初始高度加0.6m，碰撞带包含机体上下外廓、10度倾斜扫掠、误差余量。上下外廓仍使用待复核的未载货估计；不能用于带货放行。升降/不在高度带期间禁止二维导航。nvblox当前版本高度带为初始化参数，更换高度带须新建地图并核验，不假定在线改参数已生效。
2. 地面真实传感器隔离测试 `evidence/nvblox_live_20260929_120756/` 通过出流检查：VIO26.80Hz、深度25.60Hz、IMU199.18Hz、ESDF4.85Hz，1429条跟踪状态均为1；Nav2生命周期active。飞机在地面时高度保护始终不通过，符合预期。未发送飞行输入或导航目标。
3. 新增 `scripts/map_corridor_check.py`，以未知不可通行、0.60m方形包络的保守外接圆加余量检查保存地图。该次地图的(1.2,0)至(2.4,0)存在离线路径；(0,0)起点未知，因此从实际起点没有可接受路径。这是快照A*审计，不是实际Nav2路径跟踪或穿箱验收。人工净空声明不自动改写未知地图。
4. H轮廓检测已修正多阈值、正反极性、描边和方向问题；此前5.6所述完整H样本未检出已被新结果覆盖。`downward_20260929_113856/image.png`现检出一候选，中心约(315.69,237.53)。小样本测试覆盖白/黑/蓝底、旋转和部分非H负例，不是可靠性认证。
5. `scripts/h_target_preview.py`发布像素候选状态，需连续5帧且新鲜；**米制位置、边界、下降许可始终为false**。`evidence/h_preview_dds_20260929_121415/report.json`隔离回放通过稳定候选、断流抑制、空白抑制、无飞行输入检查。
6. 最新真实地面采集 `evidence/downward_20260929_122034/report.json`：283帧、28.34Hz，640×480，无有效内参；100条H状态中稳定候选0条。截图仅见模糊局部黑线、无完整H。这与完整样本回放是两种不同测试，不能混称实景识别通过。允许地面时H不可见，但起飞须依靠独立验收的定位/起降链；高度合适后重新捕获H，未捕获不能盲目对准下降。
7. 操作员确认下视相机相对PX4为FLU `[0.12,0,-0.055]`米，黑环**内径0.60m**（不是外径），起点周围1.2×1.2m、地面至1.3m净空。已写入 `config/landing_candidate.yaml`。外径、四电机中心包络及误差预算未核验；0.60m内径不能直接证明含罩0.60m飞机有安全降落余量。操作员回答“处于飞机正中央”描述靶位置，**未确认图像上方对应机体哪个方向**。
8. `scripts/landing_projection.py`加入相机偏移、畸变、姿态和地面交点几何，仅在内参/光轴方向独立验证且分辨率一致时计算。旧1280×720标定不能直接套当前640×480。对准仅输出限速预览，下降锁定。
9. 新增纯离线 `scripts/navigation_landing_shadow.py`：等待巡航→导航→H对准→降落请求意图→完成；断流/越界/接管/丢靶保护，倾角未知或超过10度禁止水平运动。所有输出 `flight_authorized=false`；无ROS/PX4输出。5项状态机测试通过。旧起降相关145项测试此前通过。

**下一阶段阻碍及顺序：**先完成下视相机方向实测与当前模式内参确认、环边界和机体包络核验；补齐起点/升空扫掠空间覆盖；再做单一控制权节点的导航接入和带动力学仿真的延迟、制动及故障验证。目前既有live-flight仍是起飞/悬停/降落且横移超0.3m中止，不能直接接Nav2速度，也不能删除该保护冒充导航部署完成。搜索/三件投放/窄门仍为后续任务。实际测试需单独现场确认与授权；时限不能替代放行条件。

### 5.8 制动空间保护补充（2026-09-29，离线）

审查发现5.7中的影子状态机仅检查当前机体占地已知空闲，不足以证明移动后的制动空间安全。新增 `scripts/stopping_space.py`：在覆盖整机高度的已知自由二维栅格中，按机体外接圆、定位余量、当前/指令速度较大值、延迟及制动距离检查全方向保守圆盘；任何接触障碍/未知或地图边界均拒绝。输入必须明确是布尔自由空间，不能将原始ESDF数值直接当free。

导航及H对准预览均要求显式提供该类空间检查，无检查器默认不移动；检查不通过还会重置H稳定对准计时。新增测试覆盖起点安全但制动范围受阻、当前速度非零而指令为零、未知格、地图边缘和无效制动参数；另有地图变化实时抑制/恢复的状态机集成测试。所有这些均为纯离线测试，无飞控接口。检查只覆盖静态二维切片，不能证明动态障碍、垂直升降空间或真实制动性能；延迟和可达减速度仍须独立实测，不能以规划配置值替代。

### 5.9 下视方向核验第一帧（2026-09-29）

用户将纸片放到下视镜头正下方（降落区前方）。隔离域178短时采集 `evidence/downward_20260929_123344/`：275帧、27.49Hz、640×480，无有效内参；画面为带鹰图案的纸片，覆盖全幅，72条H状态均不稳定。此帧仅作为方向核验基线，不能从纸上文字朝向推定飞机朝向。下一步飞机与纸片朝向均不变，仅将纸片沿机头方向平移约2cm，再比较同一图案的像素位移；仍需第二轴核验，未写入光轴旋转或启用米制控制。

随后用户确认纸片向机头平移2cm；`evidence/downward_20260929_123629/`采集209帧、26.87Hz。图像可见鹰图案向上移动。新增 `image_direction_check.py` 的双向光流检查因匹配不足拒绝，不记为自动验收通过（3项合成单元测试通过）。独立SIFT复核得到75对匹配、RANSAC内点69对，中位像素位移约 `[+15.87,-124.34]`，拟合伴有约-2.16度旋转；支持“机头大致对应图像上方”，不能据此认定精确外参。仅记录临时方向观测，`optical_axes_verified=false`不变。下一步以123629为基准，沿机体右侧平移约1cm并保持纸片不旋转，验证第二轴/镜像。当前两次采集均有界结束，无飞控输入。

用户随后确认向机体右侧平移1cm；`evidence/downward_20260929_123956/`采集223帧、28.67Hz。人工可见鹰图案主要右移；双向光流以运动不一致拒绝，SIFT独立复核96对匹配/84内点，中位位移约 `[+118.02,+27.21]`，拟合约-2.44度旋转、1.025缩放。两次支持粗略朝向“图像上→机头、图像右→机体右”，不支持精确外参。由于手工位移、旋转及比例变化，不能用2cm/1cm与像素比反推内参。已记录方向观测，但未填入已验证旋转矩阵，米制控制仍锁定。后续确认旧标定是否来自此相机/同焦点，或重新进行实际模式标定。

### 5.10 含速度延迟的离线演练（2026-09-29）

PX4混合控制源码核对：参数导出头记录1.16.2 / 54f0455ffc000000；当前源码工作树却是1.18-alpha且有用户修改，未切换或覆盖它。改用只读git show读取本地保留的精确提交 `54f0455ffcd755534539a7cf33a09a20bf71d29d`。该提交PositionControlTest.cpp的InputCombinationsPositionVelocity明确输入vx/vy和position.z并断言有效；PositionControl.cpp逐轴处理NaN并要求XY成对；commander/ModeUtil/control_mode.cpp中OffboardControlMode.position启用位置/速度/高度等控制层。说明接口受该版本源码支持，不需要为此升级固件，但未重新运行其C++测试或真实PX4 SITL，不能称为飞行验证。

`navigation_frame_contract.build_shadow_messages`现可构造实际px4_msgs的OffboardControlMode与TrajectorySetpoint对象：XY位置NaN、Z位置有限、XY速度有限、Z速度NaN、加速度/jerk/yawspeed为NaN、有限yaw；无VehicleCommand、无发布器。新增2项实际消息类型测试通过。仍未改变现有live-flight序列化器或控制出口，整机导航接入仍待完成。

导航接入接口核查：当前 `flight_messages.py` 与既有live-flight只支持完整XYZ位置目标，不能直接转发Nav2速度。新增 `navigation_frame_contract.py` 作为离线适配契约：已验证配对姿态/位置下，以ROS起始航向、PX4本地heading及FLU/FRD轴变换计算位置、速度和yaw，不能假定odom朝北或两者同原点。VIO会话/PX4任一reset counter变化永久使该对齐失效；输入过期、速度超0.15m/s、垂直速度非零、无单写者验证均拒绝。4项测试通过。生成的“XY速度+Z位置”字段仅为离线预览，尚未经对应PX4版本混合目标SITL验证；刻意未接入既有serializer、未发布任何/fmu/in消息。因此这不是导航单出口接入已完成的结论，既有实飞代码和0.3m横移保护保持原样。

移开纸片复测：用户确认移开，采集 `evidence/downward_20260929_132256/`，1280×720、69帧、有效段8.86Hz，CameraInfo与选定标定完全匹配，接收延迟中位15.42ms/P95为18.38ms，最大帧间隔212.43ms。72条H状态稳定数0；人工查看确为局部黑线/标志裁切，没有完整H或环。相机按30秒有界时长退出，不再反复以相同地面视野要求H正例通过，也不要求架高飞机。任务设计允许地面H不可见，由已独立验收起降/定位链起飞后在适当高度捕获目标；但当前仅传感器授权，不因此启动实飞。H下降全过程（近地出视野后如何维持落点、触地确认）及导航单出口尚未实现验收，继续保持下降禁用。

降落请求撤销保护：离线MissionShadow的LAND_REQUEST现每周期重新检查H新鲜度/米制标记、位置速度稳定、边界、地图、制动空间、高度与倾角。任何失效均撤销尚未执行的降落意图并重置对准计时，恢复不能立即复用之前的稳定结果。8项状态机测试和2项简化延迟演练通过；此状态机仍无飞控出口，撤销逻辑不代表已能撤销PX4实际LAND命令。DONE要求影子观测landed且非airborne，不能把矛盾观测当作落地。

**最新在线结果（覆盖本节稍早“加载待核验”）：**用户明确授权重新启动相机容器做仅传感器验证。确认入口为deploy-entrypoint加sleep infinity后启动，未运行起降入口。隔离域178短时1280×720 YUYV采集 `evidence/downward_20260929_131847/`：62帧，有效采集段8.65Hz；CameraInfo尺寸、K、D、畸变模型与选定棋盘格参数一致。图像时间戳到接收延迟最小10.93ms、中位14.63ms、P95为19.12ms、最大21.28ms；最大接收间隔217.51ms。此延迟不含完整H检测/控制，亦非曝光到执行的端到端实测。日志相机名称与旧文件名称不同告警，但实际矩阵一致且确认已加载，不能误判成内参加载失败。runtime_calibration_load_verified已置true并关联证据，metric_accuracy_verified仍false。

截图仍为鹰图案纸片，遮挡H，100条H状态稳定数0，属于该负例画面结果，不是H正例验收。已请用户移开纸片，不要求抬高飞机。相机采集按35秒时限结束；容器保留空闲，无PX4控制输入程序启动。未解锁、未改PX4参数/已验证起降链。内参来源由用户确认，实际图像模式和加载已验；真实米制误差、精确外参、落区边界、起点地图覆盖与单出口导航控制仍待验，不能宣称避障/着陆完成。

异常输入回归补充：修复 `landing_projection.AlignmentPreview` 对空向量未限制维度的问题，防止空数组范数为零被误认为对准。现在误差/速度必须是有限二维向量，时刻必须为有限标量；缺失、嵌套、错误长度、非数字、NaN均输出零预览速度并重置稳定计时。6项投影/对准测试、2项简化延迟演练、7项影子状态机测试通过；未启动硬件或修改飞行链。

**最新标定确认与配置更新（覆盖下文“来源不确定”的历史状态）：**用户明确确认桌面ZIP中的标定确由当前下视相机生成，并授权直接使用。已采用原始棋盘格JSON对应的ROS参数，保存为 `config/downward_camera_info.yaml`；未采用FOV/stream1单距离经验修正。相机来源与使用授权已写入 `config/landing_candidate.yaml`。`downward_camera_preview.launch.py`已由640×480改为1280×720、YUYV、请求10fps，通过文件URI加载该标定，URI按运行环境路径计算，兼容容器挂载。旧包与原项目文件未修改。配置测试逐项核对尺寸、K和畸变与原JSON完全一致，1项通过，launch语法编译通过。未重启当前已退出的容器、未启动相机、未改PX4或起降链，因此当前记录为“已配置，实际加载/出流待核验”，不是在线验证通过。

相机位置继续采用PX4前12cm、下5.5cm、左右0；粗方向为图像上朝机头、图像右朝机体右。用户确认的是标定来源及使用，不将其扩展为“镜头之后未调焦”或精确外参已通过；焦点历史、实际模式与CameraInfo一致性、空间位置误差仍需核验。H下降保持禁用，黑环内径60cm/外径未核实、起飞净空1.2×1.2×1.3m等信息继续保留。实际避障与H降落未完成，后续仍需起点覆盖、唯一飞控出口接入和实机动态验收。

后续主机只读模式查询：下视LRCP_H-720P仍枚举在稳定by-id路径，支持1280×720；YUYV为10/5fps，MJPEG为30/25/20/15/10/5fps。当前640×480 YUYV可30fps。因此切换旧标定分辨率并非只改尺寸，必须实测帧率、解码延迟和图像模式。未执行模式切换或加载旧内参。桌面/下载/图片/原圆靶项目范围内未找到JSON列出的棋盘格原始照片；不是全盘不存在的结论。查询时isaac_ros_dev状态为Exited(255)，相机预览/截图/实时起降进程未检出，未自动重启容器；再次硬件采集前须重新确认现场状态。

关联资料核对（2026-09-29）：桌面 `/home/cfly/Desktop/uav_circle_distance_orin_20260529.zip` 确有棋盘格标定JSON、ROS camera_info YAML、标定脚本及FOV经验参数。JSON与现有 `/home/cfly/uav_circle_distance_orin/configs/drone_calibration.json` SHA256完全相同（bffa64dc5eed56de8d10bba74d84f92d3b682cd5fb19130212ad84445d42162d）。记录1280×720、7×10内角点、19mm格长、17张采用/4张剔除，自报RMS0.263px。包内只有圆靶示例图，没有原始棋盘格照片；来源只记/dev/video0，无序列号/焦点证据。用户不确定来源。当前640×480不能直接套该结果；FOV单距离修正不替代H平面定位标定。此次只读查看压缩内容，未覆盖文件、未执行包内程序或加载参数进飞行链。

新增 `tests/test_navigation_lag_rehearsal.py`，以1.2m双纸箱通道、0.6m机体、理想已知地图/目标，演练导航→H对准→降落请求意图。采用100ms指令队列、一阶0.3s速度响应、0.2m/s²限加速度；参数是假设，不是PX4实测。正常场景最终位置x=1.97552m（目标2m）、速度0.00846m/s，到达LAND_REQUEST，模型中无碰撞；地图过期时停止移动意图，定位失效进入ABORT，接管进入HANDOVER。两项测试覆盖以上四场景并通过。

严格边界：这是纯XY简化模型，不是PX4/Gazebo SITL，不执行实际A*/DWB，不验证真实接管后的制动行为，没有模拟风/地效/垂直下降/真实H误检。LAND_REQUEST只表示意图，未验证着陆。所有场景flight_authorized=false。既有live-flight和飞控参数保持不变，尚缺导航单出口适配及实机动力学验证，不应称为完整避障降落仿真通过。

## 6. QGC通过Jetson自动连接PX4（2026-09-29，已实测）

用户要求实现电脑经同一Wi-Fi自动连接，并确认PX4使用USB接Jetson。诊断时USB可收到PX4心跳（system1/component1，读取时未解锁），但没有运行MAVLink转发程序或自启动服务；并非USB参数未开启。未执行旧restore_qgc_usb.sh（其包含参数写入及重启），未改PX4参数、未重启飞控、未占用Telem1。

### 6.1 网络与设备

- Wi-Fi热点SSID：**Mate70**，WPA2个人、5GHz，网关 **192.168.43.1**。
- Jetson：**192.168.43.228/24**，接口 `wlP1p1s0`。
- 用户电脑：**192.168.43.89**，当前QGC监听 **UDP14550**。
- USB稳定路径：`/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00`，当前映射ttyACM0；程序不依赖可变化的ttyACM序号。
- 转发服务本地绑定当前Wi-Fi IPv4的 **UDP14555**，向当前子网广播地址（本次192.168.43.255）的UDP14550发现QGC；收到GCS心跳后单播给对应IP/端口。电脑IP不硬编码，变更后可重新发现。

### 6.2 已部署配置与边界

- 转发程序：`/home/cfly/ros2_ws/robocup_nav/scripts/qgc_wifi_bridge.py`。
- 用户服务：`/home/cfly/.config/systemd/user/robocup-qgc-bridge.service`，已enable并active。
- 运行状态：`/home/cfly/.local/state/robocup-qgc/status.json`，包含新鲜时间戳、USB状态、QGC对端与双向字节数。不记录大量遥测文件。
- 使用USB独占打开和进程锁，防止本服务重复运行；不与MAVProxy/其它USB读取器同时使用。Telem1/DDS与此链路分开，未自动启动或停止DDS。
- 每3秒检查Wi-Fi地址及USB设备，异常等待后重连；GCS心跳超过10秒未更新则回到广播发现。USB拔插与热点切换恢复逻辑已实现，但本轮没有物理拔插/换热点实测。
- 仅转发真实PX4遥测和已发现QGC的MAVLink数据，不生成GCS心跳、不自行发送解锁/起飞/参数命令，因此不会用虚假心跳掩盖QGC断开。用户在QGC执行的操作会正常转发，勿在未批准时解锁或改参。
- 接收方限制为当前Wi-Fi子网且一次一个GCS对端。这不是身份认证或加密；仅在可信私人热点使用，不能开放给不可信共享Wi-Fi。
- 自启动是**cfly用户服务**，已确认GDM配置AutomaticLoginEnable=True、AutomaticLogin=cfly，因此正常桌面启动后自动运行。当前Linger=no：不是未登录时也运行的系统服务；若禁用自动登录/桌面，需另行配置。未实际重启整机验收开机流程。

### 6.3 实测证据

- 电脑192.168.43.89可达，2次ping成功，约4～6ms。
- 服务首次启动，状态记录QGC对端 `[192.168.43.89,14550]`，PX4→Wi-Fi 197578字节、QGC→PX4 2324字节。
- 仅重启转发服务后，约数秒重新发现同一QGC；新会话状态PX4→Wi-Fi 47765字节、QGC→PX4 94字节，服务active。
- **用户随后明确确认“已显示飞机和遥测”**，端到端自动连接通过，不只是服务运行。
- 2项转发协议单元测试通过（真实GCS心跳原样转发、10秒超时重新发现、无效/外网/第二对端拒绝）；systemd服务文件验证通过。

### 6.4 日常使用和回滚

保持PX4 USB连接Jetson、Jetson与电脑在同一可信Wi-Fi，打开QGC并保留UDP自动连接。一般无需手动加链路；Windows若提示防火墙，允许QGC通过当前私人网络。更换热点若存在客户端隔离/屏蔽广播，需检查网络策略，不能保证所有热点都自动发现。

查看：`systemctl --user status robocup-qgc-bridge.service`，或读取上述status.json并核对时间戳。状态文件不是永久在线证明。
停止并取消自启动（回滚）：`systemctl --user disable --now robocup-qgc-bridge.service`。文件保留，可再次enable；没有固件/飞控参数需要回滚。需要其它程序占用PX4 USB时应先停止此服务，不要直接抢占端口。

## 7. 恢复避障/H降落工作时的最新软件状态

下降候选逻辑：新增 `descent_shadow.py`，仅纯离线预览，不接入PX4。进入下降需已验证锁定落点、降落竖直空间、误差界、连续定位会话和新鲜H；较高位置丢H输出零下降意图；低于候选末段阈值前必须再次有新鲜H，之后仅在锁定落点、定位和预算持续有效时允许有时限的预览下降；越界/定位重置/安全条件丢失锁定ABORT，接管HANDOVER。触地后不发自动解锁/强制停桨，只有持续2秒landed且未解锁观测才DONE。

5项测试通过，含简化垂直积分模拟、丢靶、近地盲入拒绝、预算/定位/净空失效、已触地但仍解锁不认定完成。测试假设0.10m/s下降、末段0.05m/s、最低机体点距地0.25m进入末段、最多6秒及30秒全程，**这些是候选测试参数，不是现场测量或放行值**。实际H在何高度出视野仍未测，若在0.25m之前已看不到则此候选会拒绝继续下降，需要验证并调整阶段设计，不能直接放宽门槛。clearance是已核地平面到最低机体点距离，不是盲区内TFmini数值。状态机尚未连到导航pipeline或真实执行器，不把简化积分模型称为PX4 SITL/实机着陆通过。

**最新电机尺寸（覆盖下文待测状态）：**用户报告“对角电机准确400mm”，已写入landing_candidate.yaml。若两对角均为400mm且机架对称，则电机中心圆半径0.20m，与内径0.60m黑环的理想径向差为0.10m。用户未分别列出两条对角数值，也未核对电机圆心相对PX4参考点偏置，因此只记录名义圆半径，motor_layout_relative_to_px4_verified仍false。允许中心误差必须小于0.10m减去靶定位/飞行定位/下降漂移及中心偏置等保守预算；不能把10cm直接当对准阈值。计分电机圆与物理避障保护罩包络仍是不同几何。

降落计分几何补充：新增 `landing_envelope.py`，以相对PX4参考点的四电机中心坐标构造保守外接圆，并从已确认黑环内半径0.30m中扣除靶定位误差、飞行定位误差与下降漂移界，计算允许的中心误差。不是以保护罩宽度替代电机圆，也不把包络计分通过当物理碰撞安全。3项离线测试通过：示例电机坐标±0.16m、误差预算共0.05m时，中心余量约0.0237m，固定5cm对准阈值仍会不满足；该例不是本机尺寸。已询问两对对角电机中心间距；精确应用还需机架相对PX4中心偏置和真实误差界。当前helper未自动接入实机降落，下降仍禁用。

新增 `navigation_shadow_pipeline.py` 将影子任务状态机、会话坐标对齐和实际px4_msgs对象构造连接起来，无ROS发布器、不接真实/fmu/in。4项联合测试通过：保持固定巡航Z而非追随当前高度，过期导航指令转零XY速度且不继续偏航，地面/接管/定位故障不生成候选消息，控制权或对齐丢失锁定，LAND_REQUEST只保持高度不发LAND。只是离线联合接口测试，不是实际单出口飞行链部署完成。QGC自动连接的完成不会解除新导航/下降禁用状态；后续仍需导航整链、实际地图起点覆盖和H下降全过程验收。

## 8. 确定事实与规则差异确认清单（2026-09-29）

用户最新指示：若与原数据不对应，先询问用户，由用户回答；确定数据写入本文。已完整读取新解析PDF四页文字并查看四页图示。以下“文件写明”不等于用户已裁定其覆盖原规则；配置尚未切换。文件元数据标注创建于2021年，元数据不能单独证明本届适用或不适用，因此不自行裁定版本优先级。

### 8.1 已确定，继续保留

| 项目 | 已确认数据 | 来源与适用范围 |
|---|---|---|
| 实机电机对角距 | 400 mm | 用户实测，不等于含罩外廓 |
| 含保护罩最大外廓 | 不超过600 mm | 用户报告；用于保守避障候选，不用电机距替代 |
| 当前实体黑环 | 内径600 mm | 用户实测；不是外径，未据此认定官方图案尺寸 |
| 下视镜头相对PX4 | 前120 mm、下55 mm、左右0 | 用户确认 |
| 下视粗方向 | 图像上对应机头，图像右对应机体右 | 两次纸片平移与图像复核；不是精确外参标定 |
| 下视标定来源 | 当前下视相机的原始棋盘格标定，可采用 | 用户明确确认桌面ZIP来源并授权使用 |
| 下视实际模式/内参加载 | 1280×720，YUYV，请求10fps，实测约8.65～8.86Hz；K/D加载匹配 | 两次在线报告见5.10；实际米制精度仍未验 |
| 当前测试场地 | 8×5×3 m | 用户现场提供，不改成比赛场地尺寸 |
| 当前双箱摆位 | 中间净宽1.2 m，机头距最近箱面1.2 m | 用户现场确认，不代表全部比赛障碍 |
| 起点净空 | 1.2×1.2 m、地面至1.3 m | 用户确认；不自动清空地图未知格 |
| QGC链路 | PX4 USB→Jetson→Mate70 Wi-Fi→电脑QGC | 已双向通信、服务重启重连，并经用户确认显示飞机遥测 |
| 当前网络 | Jetson192.168.43.228，电脑192.168.43.89，网关192.168.43.1；QGC UDP14550 | 用户提供电脑地址、Jetson实查；地址可能随DHCP变化，服务自动发现 |

### 8.2 文件差异清单（询问时历史，后续裁定见8.3）

| 项目 | 原记录 | 新解析PDF写明 | 处理状态 |
|---|---|---|---|
| 比赛场地 | 10×10×4 m | 10×8×4 m；任务区8×8 m（p1） | 已询问，未覆盖 |
| 障碍规格 | 80cm纸箱等、低速0.5～0.8m策略 | 楼房/树木高1～3m，现场确定；圆柱障碍直径≤30cm；另有“大小2～4m²”表述（p1） | 已询问；面积适用对象不明，不强行推成树径或箱尺寸 |
| 障碍通道 | 宽1.5m、门约0.8m | 同为1.5m通道、两道0.8m门，门位置调整；障碍区限高1.5m且有顶网（p2） | 尺寸吻合；不把1.5m上限直接设为机体中心目标高度 |
| 投递目标 | 曾包含红十字靶、权重10 | 4静态+1动态，坦克为移动靶；帐篷/地堡/桥梁/装甲车/坦克权重1/1.5/2/2.5/5，投放区均1×1m（p3） | 已询问，不自动更改识别类别/任务优先级 |
| 起降标志 | 黑环内径60cm是现场测量；两处位置此前未定 | 两处H标志图片均80×80cm；任务区距底边4m、右边0.7m；隧道端距上边0.7m、右边0.75m（p2～3） | 已询问；80cm整图与60cm环内径不必然冲突；不当作环直径，也不直接生成odom目标 |
| 降落判分 | 按四电机中心圆与黑环判断 | 解析未给该判分细则 | 已询问是否保留原判分，不能据缺失自行删除或认可 |
| 快递 | 前文未完整建模载货外廓 | 约100g/件，14.5×8.5×10cm，3件；可一次或多次搭载，每快递点只计一件（p3～4） | 文件事实已记录，是否覆盖原载荷规格待确认；不假定本机挂载实物等于该值 |

新解析未给出相机内参、DWB增益/速度、PX4控制参数、近地下降阈值等，不能“按规则”推导这些工程参数。前述0.15m/s导航、0.10/0.05m/s下降及0.25m末段高度仍只是离线候选，不是规则规定或用户确认的实飞参数。冲突解决前，不启用新比赛任务配置，不更改已验证起降基线；QGC自动连接保持独立运行。

### 8.3 用户已裁定的合并规格（当前有效）

用户答复：场地/障碍“采用新解析的场地与障碍规格”；投递“原记录为准：包含红十字”；起降标志、60cm内径及电机圆判分问题回答“是的，是的”。据此记录：

- 比赛场地10×8×4m、任务区8×8m；障碍物高度1～3m现场确定，圆柱障碍直径上限0.30m。新解析“2～4m²”适用对象仍不明确，暂不用于自动构造障碍几何。
- 隧道宽1.5m、两道门各宽0.8m且位置调整，障碍区限高1.5m、有顶网。该高度是规则上限，目标机体中心高度还必须扣除机体上外廓、倾斜和误差裕量，不直接设置为1.5m。
- 起降标志共两处，每处整图0.80×0.80m，黑环内径0.60m；80cm不是环直径。保留原规则判分：四电机中心圆全部在黑环内10分、压环5分、不重叠0分。已知电机对角0.40m不改变含罩避障包络。
- 保留原规则投递：4个1×1m标准投放区，另1个0.35×0.35m红十字随机靶；权重帐篷1、地堡1.5、桥梁2、装甲车2.5、红十字10，三个快递。**不采用新解析的动态坦克任务或坦克权重5。**原2026文档仍在图案候选中列坦克，但其计分段未赋坦克权重，不能擅自填5；若现场出现坦克需再询问。随机摆位不等于运动靶。
- 新解析边线距离只记录为场地参考：任务区起降中心距底边4m/右边0.7m，隧道端距上边0.7m/右边0.75m。尚未完成场地坐标到odom配准，不能直接当导航目标。
- 当前测试场地仍为8×5×3m，双箱净距/前距均1.2m，不把它替换成比赛地图。0.6m巡航和已有箱体测试只是局部验证，不代表已覆盖1～3m高障碍、未知高度投递或全场任务。

已建立独立 `config/competition_rules.yaml`，记录来源、合并选择和未知值；landing_candidate同步80cm整图与已确认判分。3项规格回归测试验证场地/测试场地分离、保留红十字不引入动态坦克、整图与环径分离且未启用实飞。没有修改PX4参数、相机内参、Nav2速度或既有起降链。

方案影响：保留A*+DWB作为受限高度层的候选，不将固定高度二维导航等同于完整比赛能力。对1～3m障碍与未知投放高度，需要深度三维占据/净空检查、经验证的高度层切换和目标高度估计；nvblox当前固定高度带不能盲目跨高度复用。动态坦克追踪不加入此次任务。快递尺寸/载货外廓、文件未明确细则和其它冲突继续先询问，不猜测。

## 9. 实际Nav2插件到候选消息链的隔离联合验证（2026-09-29）

扩展 `tests/nvblox_navigation_test.py --trial-profile`：在独立ROS域176，用真实Nav2 nvblox代价地图插件、NavFn A*、DWB和既有传感器过期保护，在合成ESDF场景中生成导航输出，再经过MissionShadow/ShadowPipeline/会话坐标变换构造实际px4_msgs消息对象。候选消息仅在内存检查，未创建PX4发布器。地图、飞行状态、坐标对齐与机体姿态为明确的合成夹具，非当前实景、非运动跟踪、更非PX4动力学仿真。

首两次失败分别为容器没有加载px4_msgs及host symlink-install的/home/cfly绝对软链接在/workspaces挂载下失效，报告保留于 `evidence/nvblox_navigation_20260929_141253/`、`...141341/`。没有重建/升级/改软链接；仅对container-nvblox-test入口增加现有build/px4_msgs/rosidl_generator_py及install/px4_msgs/lib路径，其它入口不变。

修复后 `evidence/nvblox_navigation_20260929_141415/report.json`通过；再增加非零有效DWB指令必须穿过候选链的严格断言，最终 `evidence/nvblox_navigation_20260929_141500/report.json`通过：

- 生命周期active，A*规划出绕障路径，DWB有受保护输出。
- 故障测试前构造23组候选消息，其中4组来自有效非零DWB指令；验证FLU/FRD速度符号、固定巡航Z、0.15m/s上限及无VehicleCommand。
- 深度、ESDF和里程计过期均抑制导航输出；封闭通道拒绝路径。
- 隔离图中没有/fmu/in话题；flight_validation=false。所有测试进程已结束。
- run.sh语法检查通过，QGC自动连接服务仍active，未停止USB桥、未修改PX4或实飞链。

这个结果证明了真实规划插件与候选接口的软件衔接，不证明实际位置跟踪、绕箱净空、下降着陆或真实控制权切换。下一步必须补充动态闭环/唯一出口集成及现场定位、地图和H米制误差证据；不能依据该报告直接装桨或放行飞行。

### 9.1 动态闭环未通过：不能把转向指令计为前进验收

续核（2026-09-29）：用户再次提交的三项规则裁定与第8.3节、`config/competition_rules.yaml`一致，无需再次询问相同问题。其余未明确的载货外廓、障碍面积适用对象等继续保留待确认，不自行覆盖。

新增的 `tests/nvblox_closed_loop.py` 使用真实Nav2插件和简化XY/偏航响应模型，仅在隔离域176运行，不是PX4 SITL。保留两次失败证据 `evidence/nvblox_closed_loop_20260929_142433/` 和 `evidence/nvblox_closed_loop_20260929_142727/`。前一次夹具遗漏角速度反馈；补充反馈后第二次仍失败：

- 起点与最终位置均为 `(1.0, 2.5)` m，目标距离仍为4.0 m；FollowPath约15秒后报 `Failed to make progress`，动作状态6。
- 受保护前进速度最大值为0，偏航角速度最大值约0.13681 rad/s；349组候选消息的平移速度均为0。候选链没有报告fault或抑制原因，不据此直接断定DWB根因。
- 保守包络没有碰撞，隔离图没有 `/fmu/in/` 话题；这只说明该次原地测试没有碰撞/飞控输入，不代表绕障安全或任务完成。
- 第9节静态测试把“平移或转向非零”计入非零指令数，其通过只证明接口连通，不能当作有效前进/路径跟踪证据。

待排查的是原始DWB输出、各轨迹评分与速度采样；低加速度采样和5cm网格可能影响起步评分，目前仅是假设，不能记为已定位或已修复。不通过关闭进度保护、放宽碰撞边界或启动实飞来绕过失败。下一验收应要求虚拟飞机确实前进、绕障、到达目标，再做故障回归；H对准下降与唯一飞控出口仍需另外完成。当前实际自主避障与H降落未通过，已验证起降基线不作修改。

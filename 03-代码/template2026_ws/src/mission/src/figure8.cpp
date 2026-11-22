#include <template.h>

/************************************************************************
  figure8 —— 自编动作（∞ 双纽线连续轨迹）任务节点
  2026-11-18 新增，与 square/hover/avoidance 并列，不改动它们。

  与"定点"任务的区别：
    1) 参考点是连续曲线上的点，不是几个离散航点；
    2) 带 xy 速度前馈 + 弧长进度门控，解决"位置滞后 → 图形缩水"；
    3) 完成判据用实际累计弧长，不用计时；
    4) 全程记录横向跟踪误差，落地后可 scp 回地面出图。

  状态机： TAKEOFF → SETTLE → TRACK → RETURN → LAND（偏离超限进 HOVER_ALERT）
*************************************************************************/

// ==================== 状态机 ====================
int mission_num = 0;
float if_debug = 0;   // 1: 程序自动切 OFFBOARD；0: 等飞手拨 SwD
float err_max = 0.15; // 到点判据（起飞点 / 回原点）

// ==================== 几何参数 ====================
float curve_a = 0.5f;    // 双纽线半长轴 [m]
float y_scale = 1.41f;   // y 方向拉伸（1.41 → y 幅值 0.25 m）
float altitude = 0.8f;   // 相对起飞点高度 [m]

// ==================== 跟踪参数 ====================
float v_max = 0.45f;     // 最大弧长速度 [m/s]
float v_min = 0.30f;     // 最小弧长速度 [m/s]
float a_lat_max = 1.2f;  // 曲率限速用横向加速度上限 [m/s²]
float k_ct = 1.5f;       // 横向纠偏增益（仅 vel 模式）
float k_gate = 0.15f;    // 进度门控裕度 [m]：实际落后超过它就冻结参考点
int ctrl_mode = TRAJ_CTRL_POS_FF; // TRAJ_CTRL_POS_FF / TRAJ_CTRL_VEL
int track_mode = 0;      // 0=progress(带门控) 1=timed(纯时间推进，对照组)

// ==================== 圈数与 yaw ====================
int laps = 3;            // 圈数
int yaw_mode = 1;        // 0=fixed 1=sweep 2=tangent
float yaw_sweep_amp = 0.25f; // sweep 幅度 [rad]
float yaw_rate_max = 0.8f;   // yaw 指令变化率上限 [rad/s]

// ==================== 安全 ====================
float dev_max = 0.30f;   // 横向偏差上限 [m]
float dev_time = 0.5f;   // 超限持续多久触发悬停 [s]
float timeout_scale = 2.0f; // 总时长上限 = scale × 名义时长
float z_amp = 0.0f;      // >0 时叠加 z 起伏，做"三维 ∞"
bool csv_on = true;

// ==================== 运行期对象 ====================
PathLUT g_lut;
TrajStat g_stat;
TrajCsv g_csv;
std::vector<geometry_msgs::Point> g_trail;
std::string g_curve_name = "bernoulli";

// 跟踪循环内部状态
static bool g_track_inited = false;
static ros::Time g_track_t0, g_track_prev;
static float g_s_start = 0.0f;   // 入轨弧长（由 find_enter_s 自动定位到起飞点）
static float g_s_cmd_acc = 0.0f; // 参考点累计弧长
static float g_s_act_acc = 0.0f; // 实际累计弧长
static float g_s_prev = 0.0f;    // 上一周期实际弧长（投影窗口中心）
static float g_v_f = 0.0f;       // 低通后的速度
static float g_yaw_cmd = 0.0f;   // 限速后的 yaw 指令
static float g_t_bad = 0.0f;     // 偏差超限累计时间
static float g_stall_t = 0.0f;   // 实际进度停滞累计时间（防门控死锁）
static long g_loop_cnt = 0;

// 找入轨弧长：曲线过起飞原点 (0,0) 的分支里，取切向最接近 +x 的那个。
// 双纽线有两个分支过原点（自交点），这一步比写死 L/2 更稳，换曲线也不会错。
float find_enter_s(const PathLUT &lut)
{
	float best_s = 0.0f, best_score = 1e9f;
	for (size_t i = 0; i < lut.x.size(); ++i)
	{
		if (hypotf(lut.x[i], lut.y[i]) > 0.02f)
			continue; // 只考虑过原点的分支
		const float ang = fabsf(atan2f(lut.ty[i], lut.tx[i])); // 与 +x 的夹角
		if (ang < best_score)
		{
			best_score = ang;
			best_s = lut.s[i];
		}
	}
	if (best_score > 1e8f)
	{
		ROS_WARN("曲线不过起飞原点，入轨点取 s=0");
		return 0.0f;
	}
	return best_s;
}

// 相对起飞原点的当前位置
static inline void rel_pos(float &px, float &py, float &pz)
{
	px = local_pos.pose.pose.position.x - init_position_x_take_off;
	py = local_pos.pose.pose.position.y - init_position_y_take_off;
	pz = local_pos.pose.pose.position.z - init_position_z_take_off;
}

// 伯努利双纽线：x = a·cosθ/(1+sin²θ)，y = k·a·sinθcosθ/(1+sin²θ)
// 曲率处处连续；θ=π/2 处为自交点，正好落在起飞点 (0,0)。
// gerono：x = a·cosθ，y = k·a·sinθcosθ（有尖角，仅作对照实验用）。
void build_curve(PathLUT &lut)
{
	std::vector<float> xs, ys;
	const int N = 1440;
	const bool gerono = (g_curve_name == "gerono");
	for (int i = 0; i < N; ++i)
	{
		const float t = 2.0f * (float)M_PI * (float)i / (float)N;
		const float s = sinf(t), c = cosf(t);
		if (gerono)
		{
			xs.push_back(curve_a * c);
			ys.push_back(y_scale * curve_a * s * c);
		}
		else
		{
			const float d = 1.0f + s * s;
			xs.push_back(curve_a * c / d);
			ys.push_back(y_scale * curve_a * s * c / d);
		}
	}
	path_build(lut, xs, ys, 2000);
}

void print_param()
{
	std::cout << "=== 控制参数 ===" << std::endl;
	std::cout << "curve        : " << g_curve_name << std::endl;
	std::cout << "a / y_scale  : " << curve_a << " / " << y_scale << std::endl;
	std::cout << "弧长 L       : " << g_lut.total << " m  (采样点数 " << g_lut.x.size() << ")" << std::endl;
	std::cout << "altitude     : " << altitude << std::endl;
	std::cout << "err_max      : " << err_max << std::endl;
	std::cout << "v_max/v_min  : " << v_max << " / " << v_min << std::endl;
	std::cout << "a_lat_max    : " << a_lat_max << std::endl;
	std::cout << "mode         : " << (track_mode == 0 ? "progress" : "timed")
			  << "  ctrl: " << (ctrl_mode == TRAJ_CTRL_VEL ? "vel" : (ctrl_mode == TRAJ_CTRL_POS ? "pos" : "pos_ff")) << std::endl;
	std::cout << "laps         : " << laps << std::endl;
	std::cout << "yaw_mode     : " << (yaw_mode == 0 ? "fixed" : (yaw_mode == 1 ? "sweep" : "tangent"))
			  << "  amp: " << yaw_sweep_amp << std::endl;
	std::cout << "dev_max      : " << dev_max << " (dev_time " << dev_time << " s)" << std::endl;
	const float T_lap = g_lut.total / std::max(v_min, 0.10f);
	std::cout << "估算单圈     : " << T_lap << " s   " << laps << " 圈: " << laps * T_lap << " s（按 v_min）" << std::endl;
	std::cout << "超时上限     : " << timeout_scale * laps * T_lap << " s" << std::endl;
	std::cout << "csv          : " << (csv_on ? "on" : "off") << std::endl;
	std::cout << "if_debug     : " << if_debug << std::endl;
	if (if_debug == 1)
		cout << "自动offboard" << std::endl;
	else
		cout << "遥控器offboard" << std::endl;
}

// 单步跟踪：返回 true 表示圈数已跑满
bool track_step()
{
	float px, py, pz;
	rel_pos(px, py, pz);

	if (!g_track_inited)
	{
		g_track_t0 = g_track_prev = ros::Time::now();
		g_s_start = find_enter_s(g_lut); // 自交点 = 起飞点，起飞即入轨
		g_s_cmd_acc = 0.0f;
		g_s_act_acc = 0.0f;
		g_s_prev = g_s_start;
		g_v_f = v_min;
		g_yaw_cmd = 0.0f; // sweep/tangent 的初值都从 0 附近起步
		g_t_bad = 0.0f;
		g_stall_t = 0.0f;
		g_loop_cnt = 0;
		g_stat.reset();
		g_trail.clear();
		if (csv_on)
			g_csv.open("fig8");
		g_track_inited = true;
		ROS_INFO("入轨：s0=%.3f m（自交点），目标 %.1f 圈 = %.2f m",
				 g_s_start, (float)laps, laps * g_lut.total);
	}

	const ros::Time now = ros::Time::now();
	float dt = (now - g_track_prev).toSec();
	g_track_prev = now;
	dt = std::max(1e-3f, std::min(0.1f, dt));
	++g_loop_cnt;

	// 1) 参考点几何 + 曲率限速（在 s_ref 处取，天然有预瞄）
	float s_ref = g_s_start + g_s_cmd_acc;
	float qx, qy, tx, ty, kappa;
	path_sample(g_lut, s_ref, qx, qy, tx, ty, kappa);
	const float v_t = path_speed_limit(kappa, a_lat_max, v_min, v_max);
	g_v_f += (v_t - g_v_f) * std::min(1.0f, dt / 0.4f);

	// 2) 名义推进
	g_s_cmd_acc += g_v_f * dt;

	// 3) 实际进度：把当前位置投影到曲线上（窗口 ±0.4 m，防自交点跳瓣）
	float s_now, e_cross;
	path_project(g_lut, px, py, g_s_prev, 0.4f, s_now, e_cross);
	float ds = s_now - g_s_prev;
	if (ds > g_lut.total * 0.5f)
		ds -= g_lut.total;
	if (ds < -g_lut.total * 0.5f)
		ds += g_lut.total;
	g_s_act_acc += ds;
	g_s_prev = s_now;

	// 4) 停滞检测：真机上飞机可能“追到胡萝卜就停住”，此时 s_act 不再增长
	if (fabsf(ds) < 1e-4f)
		g_stall_t += dt;
	else
		g_stall_t = 0.0f;

	// 5) 进度门控：实际落后太多就冻结参考点（先保形状，不赶时间）
	//    但停滞超过 0.5 s 必须释放，否则“飞机等胡萝卜、胡萝卜等飞机”死锁
	if (track_mode == 0 && (g_s_cmd_acc - g_s_act_acc) > k_gate && g_stall_t < 0.5f)
	{
		g_s_cmd_acc = g_s_act_acc + k_gate;
	}
	else if (track_mode == 0 && g_stall_t >= 0.5f)
	{
		ROS_WARN_THROTTLE(1.0, "检测到进度停滞 %.1f s，已释放门控强制推进", g_stall_t);
	}
	if (g_s_cmd_acc < 0.0f)
		g_s_cmd_acc = 0.0f;

	// 5) 取推进后的参考点，组装参考
	s_ref = g_s_start + g_s_cmd_acc;
	path_sample(g_lut, s_ref, qx, qy, tx, ty, kappa);
	const float nx = -ty, ny = tx;
	const float theta = 2.0f * (float)M_PI * (g_s_cmd_acc / g_lut.total);

	TrajRef r;
	r.x = qx;
	r.y = qy;
	r.z = altitude + (z_amp != 0.0f ? z_amp * sinf(2.0f * theta) : 0.0f);
	r.vx = g_v_f * tx;
	r.vy = g_v_f * ty;
	if (ctrl_mode == TRAJ_CTRL_VEL)
	{
		// 纯速度模式：把横向偏差折成指向曲线的速度分量
		float v_corr = -k_ct * e_cross;
		v_corr = std::max(-0.25f, std::min(0.25f, v_corr));
		r.vx += v_corr * nx;
		r.vy += v_corr * ny;
		const float sp = hypotf(r.vx, r.vy);
		const float lim = v_max * 1.3f;
		if (sp > lim && sp > 1e-6f)
		{
			r.vx *= lim / sp;
			r.vy *= lim / sp;
		}
	}
	r.kappa = kappa;
	r.s = g_s_cmd_acc;

	// 6) yaw：三档 + 变化率限幅（square 实测 yaw 是瓶颈，必须限速）
	float yaw_t = 0.0f;
	if (yaw_mode == 0)
		yaw_t = 0.0f;
	else if (yaw_mode == 1)
		yaw_t = yaw_sweep_amp * sinf(2.0f * theta); // 每圈摆两次，起点终点都是 0
	else
		yaw_t = atan2f(ty, tx);
	float dyaw = yaw_t - g_yaw_cmd;
	while (dyaw > (float)M_PI)
		dyaw -= 2.0f * (float)M_PI;
	while (dyaw < -(float)M_PI)
		dyaw += 2.0f * (float)M_PI;
	const float dyaw_lim = yaw_rate_max * dt;
	dyaw = std::max(-dyaw_lim, std::min(dyaw_lim, dyaw));
	g_yaw_cmd += dyaw;
	r.yaw = g_yaw_cmd;

	traj_send(r, ctrl_mode);

	// 7) 统计 / 轨迹 / 落盘 / 日志
	g_stat.push(e_cross, px, py);
	if (g_loop_cnt % 4 == 1)
	{
		geometry_msgs::Point p;
		p.x = local_pos.pose.pose.position.x;
		p.y = local_pos.pose.pose.position.y;
		p.z = local_pos.pose.pose.position.z;
		g_trail.push_back(p);
		if (g_trail.size() > 3000)
			g_trail.erase(g_trail.begin());
	}
	if (csv_on)
		g_csv.write((now - g_track_t0).toSec(), g_s_cmd_acc, g_s_act_acc,
					e_cross, g_v_f, kappa, px, py, g_yaw_cmd);
	ROS_INFO_THROTTLE(0.5,
					  "s=%.2f/%.2f m  e=%.3f m  v=%.2f m/s  κ=%.2f  yaw=%.1f°  %.1f 圈",
					  g_s_act_acc, laps * g_lut.total, e_cross, g_v_f, kappa,
					  g_yaw_cmd * 180.0f / (float)M_PI, g_s_act_acc / g_lut.total);

	// 8) 安全：横向偏差超限 → 悬停告警
	if (fabsf(e_cross) > dev_max)
		g_t_bad += dt;
	else
		g_t_bad = 0.0f;
	if (g_t_bad > dev_time)
	{
		ROS_ERROR("!! 横向偏差超限 (%.2f m > %.2f m) 持续 %.1f s，已转悬停，请拨 SwD 接管",
				  e_cross, dev_max, g_t_bad);
		if (csv_on)
			g_csv.close();
		g_track_inited = false;
		mission_num = 5;
		return false;
	}

	// 9) 超时保护 → 直接回原点降落
	const float T_nom = laps * g_lut.total / std::max(v_min, 0.10f); // 按最慢速度估，避免进度门控降速后误超时
	if ((now - g_track_t0).toSec() > timeout_scale * T_nom)
	{
		ROS_WARN("跟踪超时 (%.1f s)，转回原点降落", (now - g_track_t0).toSec());
		if (csv_on)
			g_csv.close();
		g_track_inited = false;
		mission_num = 3;
		return false;
	}

	// 10) 完成判据：按实际累计弧长，不按时间
	if (g_s_act_acc >= laps * g_lut.total - 0.02f)
	{
		const float close_err = hypotf(px, py); // 期望回到 (0,0)
		ROS_INFO("∞ 轨迹完成：实际弧长 %.2f m，闭合误差 %.3f m，用时 %.1f s",
				 g_s_act_acc, close_err, (now - g_track_t0).toSec());
		std::cout << "===== 跟踪结果 =====" << std::endl;
		g_stat.report("TRACK");
		std::cout << "闭合误差: " << close_err << " m" << std::endl;
		std::cout << "x 幅值: " << (g_stat.max_x - g_stat.min_x) / 2.0f
				  << " m   y 幅值: " << (g_stat.max_y - g_stat.min_y) / 2.0f << " m" << std::endl;
		if (csv_on)
			g_csv.close();
		g_track_inited = false;
		return true;
	}
	return false;
}

int main(int argc, char **argv)
{
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "figure8");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
	ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
	traj_marker_init(nh, "/mission/fig8_marker");

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	ros::Rate rate(20); // OFFBOARD 要求 >2 Hz，沿用 20 Hz

	// ---- 参数 ----
	nh.param<std::string>("curve", g_curve_name, "bernoulli");
	nh.param<float>("curve_a", curve_a, 0.5f);
	nh.param<float>("curve_y_scale", y_scale, 1.41f);
	nh.param<float>("altitude", altitude, 0.8f);

	nh.param<float>("v_max", v_max, 0.45f);
	nh.param<float>("v_min", v_min, 0.30f);
	nh.param<float>("a_lat_max", a_lat_max, 1.2f);
	nh.param<float>("k_ct", k_ct, 1.5f);
	nh.param<float>("k_gate", k_gate, 0.15f);
	{
		std::string cm = "pos_ff", tm = "progress", ym = "sweep";
		nh.param<std::string>("ctrl_mode", cm, "pos_ff");
		nh.param<std::string>("track_mode", tm, "progress");
		nh.param<std::string>("yaw_mode", ym, "sweep");
		if (cm == "vel")
			ctrl_mode = TRAJ_CTRL_VEL;
		else if (cm == "pos")
			ctrl_mode = TRAJ_CTRL_POS;
		else
			ctrl_mode = TRAJ_CTRL_POS_FF;
		track_mode = (tm == "timed") ? 1 : 0;
		yaw_mode = (ym == "fixed") ? 0 : ((ym == "tangent") ? 2 : 1);
	}

	nh.param<int>("laps", laps, 3);
	nh.param<float>("yaw_sweep_amp", yaw_sweep_amp, 0.25f);
	nh.param<float>("yaw_rate_max", yaw_rate_max, 0.8f);

	nh.param<float>("err_max", err_max, 0.15f);
	nh.param<float>("dev_max", dev_max, 0.30f);
	nh.param<float>("dev_time", dev_time, 0.5f);
	nh.param<float>("timeout_scale", timeout_scale, 2.0f);
	nh.param<float>("z_amp", z_amp, 0.0f);
	nh.param<bool>("csv", csv_on, true);
	nh.param<float>("if_debug", if_debug, 0.0f);

	build_curve(g_lut);
	if (!g_lut.ready)
	{
		ROS_ERROR("曲线建表失败，退出");
		return 1;
	}
	print_param();

	int choice = 0;
	std::cout << "1 to go on , else to quit" << std::endl;
	std::cin >> choice;
	if (choice != 1)
		return 0;
	ros::spinOnce();
	rate.sleep();

	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	ros::Time last_request = ros::Time::now();

	// ---- 起飞：飞到 (0,0,altitude)，与 square 完全一致 ----
	while (ros::ok())
	{
		if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
		{
			if (if_debug == 1)
			{
				if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
					ROS_INFO("Offboard enabled");
			}
			else
			{
				ROS_INFO("Waiting for OFFBOARD mode");
			}
			last_request = ros::Time::now();
		}
		else
		{
			if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
			{
				if (arming_client.call(arm_cmd) && arm_cmd.response.success)
					ROS_INFO("Vehicle armed");
				last_request = ros::Time::now();
			}
		}
		if (fabs(local_pos.pose.pose.position.z - init_position_z_take_off - altitude) < 0.2)
		{
			if (ros::Time::now() - last_request > ros::Duration(1.0))
			{
				mission_num = 1;
				last_request = ros::Time::now();
				break;
			}
		}
		mission_pos_cruise(0, 0, altitude, 0, err_max);
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}

	// ---- 任务状态机 ----
	while (ros::ok())
	{
		switch (mission_num)
		{
		case 1: // 稳姿态
			if (hover(1.0f))
				mission_num = 2;
			break;

		case 2: // ∞ 跟踪
			if (track_step())
			{
				mission_num = 3;
				last_request = ros::Time::now();
			}
			break;

		case 3: // 回原点：用"径向"判据，不用 mission_pos_cruise 的分轴判据
			{
				const float rx = local_pos.pose.pose.position.x - init_position_x_take_off;
				const float ry = local_pos.pose.pose.position.y - init_position_y_take_off;
				const float rad = hypotf(rx, ry);
				mission_pos_cruise(0, 0, altitude, 0, err_max); // 一直压向原点
				if (rad < 0.05f)
				{
					ROS_INFO("已回到原点，径向偏差 %.3f m", rad);
					mission_num = 4;
					last_request = ros::Time::now();
				}
				else if (ros::Time::now() - last_request >= ros::Duration(10.0))
				{
					ROS_WARN("回原点超时，当前径向偏差 %.3f m，直接降落", rad);
					mission_num = 4;
					last_request = ros::Time::now();
				}
			}
			break;

		case 4: // 降落
			if (precision_land())
			{
				mission_num = -1;
				last_request = ros::Time::now();
			}
			break;

		case 5: // 偏差超限悬停告警（等人接管，不自动降落）
			hover(600.0f);
			ROS_ERROR_THROTTLE(2.0, "偏差超限悬停中 —— 请拨 SwD 接管/降落");
			break;
		}

		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();

		if (mission_num == -1)
		{
			ROS_INFO("任务结束。落地后请手动上锁。");
			if (g_track_inited)
				g_csv.close();
			return 0;
		}
	}
	return 0;
}


/************************************************************************
函数 11~16：连续轨迹跟踪原语（2026-11-18 新增，自编 ∞ 动作）
  面向"连续曲线"任务（双纽线、圆弧、任意闭合路径），
  与 mission_pos_cruise（单点位置控制）互补，不改动其任何行为。
  坐标系、type_mask 约定与 mission_pos_cruise 完全一致：
  传入坐标为"相对起飞原点"的 ENU 数值，由 mavros 负责转 NED。
*************************************************************************/
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

// ---------------- 11. 轨迹参考点 + 打包发送 ----------------
#define TRAJ_CTRL_POS_FF 0 // 位置 + xy 速度前馈（PX4 位置环 + 前馈）
#define TRAJ_CTRL_VEL    1 // xy 纯速度 + z 位置定高（同 avoidance 的用法）
#define TRAJ_CTRL_POS    2 // 纯位置（同 mission_pos_cruise 的 mask，不带前馈）

struct TrajRef
{
	float x, y, z; // 目标位置，相对起飞原点 [m]
	float vx, vy;  // 前馈速度 [m/s]（z 方向不用，恒 0）
	float yaw;     // [rad]
	float s;       // 弧长进度 [m]（仅日志）
	float kappa;   // 曲率 [1/m]（仅日志）
};

// 把轨迹参考点写进全局 setpoint_raw；publish 由调用方负责（保持 20 Hz 流）。
inline void traj_send(const TrajRef &r, int ctrl_mode)
{
	if (ctrl_mode == TRAJ_CTRL_VEL)
	{
		// 屏蔽 px/py/vz/加速度/FORCE/yaw_rate：xy 用速度，z 用位置
		setpoint_raw.type_mask = 1 + 2 + 32 + 64 + 128 + 256 + 512 + 2048;
		setpoint_raw.position.x = 0.0f; // 被屏蔽，占位
		setpoint_raw.position.y = 0.0f;
		setpoint_raw.position.z = r.z + init_position_z_take_off;
	}
	else if (ctrl_mode == TRAJ_CTRL_POS)
	{
		// 纯位置：vx/vy 一起屏蔽（与 mission_pos_cruise 完全一致）
		setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048;
		setpoint_raw.position.x = r.x + init_position_x_take_off;
		setpoint_raw.position.y = r.y + init_position_y_take_off;
		setpoint_raw.position.z = r.z + init_position_z_take_off;
	}
	else
	{
		// 屏蔽 vz/加速度/FORCE/yaw_rate：位置 + xy 速度前馈
		setpoint_raw.type_mask = 32 + 64 + 128 + 256 + 512 + 2048;
		setpoint_raw.position.x = r.x + init_position_x_take_off;
		setpoint_raw.position.y = r.y + init_position_y_take_off;
		setpoint_raw.position.z = r.z + init_position_z_take_off;
	}
	setpoint_raw.coordinate_frame = 1; // FRAME_LOCAL_NED（数值按 ENU 给）
	setpoint_raw.velocity.x = r.vx;
	setpoint_raw.velocity.y = r.vy;
	setpoint_raw.velocity.z = 0.0f;
	setpoint_raw.yaw = r.yaw;
}

// ---------------- 12. 弧长参数化路径表（等弧长采样 + 切向 + 曲率） ----------------
struct PathLUT
{
	std::vector<float> s;     // 累计弧长 [m]，等间隔
	std::vector<float> x, y;  // 位置 [m]
	std::vector<float> tx, ty;// 单位切向
	std::vector<float> kappa; // 曲率 [1/m]
	float total = 0.0f;
	bool ready = false;
};

// 用任意闭合曲线的原始采样点建表；resample_n 为等弧长重采样点数。
inline void path_build(PathLUT &lut,
					   const std::vector<float> &raw_x,
					   const std::vector<float> &raw_y,
					   int resample_n = 2000)
{
	lut.s.clear();
	lut.x.clear();
	lut.y.clear();
	lut.tx.clear();
	lut.ty.clear();
	lut.kappa.clear();
	lut.ready = false;
	lut.total = 0.0f;

	const int n = (int)raw_x.size();
	if (n < 8 || (int)raw_y.size() != n || resample_n < 16)
		return;

	// 1) 原始点累计弧长（末尾闭合回起点）
	std::vector<float> cs(n + 1, 0.0f);
	for (int i = 0; i < n; ++i)
	{
		const int j = (i + 1) % n;
		cs[i + 1] = cs[i] + hypotf(raw_x[j] - raw_x[i], raw_y[j] - raw_y[i]);
	}
	const float L = cs[n];
	if (L <= 1e-3f)
		return;

	// 2) 等弧长重采样（s[k] = L*k/resample_n，后面可直接用索引算 s）
	lut.s.reserve(resample_n);
	lut.x.reserve(resample_n);
	lut.y.reserve(resample_n);
	int seg = 0;
	for (int k = 0; k < resample_n; ++k)
	{
		const float target = L * (float)k / (float)resample_n;
		while (seg < n - 1 && cs[seg + 1] < target)
			++seg;
		const int i = seg, j = (seg + 1) % n;
		const float seg_len = cs[seg + 1] - cs[seg];
		const float u = (seg_len > 1e-9f) ? (target - cs[seg]) / seg_len : 0.0f;
		lut.s.push_back(target);
		lut.x.push_back(raw_x[i] + u * (raw_x[j] - raw_x[i]));
		lut.y.push_back(raw_y[i] + u * (raw_y[j] - raw_y[i]));
	}
	lut.total = L;

	// 3) 中心差分求切向与曲率（闭合索引）
	const float ds = L / (float)resample_n;
	for (int k = 0; k < resample_n; ++k)
	{
		const int kp = (k + 1) % resample_n;
		const int km = (k - 1 + resample_n) % resample_n;
		float dx = lut.x[kp] - lut.x[km];
		float dy = lut.y[kp] - lut.y[km];
		const float len = hypotf(dx, dy);
		if (len < 1e-9f)
		{
			dx = 1.0f;
			dy = 0.0f;
		}
		else
		{
			dx /= len;
			dy /= len;
		}
		lut.tx.push_back(dx);
		lut.ty.push_back(dy);
	}
	for (int k = 0; k < resample_n; ++k)
	{
		const int kp = (k + 1) % resample_n;
		const int km = (k - 1 + resample_n) % resample_n;
		float da = atan2f(lut.ty[kp], lut.tx[kp]) - atan2f(lut.ty[km], lut.tx[km]);
		while (da > M_PI)
			da -= 2.0f * M_PI;
		while (da < -M_PI)
			da += 2.0f * M_PI;
		lut.kappa.push_back(fabsf(da) / (2.0f * ds));
	}
	lut.ready = true;
}

// 按弧长 s 取参考点（O(1)，s 自动折回 [0,total)）
inline bool path_sample(const PathLUT &lut, float s,
						float &x, float &y,
						float &tx, float &ty, float &kappa)
{
	if (!lut.ready)
		return false;
	const int m = (int)lut.s.size();
	s = fmodf(s, lut.total);
	if (s < 0.0f)
		s += lut.total;
	float f = s / lut.total * (float)m;
	int i = (int)floorf(f);
	if (i >= m)
		i = m - 1;
	if (i < 0)
		i = 0;
	const int j = (i + 1) % m;
	const float u = f - (float)i;
	x = lut.x[i] + u * (lut.x[j] - lut.x[i]);
	y = lut.y[i] + u * (lut.y[j] - lut.y[i]);
	tx = lut.tx[i] + u * (lut.tx[j] - lut.tx[i]);
	ty = lut.ty[i] + u * (lut.ty[j] - lut.ty[i]);
	const float tl = hypotf(tx, ty);
	if (tl > 1e-9f)
	{
		tx /= tl;
		ty /= tl;
	}
	kappa = lut.kappa[i] + u * (lut.kappa[j] - lut.kappa[i]);
	return true;
}

// 把当前位置投影到路径上（窗口化 + 只向前），得到实际弧长 s_out 与有符号横向偏差 e_cross。
// window: 搜索半窗 [m]（防止在自交点跳到另一瓣）；e_cross > 0 表示偏在法向 N=(-ty,tx) 一侧。
inline bool path_project(const PathLUT &lut, float px, float py,
						 float s_prev, float window,
						 float &s_out, float &e_cross)
{
	if (!lut.ready)
		return false;
	const int m = (int)lut.s.size();
	const float ds = lut.total / (float)m;
	int kc = (int)floorf(fmodf(s_prev, lut.total) / ds);
	int w = (int)ceilf(window / ds);
	if (w < 2)
		w = 2;

	float best_d2 = 1e18f;
	float best_s = s_prev;
	float best_nx = 1.0f, best_ny = 0.0f;
	float best_qx = 0.0f, best_qy = 0.0f;

	for (int d = -w; d <= w; ++d)
	{
		const int k = (((kc + d) % m) + m) % m;
		const int k2 = (k + 1) % m;
		const float ax = lut.x[k], ay = lut.y[k];
		float bx = lut.x[k2] - ax, by = lut.y[k2] - ay;
		const float l2 = bx * bx + by * by;
		float u = 0.0f;
		if (l2 > 1e-12f)
			u = ((px - ax) * bx + (py - ay) * by) / l2;
		u = std::max(0.0f, std::min(1.0f, u));
		const float qx = ax + u * bx, qy = ay + u * by;
		const float d2 = (px - qx) * (px - qx) + (py - qy) * (py - qy);
		if (d2 < best_d2)
		{
			best_d2 = d2;
			best_s = ((float)k + u) * ds;
			const float bl = sqrtf(l2);
			if (bl > 1e-9f)
			{
				best_nx = -by / bl;
				best_ny = bx / bl;
			}
			best_qx = qx;
			best_qy = qy;
		}
	}
	s_out = fmodf(best_s, lut.total);
	if (s_out < 0.0f)
		s_out += lut.total;
	e_cross = (px - best_qx) * best_nx + (py - best_qy) * best_ny;
	return true;
}

// 曲率限速：v = min(v_max, sqrt(a_lat_max * R))
inline float path_speed_limit(float kappa, float a_lat_max, float v_min, float v_max)
{
	const float R = 1.0f / std::max(kappa, 1e-3f);
	float v = sqrtf(a_lat_max * std::min(R, 10.0f));
	v = std::max(v_min, std::min(v_max, v));
	return v;
}

// ---------------- 13. 跟踪误差统计 ----------------
struct TrajStat
{
	long n = 0;
	double sum_e2 = 0.0;
	float max_abs_e = 0.0f;
	float min_x = 1e9f, max_x = -1e9f;
	float min_y = 1e9f, max_y = -1e9f;

	void reset()
	{
		n = 0;
		sum_e2 = 0.0;
		max_abs_e = 0.0f;
		min_x = 1e9f; max_x = -1e9f;
		min_y = 1e9f; max_y = -1e9f;
	}
	void push(float e, float px, float py)
	{
		++n;
		sum_e2 += (double)e * (double)e;
		if (fabsf(e) > max_abs_e)
			max_abs_e = fabsf(e);
		min_x = std::min(min_x, px); max_x = std::max(max_x, px);
		min_y = std::min(min_y, py); max_y = std::max(max_y, py);
	}
	double rmse() const { return n > 0 ? sqrt(sum_e2 / (double)n) : 0.0; }
	void report(const char *tag = "TRACK") const
	{
		ROS_INFO("[%s] n=%ld RMSE=%.3f m  max|e|=%.3f m  x∈[%.3f,%.3f] y∈[%.3f,%.3f]",
				 tag, n, rmse(), max_abs_e, min_x, max_x, min_y, max_y);
	}
};

// ---------------- 14. RViz 可视化（理论曲线 + 实际轨迹 + 当前位置） ----------------
inline void traj_marker_init(ros::NodeHandle &nh,
							 const std::string &topic = "/mission/traj_marker")
{
	marker_pub = nh.advertise<visualization_msgs::Marker>(topic, 10);
}

inline void traj_marker_publish(const PathLUT &lut,
								float z,
								const std::vector<geometry_msgs::Point> &trail,
								int id_curve = 0, int id_trail = 1, int id_now = 2)
{
	if (!marker_pub)
		return;
	visualization_msgs::Marker mk;
	mk.header.frame_id = "world";
	mk.header.stamp = ros::Time::now();
	mk.action = visualization_msgs::Marker::ADD;
	mk.pose.orientation.w = 1.0;

	// 理论曲线（蓝）
	if (lut.ready)
	{
		mk.ns = "fig8_reference";
		mk.id = id_curve;
		mk.type = visualization_msgs::Marker::LINE_STRIP;
		mk.scale.x = 0.012;
		mk.color.r = 0.1f; mk.color.g = 0.4f; mk.color.b = 1.0f; mk.color.a = 0.9f;
		mk.points.clear();
		for (size_t i = 0; i < lut.x.size(); i += 4)
		{
			geometry_msgs::Point p;
			p.x = lut.x[i] + init_position_x_take_off;
			p.y = lut.y[i] + init_position_y_take_off;
			p.z = z + init_position_z_take_off;
			mk.points.push_back(p);
		}
		marker_pub.publish(mk);
	}

	// 实际轨迹（绿）
	if (trail.size() >= 2)
	{
		mk.ns = "fig8_actual";
		mk.id = id_trail;
		mk.type = visualization_msgs::Marker::LINE_STRIP;
		mk.scale.x = 0.02;
		mk.color.r = 0.0f; mk.color.g = 1.0f; mk.color.b = 0.2f; mk.color.a = 0.9f;
		mk.points = trail;
		marker_pub.publish(mk);
	}

	// 当前位置（红球）
	if (!trail.empty())
	{
		mk.ns = "fig8_now";
		mk.id = id_now;
		mk.type = visualization_msgs::Marker::SPHERE;
		mk.scale.x = 0.06; mk.scale.y = 0.06; mk.scale.z = 0.06;
		mk.color.r = 1.0f; mk.color.g = 0.1f; mk.color.b = 0.1f; mk.color.a = 1.0f;
		mk.points.clear();
		mk.pose.position = trail.back();
		marker_pub.publish(mk);
	}
}

// ---------------- 15. 跟踪数据落盘（CSV，落地后 scp 回地面出图） ----------------
struct TrajCsv
{
	std::string path;
	FILE *fp = nullptr;

	bool open(const std::string &prefix = "fig8")
	{
		const char *home = getenv("HOME");
		if (home == nullptr)
			return false;
		char buf[512];
		time_t now = time(nullptr);
		struct tm *t = localtime(&now);
		snprintf(buf, sizeof(buf), "%s/.ros/%s-%04d%02d%02d-%02d%02d%02d.csv",
				 home, prefix.c_str(),
				 t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
				 t->tm_hour, t->tm_min, t->tm_sec);
		path = buf;
		fp = fopen(path.c_str(), "w");
		if (fp == nullptr)
			return false;
		fprintf(fp, "t,s_cmd,s_act,e_cross,v_cmd,kappa,px,py,yaw\n");
		return true;
	}
	void write(float t, float s_cmd, float s_act, float e, float v, float k,
			   float px, float py, float yaw)
	{
		if (fp == nullptr)
			return;
		fprintf(fp, "%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f\n",
				t, s_cmd, s_act, e, v, k, px, py, yaw);
	}
	void close()
	{
		if (fp != nullptr)
		{
			fclose(fp);
			fp = nullptr;
			ROS_INFO("跟踪数据已保存: %s", path.c_str());
		}
	}
};

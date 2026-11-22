#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_set>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <visualization_msgs/Marker.h>

using namespace std;

#define ALTITUDE 0.8f
#define AIM_ALTITUDE 0.8f
#define AIM_CATAPULT_ALTITUDE 0.65f
#define AIM_RED_ALTITUDE 0.25f

//调平高度
float z_offset1 = 0.0;
float z_offset2 = 0.0;
float z_offset3 = 0.0;
float z_offset4 = 0.0;
vector<float> z_offset = {0.0,0.0,0.0,0.0,0.0}; //从1开始存储

int box_num = 1; //投货编号

mavros_msgs::PositionTarget setpoint_raw;

/************************************************************************
函数 1：无人机状态回调函数
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}

/************************************************************************
函数 2：回调函数接收无人机的里程计信息
*************************************************************************/
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	local_pos = *msg;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
	{
		init_position_x_take_off = local_pos.pose.pose.position.x;
		init_position_y_take_off = local_pos.pose.pose.position.y;
		init_position_z_take_off = local_pos.pose.pose.position.z;
		init_yaw_take_off = yaw;
		flag_init_position = true;
	}
}


/************************************************************************
函数 3: 起飞系位置控制
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
	if (mission_pos_cruise_flag == false)
	{
		mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
		mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
		mission_pos_cruise_flag = true;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + init_position_x_take_off;
	setpoint_raw.position.y = y + init_position_y_take_off;
	setpoint_raw.position.z = z + init_position_z_take_off;
	setpoint_raw.yaw = target_yaw;
	ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f), init_pose(%.2f, %.2f, %.2f, %.2f)", 
        local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI, 
        x, y, z, target_yaw * 180.0 / M_PI, 
        init_position_x_take_off, init_position_y_take_off, init_position_z_take_off, init_yaw_take_off * 180.0 / M_PI);
	if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		mission_pos_cruise_flag = false;
		return true;
	}
	return false;
}
/************************************************************************
函数 4：机体系位置控制
！！！使用时需立刻退出，不要在if中放置时间，否则会一直累加当前位置！！！
*************************************************************************/
float current_position_cruise_last_position_x = 0;
float current_position_cruise_last_position_y = 0;
bool current_position_cruise_flag = false;
bool current_position_cruise(float x, float y, float z, float target_yaw, float error_max);
bool current_position_cruise(float x, float y, float z, float target_yaw, float error_max)
{
	if (current_position_cruise_flag == false)
	{
		current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
		current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
		current_position_cruise_flag = true;
	}
	// float p_xy = 0.8;
	setpoint_raw.type_mask = /*1 + 2 + 4 */+ 8 + 16  + 32 + 64 + 128 + 256 + 512 /*+ 1024 */+ 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + current_position_cruise_last_position_x;
	setpoint_raw.position.y = y + current_position_cruise_last_position_y;
	setpoint_raw.position.z = z;
	setpoint_raw.yaw = target_yaw;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI, x + current_position_cruise_last_position_x, y + current_position_cruise_last_position_y, z, target_yaw * 180.0 / M_PI);
	if (fabs(local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x) < error_max && fabs(local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y) < error_max && fabs(local_pos.pose.pose.position.z - z) < error_max && fabs(yaw - target_yaw) < 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		// current_position_cruise_flag = false;
		setpoint_raw.yaw_rate = 0.0; // 重置yaw_rate
		setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 + /*1024 + */2048;
		setpoint_raw.yaw = yaw; // 确保最终yaw角度正确
		return true;
	}
	return false;
}

/************************************************************************
函数 5:降落
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land();
bool precision_land()
{
	if (!precision_land_init_position_flag)
	{
		precision_land_init_position_x = local_pos.pose.pose.position.x;
		precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
		precision_land_init_position_flag = true;
	}
	setpoint_raw.position.x = precision_land_init_position_x;
	setpoint_raw.position.y = precision_land_init_position_y;
	setpoint_raw.position.z = -0.15;
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
    if(ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        return true;
    }
    return false;
}

/************************************************************************
函数 6:lib计时器
！！！若使用超限，需要将lib_time_init_flag置为true！！！
*************************************************************************/
ros::Time last_time;
bool lib_time_init_flag = true;  // 记录是否为第一次触发
bool lib_time_record_func(float duration, ros::Time current_time) {
      // 静态变量，记录上次触发时间
	if(lib_time_init_flag)
	{
		last_time = current_time;
		lib_time_init_flag = false; // 只在第一次调用时初始化
	}

    ros::Duration elapsed = current_time - last_time;

    // 如果已经过去超过 duration 秒，返回 true 并更新时间
    if (elapsed.toSec() >= duration) {
		lib_time_init_flag = true;
		return true;
    }
    return false;
}

/************************************************************************
函数 7: hover保持悬停time秒
*************************************************************************/
bool hover_started = false;
ros::Time hover_last_time;
float hover_x = 0;
float hover_y = 0;
float hover_z = 0;
float hover_yaw = 0;
bool hover(float time_duration);
bool hover(float time_duration)
{
    if(!hover_started)
    {
        hover_last_time = ros::Time::now();
        hover_started = true;
        hover_x = local_pos.pose.pose.position.x;
        hover_y = local_pos.pose.pose.position.y;
        hover_z = local_pos.pose.pose.position.z;
        hover_yaw = yaw;
    }
    ros::Duration elapsed = ros::Time::now() - hover_last_time;
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = hover_x;
    setpoint_raw.position.y = hover_y;
    setpoint_raw.position.z = hover_z;
    setpoint_raw.yaw = hover_yaw;
    ROS_INFO("Hovering at (%.2f, %.2f, %.2f) for %.2f seconds", hover_x, hover_y, hover_z, elapsed.toSec());
     // 如果已经过去超过 time_duration 秒，返回 true 并重置状态 
    if (elapsed.toSec() >= time_duration) {
        hover_started = false; // Reset for next hover
        return true;
    }
    return false;
}

/************************************************************************
函数 8: 绕圈，以传入点为圆心，当前点到传入点距离为半径，Ts为周期绕圆
*************************************************************************/
bool init_circle_flag = false;
float init_plane_x = 0;
float init_plane_y = 0;
float circle_radius = 0;
float beta_angle = 0;
ros::Time circle_start_time;
bool circle(float center_x, float center_y, float Ts);
bool circle(float center_x, float center_y, float Ts)
{
    if (!init_circle_flag)
    {
        // 记录初始位置（用于计算初始角度）
        init_plane_x = local_pos.pose.pose.position.x;
        init_plane_y = local_pos.pose.pose.position.y;
        circle_radius = sqrt(pow(init_plane_x - center_x, 2) + pow(init_plane_y - center_y, 2));
        
        // 初始角度：从圆心指向无人机的向量的角度（标准 atan2(y, x)）
        // 注意：atan2(dy, dx)，其中 dy = y - center_y, dx = x - center_x
        beta_angle = atan2(init_plane_y - center_y, init_plane_x - center_x);
        
        init_circle_flag = true;
        circle_start_time = ros::Time::now();
    }

    ros::Time current_time = ros::Time::now();
    float elapsed_time = (current_time - circle_start_time).toSec();
    float theta = (2 * M_PI / Ts) * elapsed_time;  // 角速度 * 时间

    // 当前目标角度 = 初始角度 + theta（逆时针旋转）
    float current_angle = beta_angle + theta;

    // 目标位置：圆心 + 半径 * (cos, sin)
    setpoint_raw.position.x = center_x + circle_radius * cos(current_angle);
    setpoint_raw.position.y = center_y + circle_radius * sin(current_angle);
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;  // 或者可以设置为朝向飞行方向：current_angle

    // 设置 type_mask 和 coordinate_frame（保持你的设置）
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048; // 位置控制
    setpoint_raw.coordinate_frame = 1; // FRAME_LOCAL_NED

    ROS_INFO("Circle flying to (%.2f, %.2f, %.2f)", 
             setpoint_raw.position.x, setpoint_raw.position.y, setpoint_raw.position.z);
    ROS_WARN("Angle: %.2f deg, Time: %.2f s", current_angle * 180.0 / M_PI, elapsed_time);

    float real_radius = sqrt(pow(local_pos.pose.pose.position.x - center_x, 2) + 
                             pow(local_pos.pose.pose.position.y - center_y, 2));
    ROS_WARN("Expected radius: %.2f, Real radius: %.2f", circle_radius, real_radius);

    if (theta >= 2 * M_PI)
    {
        init_circle_flag = false;
        ROS_INFO("Circle flight complete.");
        return true;
    }
    else
    {
        return false;
    }
}
/************************************************************************
函数 9: 绕圈（提高版，基于角度增量，绕完一圈返回 true）
传入圆心 (center_x, center_y) 和每次角度增量 d_angle（弧度，可正可负）
每次调用生成下一个目标点，并累计角度
当累计旋转角度 ≥ 2π 时，返回 true（表示一圈完成）
*************************************************************************/

bool init_circle_advanced_flag = false;
float circle_adv_radius = 0;
float circle_adv_center_x = 0;
float circle_adv_center_y = 0;
float planned_angle = 0;

// 计划切向速度（vx/vy）跟踪：由 d_angle + 调用周期 dt 估算角速度
ros::Time circle_adv_prev_time;
bool circle_adv_prev_time_inited = false;
float circle_adv_last_vx = 0;
float circle_adv_last_vy = 0;

// 实际飞行状态跟踪
float actual_start_angle = 0;
float actual_unwrapped_angle = 0;
float prev_actual_angle = 0;
float actual_total_delta = 0;
float param_vel = 1.0;
// 径向速度闭环（用于纠正半径偏差）
float param_rad_k = 1.2;
float param_vrad_max = 0.55;
float param_vxy_max = 1.5;
float diff_angle_param = 0.7; // 超前角增益

// 统计：绕圈平均时间
ros::Time circle_adv_start_time;
bool circle_adv_start_time_inited = false;

bool circle_advanced(float center_x, float center_y, float d_angle, int circle_times, double circle_input_radius)
{
    float plane_x = local_pos.pose.pose.position.x;
    float plane_y = local_pos.pose.pose.position.y;
    float curr_actual_angle = atan2(plane_y - center_y, plane_x - center_x);

    if (!init_circle_advanced_flag)
    {
        // 初始化轨迹
        // circle_adv_radius = sqrt(pow(plane_x - center_x, 2) + pow(plane_y - center_y, 2));
        circle_adv_radius = circle_input_radius;
        circle_adv_center_x = center_x;
        circle_adv_center_y = center_y;

        // 初始化实际角度跟踪
        actual_start_angle = curr_actual_angle;
        actual_unwrapped_angle = curr_actual_angle;
        prev_actual_angle = curr_actual_angle;
        actual_total_delta = 0;

        // 初始化计划角度（用于生成 setpoint）
        planned_angle = curr_actual_angle;

        // 初始化 dt 估计
        circle_adv_prev_time = ros::Time::now();
        circle_adv_prev_time_inited = true;
        circle_adv_last_vx = 0;
        circle_adv_last_vy = 0;

        // 初始化绕圈计时
        circle_adv_start_time = circle_adv_prev_time;
        circle_adv_start_time_inited = true;

        init_circle_advanced_flag = true;

        ROS_INFO("Advanced circle initialized. Radius: %.2f, Start angle: %.2f deg",
                 circle_adv_radius, actual_start_angle * 180.0 / M_PI);
    }

    // === 1. 更新计划轨迹（用于发 setpoint）===
    // 若飞机跟不上 planned_angle，会导致 planned 与 actual 相位误差不断累积。
    // 这里改为每次基于“当前实际角度 + 超前角 d_angle”生成下一个目标点，避免 planned 漂移。
    planned_angle = curr_actual_angle + d_angle;

    setpoint_raw.position.x = circle_adv_center_x + circle_adv_radius * cos(planned_angle + diff_angle_param);
    setpoint_raw.position.y = circle_adv_center_y + circle_adv_radius * sin(planned_angle + diff_angle_param);
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    // === 1.1 计算并发送切向速度（vx/vy）===
    // 由每次角度增量 d_angle 与本次调用间隔 dt 估算角速度 omega
    // 切向速度：v = omega * r；方向为切向 [-sin(theta), cos(theta)]（theta=planned_angle）
    // 实际半径（用于径向纠偏）
    const float dx = plane_x - circle_adv_center_x;
    const float dy = plane_y - circle_adv_center_y;
    const float real_radius = sqrt(dx * dx + dy * dy);

    // 切向速度（只存切向项，便于 dt 异常时复用）
    float vx_tan = circle_adv_last_vx;
    float vy_tan = circle_adv_last_vy;
    if (circle_adv_prev_time_inited)
    {
        const ros::Time now = ros::Time::now();
        const float dt = (now - circle_adv_prev_time).toSec();
        circle_adv_prev_time = now;

        if (dt > 1e-3f)
        {
            const float omega = d_angle / dt;               // rad/s（带符号）
            const float v_tan = omega * circle_adv_radius;  // m/s（带符号）
            // vx = -v_tan * sin(planned_angle);
            // vy =  v_tan * cos(planned_angle);
            vx_tan = -v_tan * sin(curr_actual_angle) * param_vel;
            vy_tan =  v_tan * cos(curr_actual_angle) * param_vel;
            circle_adv_last_vx = vx_tan;
            circle_adv_last_vy = vy_tan;
        }
    }

    // 径向速度纠偏：v_rad = k_r * (r_cmd - r_real)，方向为径向 [cos(theta), sin(theta)]
    const float r_err = circle_adv_radius - real_radius;
    float v_rad = param_rad_k * r_err;
    if (v_rad > param_vrad_max) v_rad = param_vrad_max;
    if (v_rad < -param_vrad_max) v_rad = -param_vrad_max;

    float vx = vx_tan + v_rad * cos(curr_actual_angle);
    float vy = vy_tan + v_rad * sin(curr_actual_angle);

    // 总水平速度限幅，避免过激导致半径漂移更大
    const float v_xy = sqrt(vx * vx + vy * vy);
    if (v_xy > param_vxy_max && v_xy > 1e-3f)
    {
        const float scale = param_vxy_max / v_xy;
        ROS_WARN("Velocity limited: v_xy=%.2f > %.2f, scale=%.2f", v_xy, param_vxy_max, scale);
        vx *= scale;
        vy *= scale;
    }

    setpoint_raw.velocity.x = vx;
    setpoint_raw.velocity.y = vy;
    setpoint_raw.velocity.z = 0;

    // 同时发送 position + vx/vy：不要屏蔽 vx/vy；这里屏蔽 vz、加速度、yaw_rate
    setpoint_raw.type_mask = 32 + 64 + 128 + 256 + 512 + 2048;
    setpoint_raw.coordinate_frame = 1;

    // === 2. 更新实际累计转角（关键！）===
    // 解卷绕：处理角度从 π → -π 或 -π → π 的跳变
    float delta = curr_actual_angle - prev_actual_angle;
    if (delta > M_PI) delta -= 2 * M_PI;
    if (delta < -M_PI) delta += 2 * M_PI;
    actual_unwrapped_angle += delta;
    prev_actual_angle = curr_actual_angle;

    actual_total_delta = fabs(actual_unwrapped_angle - actual_start_angle);

    // === 2.1 统计：平均每 360° 所需时间 ===
    double avg_sec_per_circle = -1.0;
    if (circle_adv_start_time_inited)
    {
        const double elapsed_sec = (ros::Time::now() - circle_adv_start_time).toSec();
        const double circles_done = static_cast<double>(actual_total_delta) / (2.0 * M_PI);
        if (elapsed_sec > 1e-3 && circles_done > 1e-3)
        {
            avg_sec_per_circle = elapsed_sec / circles_done;
        }
    }

    // === 3. 日志 ===
    ROS_INFO("Target: (%.2f, %.2f), Vxy(vx,vy)=(%.2f,%.2f), Planned Angle: %.2f°, Actual Angle: %.2f°, Actual Δ: %.2f°, Circles: %.2f, AvgT/360°: %.2fs",
             setpoint_raw.position.x, setpoint_raw.position.y,
             setpoint_raw.velocity.x, setpoint_raw.velocity.y,
             planned_angle * 180.0 / M_PI,
             curr_actual_angle * 180.0 / M_PI,
             actual_total_delta * 180.0 / M_PI,
             actual_total_delta / (2 * M_PI),
             avg_sec_per_circle);
    ROS_WARN("Expected radius: %.2f, Real radius: %.2f, r_err: %.2f, v_rad: %.2f", circle_adv_radius, real_radius, r_err, v_rad);

    // === 4. 判断：基于实际飞行角度是否满一圈 ===
    // if (actual_total_delta >= 2 * M_PI - M_PI/180*130)  // 提前退出
    if (actual_total_delta >= circle_times * 2 * M_PI - M_PI/180*10)  // 提前退出
    {
        init_circle_advanced_flag = false;
        circle_adv_prev_time_inited = false;
        circle_adv_start_time_inited = false;
        ROS_INFO("Actual full circle completed! Total actual rotation: %.2f°",
                 actual_total_delta * 180.0 / M_PI);
        return true;
    }
    else
    {
        return false;
    }
}


/************************************************************************
函数 10: rviz当前位置轨迹可视化
*************************************************************************/
// 全局变量
ros::Publisher marker_pub;
std::deque<geometry_msgs::Point> path_points;
const int MAX_POINTS = 1000000;  // 最大轨迹点数
const float TRAJECTORY_WIDTH = 0.05; // 轨迹线宽
// 位置回调函数
inline void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 添加新点到轨迹
    geometry_msgs::Point new_point;
    new_point.x = msg->pose.position.x;
    new_point.y = msg->pose.position.y;
    new_point.z = msg->pose.position.z;
    
    path_points.push_back(new_point);
    
    // 限制轨迹长度
    if (path_points.size() > MAX_POINTS) {
        path_points.pop_front();
    }
    
    // 创建轨迹Marker
    visualization_msgs::Marker trajectory;
    //trajectory.header.frame_id = msg->header.frame_id;
    trajectory.header.frame_id = "world";
    trajectory.header.stamp = ros::Time::now();
    trajectory.ns = "drone_trajectory";
    trajectory.id = 0;
    trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory.action = visualization_msgs::Marker::ADD;
    
    // 轨迹样式设置
    trajectory.scale.x = TRAJECTORY_WIDTH;  // 线宽
    trajectory.color.r = 0.0;   // 红色分量
    trajectory.color.g = 1.0;   // 绿色分量
    trajectory.color.b = 0.0;   // 蓝色分量
    trajectory.color.a = 0.8;   // 透明度
    
    // 轨迹点数据
    trajectory.points = std::vector<geometry_msgs::Point>(
        path_points.begin(), path_points.end());
    
    // 发布轨迹
    marker_pub.publish(trajectory);
    
    // 创建当前位置标记
    visualization_msgs::Marker position_marker;
    position_marker.header = trajectory.header;
    position_marker.ns = "drone_position";
    position_marker.id = 1;
    position_marker.type = visualization_msgs::Marker::SPHERE;
    position_marker.action = visualization_msgs::Marker::ADD;
    position_marker.pose.position = new_point;
    position_marker.pose.orientation.w = 1.0;
    position_marker.scale.x = 0.3;  // 球体直径
    position_marker.scale.y = 0.3;
    position_marker.scale.z = 0.3;
    position_marker.color.r = 1.0;
    position_marker.color.g = 0.0;
    position_marker.color.b = 0.0;
    position_marker.color.a = 1.0;
    
    // 发布位置标记
    marker_pub.publish(position_marker);
}

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

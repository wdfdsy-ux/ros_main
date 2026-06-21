#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "arm_controller/move.h"
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include <set>
#include <cmath>

ros::ServiceClient armmove_client;
ros::ServiceClient pick_client;
ros::ServiceClient put_client;
ros::Subscriber ar_sub;
bool tag_100_detected = false;
bool cargo_scan_done = false;
std::set<int> cargo_ids_detected;

bool arm_move(float x, float y, float z)
{
    ROS_INFO("等待服务 /goto_position 启动...");
    armmove_client.waitForExistence();
    ROS_INFO("服务已连接！");
    arm_controller::move srv;
    srv.request.pose.position.x = x;
    srv.request.pose.position.y = y;
    srv.request.pose.position.z = z;

    if (armmove_client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("机械臂移动成功：目标(%.2f, %.2f, %.2f)", x, y, z);
            return true;
        } else {
            ROS_ERROR("机械臂移动失败：%s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("机械臂服务调用失败！");
        return false;
    }
}

void safe_retract()
{
    ROS_WARN("检测失败，正在收回机械臂至安全点...");
    arm_move(150, 0, 120);
}

void set_pump(bool state)
{
    ROS_INFO("等待抓取服务启动...");
    pick_client.waitForExistence();
    put_client.waitForExistence();
    ROS_INFO("服务已连接！");
    std_srvs::Empty srv;
    if (state) {
        pick_client.call(srv);
        ROS_INFO("吸盘已开启");
    } else {
        put_client.call(srv);
        ROS_INFO("吸盘已关闭");
    }
}

void arMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers)
{
    for (const auto& marker : markers->markers) {
        // 检测定位标签 100
        if (marker.id == 100 && !tag_100_detected) {
            tag_100_detected = true;
            ROS_INFO("定位成功！已识别到AR标签 100");
        }
        // 检测货物标签 1~4
        if (marker.id >= 1 && marker.id <= 4 && tag_100_detected && !cargo_scan_done) {
            if (cargo_ids_detected.find(marker.id) == cargo_ids_detected.end()) {
                cargo_ids_detected.insert(marker.id);
                ROS_INFO("检测到货物AR标签 ID=%d", marker.id);
            }
        }
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "grab_node");
    ros::NodeHandle nh;

    armmove_client = nh.serviceClient<arm_controller::move>("/goto_position");
    pick_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/on");
    put_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/off");

    // ===== Step 1: 机械臂移到安全点 =====
    ROS_INFO("===== Step 1: 移至安全点 (150, 0, 120) =====");
    if (!arm_move(150, 0, 120)) {
        return -1;
    }

    // ===== Step 2: 机械臂移到定位点 =====
    ROS_INFO("===== Step 2: 移至定位点 (165, -210, 70) =====");
    if (!arm_move(165, -210, 70)) {
        return -1;
    }

    // ===== Step 3: 被动识别定位标签 100 (超时10秒) =====
    ROS_INFO("===== Step 3: 等待AR标签 100 识别... (超时10秒) =====");
    ar_sub = nh.subscribe("hand_camera/ar_pose_marker", 10, arMarkerCallback);

    ros::Time tag_wait_start = ros::Time::now();
    ros::Rate rate(10);
    while (ros::ok() && !tag_100_detected &&
           (ros::Time::now() - tag_wait_start).toSec() < 10.0) {
        ros::spinOnce();
        rate.sleep();
    }
    if (!tag_100_detected) {
        ROS_ERROR("定位失败：未识别到AR标签 100");
        safe_retract();
        return -1;
    }

    // ===== Step 4: 机械臂移到货物识别点 =====
    ROS_INFO("===== Step 4: 移至货物识别点 (180, -180, 90) =====");
    if (!arm_move(180, -180, 90)) {
        return -1;
    }

    // ===== Step 5: 扫描货物标签 (ID: 1, 2, 3, 4) =====
    ROS_INFO("===== Step 5: 扫描货物AR标签 (ID:1~4) =====");
    ros::Time scan_start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - scan_start).toSec() < 3.0) {
        ros::spinOnce();
        rate.sleep();
    }
    cargo_scan_done = true;

    if (cargo_ids_detected.empty()) {
        ROS_ERROR("识别失败：未检测到任何货物标签 (ID:1~4)");
        safe_retract();
        return -1;
    }

    int min_id = *cargo_ids_detected.begin();
    std::string ids_str;
    for (int id : cargo_ids_detected) {
        ids_str += std::to_string(id) + " ";
    }
    ROS_INFO("共检测到 %zu 个货物标签 [%s]，最小ID=%d",
             cargo_ids_detected.size(), ids_str.c_str(), min_id);

    // ===== Step 6: 移动到最小ID货物标签正上方 40mm =====
    ROS_INFO("===== Step 6: 移至货物标签 %d 上方40mm =====", min_id);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    std::string target_frame = "ar_marker_" + std::to_string(min_id);

    try {
        listener.waitForTransform("Base", target_frame, ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform("Base", target_frame, ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("获取货物标签TF坐标失败: %s", ex.what());
        safe_retract();
        return -1;
    }

    float cargo_x = transform.getOrigin().x() * 1000;
    float cargo_y = transform.getOrigin().y() * 1000;
    float cargo_z_surface = transform.getOrigin().z() * 1000;  // 货物表面
    float tag_z = cargo_z_surface + 40;  // 上方40mm（安全接近高度）

    // 全局安全高度：保证高于工作区内所有物块，所有水平移动在此高度进行
    float SAFE_HEIGHT;
    nh.param("safe_lift_height", SAFE_HEIGHT, 80.0f);

    ROS_INFO("货物标签位置: (%.2f, %.2f, %.2f)mm", cargo_x, cargo_y, cargo_z_surface);
    ROS_INFO("目标位置(上方40mm): (%.2f, %.2f, %.2f)mm", cargo_x, cargo_y, tag_z);

    // ── Step 6+: 先原地升到安全高度 → 再平移到偏置接近点 → 下降到接近高度 ──
    // 计算侧向偏置方向：垂直于 [扫描点→目标] 连线，从空旷侧接近，避免直上直下撞到邻居
    float approach_offset = 20.0f;  // 侧向偏置距离 20mm（可调参数）
    float scan_dx = cargo_x - 180.0f;
    float scan_dy = cargo_y - (-180.0f);
    float scan_len = std::sqrt(scan_dx * scan_dx + scan_dy * scan_dy);
    // 偏置方向 = 扫描方向逆时针旋转 90°（侧向偏移）
    float offset_x = cargo_x + (-scan_dy / scan_len) * approach_offset;
    float offset_y = cargo_y + ( scan_dx / scan_len) * approach_offset;

    ROS_INFO("===== Step 6: 升到安全高度 %.0fmm → 平移到偏置点 → 下降到接近高度 =====", SAFE_HEIGHT);
    // ① 原地升到全局安全高度（无论当前在哪个位置，先确保高度安全）
    arm_move(180, -180, SAFE_HEIGHT);
    // ② 在安全高度平移到偏置接近点正上方（此时高于所有物块，安全水平移动）
    if (!arm_move(offset_x, offset_y, SAFE_HEIGHT)) {
        safe_retract();
        return -1;
    }
    // ③ 从偏置点垂直下降到 tag_z（附近没有物块，安全）
    if (!arm_move(offset_x, offset_y, tag_z)) {
        safe_retract();
        return -1;
    }

    // ===== Step 7: 侧向滑入抓取（避免直下撞到旁边物块） =====
    ROS_INFO("===== Step 7: 侧向滑入 (%.1f, %.1f, %.0f) → 目标正上方 → 下降抓取 =====",
             offset_x, offset_y, tag_z);
    ROS_INFO("       偏置方向: 垂直于 [扫描点→目标] 连线，距离=%.0fmm", approach_offset);

    // 阶段 A: 在 tag_z 高度（高于表面40mm）水平滑入目标正上方
    //         这段水平运动只移动很小的偏移量（~20mm），不会撞到旁边物块
    arm_move(cargo_x, cargo_y, tag_z);
    // 阶段 B: 垂直下降到货物表面
    arm_move(cargo_x, cargo_y, cargo_z_surface);
    ros::Duration(1.0).sleep();
    // 开启吸盘
    set_pump(true);

    // ── 拾取后的安全退出路径（与接近路径对称） ──
    // 阶段 C: 垂直提升到 tag_z（离开表面）
    arm_move(cargo_x, cargo_y, tag_z);
    // 阶段 D: 水平侧移回到偏置点（退出紧挨的物块区域）
    arm_move(offset_x, offset_y, tag_z);
    // 阶段 E: 上升到全局安全高度
    arm_move(offset_x, offset_y, SAFE_HEIGHT);

    // ===== Step 8: 从安全高度平移到出刀点正上方，下降放置 =====
    bool point1_occupied = false;
    nh.param("placement_point_1_occupied", point1_occupied, false);

    // 此时已经在 (offset_x, offset_y, SAFE_HEIGHT)，所有水平运动都在安全高度进行
    float lift_z = SAFE_HEIGHT;

    if (!point1_occupied) {
        // 出刀点1 未被占用 → 在升高的高度平移到出刀点1正上方
        float p1_x = 107, p1_y = 115, p1_z = 42;

        ROS_INFO("===== 平移到出刀点1正上方 (%.0f, %.0f, %.2f) =====", p1_x, p1_y, lift_z);
        arm_move(p1_x, p1_y, lift_z);

        // 横纵坐标已对齐出刀点1，下降放置
        ROS_INFO("横纵坐标已对齐出刀点1，下降至放置高度");
        arm_move(p1_x, p1_y, p1_z);
        ros::Duration(1.0).sleep();
        set_pump(false);           // 关闭吸盘，松开货物
        arm_move(p1_x, p1_y, p1_z + 30);   // 出刀（向上30mm防撞）
        // 标记出刀点1 为已占用
        nh.setParam("placement_point_1_occupied", true);
        ROS_WARN("出刀点1 已标记为占用");
    } else {
        // 出刀点1 已被占用 → 在升高的高度平移到出刀点2正上方
        float p2_x = 107, p2_y = 185, p2_z = 42;

        ROS_INFO("===== 出刀点1已被占用，平移到出刀点2正上方 (%.0f, %.0f, %.2f) =====", p2_x, p2_y, lift_z);
        arm_move(p2_x, p2_y, lift_z);

        // 横纵坐标已对齐出刀点2，下降放置
        ROS_INFO("横纵坐标已对齐出刀点2，下降至放置高度");
        arm_move(p2_x, p2_y, p2_z);
        ros::Duration(1.0).sleep();
        set_pump(false);           // 关闭吸盘，松开货物
        arm_move(p2_x, p2_y, p2_z + 30);   // 出刀（向上30mm防撞）
    }

    // ===== Step 9: 回到安全点 =====
    ROS_INFO("===== Step 9: 回到安全点 (150, 0, 120) =====");
    if (!arm_move(150, 0, 120)) {
        return -1;
    }

    ROS_INFO("===== 全部完成！=====");
    return 0;
}

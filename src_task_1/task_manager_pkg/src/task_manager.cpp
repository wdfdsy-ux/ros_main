#include <ros/ros.h>
#include <cstdlib>

static const int MAX_RETRIES = 10;

bool run_with_retry(const char* cmd, int step)
{
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++)
    {
        ROS_INFO("  └─ 尝试第 %d/%d 次...", attempt, MAX_RETRIES);
        int ret = system(cmd);

        if (ret == 0)
        {
            ROS_INFO("  └─ 启动成功");
            return true;
        }

        ROS_WARN("  └─ 第 %d 次失败 (返回码 %d)", attempt, ret);

        if (attempt < MAX_RETRIES)
        {
            ROS_INFO("  └─ 等待 2 秒后重试...");
            ros::Duration(2.0).sleep();
        }
    }

    ROS_ERROR("Step %d 连续 %d 次启动失败，终止任务！", step, MAX_RETRIES);
    return false;
}

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "task_manager");
    ros::NodeHandle nh;

    int step = 1;
    const int total_steps = 9;

    // ====================================================================
    // Step 1: 导航到目标点4
    // ====================================================================
    ROS_INFO("===== Step %d/%d: 导航到目标点4 =====", step, total_steps);
    if (!run_with_retry("rosrun auto_nav_goal_4_pkg auto_nav_4_node", step))
        return -1;
    ROS_INFO("----- Step %d 完成 -----", step++);

    // ====================================================================
    // Step 2: 机械臂抓取 + 放置货物（第1次）
    // ====================================================================
    ROS_INFO("===== Step %d/%d: 机械臂抓取+放置货物 (1/2) =====", step, total_steps);
    if (!run_with_retry("rosrun Grab_pkg grab_node", step))
        return -1;
    ROS_INFO("----- Step %d 完成 (出刀点1已占用) -----", step++);

    // ====================================================================
    // Step 3: 后退 0.18m（第1次）
    // ====================================================================
    ROS_INFO("===== Step %d/%d: 后退0.18m (1/2) =====", step, total_steps);
    if (!run_with_retry("rosrun remove_back_pkg re_back_node", step))
        return -1;
    ROS_INFO("----- Step %d 完成 -----", step++);

    // ====================================================================
    // Step 4: 导航到目标点5
    // ====================================================================
    ROS_INFO("===== Step %d/%d: 导航到目标点5 =====", step, total_steps);
    if (!run_with_retry("rosrun auto_nav_goal_5_pkg auto_nav_5_node", step))
        return -1;
    ROS_INFO("----- Step %d 完成 -----", step++);

    // ====================================================================
    // Step 5: 机械臂抓取 + 放置货物（第2次）
    //                         出刀点1已被占用，自动使用出刀点2
    // ====================================================================
    ROS_INFO("===== Step %d/%d: 机械臂抓取+放置货物 (2/2) =====", step, total_steps);
    if (!run_with_retry("rosrun Grab_pkg grab_node", step))
        return -1;
    ROS_INFO("----- Step %d 完成 (使用出刀点2) -----", step++);

    // ====================================================================
    // Step 6: 后退 0.18m（第2次）
    // ====================================================================
    ROS_INFO("===== Step %d/%d: 后退0.18m (2/2) =====", step, total_steps);
    if (!run_with_retry("rosrun remove_back_pkg re_back_node", step))
        return -1;
    ROS_INFO("----- Step %d 完成 -----", step++);

    // ====================================================================
    // Step 7: 导航回起点（第1次）
    // ====================================================================
    ROS_INFO("===== Step %d/%d: 导航回起点 (1/2) =====", step, total_steps);
    if (!run_with_retry("rosrun auto_bkhome_pkg auto_bkhome_node", step))
        return -1;
    ROS_INFO("----- Step %d 完成 -----", step++);

    // ====================================================================
    // Step 8: 导航到充电桩充电
    // ====================================================================
    ROS_INFO("===== Step %d/%d: 导航到充电桩充电 =====", step, total_steps);
    if (!run_with_retry("rosrun auto_charge_pkg auto_charge_node", step))
        return -1;
    ROS_INFO("----- Step %d 完成 -----", step++);

    // ====================================================================
    // Step 9: 导航回起点（第2次）
    // ====================================================================
    ROS_INFO("===== Step %d/%d: 导航回起点 (2/2) =====", step, total_steps);
    if (!run_with_retry("rosrun auto_bkhome_pkg auto_bkhome_node", step))
        return -1;
    ROS_INFO("----- Step %d 完成 -----", step++);

    // ====================================================================
    // 全部完成 — 复位出刀点1参数，下次任务可重新使用
    // ====================================================================
    ROS_INFO("========================================");
    ROS_INFO("  全部 %d 个步骤执行完毕！", total_steps);
    ROS_INFO("========================================");
    ros::param::set("placement_point_1_occupied", false);
    ROS_INFO("出刀点1 占用状态已自动复位");
    return 0;
}

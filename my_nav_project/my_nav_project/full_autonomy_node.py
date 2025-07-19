import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations

"""
该脚本将引导机器人完成一个多点巡逻任务。
"""

def main(args=None):
    # 1. 初始化 ROS 2 客户端库
    rclpy.init(args=args)

    # 2. 实例化 BasicNavigator
    # 这是与 Nav2 交互的主要入口点
    navigator = BasicNavigator()

    # 3. 设置机器人的初始位姿
    # 您必须根据机器人在仿真或现实世界中的实际起始位置来设置此值
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # 4. 等待 Nav2 完全激活
    # 如果您在启动 Nav2 时设置了 autostart=false，则应改用 navigator.lifecycleStartup()
    navigator.waitUntilNav2Active()
    print('Nav2 系统已激活，准备开始导航任务。')

    # 5. 定义巡逻的路径点
    # 每个路径点都是一个 PoseStamped 对象
    # 坐标和朝向需要根据您的地图进行调整
    
    # 将欧拉角（yaw）转换为四元数
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57)
    
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 2.0
    goal_pose1.pose.position.y = 0.5
    goal_pose1.pose.orientation.w = 1.0

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.5
    goal_pose2.pose.position.y = -1.5
    goal_pose2.pose.orientation.z = q_z
    goal_pose2.pose.orientation.w = q_w

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 0.5
    goal_pose3.pose.position.y = -2.0
    goal_pose3.pose.orientation.w = 1.0

    waypoints = [goal_pose1, goal_pose2, goal_pose3]

    # 6. 开始执行路径点跟随任务
    print('开始执行多点巡逻任务...')
    navigator.followWaypoints(waypoints)

    # 7. 监控任务执行状态
    i = 0
    while not navigator.isTaskComplete():
        ################################################
        # 您可以在这里执行其他任务，例如获取传感器数据等。#
        ################################################
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'正在前往第 {feedback.current_waypoint + 1}/{len(waypoints)} 个路径点...')

    # 8. 获取最终任务结果
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('任务成功！所有路径点均已到达。')
    elif result == TaskResult.CANCELED:
        print('任务被取消！')
    elif result == TaskResult.FAILED:
        print('任务失败！')
    else:
        print('任务状态未知！')

    # 9. 安全关闭 Nav2 系统
    navigator.lifecycleShutdown()

    # 10. 退出程序
    exit(0)


if __name__ == '__main__':
    main()
# 导入 LaunchDescription 和 Node
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义 generate_launch_description 函数
def generate_launch_description():
    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 声明 talker 节点
    talker_node = Node(
        package='learning_topic', # 或者你的自定义包名
        executable='topic_publisher',     # 或者你的自定义节点可执行文件名
        name='my_talker',        # 自定义节点名称
        parameters=[{'message_content': 'Hello from Launch!'}] # 示例：传递参数
    )

    # 声明 listener 节点
    listener_node = Node(
        package='learning_topic', # 或者你的自定义包名
        executable='topic_subscriber',   # 或者你的自定义节点可执行文件名
        name='my_listener',      # 自定义节点名称
        # 示例：重映射话题
        # remappings=[
        #     ('/chatter', '/my_chatter_topic')
        # ]
    )

    # 将节点添加到 LaunchDescription
    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld

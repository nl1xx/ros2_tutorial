import sys
import rclpy                                  
from rclpy.node import Node                 
from learning_interfaces.srv import Add2int # 自定义的服务接口

class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)                                    
        # 创建服务客户端对象（服务接口类型，服务名）   
        self.client = self.create_client(Add2int, 'add_two_ints') 
        # 循环等待服务器端成功启动
        while not self.client.wait_for_service(timeout_sec=1.0):     
            self.get_logger().info('service not available, waiting again...') 
        # 创建服务请求的数据对象
        self.request = Add2int.Request()
        
    # 创建一个发送服务请求的函数                           
    def send_request(self):
        self.request.a = int(sys.argv[1])
        self.request.b = int(sys.argv[2])
        # 异步方式发送服务请求
        self.future = self.client.call_async(self.request)           

def main(args=None):
    rclpy.init(args=args)                        
    node = adderClient("service_adder_client")   
    node.send_request()                          

    # ROS2系统正常运行
    while rclpy.ok(): 
        # 循环执行一次节点                           
        rclpy.spin_once(node)                   

        # 数据是否处理完成
        if node.future.done():                   
            try:
                # 接收服务器端的反馈数据
                response = node.future.result()  
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # 将收到的反馈信息打印输出
                node.get_logger().info('Result of add_two_ints: for %d + %d = %d' % (node.request.a, node.request.b, response.result))
            break

    node.destroy_node()                          
    rclpy.shutdown()

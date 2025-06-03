import rclpy                                     
from rclpy.node import Node              
# 自定义的服务接口      
from learning_interfaces.srv import Add2int    

class adderServer(Node):
    def __init__(self, name):
        super().__init__(name)      
        # 创建服务器对象（接口类型、服务名、服务器回调函数）                                                     
        self.srv = self.create_service(Add2int, 'add_two_ints', self.adder_callback)  

    def adder_callback(self, request, response):   
         # 完成加法求和计算，将结果放到反馈的数据中
        response.result = request.a + request.b      
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))   
        return response                          

def main(args=None):                            
    rclpy.init(args=args)                        
    node = adderServer("service_adder_server")
    # 循环等待ROS2退出  
    rclpy.spin(node)                             
    node.destroy_node()                         
    rclpy.shutdown()

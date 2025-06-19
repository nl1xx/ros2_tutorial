import copy
import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')


show_animation = True


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


class RRT:
    def __init__(self, obstacleList, randArea,
                 expandDis=2.0, goalSampleRate=10, maxIter=200):

        self.start = None
        self.goal = None
        # 最小采样值
        self.min_rand = randArea[0]
        # 最大采样值
        self.max_rand = randArea[1]
        # 采样步长
        self.expand_dis = expandDis
        # 目标采样率, 就是说朝着目标方向生长的概率, 这里设置的是10%
        self.goal_sample_rate = goalSampleRate
        # 最大迭代次数
        self.max_iter = maxIter
        self.obstacle_list = obstacleList
        # 存放RRT树上的节点的
        self.node_list = None

    def rrt_planning(self, start, goal, animation=True):
        # 计时
        start_time = time.time()
        # 起点和终点
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        # 起始节点先加进来
        self.node_list = [self.start]
        # 路径是空的
        path = None

        # 开始进行循环
        for i in range(self.max_iter):
            # 首先会进行一个采样
            rnd = self.sample()
            # 找到离采样点最近的那个节点对应的下标
            n_ind = self.get_nearest_list_index(self.node_list, rnd)
            # 找到距离最近的点
            nearestNode = self.node_list[n_ind]

            # 知道了产生的随机采样点和距离他最近的点,就可以求出两点连成直线所对应的角度,就是用的atan2方法
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            # 根据已知的数据我们就可以生成一个新的节点
            newNode = self.get_new_node(theta, n_ind, nearestNode)
            # 检查路径是否有碰撞
            noCollision = self.check_segment_collision(newNode.x, newNode.y, nearestNode.x, nearestNode.y)
            # 如果没有碰撞
            if noCollision:
                # 加入到树中
                self.node_list.append(newNode)
                if animation:
                    self.draw_graph(newNode, path)
                # 判断是否在终点附近
                if self.is_near_goal(newNode):
                    # 如果在终点的附近，那么就检测是否与障碍物发生碰撞，如果发生了碰撞，那么就直接进入下一个循环，继续迭代
                    if self.check_segment_collision(newNode.x, newNode.y, self.goal.x, self.goal.y):
                        # 没有发生碰撞，找到最后一个节点的下标
                        lastIndex = len(self.node_list) - 1
                        # 寻找相应的路径
                        path = self.get_final_course(lastIndex)
                        pathLen = self.get_path_len(path)
                        print("迭代次数{}".format(i))
                        print("current path length: {}, It costs {} s".format(pathLen, time.time() - start_time))

                        if animation:
                            self.draw_graph(newNode, path)
                        return path

    def sample(self):
        # 产生一个0-100的随机数,如果它比目标采样率大,那么在空间中随机找一个点
        if random.randint(0, 100) > self.goal_sample_rate:
            # 产生一个处于最大值和最小值之间的随机数
            rnd = [random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand)]
        else:  # goal point sampling
            # 否则直接返回终点的坐标就行了
            rnd = [self.goal.x, self.goal.y]
        return rnd

    def find_near_nodes(self, newNode):
        n_node = len(self.node_list)
        r = 50.0 * math.sqrt((math.log(n_node) / n_node))
        d_list = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2
                  for node in self.node_list]
        near_inds = [d_list.index(i) for i in d_list if i <= r ** 2]
        return near_inds

    @staticmethod
    def get_path_len(path):
        pathLen = 0
        for i in range(1, len(path)):
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            pathLen += math.sqrt((node1_x - node2_x)
                                 ** 2 + (node1_y - node2_y) ** 2)
        return pathLen

    @staticmethod
    def line_cost(node1, node2):
        """
        计算两个节点之间的距离
        :param node1: 节点1
        :param node2: 节点2
        :return: 节点距离的平方
        """
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    @staticmethod
    # 返回最小值对应的下标
    def get_nearest_list_index(nodes, rnd):
        # 计算node与采样点的距离
        dList = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex

    def get_new_node(self, theta, n_ind, nearestNode):
        # copy.deepcopy()是深拷贝，会拷贝对象及其子对象，哪怕以后对其有改动，也不会影响其第一次的拷贝。
        newNode = copy.deepcopy(nearestNode)
        # 计算出对应的xy坐标
        newNode.x += self.expand_dis * math.cos(theta)
        newNode.y += self.expand_dis * math.sin(theta)
        # 扩展的距离,累加在一起就是路径的距离
        newNode.cost += self.expand_dis
        # 新节点的父节点
        newNode.parent = n_ind
        return newNode

    def is_near_goal(self, node):
        d = self.line_cost(node, self.goal)
        if d < self.expand_dis:
            return True
        return False

    @staticmethod
    def distance_squared_point_to_segment(v, w, p):
        '''
        计算点到直线的距离
        :param v: 线段的起始点
        :param w:
        :param p: 线段外的点
        :return: 点到直线距离的平方
        '''
        # Return minimum distance between line segment vw and point p
        # 如果说线段的起始点和终止点位于同一个位置,那么点到直线的距离就是v到p的距离
        if np.array_equal(v, w):
            return (p - v).dot(p - v)  # v == w case
        l2 = (w - v).dot(w - v)  # i.e. |w-v|^2 -  avoid a sqrt
        # Consider the line extending the segment,
        # parameterized as v + t (w - v).
        # We find projection of point p onto the line.
        # It falls where t = [(p-v) . (w-v)] / |w-v|^2
        # We clamp t from [0,1] to handle points outside the segment vw.
        t = max(0, min(1, (p - v).dot(w - v) / l2))
        projection = v + t * (w - v)  # Projection falls on the segment
        return (p - projection).dot(p - projection)


    def check_segment_collision(self, x1, y1, x2, y2):
        """
        用于检查是否与障碍物发生了碰撞
        :param x1: 线段的起始坐标和终止坐标
        :param y1:
        :param x2:
        :param y2:
        :return:
        """
        # 遍历所有的障碍物
        for (ox, oy, size) in self.obstacle_list:
            # dd 点到直线的距离的平方
            dd = self.distance_squared_point_to_segment(
                np.array([x1, y1]),
                np.array([x2, y2]),
                np.array([ox, oy]))
            # 如果是小于圆的半径，就说明发生了碰撞
            if dd <= size ** 2:
                return False  # collision
        return True

    def check_collision(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)
        end_x = tmpNode.x + math.cos(theta) * d
        end_y = tmpNode.y + math.sin(theta) * d
        return self.check_segment_collision(tmpNode.x, tmpNode.y, end_x, end_y)

    def get_final_course(self, lastIndex):
        # 先把终点放进来
        path = [[self.goal.x, self.goal.y]]
        # 找点对应的父节点，如果是None那就说明找到了起点
        while self.node_list[lastIndex].parent is not None:
            node = self.node_list[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        #     加入最后的起始点
        path.append([self.start.x, self.start.y])
        return path

    def draw_graph(self, rnd=None, path=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")

        for node in self.node_list:
            if node.parent is not None:
                if node.x or node.y is not None:
                    plt.plot([node.x, self.node_list[node.parent].x], [
                        node.y, self.node_list[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacle_list:
            # self.plot_circle(ox, oy, size)
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")

        if path is not None:
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.axis([-2, 18, -2, 15])
        plt.grid(True)
        plt.pause(0.01)


def main():
    print("Start rrt planning")

    obstacleList = [
        (3, 3, 1.5),
        (12, 2, 3),
        (3, 9, 2),
        (9, 11, 2),
        (15, 8, 2.5),
        (12, 10, 1.5),
    ]

    # randArea 随机采样的范围, maxIter 最大迭代周期
    rrt = RRT(randArea=[-2, 18], obstacleList=obstacleList, maxIter=200)
    path = rrt.rrt_planning(start=[0, 0], goal=[15, 12], animation=show_animation)
    print("Done!!")

    if show_animation and path:
        plt.show()


if __name__ == '__main__':
    main()

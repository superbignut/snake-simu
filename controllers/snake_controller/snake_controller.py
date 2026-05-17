"""snake_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Camera, LED, Keyboard, InertialUnit, Gyro, Supervisor, PositionSensor, Node ,PositionSensor
import time
import numpy as np
import random

np.set_printoptions(precision=4)

# create the Robot instance.
# robot = Robot()
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
keyboard = robot.getKeyboard() # 触发键盘检测

# print(robot.getBasicTimeStep())
keyboard.enable(int(robot.getBasicTimeStep()))

# 全局坐标显示球
ball0 = robot.getFromDef("ball0")
ball1 = robot.getFromDef("ball1")
ball2 = robot.getFromDef("ball2")
ball3 = robot.getFromDef("ball3")
ball4 = robot.getFromDef("ball4")
ball5 = robot.getFromDef("ball5")
ball6 = robot.getFromDef("ball6")
ball7 = robot.getFromDef("ball7")
ball8 = robot.getFromDef("ball8")

# 局部坐标显示方块

box0 = robot.getFromDef("box0")
box1 = robot.getFromDef("box1")
box2 = robot.getFromDef("box2")
box3 = robot.getFromDef("box3")
box4 = robot.getFromDef("box4")
box5 = robot.getFromDef("box5")
box6 = robot.getFromDef("box6")
box7 = robot.getFromDef("box7")

print("box7, ", box0.getPosition())

# print(ball0.getPosition())


def getMotors()->list[Motor]:

    hinge_joint_1_motor = robot.getDevice("motor1")  # 假设电机名称是 "bar1_motor"，需根据实际场景树调整
    hinge_joint_2_motor = robot.getDevice("motor2")
    hinge_joint_3_motor = robot.getDevice("motor3")
    hinge_joint_4_motor = robot.getDevice("motor4")
    hinge_joint_5_motor = robot.getDevice("motor5")
    hinge_joint_6_motor = robot.getDevice("motor6")
    hinge_joint_7_motor = robot.getDevice("motor7")
    hinge_joint_8_motor = robot.getDevice("motor8")
    hinge_joint_9_motor = robot.getDevice("motor9")
    hinge_joint_10_motor = robot.getDevice("motor10")
    hinge_joint_11_motor = robot.getDevice("motor11")
    hinge_joint_12_motor = robot.getDevice("motor12")
    hinge_joint_13_motor = robot.getDevice("motor13")
    hinge_joint_14_motor = robot.getDevice("motor14")
    hinge_joint_15_motor = robot.getDevice("motor15")

    motor_list = [hinge_joint_1_motor, hinge_joint_2_motor, hinge_joint_3_motor, hinge_joint_4_motor,
                  hinge_joint_5_motor, hinge_joint_6_motor, hinge_joint_7_motor,hinge_joint_8_motor, 
                  hinge_joint_9_motor, hinge_joint_10_motor, hinge_joint_11_motor,hinge_joint_12_motor, 
                  hinge_joint_13_motor, hinge_joint_14_motor, hinge_joint_15_motor]

    return motor_list

def getSensors()->list[PositionSensor]:
    motor1_sensor = robot.getDevice("sensor1")
    motor2_sensor = robot.getDevice("sensor2")
    motor3_sensor = robot.getDevice("sensor3")
    motor4_sensor = robot.getDevice("sensor4")
    motor5_sensor = robot.getDevice("sensor5")
    motor6_sensor = robot.getDevice("sensor6")
    motor7_sensor = robot.getDevice("sensor7")
    motor8_sensor = robot.getDevice("sensor8")
    motor9_sensor = robot.getDevice("sensor9")
    motor10_sensor = robot.getDevice("sensor10")
    motor11_sensor = robot.getDevice("sensor11")
    motor12_sensor = robot.getDevice("sensor12")
    motor13_sensor = robot.getDevice("sensor13")
    motor14_sensor = robot.getDevice("sensor14")
    motor15_sensor = robot.getDevice("sensor15")

    # 开启传感器
    motor1_sensor.enable(timestep)
    motor2_sensor.enable(timestep)
    motor3_sensor.enable(timestep)
    motor4_sensor.enable(timestep)
    motor5_sensor.enable(timestep)
    motor6_sensor.enable(timestep)
    motor7_sensor.enable(timestep)
    motor8_sensor.enable(timestep)
    motor9_sensor.enable(timestep)
    motor10_sensor.enable(timestep)
    motor11_sensor.enable(timestep)
    motor12_sensor.enable(timestep)
    motor13_sensor.enable(timestep)
    motor14_sensor.enable(timestep)
    motor15_sensor.enable(timestep)
  
    sensor_list = [motor1_sensor, motor2_sensor, motor3_sensor, motor4_sensor,
                   motor5_sensor, motor6_sensor, motor7_sensor, motor8_sensor,
                   motor9_sensor, motor10_sensor, motor11_sensor,motor12_sensor,
                   motor13_sensor, motor14_sensor, motor15_sensor]

    return sensor_list

def getRobotNode()-> list[Node]:
    robot_nodes = []
    for i in range(16):  # This will loop from 0 to 15
        node_name = f'bar{i}'
        robot_node = robot.getFromDef(node_name)
        robot_nodes.append(robot_node)
    
    return robot_nodes

motor_list = getMotors()

sensor_list = getSensors()

node_list = getRobotNode()


def get_node_pose(num):
    # 返回第i个节点的姿态矩阵
    return node_list[num].getPose()

def get_node_position(num):
    # 获取第i个节点的位置坐标
    return node_list[num].getPosition()

def get_node_pose_3(num):
    # 返回第i个节点的姿态矩阵,只获取旋转矩阵部分 也就是 3*3 的部分
    pose =  node_list[num].getPose()
    
    pose_4x4 = np.array(pose).reshape((4, 4))
    
    pose_3x3 = pose_4x4[:3, :3] 
    
    return pose_3x3

def get_head_position():
    # 获取头节点坐标
    return get_node_position(1)

class tupleX:
    # 这是一个snake机器人的两个关节的组合，由于snake关节之间使用正交连接,
    # 因此为了给一个tuple组，从而获取两个以上的自由度，在有一个target方向
    # 的时候，可以做更合理的趋近  -- 当然也只是一个尝试
    
    # 对于任何一个 tuple：已后tuple的舵机旋转点为基点：
    # \theta_front, \theta_back 共同决定了一个tuple的终端坐标，设这个函数为F, 所以有：f_0 = F(\theta_front, \theta_back)
    # 由于每一个tuple 有一个 初始的 组装上的 姿态旋转矩阵 G0_i，所以修正后的f_0为：f_0 = G0_i * F(\theta_front, \theta_back)
    # 以上的坐标系都是基于locale坐标系来计算的
    
    # 上式中，由于引入了初始姿态矩阵，所以每一个tuple组的 F函数是相同的，只有G0_i 是不同的
    
    # f_0 为3维的空间坐标，当然由于只有两个自由度，（并且每一个舵机的运动角度受限），所以应该是一个有限的运动空间
    
    # 在运动中，蛇形机器人相较于全局坐标，会具有一个任意旋转的姿态时，假设这时的旋转矩阵为：G', 所以这时的目标坐标 f_1 = G' * f_0 = G' * G0_i * F(\theta_front, \theta_back)
    
    
    # 在webots中，因为getPose函数给出的是坐标系之间的旋转矩阵，也就是 把 局部坐标变换到全局坐标系下的 旋转矩阵, 因此得到的就已经是 G'了
    
    # 所以逆解是： \theta_front, theta_back = F^-1( Go^T * G'^T * f_1) 
    
    # 所以运动学的逆解需要提供一个目标角度，反解出 \theta_front \theta_back
    
    
    def __init__(self):
        
        # 在snake中的tuple编号
        self.tupleIndex = -1
        # 包含关节数
        self.partNums = 2
        # 当前的旋转矩阵
        self.currentG = np.array([[1, 2, 3],[4, 5, 6],[7, 8, 9]])
        # 初始的旋转矩阵
        self.initG0 = np.array([[1, 2, 3],[4, 5, 6],[7, 8, 9]])
        
        # 目标姿态坐标
        self.targeThetas = np.array([0, 0]) # front, back
        
        # 当前姿态坐标
        self.currentThetas = np.array([0, 0])
        
        # 当前位置坐标
        self.currentPosition = np.array([0, 0, 0])
        
        self.front_node:Node = None
        
        self.back_node:Node = None
        
        self.front_motor:Motor = None
        
        self.back_motor:Motor = None
        
        self.front_sensor:PositionSensor = None
        
        self.back_sensor:PositionSensor = None
        
        
        self.tmpt = 1
        
    def init_tuple_Index(self, index):
        """ 
            snake初始化时，需要最先修改,以及初始化了node、motor、sensor
        """

        self.tupleIndex = index
        
        self.front_node = node_list[index * 2]
        self.back_node = node_list[index * 2 + 1]
        
        self.front_motor = motor_list[index * 2]
        # 仿真中默认是15个电机，这里做了一点特殊判断
        if index * 2 + 1 < len(motor_list):
            self.back_motor = motor_list[index * 2 + 1]
            
        # 
        self.front_sensor = sensor_list[index * 2]
        if index * 2 + 1 < len(sensor_list):
            self.back_sensor = sensor_list[index * 2 + 1]
        
        
        
    def set_target_thetas(self, thetas, alpha=0.1):
        """
            设定前后舵机的两个角度，这里允许设定一个拟合度，因为直接把角度切过去不是很合理
            
            可以写成：
            
                current = alpha * target + (1 - alpha) * current
        """
        self.targeThetas = thetas
          
    def get_current_G(self):
        """
            获取从局部坐标变换到全局坐标的 姿态旋转矩阵
            webots：
                是getpose函数的返回值
            落地部署：
        """
        # print("testG", get_node_pose_3(self.tupleIndex * 2 + 1))#
        self.currentG = get_node_pose_3(self.tupleIndex * 2 + 1)
        
        return self.currentG
    
    def get_init_G0(self):
        """
            获取这个tuple组 较于tuple正方向的在局部坐标系下的姿态旋转矩阵
            
            机器人组装上本身就有的初始的旋转G0,以tuple的后关节为准
            
            不管是仿真环境还是真实环境，这里都是固定的
            
        """
        Z0 = np.array([[1,0,0],[0,1,0],[0,0,1]])
        Z180 = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        
        if self.tupleIndex % 2 == 0:
            # 头tuple是G0基准
            self.initG0 = Z0
        elif self.tupleIndex % 2 == 1:
            # 下一个tuple绕Z轴旋转了180
            self.initG0 = Z180
            
        return self.initG0

    
    def get_current_thetas(self):
        
        """ 
            获取当前的舵机的角度
        """
        
        # webots 中通过传感器获取角度
        front_theta = self.front_sensor.getValue()
        back_theta = self.back_sensor.getValue()
        
        self.currentThetas = np.array([front_theta, back_theta])
        
        return self.currentThetas

    def get_forward_K(self, thetas):
        """ 
            正向运动学求解
            一个tuple组有两个自由度，分别是前后两个舵机，
            用于根据角度输入，获取在后端点固定的情况下，前端的方向
            
            这个函数暂不需要
        """
        pass

    def get_inverse_K(self, normDirection):
        """
        获取运动学逆解：     
        从目标方向求解出前、后舵机需要旋转的角度 
        1. G0
        2. G1
        3. 运动学逆解 
            \theta_front, theta_back = F^-1( Go^T * G'^T * f_1) 
        """
        
        # print("norm ", normDirection)
        # print("initG0 ",self.initG0)
        # print("curreG ",self.currentG) #Todo 这个矩阵获取的不对
        
        # 1. 变换到局部坐标
        localDirection = self.get_init_G0().transpose() @ self.get_current_G().transpose() @ normDirection
        
        # print("norm ", normDirection)
        # print("initG ",self.get_init_G0())
        # print("currentG ", self.get_current_G())
        # print("localDirection: ",localDirection)
        # print("localeD:", localDirection)
        # 2.运动学逆解
        vtx, vty, vtz = localDirection
        
        # print("xyz", vtx, vty, vtz)
        
        theta_back = - np.arctan2(vtx, vtz) # 这个 - 号是坐标正方向的原因 # arctan2 的值域是 [-pi,pi]
        
        A = vty / np.sqrt(vtx * vtx + vtz * vtz)
        
        # print("A", A)
        
        theta_2 = np.arctan2(np.sqrt(vtx * vtx + vtz * vtz), vty) # 这里是因为值域的原因，将arctan 换成了arctan2
        
        # print("theta_2: ", theta_2)
        
        theta_front = np.pi - 2 * theta_2
        
        # print("this is A: ",A)
        
        
        # print("theta_f ", theta_front)
        # print("theta_b", theta_back)

        return np.array([theta_front, theta_back])
        
        """
        
     y
    ^    
    |            
    |                          
    |                  
    |                    
    |                  
    |                                 /
    |                                /
    |                               /
    |                              /
    |                             /  l2
    |                            /
    |---------------------------/-------------------->  x
     \                         /
      \         l1            /   θ_front
       \--------------------- O --------   
        \  
         \ - θ_back
          \
           \  z
            \
             \
             _\|      
    
    
    x z 
    y x
    z y
    θ_back 向 -y 的方向为 后舵机的正方向
    
    θ_front 向 z 的正方向为 前舵机正方向
    
    
    v_z = (l1 + l2 * cos(θ_front)) * cos(- θ_back)  =   (l1 + l2 * cos(θ_front)) * cos(θ_back) 
    v_x = (l1 + l2 * cos(θ_front)) * sin(- θ_back)  = - (l1 + l2 * cos(θ_front)) * sin(θ_back)
    v_y = l2 * sin(θ_front)
    
    
    对于一个 归一化的 vt 向量来说 vt = (vt_x, vt_y, vt_z) 有：
    
    - θ_back = arctan( vt_x / vt_z ), 
    
    所以: θ_back = - arctan( vt_x / vt_z ) + k \pi
    
    又因为 vt_y / sqrt (|vt_z^2 + vt_x^2|) = l2 * sin(θ_front) / (l1 + l2 * cos(θ_front))　
    
    有，cos(\theta_front + \phi) = -Al1 / (l2 * sqrt(A^2 + 1)), tan(\phi)= 1/A   ----------(1)
    
    
    
    所以: θ_front = -arctan(1/A) ± arccos(-Al1 / (l2 * sqrt(A^2 + 1))) + 2k \pi ,其中 A = vt_y / sqrt(vt_z^2 + vt_x^2) 
    
    这里发现，如果把A带进去,考虑如下直角三角形：
    
    
    


           /|  θ_2
          / |
         /  | 
        /   |   v_y
       /    |
      /     |
     /      |
    /       |  
    ---------
θ_1    sqrt(vt_z^2 + vt_x^2) 




    会发现，在l1 = l2的情况下： 
    
        tan(\theta_2) = 1 / A;   -------------------(3)
        
        cos(\pi +- \theta_2)  = - A / (sqrt(A^2 + 1))  -------------(2)
        
        
     这里如果和 (1) (2) (3)式联立：
     
     \pi +- \theta_2 = theta_front + \theta_2
    
    所以 \theta_front = \pi 或者 \pi - 2 theta_2 
    
    所以在 theta_2 小于 \pi / 2 的情况下，   \pi - 2 theta_2  都是 > 0 的，这里只要再考虑到舵机的转动角度
    
        
    """

    def set_target_direction(self, targetDirection):
        """
            设定头节点的目标坐标
            
            1. 反向求解前后舵机角度，使得tuple的朝向可以指向目标方向
            2. 更新到self.targetTheta上
        
        """
        
        tmp_nd= targetDirection * 1.0 / np.linalg.norm(targetDirection) # 这里乘一个系数 防止直接到位
        
        # print("tmp nd: ", tmp_nd)
        
        thetas = self.get_inverse_K(normDirection=tmp_nd)
        
        print("target thetas 0 ", thetas)
        
        self.set_target_thetas(thetas=thetas)


    
    def set_target_tuple(self, targetTuple, boost=1.0):
        
        """ 
            设定需要follow的目标tuple:

            这里直接设置当前tuple的目标角度是 target tuple的当前角度
        """
        
        targetThetas = targetTuple.get_current_thetas()
        
        self.set_target_thetas(thetas=targetThetas*boost)

    
    def go_forward(self, local_v=np.array([0,0,1]), speedAlpha=5):
        """
            让tuple的每一个关节向轴向 以指定速度 运动

        """
        
        # self.tmpt += 1
        
        # local_v = np.array([0,0,1]) # 水平面是 xz
        
        # vertical_v = np.array([1,0,0])
        
        # local_v[2] = self.tmpt * local_v[2] 
        # local_v[0] =  np.sin(0.3 * self.tmpt + 0.1) * vertical_v[0] *  self.tmpt * 0.4 # 速度反向是ok的，但是整体摆动不太对劲
        
        
        # local_v = local_v / np.linalg.norm(local_v)
        
        # print("localv ",local_v)
        
        # alpha = 5   
            
        global_v = speedAlpha * self.get_init_G0() @ self.get_current_G() @ local_v 
        
        # print("global_v" , global_v)
        
        self.back_node.setVelocity(global_v) # 这里目前是后节点来提供动力
    
    
    def update(self, timeDelta):
        """ 
            更新坐标, 运动执行函数
            在update之前需要将目标的theta坐标，更新到self.targetTheta中
            由update函数统一更新
        """

        # 前后舵机设定目标角度
        # print(self.tupleIndex ,self.targeThetas, self.front_motor, self.back_motor)
        
        self.front_motor.setPosition(self.targeThetas[0] * timeDelta)

        if self.back_motor != None:
            self.back_motor.setPosition(self.targeThetas[1]*timeDelta)
        # 前后模块设定速度



"""
    这是一个全局的轨迹显示类，用于在webots中显示全局轨迹坐标
    
    终点坐标用红球来代替，路径坐标是蓝色的球

"""

class globalPathX:
    # 这里假设有一个默认的全局路径
    def __init__(self):
        # 路径
        self.path = []
        # 终点球
        self.destBall = ball0
        # 路径球
        self.pathBall = []
        # 路径球数量
        self.pathBallNum = 8
        # 路径球初始化
        for i in range(1, self.pathBallNum + 1): 
            pathBallName = f'ball{i}'
            self.pathBall.append(robot.getFromDef(pathBallName))
        
    def update(self):
        """ 
            迭代更新路径， 更新之前需要先初始化self.path
        
        """
        # 头节点坐标
        headPos = get_head_position()
        
        # 判断是否到达最近的目标点
        while np.linalg.norm(self.path[0] - headPos) < 0.2:       
                # 计算是否到达坐标点，到达则弹出
                self.path.pop(0)
                break
        # 到达重点        
        if self.isFinished():
            print("Global Destination has finished.")
            return

        if len(self.path) <= self.pathBallNum:
            # 如果坐标数 <= 蓝球数
            for i in range(len(self.pathBall)):
                # 设置所有路径的坐标
                if i < len(self.path) -1:
                    self.setBallPos(i, pos=self.path[i])
                else:
                    self.hideBall(i)
        else:
            # 如果坐标数大于球数，则只显示前几个
            for i in range(len(self.pathBall)):
                self.setBallPos(i, pos=self.path[i])
        
        self.setDesBallPos(pos=self.path[-1])
        
        # 返回最近的目标节点
        return self.getNearestBallPos()
    
    def isFinished(self):
        return len(self.path) == 0

    def hideBall(self, i):
        # 看不见这个球
        self.setBallPos(i, pos=[-100,-100,-100])
    
    def setDesBallPos(self, pos):
        # 设置红球位置
        # print("des pos :", pos)
        self.destBall.getField("translation").setSFVec3f(list(pos)) 
    
    def setBallPos(self,i, pos):
        # 设置蓝球位置
        self.pathBall[i].getField("translation").setSFVec3f(list(pos)) 
        
    def getNearestBallPos(self):
        return self.path[0]
    

""" 
    全局坐标有一个问题就是, snake 只能纯粹的follow,灵活性很差 我这里是不是可以做一个
    局部规划器, 在两个全局坐标点之间, 生成一个局部轨（二维：一个曲线、三维：一个圆, 或者大于直径就行其实）
    然后做到snake的运动不是直直的，而是具有一定wind的snake-like曲线
    
    并且这里还可以考虑到，sanke-like的body能增加body的宽度，有更好的平衡性和稳定性
"""
class localPathX:
    # 在全局路径的默认的情况下，局部路径感觉要更动态一点
    # 可以每走一步更新一下，也可以一个全局节点更新一下
    # 暂时按照一个目标节点一次更新来做吧
    def __init__(self, globalPath:globalPathX):
        # 路径
        self.path = []
        # 路径块集合
        self.pathBox = []
        # 路径块数量
        self.pathBoxNum = 8
        # 局部轨迹需要知道当前的全局轨迹
        self.globalPathX = globalPath
        
        # 路径块初始化
        for i in range(self.pathBoxNum): 
            pathBoxName = f'box{i}'
            self.pathBox.append(robot.getFromDef(pathBoxName))
            
        self.flag = 1
        
        
        
    def compute_local_path(self):
        """
            根据 globalPathX 计算出局部坐标, 更新到path中
        """
        
        headPosition = node_list[0].getPosition()        
        
        targetPosition = self.globalPathX.path[0]
        
        t = np.arange(0, 1, 0.1) # 用于计算步长
        
        # p1 是控制点，用于控制局部路径的弯曲程度
        if self.flag == 1:
            p1 = (np.array(headPosition)*1.5 + targetPosition * 1.5) / 2 
        else:
            p1 = (np.array(headPosition)*0.7 + targetPosition * 0.7) / 2 
        
        
        self.path = bezier_curve_2(p0=headPosition, p1=p1, p2=targetPosition)
        
        self.flag *= -1
        
        

    
    def update(self):
        # 更新局部坐标显示
        # 如果到达了最后一个局部坐标，则触发一次计算
        headPos = get_head_position()
        
        # 如果全局坐标空了，则局部也结束
        if self.globalPathX.isFinished():
            print("is finished")
            return
        
        if len(self.path) == 0:
            # 这里是因为最开始可能是空的
            self.compute_local_path()
            # print("is finished")
            return self.update()
        
        while np.linalg.norm(self.path[0] - headPos) < 0.05:
            # 计算是否到达坐标点，到达则弹出
            self.path.pop(0)
            break
        
        if len(self.path) == 0:
            # 如果路径空了，则需要重新拉去一个
            # 如果这里是每次强制计算，则就是最大频率的局部坐标计算
            self.compute_local_path()
            # print("is finished")
            return self.update()

        # 如果坐标数 <= box数
        if len(self.path) <= self.pathBoxNum:
            # 设置所有路径的坐标
            for i in range(len(self.pathBox)):
                if i < len(self.path):
                    self.setBoxPos(i, pos=self.path[i])
                else:
                    self.hideBox(i)
        else:
            # 如果大于，则只显示前几个
            for i in range(len(self.pathBox)):
                self.setBoxPos(i, pos=self.path[i])
                
        return self.getNearestBoxPos()
    
    
    def hideBox(self, i):
        # 看不见这个box
        self.setBoxPos(i, pos=[-100,-100,-100])
    
    def setBoxPos(self, i, pos):
        # 设置路径box位置
        self.pathBox[i].getField("translation").setSFVec3f(list(pos)) 
        
    def getNearestBoxPos(self):
        # 返回最近的一个局部路径的坐标
        return self.path[0]

class snakeX:
    
    # 包含了多个tuple组的snake，用于设定目标点坐标以及迭代更新
    def __init__(self):
        # tuple组的个数
        self.tupleNums = 8
        # tuple数组
        self.tupleList:list[tupleX] = [tupleX() for _ in range(self.tupleNums)]
        
        # 全局轨迹对象，用于保存、显示全局轨迹
        self.globalTraj = globalPathX()
        
        # 局部轨迹对象，传入全局轨迹
        self.localTraj = localPathX(globalPath=self.globalTraj)
        
        # 每一个tuple，初始化初始旋转坐标
        for index, _ in enumerate(self.tupleList):
            # 1. 设定编号与对应初始化
            self.tupleList[index].init_tuple_Index(index)
            # 2. 获取初始旋转矩阵
            self.tupleList[index].get_init_G0()
    
    def compute_target_position(self, targetDirection, timeDelta):
        """
            向目标节点进行一次更新，主要策略为火车头策略
            1. 首个tuple根据前进方向确定下一个位置姿态
            2. 后续tuple完成对前一个节点的姿态跟随
        """
        
        for index in range(len(self.tupleList)):
            # 遍历tuple，设定目标坐标
            if index == 0:
                # 如果是头tuple, 则设置目标坐标
                self.tupleList[index].set_target_direction(targetDirection=targetDirection)
            else:
                # 如果是后续节点，则跟随前一个节点的姿态

                self.tupleList[index].set_target_tuple(targetTuple=self.tupleList[index - 1], boost=1.2) # 这个boost 决定了后续的角度的放大倍数
            
        
        self.update_all(timeDelta=timeDelta)
        
    
    def update_all(self,timeDelta):
        for index in range(len(self.tupleList)):
            self.tupleList[index].update(timeDelta=timeDelta)
            
            self.tupleList[index].go_forward(local_v=np.array([0,0,1]), speedAlpha= 6.0 / np.exp(index)) # 这个alpha 决定了速度的大小
        
    
    def follow_key_board(self):
        """ 
            键盘控制前进方向
        """
        direction1 = np.array([10, -10, 0])
        direction2 = np.array([-10, -10, 0])
        
        key = keyboard.getKey()
        
        if key == keyboard.LEFT:
            direction = direction1
            self.compute_target_position(targetDirection=direction,timeDelta=1)
        elif key == keyboard.RIGHT:    
            direction = direction2
            self.compute_target_position(targetDirection=direction,timeDelta=1)

    
    def follow_gloal_trajectory(self):
        """ 
            局部规划器：输入全局轨迹，计算这个方向的局部路径曲线，进而进行运动
            或者最开始直接直接运动就行
        """
        # 迭代返回最近的全局坐标点
        nearestGloablTargetPosition = self.globalTraj.update()
        
        # 迭代返回最近的局部坐标点
        nearestLocalTargetPosition = self.localTraj.update()
        
        # 朝着目标点运动
        self.compute_target_position(targetDirection=nearestLocalTargetPosition - get_head_position(),timeDelta=1)
        
        # print("nearest target: ", nearestGloablTargetPosition)

        # self.compute_global_trajectory(targetPosition=targetPosition)
           
    def compute_global_trajectory(self, targetPosition):
        """ 
            全局规划器：输入目标点，计算这个方向的全局路径
            
            # 先假设全局坐标就只有单一的几个点，试试看
            
            # 逐步过渡到三维空间的轨迹追踪(在World中加一个圆柱体)
            
        """
        
        headPosition = node_list[0].getPosition()
        
        direction = targetPosition - headPosition
        
        t = np.arange(0, 1, 0.1) # 用于计算步长
        
        # self.globalTraj = []
        
        for index, item_t in enumerate(t):

            tmpPos = headPosition + item_t * direction

            self.globalTraj.path = bezier_curve_3(headPosition, headPosition + np.array([-1, -1, 0]),  targetPosition + np.array([-1, -2, 0]), targetPosition)
            
 
def bezier_curve_3(p0, p1, p2, p3, num_points=10):
    """
        3次贝塞尔曲线, p0起点, p3终点, p1/p2控制点
    """
    t = np.linspace(0, 1, num_points)
    t = t[:, None] # t.shape = 10 * 1
    # p.shape = 1* 3
    curve = (1-t)**3 * p0 + 3*(1-t)**2*t * p1 + 3*(1-t)*t**2 * p2 + t**3 * p3
    # 
    return list(curve)

def bezier_curve_2(p0, p1, p2, num_points=10):
    """
        2次贝塞尔曲线, p0起点, p3终点, p1控制点
    """
    t = np.linspace(0, 1, num_points)
    t = t[:, None] # t.shape = 10 * 1
    # p.shape = 1* 3
    curve = (1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2
    # 
    return list(curve)
    
ss = snakeX()

des = np.array([1, -1, 0])

# 计算全局轨迹
ss.compute_global_trajectory(targetPosition=des)

while robot.step(timestep) != -1:
    """
        给定一个归一化的方向，机器人会朝着这个反向，不断步进
        
        这个归一化的运动方向是相对于 snake头节点来说的
    
    """
    # follow 全局轨迹
    ss.follow_gloal_trajectory()
    
    # ss.follow_key_board()
    
    # ss.compute_target_position(targetDirection=targetDirection,timeDelta=0.96)
    

"""

while robot.step(timestep) != -1:

currentPosition_list = [sr.getValue() for sr in sensor_list]

for i in range(len(motor_list)):
    theta = 1.0 * np.sin(np.pi * t + 1.26 * i)
    motor_list[i].setPosition(theta) 

t += 0.2

time.sleep(0.05)
    

"""
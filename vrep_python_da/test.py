
import sys
import ast
import libpyauboi5
import logging
from logging.handlers import RotatingFileHandler
from PySide6.QtWidgets import QApplication,QTextEdit, QWidget, QVBoxLayout, QPushButton, QLineEdit, QFileDialog, QMainWindow, QHBoxLayout, QLabel
import os
from math import pi
from robotcontrol import Auboi5Robot, RobotErrorType, RobotCoordType, RobotCoordCalMethod,RobotError,RobotIOType,RobotUserIoName
import time
import json
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
import keyboard
import math
try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

logger = logging.getLogger('main.robotcontrol')
def logger_init():
    # Log等级总开关
    logger.setLevel(logging.INFO)

    # 创建log目录
    if not os.path.exists('./logfiles'):
        os.mkdir('./logfiles')

    # 创建一个handler，用于写入日志文件
    logfile = './logfiles/robot-ctl-python.log'

    # 以append模式打开日志文件
    # fh = logging.FileHandler(logfile, mode='a')
    fh = RotatingFileHandler(logfile, mode='a', maxBytes=1024*1024*50, backupCount=30)

    # 输出到file的log等级的开关
    fh.setLevel(logging.INFO)

    # 再创建一个handler，用于输出到控制台
    ch = logging.StreamHandler()

    # 输出到console的log等级的开关
    ch.setLevel(logging.INFO)

    # 定义handler的输出格式
    # formatter = logging.Formatter("%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s")
    formatter = logging.Formatter("%(asctime)s [%(thread)u] %(levelname)s: %(message)s")

    # 为文件输出设定格式
    fh.setFormatter(formatter)

    # 控制台输出设定格式
    ch.setFormatter(formatter)

    # 设置文件输出到logger
    logger.addHandler(fh)

    # 设置控制台输出到logger
    logger.addHandler(ch)

def rad_to_degree(q):
        temp=[]
        for i in range(len(q)):
            temp.append(q[i]*180/pi)
        return temp
def create_matrix(roll, pitch, yaw, px, py, pz):
    R11 = np.cos(pitch) * np.cos(yaw)
    R12 = np.cos(yaw) * np.sin(roll) * np.sin(pitch) - np.cos(roll) * np.sin(yaw)
    R13 = np.cos(roll) * np.cos(yaw) * np.sin(pitch) + np.sin(roll) * np.sin(yaw)
    R21 = np.cos(pitch) * np.sin(yaw)
    R22 = np.cos(roll) * np.cos(yaw) + np.sin(roll) * np.sin(pitch) * np.sin(yaw)
    R23 = -np.cos(yaw) * np.sin(roll) + np.cos(roll) * np.sin(pitch) * np.sin(yaw)
    R31 = -np.sin(pitch)
    R32 = np.cos(pitch) * np.sin(roll)
    R33 = np.cos(roll) * np.cos(pitch)

    T = np.array([[R11, R12, R13, px],
                  [R21, R22, R23, py],
                  [R31, R32, R33, pz],
                  [0, 0, 0, 1]])

    return T
def run_robot_1():
        # self.x = float(self.line_edit_x.text())
        # self.y = float(self.line_edit_y.text())
        # self.z = float(self.line_edit_z.text())
        Auboi5Robot.initialize() # khởi tạo hệ thống
        robot = Auboi5Robot() # Tạo lớp điều khiển cánh tay robot
        handle = robot.create_context() # Tạo ngữ cảnh
        logger.info("robot.rshd={0}".format(handle)) # Bối cảnh in 
        # Đọc nội dung từ file .txt
        file_home = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\home_point.txt"  # Thay đổi đường dẫn đến tệp .txt thực tế
        with open(file_home, 'r') as file:
            content = file.read()
        home_point = json.loads(content)
        w,x,y,z = home_point['ori']
        ori= (w,x,y,z)
        rpy= robot.quaternion_to_rpy( ori)
        rpy= rad_to_degree(rpy)
        print(rpy)
        ori_tool= (-3.140160, 0.004737, 0.16747, 0.0)
        ori_rpy= robot.quaternion_to_rpy(ori_tool)
        ori_rpy=rad_to_degree(ori_rpy)
        print("rpy of tool: ", ori_rpy)

        file_throw = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\throw_point.txt"  # Thay đổi đường dẫn đến tệp .txt thực tế
        with open(file_throw, 'r') as file:
            content = file.read() 
        throw_point = json.loads(content)
        
        logger_init() # Khởi tạo bộ ghi nhật ký
        logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time())) # bắt đầu kiểm tra
        # Auboi5Robot.initialize() # khởi tạo hệ thống
        # robot = Auboi5Robot() # Tạo lớp điều khiển cánh tay robot
        # handle = robot.create_context() # Tạo ngữ cảnh
        # logger.info("robot.rshd={0}".format(handle)) # Bối cảnh in 
        try:
            ip = '192.168.3.39'
            port = 8899
            result = robot.connect(ip, port)
            if result != RobotErrorType.RobotError_SUCC:
                logger.info("connect server{0}:{1} failed.".format(ip, port))
            else:
                robot.enable_robot_event()
                robot.init_profile()
                # user_coord = {'coord_type': RobotCoordType.Robot_Base_Coordinate,
                #           'calibrate_method': 0,
                #           'calibrate_points':
                #               {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                #                "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                #                "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
                #           'tool_desc':
                #               {"pos": (-0.016307, 0.0171,  0.086615),
                #                "ori": (0.026213, -0.087676, -1.98273, 0.0)}
                #           }
                user_tool={"pos": (-0.016307, 0.0171, 0.086615),
                    "ori": (-3.140160, 0.004737, 0.16747, 0.0)}
                # point = [self.x,self.y,self.z]
                # print(point[0])

                robot.set_end_max_line_velc(0.1)
                robot.set_joint_maxvelc((0.1,0.1,0.1,0.1,0.1,0.1))

                # print(self.x, self.y, self.z)

                robot.set_base_coord()
                robot.set_tool_kinematics_param(user_tool)
        
        

                robot.move_line(home_point['joint'])
                file_path= r'C:\Users\thanhgyza\Downloads\vrep_python_da\point_xla.txt'
                data = np.loadtxt(file_path, delimiter=',')
                print(data[0])  # trích xuất tọa độ x: data[0][0]
                for i in range(len(data)):
                    robot.move_to_target_in_cartesian([data[i][0],data[i][1],data[i][2]],rpy_xyz=(89.585632,-7.597886, 89.489021))
                    get_pos= robot.get_current_waypoint()
                    print(get_pos)

                #robot.set_relative_offset_on_base((0,0,0.05),(1,0,0,0))
                
                # robot.move_to_target_in_cartesian([point[0],point[1],point[2]],rpy_xyz=(89.585632,-7.597886, 89.489021))
                #robot.set_relative_offset_on_base((0,0,0),(1,0,0,0))
                robot.move_to_target_in_cartesian([0.725,-0.3,0.5],rpy_xyz=(89.585632,-7.597886, 89.489021))
                # robot.set_board_io_status(5,"U_DO_00",1)
                #robot.set_relative_offset_on_base((0,0,0.05),(1,0,0,0))
                #robot.move_to_target_in_cartesian([point[0],point[1],point[2]],rpy_xyz=(89.585632,-7.597886, 89.489021))

                robot.move_line(throw_point['joint'])
                robot.set_relative_offset_on_base((0,0,0),(1,0,0,0))
                robot.move_line(throw_point['joint'])
                robot.set_relative_offset_on_base((0,0,0.1),(1,0,0,0))
                robot.move_line(throw_point['joint'])
                robot.set_relative_offset_on_base((0,0,0),(1,0,0,0))
                input()
                
                robot.move_line(home_point['joint'])
                robot.disconnect() # Ngắt kết nối robot

        except KeyboardInterrupt:
            robot.move_stop()
            
        except RobotError as e:
            logger.error("robot Event:{0}".format(e))

        finally:
            if robot.connected:
                robot.disconnect()
            Auboi5Robot.uninitialize()
            #self.info.append("------------Kết thúc chương trình chạy thủ công-------------")
            #print("------------Kết thúc chương trình chạy thủ công-------------")




# if __name__ == '__main__':
#     run_robot_1()
# bảng DH
a = [0 , 0.647, 0.6005, 0, 0, 0 ]
alpha = [np.pi/2, 0, 0, -np.pi/2, np.pi/2,0]
d = [0.1632, 0.197, -0.1235, 0.1278, 0.1025, 0.094+0.105]
theta = [0, np.pi/2, 0, -np.pi/2, 0, 0]

# Tạo một mảng chứa các khớp của robot
links = []
for i in range(6):
    link = RevoluteDH(d[i], a[i], alpha[i], theta[i])
    links.append(link)

# Tạo đối tượng robot từ mảng các khớp
robot_i10 = DHRobot(links, name='MyRobot')      
# khởi tạo ma trận ban đầu
# T= create_matrix(0,np.pi/2,0,0,-0.4003,1.513)  
config = [0,0,0,0,0,0]
mask= [1,1,1,1,1,1]   
           
            
enable_robot_i10= True
print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    vrep.simxSetStringSignal(clientID,'connect_status','ok',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'in4_ip','127.0.0.1',vrep.simx_opmode_oneshot)
    time.sleep(0.1)
    print ('Connected to remote API server')

    # connect to robot
    logger_init() # Khởi tạo bộ ghi nhật ký
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time())) # bắt đầu kiểm tra
    Auboi5Robot.initialize() # khởi tạo hệ thống
    robot = Auboi5Robot() # Tạo lớp điều khiển cánh tay robot
    handle = robot.create_context() # Tạo ngữ cảnh
    logger.info("robot.rshd={0}".format(handle)) # Bối cảnh in 

    try:
        ip = '192.168.3.39'
        port = 8899
        result = robot.connect(ip, port)
        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            robot.enable_robot_event()
            robot.init_profile()
            user_tool={"pos": (-0.016307, 0.0171, 0.086615),
                "ori": (-3.140160, 0.004737, 0.16747, 0.0)}

            robot.set_end_max_line_velc(0.1)
            robot.set_joint_maxvelc((0.1,0.1,0.1,0.1,0.1,0.1))
            robot.set_base_coord()
            robot.set_tool_kinematics_param(user_tool)
            get_pos_pre= robot.get_current_waypoint()
            time.sleep(0.1)

            while True:
                get_pos= robot.get_current_waypoint()
                print(get_pos)
                px,py,pz=get_pos['pos']
                w,x,y,z = get_pos['ori']
                distance_pos= math.sqrt((px - get_pos_pre['pos'][0])**2 + (py - get_pos_pre['pos'][1])**2 + (pz - get_pos_pre['pos'][2])**2)
                if (distance_pos>0.01):
                    ori= (w,x,y,z)
                    rpy= robot.quaternion_to_rpy(ori)
                    T=create_matrix(rpy[0],rpy[1],rpy[2],px,py,pz)
                    joint= robot_i10.ikine_LM(T, config, mask=mask, ilimit=50)
                    config= joint.q
                    get_pos_pre= get_pos
                    # result = f"1 {config[0]} {config[1]} -2.075728119605464 2.0139038892100487 1.428503667419843 -1.570796336581565 1"
                    numberString = "1  " + "".join(str(config[0]))+" " + "".join(str(config[1]))+" "+ "".join(str(config[2]))+" "+ "".join(str(config[3]))+" "+ "".join(str(config[4]))+" "+ "".join(str(config[5])) + "  1"
                    print(numberString)

                    vrep.simxSetStringSignal(clientID,'numberString',numberString,vrep.simx_opmode_oneshot)
                    while True:
                        [errorCode, stringData] = vrep.simxGetStringSignal(clientID, 'phanhoi', vrep.simx_opmode_blocking)
                        stringData = stringData.decode() 
                        #print(stringData)
                        if stringData == '111':
                            print('send data joint ok')
                            vrep.simxSetStringSignal(clientID, 'phanhoi', '', vrep.simx_opmode_oneshot)
                            break
                else:
                    time.sleep(0.1)
                
                time.sleep(0.02)
                if keyboard.is_pressed('1'):
                    break

            
            # robot.move_to_target_in_cartesian([0.725,-0.3,0.5],rpy_xyz=(89.585632,-7.597886, 89.489021))
            
            robot.disconnect() # Ngắt kết nối robot

    except KeyboardInterrupt:
        robot.move_stop()
        
    except RobotError as e:
        logger.error("robot Event:{0}".format(e))

    finally:
        if robot.connected:
            robot.disconnect()
        Auboi5Robot.uninitialize()


    vrep.simxSetStringSignal(clientID, 'ketthuc', '11', vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)
    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
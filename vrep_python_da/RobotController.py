import sys
import ast
import libpyauboi5
import logging
from logging.handlers import RotatingFileHandler
from PySide6.QtWidgets import QApplication,QTextEdit, QWidget, QVBoxLayout, QPushButton, QLineEdit, QFileDialog, QMainWindow, QHBoxLayout, QLabel
import os
from math import pi
from robotcontrol import Auboi5Robot, RobotErrorType, RobotCoordType, RobotCoordCalMethod,RobotError,RobotIOType,RobotUserIoName
import multiprocessing
import time
import json

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
class JointInfoApp(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.origin_point = None
        self.return_point = None

    def init_ui(self):
        self.point_labels = []  # Danh sách để lưu các nhãn
        self.line_edits = []  # Danh sách để lưu các QLineEdit
        self.setWindowTitle("Bảng điều khiển")
        self.setGeometry(100, 100, 700, 500)

        main_widget = QWidget(self)
        main_layout = QVBoxLayout(main_widget)

        # Widget 1: Các điểm và tọa độ
        widget_points_coords = QWidget(self)
        layout_points_coords = QHBoxLayout(widget_points_coords)
        
        # Widget cho các điểm
        widget_points = QWidget(self)
        layout_points = QVBoxLayout(widget_points)

        for i in range(3):
            point_label = QLabel(f"Point {i+1}:")
            self.point_labels.append(point_label)
            line_edit = QLineEdit()
            self.line_edits.append(line_edit) 
            layout_points.addWidget(point_label)
            layout_points.addWidget(line_edit)

        # Widget cho các tọa độ
        widget_coords = QWidget(self)
        layout_coords = QVBoxLayout(widget_coords)

        label_x = QLabel("x:")
        label_y = QLabel("y:")
        label_z = QLabel("z:")
        self.line_edit_x = QLineEdit()
        self.line_edit_y = QLineEdit()
        self.line_edit_z = QLineEdit()

        layout_coords.addWidget(label_x)
        layout_coords.addWidget(self.line_edit_x)
        layout_coords.addWidget(label_y)
        layout_coords.addWidget(self.line_edit_y)
        layout_coords.addWidget(label_z)
        layout_coords.addWidget(self.line_edit_z)
        layout_points_coords.addWidget(widget_points)
        layout_points_coords.addWidget(widget_coords)

        # Widget 2: Các nút nhấn
        widget_buttons = QWidget(self)
        
        layout_buttons = QHBoxLayout(widget_buttons)
        
        self.btn_add_origin = QPushButton('Add Origin')
        self.btn_add_home = QPushButton('Add Home Point')
        self.btn_add_throw = QPushButton('Add Throw Point')
        self.btn_get_joint = QPushButton('Get Joint Info')
        self.btn_run_robot = QPushButton('Run Robot')
        self.btn_run_robot_1 = QPushButton('Run Robot base')
        self.btn_run_auto = QPushButton('Run Auto')
        

        layout_buttons.addWidget(self.btn_add_origin)
        layout_buttons.addWidget(self.btn_add_home)
        layout_buttons.addWidget(self.btn_add_throw)
        layout_buttons.addWidget(self.btn_get_joint)
        layout_buttons.addWidget(self.btn_run_robot)
        layout_buttons.addWidget(self.btn_run_robot_1)
        layout_buttons.addWidget(self.btn_run_auto)

        widget_info = QWidget(self)
        layout_info = QHBoxLayout(widget_info)
        self.info = QTextEdit()
        self.info.setReadOnly(True)  # Đặt chế độ chỉ đọc
        layout_info.addWidget(self.info)

        main_layout.addWidget(widget_points_coords)
        main_layout.addWidget(widget_buttons)
        main_layout.addWidget(widget_info)

        # Kết nối nút nhấn
        self.btn_add_origin.clicked.connect(self.add_origin)
        self.btn_add_home.clicked.connect(self.add_home_point)
        self.btn_add_throw.clicked.connect(self.add_throw_point)
        self.btn_get_joint.clicked.connect(self.get_joint_info2)
        self.btn_run_robot.clicked.connect(self.run_robot)
        self.btn_run_robot_1.clicked.connect(self.run_robot_1)
        #self.btn_run_auto.clicked.connect(self.run_robot_auto)

    def add_origin(self):
        
        logger_init() # Khởi tạo bộ ghi nhật ký
        logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time())) # bắt đầu kiểm tra
        Auboi5Robot.initialize() # khởi tạo hệ thống
        robot = Auboi5Robot() # Tạo lớp điều khiển cánh tay robot
        handle = robot.create_context() # tạo ngữ cảnh
        logger.info("robot.rshd={0}".format(handle)) # bối cảnh in

        try:
            ip = '192.168.3.39'
            port = 8899
            result = robot.connect(ip, port) 
            if result != RobotErrorType.RobotError_SUCC:
                logger.info("connect server{0}:{1} failed.".format(ip, port))
            else:
                robot.enable_robot_event()
                robot.init_profile()
                user_coord = {'coord_type': RobotCoordType.Robot_World_Coordinate,
                'calibrate_method': RobotCoordCalMethod.CoordCalMethod_xOxy,
                'calibrate_points':
                {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), #Khởi tạo ban đầu
                "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
                'tool_desc':
                {"pos": (0.047283, 0.02, 0.119),
                    "ori": (1.0, 0.0, 0.0, 0.0)}
                            }
                self.origin_point= robot.get_current_waypoint()
                file_path = r"D:\Downloads\ketnoirobot\point_data.txt"
                with open(file_path, 'a') as file:
                    data_str = str(self.origin_point).replace("'", '"')
                
                    file.write(data_str + '\n')
                
              
        except KeyboardInterrupt:
            robot.move_stop()
            
        except RobotError as e:
            logger.error("robot Event:{0}".format(e))
            
        finally:
            if robot.connected:
                robot.disconnect()
            Auboi5Robot.uninitialize()
            #self.info.append("Dữ liệu Origin đã được lưu vào:", file_path)
            
            #print("Dữ liệu Origin đã được lưu vào:", file_path)

            
    def add_throw_point(self):
        logger_init() # Khởi tạo bộ ghi nhật ký
        logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time())) # bắt đầu kiểm tra
        Auboi5Robot.initialize() # khởi tạo hệ thống
        robot = Auboi5Robot() # Tạo lớp điều khiển cánh tay robot
        handle = robot.create_context() # tạo ngữ cảnh
        logger.info("robot.rshd={0}".format(handle)) # bối cảnh in
        try:
            ip = '192.168.3.39'
            port = 8899
            result = robot.connect(ip, port)
            
            if result != RobotErrorType.RobotError_SUCC:
                logger.info("connect server{0}:{1} failed.".format(ip, port))
            else:
                
                robot.enable_robot_event()
                robot.init_profile()
                user_coord = {'coord_type': RobotCoordType.Robot_World_Coordinate,
                'calibrate_method': RobotCoordCalMethod.CoordCalMethod_xOxy,
                'calibrate_points':
                {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), #Khởi tạo ban đầu
                "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
                'tool_desc':
                {"pos": (0.047283, 0.02, 0.119),
                    "ori": (1.0, 0.0, 0.0, 0.0)}
                            }
                self.throw_point= robot.get_current_waypoint()

                file_path = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\throw_point.txt"
                with open(file_path, 'a') as file:
                    data_str = str(self.throw_point).replace("'", '"')
                    
                    file.write(data_str + '\n')
                
                
        except KeyboardInterrupt:
            robot.move_stop()
            
        except RobotError as e:
            logger.error("robot Event:{0}".format(e))
            
        finally:
            print("haha3")
            if robot.connected:
                robot.disconnect()
            Auboi5Robot.uninitialize()
            #self.info.append("Dữ liệu điểm thả vật đã được lưu vào:", file_path)
            #print("Dữ liệu điểm thả vật đã được lưu vào:", file_path)
        

    def add_home_point(self):
        logger_init() # Khởi tạo bộ ghi nhật ký
        logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time())) # bắt đầu kiểm tra
        Auboi5Robot.initialize() # khởi tạo hệ thống
        robot = Auboi5Robot() # Tạo lớp điều khiển cánh tay robot
        handle = robot.create_context() # tạo ngữ cảnh
        logger.info("robot.rshd={0}".format(handle)) # bối cảnh in
        try:
            ip = '192.168.3.39'
            port = 8899
            result = robot.connect(ip, port)
            
            if result != RobotErrorType.RobotError_SUCC:
                logger.info("connect server{0}:{1} failed.".format(ip, port))
            else:
                
                robot.enable_robot_event()
                robot.init_profile()
                user_coord = {'coord_type': RobotCoordType.Robot_World_Coordinate,
                'calibrate_method': RobotCoordCalMethod.CoordCalMethod_xOxy,
                'calibrate_points':
                {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), #Khởi tạo ban đầu
                "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
                'tool_desc':
                {"pos": (0.047283, 0.02, 0.119),
                    "ori": (1.0, 0.0, 0.0, 0.0)}
                            }
                self.home_point= robot.get_current_waypoint()
                file_path = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\home_point.txt"

                with open(file_path, 'a') as file:
                    data_str = str(self.home_point).replace("'", '"')
                    
                    file.write(data_str + '\n')
                
        except KeyboardInterrupt:
            robot.move_stop()
            
        except RobotError as e:
            logger.error("robot Event:{0}".format(e))
            
        finally:
            if robot.connected:
                robot.disconnect()
            Auboi5Robot.uninitialize()
            #self.info.append("Dữ liệu điểm Home đã được lưu vào:", file_path)
            #print("Dữ liệu điểm Home đã được lưu vào:", file_path)


        

    # def get_joint_info(self):
    #     file_dialog = QFileDialog()
    #     file_path, _ = file_dialog.getOpenFileName(self, "Open File", "", "Text files (*.txt)")
        
    #     if file_path:
    #         with open(file_path, 'r') as file:
    #             content = file.readlines()

    #         self.joint_info = []
    #         for line in content:
    #             joint_data = ast.literal_eval(line)
    #             self.joint_info.append(joint_data['joint'])

    #         joint_arrays = []
    #         for i, line_edit in enumerate(self.line_edits):
    #             if i < len(self.joint_info):
    #                 joint_values = self.joint_info[i]
    #                 joint_str = ', '.join([str(val) for val in joint_values])
    #                 line_edit.setText(joint_str)
    #                 joint_array = ', '.join([str(val) for val in joint_values])
    #                 joint_arrays.append(joint_array)

            # first_joint_array = joint_arrays[0]
            # second_joint_array = joint_arrays[1]
            # third_joint_array = joint_arrays[2]
            # value_strs1 = first_joint_array.split(', ') # Tách chuỗi thành danh sách các chuỗi con dựa trên dấu phẩy
            # value_strs2 = second_joint_array.split(', ')
            # value_strs3 = third_joint_array.split(', ')
            # joint_point1 = [float(val) for val in value_strs1] # Chuyển các chuỗi con thành các giá trị số float và lưu vào mảng
            # joint_point2 = [float(val) for val in value_strs2]
            # joint_point3 = [float(val) for val in value_strs3]
            # self.joint_point1 = tuple(joint_point1)
            # self.joint_point2 = tuple(joint_point2)
            # self.joint_point3 = tuple(joint_point3)
            # print("Thêm điểm hệ tọa độ thành công")



    def get_joint_info2(self):
        file_path = r"D:\Downloads\ketnoirobot\point_data.txt"
        if file_path:
            with open(file_path, 'r') as file:
                content = file.readlines()

            self.joint_info = []
            for line in content:
                joint_data = ast.literal_eval(line)
                self.joint_info.append(joint_data['joint'])

            joint_arrays = []
            for i, line_edit in enumerate(self.line_edits):
                if i < len(self.joint_info):
                    joint_values = self.joint_info[i]
                    joint_str = ', '.join([str(val) for val in joint_values])
                    line_edit.setText(joint_str)
                    joint_array = ', '.join([str(val) for val in joint_values])
                    joint_arrays.append(joint_array)

            first_joint_array = joint_arrays[0]
            second_joint_array = joint_arrays[1]
            third_joint_array = joint_arrays[2]
            value_strs1 = first_joint_array.split(', ') # Tách chuỗi thành danh sách các chuỗi con dựa trên dấu phẩy
            value_strs2 = second_joint_array.split(', ')
            value_strs3 = third_joint_array.split(', ')
            joint_point1 = [float(val) for val in value_strs1] # Chuyển các chuỗi con thành các giá trị số float và lưu vào mảng
            joint_point2 = [float(val) for val in value_strs2]
            joint_point3 = [float(val) for val in value_strs3]
            self.joint_point1 = tuple(joint_point1)
            self.joint_point2 = tuple(joint_point2)
            self.joint_point3 = tuple(joint_point3)
            #self.info.append("Thêm điểm hệ tọa độ thành công", file_path)
            #print("Thêm điểm hệ tọa độ thành công",file_path)



    def run_robot(self):
        self.x = float(self.line_edit_x.text())
        self.y = float(self.line_edit_y.text())
        self.z = float(self.line_edit_z.text())
        # Đọc nội dung từ file .txt
        file_home = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\home_point.txt"  # Thay đổi đường dẫn đến tệp .txt thực tế
        with open(file_home, 'r') as file:
            content = file.read()
        self.home_point = json.loads(content)

        file_throw = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\throw_point.txt"  # Thay đổi đường dẫn đến tệp .txt thực tế
        with open(file_throw, 'r') as file:
            content = file.read() 
        self.throw_point = json.loads(content)
        
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
                user_coord = {'coord_type': RobotCoordType.Robot_World_Coordinate,
                'calibrate_method': RobotCoordCalMethod.CoordCalMethod_xOxy,
                'calibrate_points':
                {"point1": self.joint_point1,
                "point2": self.joint_point2,
                "point3": self.joint_point3},
                          
                'tool_desc':
                {"pos": (-0.016307, 0.0171, 0.086615),
                    "ori": (1.0, 0.0, 0.0, 0.0)}
                }           
                self.origin= robot.get_current_waypoint()
                user_tool={"pos": (-0.016307, 0.0171, 0.086615),
                    "ori": (1.0, 0.0, 0.0, 0.0)}
                ori2=(-1.59864527158  ,-0.00681940281, -0.1103831391)

                point1=(self.x,self.y,0.2)
                point2=(self.x,self.y,self.z)
        
                robot.set_user_coord(user_coord) 
                robot.set_tool_kinematics_param(user_tool)
               
                kq2=robot.user_to_base(point2,robot.rpy_to_quaternion(ori2),user_coord,user_tool)
                kq1=robot.user_to_base(point1,robot.rpy_to_quaternion(ori2),user_coord,user_tool)
                #print(self.home_point['joint'])
                #robot.move_line(self.home_point['joint'])

                robot.set_relative_offset_on_user((0,0,0.05),(1,0,0,0),user_coord)
                
                robot.move_to_target_in_cartesian(kq2['pos'],rpy_xyz=(0,0, 0))
                robot.set_relative_offset_on_user((0,0,0),(1,0,0,0),user_coord)
                robot.move_to_target_in_cartesian(kq2['pos'],rpy_xyz=(0,0, 0))
                #robot.set_board_io_status(5,"U_DO_00",1)
                robot.set_relative_offset_on_user((0,0,0.05),(1,0,0,0),user_coord)
                robot.move_to_target_in_cartesian(kq2['pos'],rpy_xyz=(0,0, 0))
                
                robot.move_line(self.throw_point['joint'])
                robot.set_relative_offset_on_user((0,0,0),(1,0,0,0),user_coord)
                robot.move_line(self.throw_point['joint'])
                robot.set_relative_offset_on_user((0,0,0.05),(1,0,0,0),user_coord)
                robot.move_line(self.throw_point['joint'])
                robot.set_relative_offset_on_user((0,0,0),(1,0,0,0),user_coord)
                input()
                robot.move_line(self.home_point['joint'])
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


    def run_robot_1(self):
        self.x = float(self.line_edit_x.text())
        self.y = float(self.line_edit_y.text())
        self.z = float(self.line_edit_z.text())
        # Đọc nội dung từ file .txt
        file_home = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\home_point.txt"  # Thay đổi đường dẫn đến tệp .txt thực tế
        with open(file_home, 'r') as file:
            content = file.read()
        self.home_point = json.loads(content)

        file_throw = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\throw_point.txt"  # Thay đổi đường dẫn đến tệp .txt thực tế
        with open(file_throw, 'r') as file:
            content = file.read() 
        self.throw_point = json.loads(content)
        
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
                point = [self.x,self.y,self.z]
                print(point[0])

                robot.set_end_max_line_velc(0.1)
                robot.set_joint_maxvelc((0.1,0.1,0.1,0.1,0.1,0.1))

                print(self.x, self.y, self.z)

                robot.set_base_coord()
                robot.set_tool_kinematics_param(user_tool)
        
        

                robot.move_line(self.home_point['joint'])

                #robot.set_relative_offset_on_base((0,0,0.05),(1,0,0,0))
                
                robot.move_to_target_in_cartesian([point[0],point[1],point[2]],rpy_xyz=(89.585632,-7.597886, 89.489021))
                #robot.set_relative_offset_on_base((0,0,0),(1,0,0,0))
                robot.move_to_target_in_cartesian([0.725,-0.3,0.5],rpy_xyz=(89.585632,-7.597886, 89.489021))
                # robot.set_board_io_status(5,"U_DO_00",1)
                #robot.set_relative_offset_on_base((0,0,0.05),(1,0,0,0))
                #robot.move_to_target_in_cartesian([point[0],point[1],point[2]],rpy_xyz=(89.585632,-7.597886, 89.489021))

                robot.move_line(self.throw_point['joint'])
                robot.set_relative_offset_on_base((0,0,0),(1,0,0,0))
                robot.move_line(self.throw_point['joint'])
                robot.set_relative_offset_on_base((0,0,0.1),(1,0,0,0))
                robot.move_line(self.throw_point['joint'])
                robot.set_relative_offset_on_base((0,0,0),(1,0,0,0))
                input()
                
                robot.move_line(self.home_point['joint'])
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



    def run_robot_auto(self):
       # Đọc nội dung từ file .txt
        file_home = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\home_point.txt"  # Thay đổi đường dẫn đến tệp .txt thực tế
        with open(file_home, 'r') as file:
            content = file.read()
        self.home_point = json.loads(content)

        file_throw = r"C:\Users\thanhgyza\Downloads\VisionApp_NoRobot\throw_point.txt"  # Thay đổi đường dẫn đến tệp .txt thực tế
        with open(file_throw, 'r') as file:
            content = file.read()
        self.throw_point = json.loads(content)

        file_obj = "result/obj_rotated.txt"   
        with open(file_obj, "r") as file:
            content = file.read()
        coordinate_pairs = content.split(';')
        coordinates_list = []
        for pair in coordinate_pairs:
            if pair:
                values = pair.strip("[]").split(',')
                float_values = [float(value) for value in values]
                coordinates_list.append(float_values)

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
                user_coord = {'coord_type': RobotCoordType.Robot_World_Coordinate,
                'calibrate_method': RobotCoordCalMethod.CoordCalMethod_xOxy,
                'calibrate_points':
                {"point1": self.joint_point1,
                "point2": self.joint_point2,
                "point3": self.joint_point3},
                'tool_desc':
                {"pos": (-0.016307, 0.0171, 0.086615),
                    "ori": (1.0, 0.0, 0.0, 0.0)}
                            }
                self.origin= robot.get_current_waypoint()
                user_tool={"pos": (-0.016307, 0.0171, 0.086615),
                    "ori": (1.0, 0.0, 0.0, 0.0)}
                ori2=(-1.5730576102826257  ,0.02451737304108541, -0.02558942408083195)
                robot.move_line(self.home_point['joint'])
                # Tiến hành di chuyển và thao tác với robot theo tọa độ
                for idx, coordinates in enumerate(coordinates_list, start=1):
                    x, y = coordinates
                    z = 0.1
                    point = (x, y, z)
                    robot.set_user_coord(user_coord) 
                robot.set_tool_kinematics_param(user_tool)
               
                kq2=robot.user_to_base(point,robot.rpy_to_quaternion(ori2),user_coord,user_tool)
                print(self.home_point['joint'])
                robot.move_line(self.home_point['joint'])

                robot.set_relative_offset_on_user((0,0,0.05),(1,0,0,0),user_coord)
                
                robot.move_to_target_in_cartesian(kq2['pos'],rpy_xyz=(-90.129562, 1.404742, -1.466166))
                robot.set_relative_offset_on_user((0,0,0),(1,0,0,0),user_coord)
                robot.move_to_target_in_cartesian(kq2['pos'],rpy_xyz=(-90.129562, 1.404742, -1.466166))
                #robot.set_board_io_status(5,"U_DO_00",1)
                robot.set_relative_offset_on_user((0,0,0.05),(1,0,0,0),user_coord)
                robot.move_to_target_in_cartesian(kq2['pos'],rpy_xyz=(-90.129562, 1.404742, -1.466166))
                
                robot.move_line(self.throw_point['joint'])
                robot.set_relative_offset_on_user((0,0,0),(1,0,0,0),user_coord)
                robot.move_line(self.throw_point['joint'])
                robot.set_relative_offset_on_user((0,0,0.05),(1,0,0,0),user_coord)
                robot.move_line(self.throw_point['joint'])
                robot.set_relative_offset_on_user((0,0,0),(1,0,0,0),user_coord)
            robot.move_line(self.home_point['joint'])
            robot.disconnect() # Ngắt kết nối robot

        except KeyboardInterrupt:
            robot.move_stop()
    
            
        except RobotError as e:
            logger.error("robot Event:{0}".format(e))

        finally:
            if robot.connected:
                robot.disconnect()
            Auboi5Robot.uninitialize()
            print("------------Kết thúc chương trình chạy AUTO-------------")
       
    def run_tasks_concurrently(self):
        process1 = multiprocessing.Process(target=self.init_ui)
        process2 = multiprocessing.Process(target=self.add_origin)
        process3 = multiprocessing.Process(target=self.get_joint_info2)
        process4 = multiprocessing.Process(target=self.add_home_point)
        process5 = multiprocessing.Process(target=self.add_throw_point)
        process6 = multiprocessing.Process(target=self.run_robot)
        # process7 = multiprocessing.Process(target=self.run_robot_auto)

        process1.start()
        process2.start()
        process3.start()
        process4.start()
        process5.start()
        process6.start()
        # process7.start()

        process1.join()
        process2.join()
        process3.join()
        process4.join()
        process5.join()
        process6.join()
        # process7.join()


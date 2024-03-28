from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np
import cv2
import concurrent.futures
import threading
import multiprocessing

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
results = []  # Mảng để lưu kết quả tính toán
lock = threading.Lock()  # Lock để đồng bộ hóa truy cập vào results
joint_ref= [0,0,0,0,0,0]

# Hàm chạy trong mỗi luồng
def worker(points):
    for i,point in enumerate(points):
        T[0][3] = point[0]
        T[1][3] = point[1]
        T[2][3] = point[2]

        if i == 0:
            joint = robot.ikine_LM(T, [0,0,0,0,0,0], mask=mask, ilimit=50)
            joint_ref= joint.q
            results.append(joint.q)
        else:
            joint1 = robot.ikine_LM(T, [0,0,0,0,0,0], mask=mask, ilimit=50)
            joint_ref = joint1.q
            results.append(joint1.q)    
    
def img_processing(path):
    img = cv2.imread(path)

    imageSize = img.shape[:2]
    print(f"imgsize: {imageSize[0]}x{imageSize[1]} pixels")

    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, binaryImg = cv2.threshold(grayImg, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

    boundaryPoints, _ = cv2.findContours(binaryImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Vẽ đường viền lên ảnh gốc
    cv2.drawContours(img, [boundaryPoints[0]], 0, (255, 255, 0), 1)

    for point in boundaryPoints[0]:
        x, y = point[0]
        cv2.circle(img, (x, y), 3, (0, 0, 255), -1)

    # Hiển thị ảnh gốc với đường viền đã vẽ
    cv2.imshow("Image with Contour", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return boundaryPoints

def matrix_conver(point1, point2, point3):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    x3, y3, z3 = point3
    
    # Tính vector hướng của X
    direction_x = np.array([x2 - x1, y2 - y1, z2 - z1])
    direction_x = direction_x / np.linalg.norm(direction_x)
    
    # Tính vector hướng của Z
    direction_z = np.cross(direction_x, np.array([x3 - x1, y3 - y1, z3 - z1]))
    direction_z = direction_z / np.linalg.norm(direction_z)
    
    # Tính vector hướng của Y
    direction_y = np.cross(direction_z, direction_x)
    direction_y = direction_y / np.linalg.norm(direction_y)
    
    intersection_point = np.array([[direction_x[0], direction_y[0], direction_z[0], x1],
                                   [direction_x[1], direction_y[1], direction_z[1], y1],
                                   [direction_x[2], direction_y[2], direction_z[2], z1],
                                   [0, 0, 0, 1]])
    
    return intersection_point

'''
    a = [0 , 0.647, 0.6005, 0, 0, 0 ];
    alpha = [pi/2, 0, 0, -pi/2, pi/2,0];
    d = [0.1632, 0.197, -0.1235, 0.1278, 0.1025, 0.094+0.105];
    theta = [0, pi/2, 0, -pi/2, 0, 0];
    '''
# Định nghĩa thông số của robot
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

# xử lý ảnh:
path = r"C:\Users\thanhgyza\Downloads\do_an_2_matlab (1)\image3.png"
boundaryPoints = img_processing(path)
pos_img=[]
for boundaryPoint in boundaryPoints:
    pos = np.flip(boundaryPoint, axis=2).swapaxes(1, 2)
    pos_img.append(pos)

print("Boundary points:")
print(boundaryPoints)

## chuyển tọa độ ảnh sang tọa base robot
# định dạng lại mảng 2 chiều
point_2d = pos.reshape(-1, 2)
# thêm 1 cột 0
point_3d = np.concatenate((point_2d, np.zeros((point_2d.shape[0], 1))), axis=1)

# hệ số calib camera
a=0.002; # m/pixel
point_img= point_3d*a

# Thêm cột giá trị bằng 1 vào mảng
point_img = np.concatenate((point_img, np.ones((point_img.shape[0], 1))), axis=1)
#print(point_img)

A = [0.725, 0.5, 1]; #x1
B = [0.725, 0.5, 0.9];   #x2
C= [0.725,0.4,1];    #y1
matrix_base= matrix_conver(A,B,C) # ma trận chuyển đổi về base
#print(matrix_base)

point_conver=[]
for point in point_img:
    result = np.dot(matrix_base, point)
    point_conver.append(result)

print(point_conver)

# Chuyển đổi mảng thành chuỗi
formatted_arr = '\n'.join([','.join(map(str, sublist)) for sublist in point_conver])
# In chuỗi đã định dạng
# print(formatted_arr)
file_path= r'C:\Users\thanhgyza\Downloads\vrep_python_da\point_xla.txt'
with open(file_path, 'w') as f:
    f.write(formatted_arr)

data = np.loadtxt(file_path, delimiter=',')
print(len(data))  # trích xuất tọa độ x: data[0][0]

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
robot = DHRobot(links, name='MyRobot')

# # Tính toán ma trận biến đổi của robot cho một bộ giá trị góc
# joint = [0, 0, 0, 0, 0, 0]  # Giá trị góc cho mỗi khớp
# T = robot.fkine(joint)  # Tính toán ma trận biến đổi của robot
# #print(T)

# # Vị trí và hướng mong muốn của công cụ (end-effector)
# position = [0.76, 0.3, 0.5]  # Vị trí (x, y, z)
# orientation = SE3.RPY([0, 0, np.pi/2])  # Hướng (roll, pitch, yaw)
# #print(orientation)
# config = [0,0,0,0,0,0]
# mask= [1,1,1,1,1,1]

# # In kết quả
# print("Các giá trị góc của các khớp:")

# matran= create_matrix(0,np.pi/2,0,0.76,0.3,0.5)
# print(matran[1][3])
# q = robot.ikine_LM(matran, config, mask=mask, ilimit=50)
# print(q.q)
# # test lại
# qq= robot.fkine(q.q)
# print(qq)

# file_path = r'C:\Users\thanhgyza\Downloads\do_an_2_matlab (1)\point_xla.txt'
# # Đọc dữ liệu từ file văn bản
# data = np.loadtxt(file_path, delimiter=',')
# print(data)

home_joint= [0,0,0,0,0,0]
T_home = robot.fkine(home_joint)  # Tính toán ma trận biến đổi của robot
print(T_home)
joint_ref= [0,0,0,0,0,0]
T= create_matrix(np.pi/2,-np.pi/4,np.pi/2,0,-0.4003,1.513)

config = [0,0,0,0,0,0]
mask= [1,1,1,1,1,1]
# results = []  # Mảng để lưu kết quả tính toán
# # chuyển thành các joint
# for i in range(len(point_conver)):
#     T[0][3]= point_conver[i][0]
#     T[1][3]= point_conver[i][1]
#     T[2][3]= point_conver[i][2]
#     if (i==0):
#         joint= robot.ikine_LM(T, config, mask=mask, ilimit=50)
#         joint_ref= joint.q
#         results.append(joint.q)
#     else:
#         joint1=robot.ikine_LM(T, joint_ref, mask=mask, ilimit=50)
#         joint_ref=joint1.q
#         results.append(joint1.q)
# Bắt đầu đo thời gian
start_time = time.time()

# Chia danh sách point_conver thành các phần nhỏ
# chunk_size = len(point_conver) // threading.active_count()
chunk_size= 200

chunks = [point_conver[i:i+chunk_size] for i in range(0, len(point_conver), chunk_size)]
print(chunks[0])

# Tạo và khởi chạy các luồng
threads = []
for chunk in chunks:
    thread = threading.Thread(target=worker, args=(chunk,))
    thread.start()
    threads.append(thread)

# Chờ tất cả các luồng hoàn thành
for thread in threads:
    thread.join()

print(len(results))
# Kết thúc đo thời gian
end_time = time.time()
# Tính thời gian chạy
execution_time = end_time - start_time
# In kết quả
print("Thời gian chạy:", execution_time, "giây")

result_strings = []
for i in range(len(point_conver)):
    # Chuyển đổi từng phần tử thành chuỗi và thêm vào mảng result_strings
    result_string = "1  " + "  ".join(str(value) for value in results[i]) + "  1"
    result_strings.append(result_string)

# In kết quả
for result_string in result_strings: 
    print(result_string) 

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    vrep.simxSetStringSignal(clientID,'connect_status','ok',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'in4_ip','127.0.0.1',vrep.simx_opmode_oneshot)
    time.sleep(0.1)
    print ('Connected to remote API server')

    for i in range(len(result_strings)):
        numberString= result_strings[i]
        vrep.simxSetStringSignal(clientID,'numberString',numberString,vrep.simx_opmode_oneshot)
        while True:
            [errorCode, stringData] = vrep.simxGetStringSignal(clientID, 'phanhoi', vrep.simx_opmode_blocking)
            stringData = stringData.decode() 
            #print(stringData)
            if stringData == '111':
                print('send data joint ok')
                vrep.simxSetStringSignal(clientID, 'phanhoi', '', vrep.simx_opmode_oneshot)
                #print(1)
                break


    vrep.simxSetStringSignal(clientID, 'ketthuc', '11', vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

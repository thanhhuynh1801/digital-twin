# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

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
import numpy
from aubo_kienamtics import Aubo_kinematics


ak47=Aubo_kinematics()
home_point= [0,0,0,0,0,0]
joint_ref= [0,0,0,0,0,0]
T= ak47.aubo_forward(home_point)
print(T)
file_path = r'C:\Users\thanhgyza\Downloads\do_an_2_matlab (1)\point_xla.txt'
# Đọc dữ liệu từ file văn bản
data = numpy.loadtxt(file_path, delimiter=',')
print(len(data))
results = []  # Mảng để lưu kết quả tính toán
# chuyển thành các joint
for i in range(len(data)):
    T[3]= data[i][0]
    T[7]= data[i][1]
    T[11]=data[i][2]
    if (i==0):
        joint= ak47.GetInverseResult(T,ak47.degree_to_rad(home_point))
        joint_ref= joint
        results.append(joint)
    else:
        joint1=ak47.GetInverseResult(T,ak47.degree_to_rad(home_point))
        joint_ref=joint1
        results.append(joint1)
result_strings = []
for i in range(len(data)):
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

import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

def findIntersectionPoint(point1, point2, point3):
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
def draw_coordinates(x, y, z, mt):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # x_axis
    ax.plot([x, (1 * mt[0, 0] + x)], [y, (1 * mt[1, 0] + y)], [z, (1 * mt[2, 0] + z)], 'r', linewidth=2)
    ax.text((1 * mt[0, 0] + x), (1 * mt[1, 0] + y), (1 * mt[2, 0] + z), 'x', horizontalalignment='right', fontsize=11)

    # y_axis
    ax.plot([x, (1 * mt[0, 1] + x)], [y, (1 * mt[1, 1] + y)], [z, (1 * mt[2, 1] + z)], 'g', linewidth=2)
    ax.text((1 * mt[0, 1] + x), (1 * mt[1, 1] + y), (1 * mt[2, 1] + z), 'y', horizontalalignment='right', fontsize=11)

    # z_axis
    ax.plot([x, (1 * mt[0, 2] + x)], [y, (1 * mt[1, 2] + y)], [z, (1 * mt[2, 2] + z)], 'b', linewidth=2)
    ax.text((1 * mt[0, 2] + x), (1 * mt[1, 2] + y), (1 * mt[2, 2] + z), 'z', horizontalalignment='right', fontsize=11)

    plt.show()

T00 = np.array([[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])
draw_coordinates(0,0,0,T00)

# # xla:
# path = r"C:\Users\thanhgyza\Downloads\do_an_2_matlab (1)\image1.png"
# boundaryPoints = img_processing(path)
# pos_img=[]
# for boundaryPoint in boundaryPoints:
#     pos = np.flip(boundaryPoint, axis=2).swapaxes(1, 2)
#     pos_img.append(pos)

# print("Boundary points:")
# print(boundaryPoints)

# ## chuyển tọa độ ảnh sang tọa base robot
# # định dạng lại mảng 2 chiều
# point_2d = pos.reshape(-1, 2)
# # thêm 1 cột 0
# point_3d = np.concatenate((point_2d, np.zeros((point_2d.shape[0], 1))), axis=1)

# # hệ số calib camera
# a=0.002; # m/pixel
# point_img= point_3d*a

# # Thêm cột giá trị bằng 1 vào mảng
# point_img = np.concatenate((point_img, np.ones((point_img.shape[0], 1))), axis=1)
# print(point_img)

# A = [0.725, 0.5, 1]; #x1
# B = [0.725, 0.5, 0.9];   #x2
# C= [0.725,0.4,1];    #y1
# matrix_base= findIntersectionPoint(A,B,C) # ma trận chuyển đổi về base
# print(matrix_base)

# point_conver=[]
# for point in point_img:
#     result = np.dot(matrix_base, point)
#     point_conver.append(result)

# print(point_conver)
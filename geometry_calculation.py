# import cv2
import numpy as np
import math
import time
import random
# import setup_camera as cam
from sklearn.cluster import KMeans
# import matplot_show as matplot

def get_plane_equation_from_points(para1, para2, para3):  
    x1, y1, z1 = para1[:3]
    x2, y2, z2 = para2[:3]
    x3, y3, z3 = para3[:3]
    a1 = x2 - x1 
    b1 = y2 - y1 
    c1 = z2 - z1 
    a2 = x3 - x1 
    b2 = y3 - y1 
    c2 = z3 - z1 
    a = b1 * c2 - b2 * c1 
    b = a2 * c1 - a1 * c2 
    c = a1 * b2 - b1 * a2 
    d = (- a * x1 - b * y1 - c * z1) 
    return a, b, c, d

def get_plane_equation_from_point_normal_vector(normal_vector, point):
    x,y,z = normal_vector[:3]
    A, B, C = point[:3]
    return x, y, z, -(x*A + y*B + z*C)

def get_angle_between_planes(para1, para2):
    vector_1 = para1[:3]
    vector_2 = para2[:3]
    # d = (a1 * a2) + (b1 * b2) + (c1 * c2) 
    # e1 = math.sqrt( a1 * a1 + b1 * b1 + c1 * c1) 
    # e2 = math.sqrt( a2 * a2 + b2 * b2 + c2 * c2) 
    # d = d / (e1 * e2)
    # d = round(d,1) 
    # A = math.degrees(math.acos(d)) 

    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = math.degrees(np.arccos(dot_product))


    return angle

def get_line_intersection_vector_from_two_planes(A, B):
    a,b,c = np.array(A)[:3].tolist()
    x,y,z = np.array(B)[:3].tolist()
    return [y*c - z*b, z*a - x*c, x*b - y*a]

def find_2d_corner(image, mask,object_depth,point_cloud,axis, start_loop, end_loop, direct):

    for i in range(start_loop, end_loop, direct):
        if axis == 'x':
            mask_collumn = mask[:,i]
        else:
            mask_collumn = mask[i]
        if mask_collumn.max() == 1:
            position_arr = np.where(mask_collumn == 1)
            temp_arr = []
            for temp in position_arr[0]:
                if axis == 'x':
                    if point_cloud[temp][i][0] == 0:
                        continue
                    try:
                        cur_dist = object_depth[temp][i]
                        if temp_dist > cur_dist:
                            temp_dist = cur_dist
                            box_corner = [i, temp]
                    except:
                        temp_dist = object_depth[temp][i]
                    
                    temp_arr.append(temp)
                else:
                    if point_cloud[i][temp][0] == 0:
                        continue
                    try:
                        cur_dist = object_depth[i][temp]
                        if temp_dist > cur_dist:
                            temp_dist = cur_dist
                            box_corner = [temp, i]
                    except:
                        temp_dist = object_depth[i][temp]

                    temp_arr.append(temp)

            if not temp_arr:
                continue
            try:
                box_corner
            except:
                continue
            break
    # image = cv.circle(image,tuple(box_corner), 5, (0,0,255), -1)
    return box_corner, image

def get_distance_point_plane(M, alpha):
    a1, b1, c1, d1 = alpha
    a2, b2, c2 = M
    num = (a1 * a2) + (b1 * b2) + (c1 * c2) + d1
    denom = math.sqrt( a1 * a1 + b1 * b1 + c1 * c1) 
    return num/denom

def get_distance_point_line_3d(M, direction_vector, P):
    # a1, b1, c1 = P
    # a2, b2, c2 = M
    x,y,z = direction_vector
    MP = M - P
    MPS = np.array([MP[1]*z - MP[2]*y, -MP[0]*x + MP[2]*z, MP[1]*x - MP[0]*y])
    num = np.sqrt(np.power(MPS[0], 2)+ np.power(MPS[1], 2)+ np.power(MPS[2], 2))
    nom = np.sqrt(np.power(x, 2) + np.power(y, 2) + np.power(z, 2))
    return num/nom

def get_distance_two_point_3d(A, B):
    a1, b1, c1 = A
    a2, b2, c2 = B
    denom = math.sqrt( (a1 - a2)*(a1 - a2) + (b1 - b2)*(b1 - b2) + (c1 - c2)*(c1 - c2))
    return denom

def get_distance_two_point_2d(A, B):
    a1, b1 = A
    a2, b2 = B
    denom = math.sqrt((a1 - a2)*(a1 - a2) + (b1 - b2)*(b1 - b2))
    return denom

import math

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.degrees(math.acos(dotproduct(v1, v2) / (length(v1) * length(v2))))

def get_angle_two_line_3d(direction_vector_1, direction_vector_2):
    m1 = np.sqrt(np.power(direction_vector_1[0],2)+np.power(direction_vector_1[1],2)+np.power(direction_vector_1[2],2))
    m2 = np.sqrt(np.power(direction_vector_2[0],2)+np.power(direction_vector_2[1],2)+np.power(direction_vector_2[2],2))

    cosn = np.sum(direction_vector_1*direction_vector_2)/(m1*m2)
    return math.degrees(math.acos(cosn)) 

def get_satellite_points_of_center_2d(center_point, normal_vector_2d, distance_to_center_point):
    line_equation = (normal_vector_2d[0], normal_vector_2d[1], -(normal_vector_2d[0]*center_point[0] + normal_vector_2d[1]*center_point[1]))
    x_value = (-line_equation[1]/line_equation[0], -line_equation[2]/line_equation[0])
    raw_value = np.power(x_value[1] - center_point[0],2) + np.power(center_point[1],2) - np.power(distance_to_center_point,2)
    level_1_value = 2*(x_value[0])*(x_value[1]-center_point[0]) - 2*(center_point[1])
    level_2_value = np.power(x_value[0],2) + 1

    # print("Handle Quadratic Equation: ax^2 + bx + c = 0")
    a = level_2_value
    b = level_1_value
    c = raw_value

    if a == 0:
        if b == 0:
            if c == 0:
                print("Countless!")
            else:
                print("impossible equation!")
        else:
            if c == 0:
                print("x = 0")
            else:
                print("x = ", -c / b)
    else:
        delta = np.power(b,2) - 4 * a * c
        if delta < 0:
            print("impossible equation!!")
        elif delta == 0:
            print("x = ", -b / (2 * a))
        else:
            # print("2 values!")
            y1 = int((-b - np.sqrt(delta)) / (2 * a))
            y2 = int((-b + np.sqrt(delta)) / (2 * a))
            x1 = int(x_value[0] * y1 + x_value[1])
            x2 = int(x_value[0] * y2 + x_value[1])
            return [x1,y1],[x2,y2]

def get_mask_boundary(mask_polygon):
    start_time = time.time()
    left_box_corner = right_box_corner = mask_polygon[0][0]
    top_box_corner = button_box_corner = mask_polygon[0][1]
    for coor in mask_polygon:
        if left_box_corner > coor[0]:
            left_box_corner = coor[0]
        if right_box_corner < coor[0]:
            right_box_corner = coor[0]
        if top_box_corner > coor[1]:
            top_box_corner = coor[1]
        if button_box_corner < coor[1]:
            button_box_corner = coor[1]
    end_time = time.time()
    # print('mask_polygon ellapsed time =', end_time - start_time)
    return left_box_corner, right_box_corner, top_box_corner, button_box_corner

def get_ramdom_rgb():
    r = random.randint(0,255)
    g = random.randint(0,255)
    b = random.randint(0,255)
    return (r,g,b)

def get_coordinate_system_root(iniX, iniY, iniZ, rangeV, stepV, distA, distB, distC, A_pos, B_pos, C_pos):
    for x in range(iniX,iniX + rangeV,stepV):
        for y in range(iniY,iniY + rangeV,stepV):
            for z in range(iniZ,iniZ + rangeV,stepV):
                temp_dist_A = abs(get_distance_two_point_3d([x,y,z],A_pos)- distA)
                temp_dist_B = abs(get_distance_two_point_3d([x,y,z],B_pos)- distB)
                temp_dist_C = abs(get_distance_two_point_3d([x,y,z],C_pos)- distC) 
                try:
                    dist
                except:
                    dist = temp_dist_A + temp_dist_B + temp_dist_C

                if dist > temp_dist_A + temp_dist_B + temp_dist_C:
                    dist = temp_dist_A + temp_dist_B + temp_dist_C
                    print([x,y,z])
                    print(dist)

def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, math.cos(theta),-math.sin(theta)],
                   [ 0, math.sin(theta), math.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-math.sin(theta), 0, math.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                   [ math.sin(theta), math.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

import queue
rotate_degree = queue.Queue()
def speed_up(object_in_world, object_in_eye_T, phi, iniY, iniZ, rangeV, stepV):
    
    # phi = phi / 10
    for theta in range(1780,1850,1):
        theta = theta/10
        for beta in range(880,950,1):
            beta = beta /10
            angle_to_T = 0
            R = Rz(math.radians(beta)) * Ry(math.radians(theta)) * Rx(math.radians(phi))
            for index, point_in_world in enumerate(object_in_world):
                x,y,z = point_in_world
                v2 = R * np.array([[x],[y],[z]])
                # print(np.round(v2, decimals=2))
                
                temp_point = v2.transpose().reshape(-1,).tolist()[0]
                angle_to_T = angle_to_T + abs(get_angle_two_line_3d(np.array(temp_point), np.array(object_in_eye_T[index])))
            # dist_to_T = cal.get_distance_two_point_3d(temp_point, object_in_eye_T)
            try:
                temp_dist
            except:
                temp_dist = angle_to_T
                temp_phi, temp_theta, temp_beta = [phi, theta, beta]
            if temp_dist > angle_to_T:
                temp_dist = angle_to_T
                temp_phi, temp_theta, temp_beta = [phi, theta, beta]
    print([temp_dist, temp_phi, temp_theta, temp_beta])
    rotate_degree.put([temp_dist, temp_phi, temp_theta, temp_beta])


def get_real_point(center_point_3d, recognize_distance, accurated_r_distance):
    top = math.pow(accurated_r_distance,2) - math.pow(accurated_r_distance - recognize_distance,2) + math.pow(center_point_3d[0],2) + math.pow(center_point_3d[1],2)
    button = (2*math.pow(center_point_3d[0],2)/center_point_3d[1])  + 2* center_point_3d[1]
    y = top/button
    x = center_point_3d[0]*y/center_point_3d[1]
    return [x,y, center_point_3d[2]]

def error_compensatiton(center_point_3d):
    error_para = 1
    recognized_angle = get_distance_two_point_2d([0,0], center_point_3d[:2])
    recognized_angle = math.degrees(math.atan(recognized_angle/center_point_3d[2]))
    add_up_angle = recognized_angle * error_para
    accurated_r_distance =  center_point_3d[2]*math.tan(add_up_angle*math.pi/180)
    recognize_distance = get_distance_two_point_2d([0,0],[center_point_3d[0],center_point_3d[1]])
    center_point_3d = get_real_point(center_point_3d,recognize_distance,accurated_r_distance)

    return center_point_3d


def get_rotation_coordinate_systems(iniX, iniY, iniZ, rangeV, stepV, cam_pos,point_pos, point_cam_pos):

    point_pos_T = []
    for point in point_pos:
        point_T = [-cam_pos[0]+point[0],-cam_pos[1]+point[1],-cam_pos[2]+point[2]]
        point_pos_T.append(point_T)
    
    print('point_pos_T', point_pos_T)
    # for phi in range(iniX,iniX + rangeV,stepV):
    for phi in range(-45,10,1):
        phi = phi/10
        start_time = time.time()
        speed_up(point_cam_pos, point_pos_T, phi, iniY, iniZ, rangeV, stepV)
        end_time = time.time()
        print(end_time - start_time)
        # t =  threading.Thread(target=speed_up, args=(object_in_world, object_in_eye_T, phi))
        # thread_list.append(t)
        # t.start()
        print(phi)
    # for t in thread_list:
    #     t.join()
    while not rotate_degree.empty():
        return_data = rotate_degree.get()
        try:
            smallest_angle
        except:
            smallest_angle = return_data[0]
        if smallest_angle > return_data[0]:
            smallest_angle == return_data[0]
            print(return_data)

def get_points_on_circle(pc, center):
    point_1 = pc[center[1]+ 20][center[0]]
    point_2 = pc[center[1]][center[0]+ 20]
    # normal_vector = np.array(get_plane_equation_from_points(point_0, point_1, point_2))[:3]
    return point_1, point_2

def get_pallet_corner(cicle_positions):
    ab = get_distance_two_point_3d(cicle_positions[0],cicle_positions[1])
    bc = get_distance_two_point_3d(cicle_positions[1],cicle_positions[2])
    ca = get_distance_two_point_3d(cicle_positions[0],cicle_positions[2])
    if ab > bc and ab > ca:
        return [cicle_positions[2], 2]
    elif bc > ab and bc > ca:
        return [cicle_positions[0], 0]
    else:
        return [cicle_positions[1], 1]

# def get_direction_point(cicle_position):
#     for i in range(3):
#         try:
#             temp_direct
#         except:
#             temp_direct = cicle_position[i][0]
#             direction_point = cicle_position[i]
#         if temp_direct > cicle_position[i][0]:
#             temp_direct = cicle_position[i][0]
#             direction_point = cicle_position[i]
#     return direction_point

def get_direction_point(cicle_position):
    for i in range(3):
        try:
            temp_direct
        except:
            temp_direct = cicle_position[i][1]
            direction_point = cicle_position[i]
            direction_index = i
        if temp_direct < cicle_position[i][1]:
            temp_direct = cicle_position[i][1]
            direction_point = cicle_position[i]
            direction_index = i
    return [direction_point, direction_index]

# cicle_position  = [[ 136.45716765,  270.24659981, 1175.46962148],
#  [-254.89544054,  -94.6665325,  1164.62614852],
#  [  86.851364,   -141.35726982, 1171.01745287]]

# cicle_position = [[  91.22726618,  301.12826936, 1101.54933489],
#  [-300.19088681,  -65.61333534, 1090.63033198],
#  [  40.2842625,  -108.00189096, 1106.70428725]]

# corner_point = get_pallet_corner(cicle_position)
# direction_point = get_direction_point(cicle_position)
# direction_vector = np.array(corner_point) - np.array(direction_point)
# print(get_angle_two_line_3d(direction_vector, np.array([1,0,0])))

# A = [738.7,-1657.3]
# B = [340.45,-1657.3]
# C = [340.45,-1716.23]
# AB = np.array(A) - np.array(B)
# AC = np.array(A) - np.array(C)
# print(angle([AB[0],AB[1], 0], [AC[0],AC[1], 0]))


def get_circle_center():
    camera_data = cam.setup_cam()
    object_3d_pos_list = []
    object_3d_pos_list_1 = []
    object_3d_pos_list_2 = []
    est = KMeans(n_clusters=3)
    count = 0
    while True:
        try:
            count += 1
            color_image, depth_map, point_cloud = camera_data.single_data()
            h,w,_ = color_image.shape

            left_top = [int(w*0.2),int(h*0.3)]
            right_button = [int(w*0.7),int(h*0.9)]

            # left_top = [int(w*0.4),int(h*0.4)]
            # right_button = [int(w*0.6),int(h*0.6)]

            # left_top = [int(w*0.1),int(h*0.4)]
            # right_button = [int(w*0.3),int(h*0.6)]

            # left_top = [0,int(h*0.4)]
            # right_button = [int(w*0.2),int(h*0.6)]

            # left_top = [0,0]
            # right_button = [w,h]  

            roi = color_image[left_top[1]:right_button[1], left_top[0]:right_button[0]]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            gray_blur = cv2.GaussianBlur(gray, (15, 15), 0)
                
            circles = cv2.HoughCircles(gray_blur,cv2.HOUGH_GRADIENT,1,20, param1=50,param2=30,minRadius=0,maxRadius=0)
            circles = np.uint16(np.around(circles))
            largestRadius = 0
            for i in circles[0,:]:
                center_x, center_y = (left_top[0] + i[0],left_top[1] + i[1])
                object_pos = point_cloud[center_y][center_x]*1000
                print('object_pos', np.around(object_pos,1))
                cv2.circle(color_image, (center_x, center_y), 1, [0,255,0], -1)
                # if not object_3d_pos_list 
                # if object_pos[1] > 1:
                #     continue
                object_3d_pos_list.append(object_pos)
                # object_3d_pos_list_1.append((object_pos_1*1000).astype(int))
                # object_3d_pos_list_2.append((object_pos_2*1000).astype(int))
            color_image = cv2.circle(color_image,tuple(left_top), 5,[0,0,0],1)
            color_image = cv2.circle(color_image,tuple(right_button), 5,[0,0,0],1)
            cv2.imshow('Detected coins',color_image)
            if cv2.waitKey(1) == ord('q'):
                break
            # cv2.waitKey(1)
            # if count > 100:
            #     break
        except Exception as ex:
            print('exception happend in circle', ex)
    
    est.fit(object_3d_pos_list)
    labels = est.labels_
    clustering_center_points = est.cluster_centers_
    # print(clustering_center_points)
    # data_dislay = matplot.data_show_3d(object_3d_pos_list, labels, clustering_center_points)
    return clustering_center_points


    est.fit(object_3d_pos_list_1)
    point_1 = est.cluster_centers_[0]

    est.fit(object_3d_pos_list_2)
    point_2 = est.cluster_centers_[0]
    cv2.destroyAllWindows()
    return point_0, point_1, point_2

def get_world_posotion(point_pos, cam_pos):
    P = [278.4875,  90.0875, 692.675 ]
    point_pos = error_compensatiton(point_pos)
    point_world_pos = [cam_pos[0]-point_pos[1],cam_pos[1]-point_pos[0],cam_pos[2]-point_pos[2]]
    return point_world_pos

# cam_pos = [986, 57, 920]
# point_pos_0, point_pos_1, point_pos_2 = get_circle_center()
# point_pos_0 = np.array(point_pos_0).astype(int)
# point_pos_0 = get_world_posotion(point_pos_0, cam_pos)

# point_pos_1 = np.array(point_pos_1).astype(int)
# point_pos_1 = get_world_posotion(point_pos_1, cam_pos)

# point_pos_2 = np.array(point_pos_2).astype(int)
# point_pos_2 = get_world_posotion(point_pos_2, cam_pos)

# normal_vector = np.array(get_plane_equation_from_points(point_pos_0[:3],point_pos_1[:3],point_pos_2[:3]))[:3]
# angle_C = get_angle_two_line_3d(normal_vector, [0,1,0])
# print(90 + angle_C)
# angle_B = get_angle_two_line_3d(normal_vector, [1,0,0])
# print(90 - angle_B)
# cam_pos = [986, 57, 920]
# point_pos = get_circle_center()
# point_pos = get_world_posotion(point_pos, cam_pos)
# vector_1 = [0, 1,0]
# vector_2 = [1, 1,0]

# unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
# unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
# dot_product = np.dot(unit_vector_1, unit_vector_2)
# angle = math.degrees(np.arccos(dot_product))
# print(angle)

# A = [188.08372093, 133.26511628, 638.22790698]
# B = [ 48.02723735,  19.24124514, 638.05836576]
# C = [-123.40804598,  133.02298851,  640.55747126]
# D = [226.56621005 ,119.87214612, 648.74429224]
# E = [432.3740458,  135.20610687, 701.96183206]

# Ar = [837.68, -157.97, 258]
# Br = [953.51, -17, 259.4]
# Cr = [837.32, 159.26, 259.47]
# Dr = [844.82,-177.57,292.19]
# Er  = [830.16,-388.19,229.02]


# A = [299.20647773,  52.32793522, 618.26720648]
# B = [245.18367347, 100.54081633, 616.81122449]
# C = [ 41.7260274,   42,         613.71917808]
# Ar = [911.55, -256.29, 314.05]
# Br = [862.71, -201.72, 314.32]
# Cr = [920.74,5.34,314.15]

# A = [-27.01801802, 167.22522523, 692.30630631]
# B = [206.12222222,  46.92592593, 691.46666667]
# C = [112.8,        163.51555556, 692.48444444]

# Ar = [816.61,86.88,227.95]
# Br = [937.45,-153.74,229.1]
# Cr = [821.81,-58.75,227.7]

# get_coordinate_system_root(900,0,850,100,1,
#                                 get_distance_two_point_3d([0,0,0],A),
#                                 get_distance_two_point_3d([0,0,0],B),
#                                 get_distance_two_point_3d([0,0,0],C),
#                                 Ar, Br, Cr
#                                 )

# get_rotation_coordinate_systems(-5, 175, 80, 10, 1, [970, 25, 898],[Ar,Br, Cr], [error_compensatiton(A), error_compensatiton(B), error_compensatiton(C)])
# get_rotation_coordinate_systems(-5, 175, 80, 10, 1, [970, 25, 898],[Ar,Br, Cr], [A, B, C])
# A = [226.56621005 ,119.87214612, 648.74429224]
# R = Rz(math.radians(88.796)) * Ry(math.radians(179.604)) * Rx(math.radians(0.02)) #0625
# R = Rz(math.radians(90.1)) * Ry(math.radians(179)) * Rx(math.radians(-1)) #0628  -1.12, 178.97, 90.3
# R = Rz(math.radians(-4.1089)) * Ry(math.radians(-0.3339)) * Rx(math.radians(1.238)) 
# P = [-80.26243,  58.75567, 557.5    ]
# P = P*R
# print(P)

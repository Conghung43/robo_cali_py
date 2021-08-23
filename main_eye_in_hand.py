# This program support the convertion of object location from camera by rotating it in 3D environment
# It needs the coorporation of robot hand so control robot function will be applied
# Robot position will be changed in config file

from cv2 import sqrt
import kuka_communicate as robot
import configparser
from ast import literal_eval
import cicle_detection
import setup_camera as cam
import matplot_show as mat
import time
import fit_skspatial as fit
import geometry_calculation as cal
import numpy as np
import cam_rotation_eye_hand as rot
import random
import math

class eye_hand_cali():
    def __init__(self, robot_type, number_steps, angle_config):
        self.robot_type = robot_type
        self.num_step = number_steps
        self.angle_config = angle_config
        self.pos_list_x = []
        self.pos_list_y = []
        self.robo_pos_list = []
        self.camera_data = cam.setup_cam()
        self.conn = robot.connect_kuka()
        self.robot_origin_pos = self.conn.sock.recv(1024)
        self.robot_origin_pos = self.robot_origin_pos.replace(b'\r\n',b'')
        self.robot_origin_pos = [float(v) for v in self.robot_origin_pos.decode("utf-8").split(',')]
        self.get_config_data()
        # time.sleep(4)
        self.main_function()
    def get_config_data(self):
        self.config = configparser.ConfigParser()
        self.config.read("utils/configuration.cfg", encoding="utf-8")

    def set_config_data(self, section_name, key, value):
        value = str(value)
        if not self.config.has_section(section_name):
            self.config.add_section(section_name)
        self.config.set(section_name, key, value)
        with open("utils/configuration.cfg", 'w') as configfile:
            self.config.write(configfile)

    def axis_movement(self, axis):
        section_name = axis + '_axis_eye_' + self.robot_type
        start_pos = literal_eval(self.config[section_name]['start_pos'])
        end_pos = literal_eval(self.config[section_name]['end_pos'])
        
        if axis == 'x':
            print('xaxis_confirmation')
            step_range = (end_pos[1] - start_pos[1])/ self.num_step
            for i in range(self.num_step):
                mul_value = random.randint(0, self.num_step)
                # mul_value = i
                new_pos = [start_pos[0],start_pos[1] + mul_value*step_range,start_pos[2]]
                self.robo_pos_list.append(new_pos)
                self.conn.send_binary([[new_pos[0],new_pos[1] ,new_pos[2], self.angle_config[0], self.angle_config[1], self.angle_config[2]]])
                self.conn.sock.recv(1024)
                #get object position
                centroid_pos = cicle_detection.chessboard_detection(self.camera_data,5)
                if len(centroid_pos) > 0:
                    self.pos_list_x.append(centroid_pos)
        elif axis == 'y':
            print('yaxis_confirmation')
            step_range = (end_pos[0] - start_pos[0])/ self.num_step
            for i in range(self.num_step):
                mul_value = random.randint(0, self.num_step)
                new_pos = [start_pos[0] + mul_value*step_range,start_pos[1],start_pos[2]]
                self.robo_pos_list.append(new_pos)
                self.conn.send_binary([[new_pos[0],new_pos[1],new_pos[2], self.angle_config[0], self.angle_config[1], self.angle_config[2]]])
                self.conn.sock.recv(1024)
                #get object position
                centroid_pos = cicle_detection.chessboard_detection(self.camera_data,5)
                if len(centroid_pos) > 0:
                    self.pos_list_y.append(centroid_pos)

    def eyes_xy_to_tool(self, rz, rx, ry):
        self.conn.send_binary([[self.robot_origin_pos[0],
                                self.robot_origin_pos[1],
                                self.robot_origin_pos[2], 
                                180,0,180]])
        self.conn.sock.recv(1024)
        centroid_pos_rotation0 = cicle_detection.chessboard_detection(self.camera_data,10)
        centroid_pos_rotation0 = centroid_pos_rotation0*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz))
        self.conn.send_binary([[self.robot_origin_pos[0],
                                self.robot_origin_pos[1],
                                self.robot_origin_pos[2], 
                                0, 0, 180]])
        self.conn.sock.recv(1024)
        centroid_pos_rotation1 = cicle_detection.chessboard_detection(self.camera_data,10)
        centroid_pos_rotation1 = centroid_pos_rotation1*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz))
        eyes2tool = (np.array(centroid_pos_rotation0) + np.array(centroid_pos_rotation1))/2
        if len(eyes2tool) == 1:
            eyes2tool = np.array(eyes2tool[0])*1000 # for D435
        else:
            eyes2tool = np.array(eyes2tool)*1000 # for L515
        print(eyes2tool)
        return [abs(eyes2tool[1]), abs(eyes2tool[0])]

    def eyes_z_to_tool(self, rz, rx, ry, eyes2tool_xy_plane_dist):
        self.conn.send_binary([[self.robot_origin_pos[0],
                                self.robot_origin_pos[1],
                                self.robot_origin_pos[2], 
                                180,0,180]])
        self.conn.sock.recv(1024)
        centroid_pos_rotation0 = cicle_detection.chessboard_detection(self.camera_data,100)
        centroid_pos_rotation0 = (centroid_pos_rotation0*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz)))[0].tolist()
        centroid_pos_rotation0 = cal.get_distance_two_point_3d([0,0,0],centroid_pos_rotation0[0])
        self.conn.send_binary([[self.robot_origin_pos[0],
                                self.robot_origin_pos[1],
                                self.robot_origin_pos[2], 
                                180,0,170]])
        self.conn.sock.recv(1024)
        centroid_pos_rotation1 = cicle_detection.chessboard_detection(self.camera_data,100)
        centroid_pos_rotation1 = (centroid_pos_rotation1*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz)))[0].tolist()
        centroid_pos_rotation1 = cal.get_distance_two_point_3d([0,0,0],centroid_pos_rotation1[0])
        self.conn.send_binary([[self.robot_origin_pos[0],
                                self.robot_origin_pos[1],
                                self.robot_origin_pos[2], 
                                180,0,160]])
        self.conn.sock.recv(1024)
        centroid_pos_rotation2 = cicle_detection.chessboard_detection(self.camera_data,100)
        centroid_pos_rotation2 = (centroid_pos_rotation2*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz)))[0].tolist()
        centroid_pos_rotation2 = cal.get_distance_two_point_3d([0,0,0],centroid_pos_rotation2[0])

        # eyes2tool = rot.find_z_eyes_tool((centroid_pos_rotation0[0][2]-centroid_pos_rotation1[0][2])/(centroid_pos_rotation1[0][2]-centroid_pos_rotation2[0][2]), eyes2tool_xy_plane_dist)
        # eyes2tool = rot.find_z_eyes_tool((centroid_pos_rotation2[2]-centroid_pos_rotation1[2])/(centroid_pos_rotation1[2]-centroid_pos_rotation0[2]), eyes2tool_xy_plane_dist)
        eyes2tool = rot.find_z_eyes_tool((centroid_pos_rotation2-centroid_pos_rotation1)/(centroid_pos_rotation1-centroid_pos_rotation0), eyes2tool_xy_plane_dist)
        return eyes2tool

    def mapping_eyes_robot(self,rz, rx, ry,eyes_tool_x, eyes_tool_y,eyes_tool_z):
        self.robot_origin_pos = [400,-1176,1470]
        self.conn.send_binary([[self.robot_origin_pos[0],
                                self.robot_origin_pos[1],
                                self.robot_origin_pos[2], 
                                180,0,180]])
        self.conn.sock.recv(1024)
        centroid_pos_rotation1 = cicle_detection.chessboard_detection(self.camera_data,5)
        centroid_pos_rotation1 = centroid_pos_rotation1*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz))
        if len(centroid_pos_rotation1) == 1:
            centroid_pos_rotation1 = np.array(centroid_pos_rotation1[0])*1000 # for D435
        else:
            centroid_pos_rotation1 = np.array(centroid_pos_rotation1)*1000 # for L515
        calibration_value = [eyes_tool_x, eyes_tool_y, eyes_tool_z] # trial run
        # calibration_value = [-273.88, 51.17, 400] # real environment
        point_in_world = np.array(calibration_value) + np.array([self.robot_origin_pos[0],
                                self.robot_origin_pos[1],
                                self.robot_origin_pos[2]])
        centroid_pos_rotation1 = centroid_pos_rotation1[0]
        point_in_world = [point_in_world[0] - centroid_pos_rotation1[1], 
                                point_in_world[1] - centroid_pos_rotation1[0], 
                                point_in_world[2] - centroid_pos_rotation1[2]]# code may change here
        self.conn.send_binary([[point_in_world[0],
                                point_in_world[1],
                                point_in_world[2], 
                                180,0,180]])
        return point_in_world

    def main_function(self):
        # self.axis_movement('x')
        # self.axis_movement('y')
        # rz, rx, ry = rot.find_camera_rotation(self.pos_list_x, self.pos_list_y)

        rz, rx, ry = [-11.3,-1.36,-1.4]
        eyes_tool_x, eyes_tool_y, eyes_tool_z = [16.86, 171.85, 25.6]

        # eyes_tool_x, eyes_tool_y = self.eyes_xy_to_tool(rz, rx, ry)
        # eyes2tool_xy_plane_dist = math.sqrt(pow(eyes_tool_x,2) + pow(eyes_tool_y,2))
        # eyes_tool_z = self.eyes_z_to_tool(rz, rx, ry, eyes2tool_xy_plane_dist)
        self.mapping_eyes_robot(rz, rx, ry,eyes_tool_x, eyes_tool_y,eyes_tool_z)
        section_name = 'mapping_' + self.robot_type
        self.set_config_data(section_name, 'rz',rz)
        self.set_config_data(section_name, 'rx',rx)
        self.set_config_data(section_name, 'ry',ry)
        self.set_config_data(section_name, 'eyes_tool_x',eyes_tool_x)
        self.set_config_data(section_name, 'eyes_tool_y',eyes_tool_y)
        self.set_config_data(section_name, 'eyes_tool_z',eyes_tool_z)
        print(rz, rx, ry)


run = eye_hand_cali('kr60', 20, [180, 0, 180])

# rz, rx, ry = [-9.065, -1.912, -0.894]
# eyes_tool_x, eyes_tool_y, eyes_tool_z = [75.78, -180.46, -20.82]
# def mapping_eyes_robot(rz, rx, ry,eyes_tool_x, eyes_tool_y,eyes_tool_z):
#     robot_origin_pos = [1170,0,1470]
#     centroid_pos_rotation1 = cicle_detection.chessboard_detection(cam.setup_cam())
#     centroid_pos_rotation1 = centroid_pos_rotation1*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz))
#     if len(centroid_pos_rotation1) == 1:
#         centroid_pos_rotation1 = np.array(centroid_pos_rotation1[0])*1000 # for D435
#     else:
#         centroid_pos_rotation1 = np.array(centroid_pos_rotation1)*1000 # for L515
#     # calibration_value = [eyes_tool_x, -eyes_tool_y, eyes_tool_z] # trial run
#     # calibration_value = [-273.88, 51.17, 400] # real environment
#     # point_in_world = np.array(calibration_value) + np.array([robot_origin_pos[0],
#     #                         robot_origin_pos[1],
#     #                         robot_origin_pos[2]])
#     centroid_pos_rotation1 = centroid_pos_rotation1[0]
#     point_in_world = [robot_origin_pos[0] - eyes_tool_x - centroid_pos_rotation1[1], 
#                             robot_origin_pos[1] - eyes_tool_y - centroid_pos_rotation1[0], 
#                             robot_origin_pos[2] - eyes_tool_z - centroid_pos_rotation1[2]]# code may change here
 
#     return point_in_world

# mapping_eyes_robot(rz, rx, ry,eyes_tool_x, eyes_tool_y,eyes_tool_z)
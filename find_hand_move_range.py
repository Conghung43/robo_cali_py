import cicle_detection
import numpy as np
import time

class find_robot_range:
    def __init__(self, home_pos, camera_data, limit_value, step_range, robot_connection):
        self.home_pos = home_pos
        self.camera_data = camera_data
        self.limit_value = limit_value
        self.step_range = step_range
        self.robot_connection = robot_connection

    def move_axis(self, index, direction):
        if direction == 0:
            direction_value = -self.step_range
        else:
            direction_value = self.step_range
        if index == 0:
            diff_moving = [direction_value,0,0,0,0,0]
        else:
            diff_moving = [0,direction_value,0,0,0,0]

        home_pos = self.home_pos
        while True:
            temp_home_pos = home_pos.copy()
            home_pos = np.array(home_pos) + np.array(diff_moving)
            self.robot_connection.send_binary([[home_pos[0], home_pos[1], home_pos[2], home_pos[3],home_pos[4],home_pos[5]]])
            self.robot_connection.sock.recv(1024)
            print(home_pos)
            corners = cicle_detection.chessboard_detection(self.camera_data,1, True)
            time.sleep(1)
            if corners is None or (home_pos[index] < self.limit_value[index][0] or home_pos[index] > self.limit_value[index][1]):
                return temp_home_pos[index]

    def find_range(self):
        start_x = self.move_axis(0, 0)
        print('start_x')
        # time.sleep(5)
        end_x = self.move_axis(0, 1)
        print('end_x')
        # time.sleep(5)
        start_y = self.move_axis(1, 0)
        print('start_y')
        # time.sleep(5)
        end_y = self.move_axis(1, 1)
        print('end_y')
        # time.sleep(5)
        return [start_x, end_x, start_y, end_y]

# home_pos = [880,15,400,0,90,0]
# limit_value = [[0,1500],[-500,500]]
# step_range = 50
# import setup_camera as cam
# # import matplot_show as mat
# camera_data = cam.setup_cam()
# print(find_range(home_pos, camera_data, limit_value))
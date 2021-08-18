import numpy as np
import cv2
from sklearn.cluster import KMeans

def get_circle_center(camera_data, number_cluster, crop_area):
    
    object_3d_pos_list = []
    est = KMeans(n_clusters=number_cluster)
    count = 0
    while True:
        try:
            count += 1
            color_image, _, point_cloud = camera_data.single_data()
            h,w,_ = color_image.shape

            left_top = [int(w*crop_area[0]),int(h*crop_area[2])]
            right_button = [int(w*crop_area[1]),int(h*crop_area[3])]

            roi = color_image[left_top[1]:right_button[1], left_top[0]:right_button[0]]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            gray_blur = cv2.GaussianBlur(gray, (15, 15), 0)
                
            circles = cv2.HoughCircles(gray_blur,cv2.HOUGH_GRADIENT,1,20, param1=50,param2=30,minRadius=0,maxRadius=0)
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                if i[2] > 60 or i[2] < 50:
                    continue
                center_x, center_y = (left_top[0] + i[0],left_top[1] + i[1])
                object_pos = point_cloud[center_y][center_x]*1000
                # if object_pos[2] < 600:
                #     continue
                # print('object_pos', np.around(object_pos,1))
                cv2.circle(color_image, (center_x, center_y), 1, [0,255,0], -1)
                object_3d_pos_list.append(object_pos)
            color_image = cv2.circle(color_image,tuple(left_top), 5,[250,0,0],1)
            color_image = cv2.circle(color_image,tuple(right_button), 5,[250,0,0],1)
            cv2.imshow('Detected coins',color_image)
            if cv2.waitKey(1) == ord('q'):
                break
            if count > 10:
                break
        except Exception as ex:
            print('exception happend in circle', ex)
    try:
        est.fit(object_3d_pos_list)
        labels = est.labels_
        clustering_center_points = est.cluster_centers_[0].tolist()
    except:
        clustering_center_points = []

    # mat_show = mat.data_show_3d(object_3d_pos_list)

    return clustering_center_points

# import setup_camera as cam
# import matplot_show as mat
# camera_data = cam.setup_cam()
# print(get_circle_center(camera_data, 1, [0.1,0.9,0.3,0.6]))
# print(get_circle_center(camera_data, 1, [0.4,0.6,0.1,0.9]))

def chessboard_detection(camera_data):
    nline = 6
    ncol = 6

    # img = cv2.imread('chestboard.jpg')
    # cam = camera.Zed_camera(24998496)#21786708, 24998496
    while True:
        try:
            # image,pc,depth = cam.retreive_cam_data()
            img, _, pc = camera_data.single_data()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            half_h, half_w,_ = (np.array(img.shape)/2).astype(int)
            # img = cv2.circle(img,(half_h, half_w),1,[0,0,255],1)
            # continue
            
            chessboard_pc_list = []
            ## termination criteria
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (nline, ncol), None)
            try:
                for corner in corners:
                    corner = corner.astype(int)
                    img = cv2.circle(img,tuple(corner[0]), 1, [0,0,255], -1)
                    corner_x, corner_y = np.array(corner[0])
                    chessboard_pc_list.append(pc[corner_y][corner_x])
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                cv2.imshow('image', img)
                cv2.waitKey(1)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            except Exception as ex:
                print('')
            if len(chessboard_pc_list) > 0:
                return np.mean(chessboard_pc_list, axis=0)
                # return chessboard_pc_list
            # f =  open('dummy_data/{}.npy'.format(time.time()), 'a') 
            # np.save('dummy_data/{}.npy'.format(time.time()), np.array(pc_3d_list))
        except Exception as ex:
            print('unexpect exception: ', ex)
            cv2.imshow('image', img)
            cv2.waitKey(1)

# import setup_camera as cam
# import matplot_show as mat
# camera_data = cam.setup_cam()
# chessboard_pc_list = chessboard_detection(camera_data)
# print(chessboard_pc_list*1000)
# print(np.mean(chessboard_pc_list, axis=0))
# chessboard_pc_list = np.concatenate((chessboard_pc_list,[np.mean(chessboard_pc_list, axis=0)]),axis = 0)
# mat.data_show_3d(chessboard_pc_list)

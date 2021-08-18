import geometry_calculation as cal
import fit_skspatial as fit
import matplot_show as mat
import numpy as np
import math
from threading import Thread
pos_list_x = []
pos_list_y = []
pos_list_x.append([[-128.83004760742188, 83.0177230834961, 555.2430419921875], [-142.23692321777344, 84.44670104980469, 564.6181640625], [-155.0538330078125, 85.7260513305664, 573.1389770507812], [-168.08267211914062, 87.1253433227539, 582.3611450195312], [-180.71388244628906, 88.44149017333984, 590.4305419921875], [-193.5456085205078, 89.76524353027344, 598.9166259765625], [-206.59786987304688, 91.16416931152344, 608.416748046875], [-218.0603485107422, 92.35000610351562, 615.6111450195312], [-231.1671600341797, 93.67033386230469, 624.7708129882812], [-243.32960510253906, 94.74584197998047, 631.52783203125], [-256.8137512207031, 96.06051635742188, 640.6181030273438], [-269.6240539550781, 97.51742553710938, 649.9583129882812], [-282.02264404296875, 98.58100128173828, 657.632080078125], [-294.5813293457031, 99.7491226196289, 666.3818969726562], [-307.03057861328125, 101.1264877319336, 675.048583984375], [-319.5606689453125, 102.2698745727539, 683.6180419921875], [-331.76080322265625, 103.40960693359375, 691.3611450195312], [-344.43524169921875, 104.70219421386719, 700.9861450195312], [-356.5699768066406, 105.73969268798828, 708.9306640625], [-369.2362365722656, 106.98151397705078, 718.0139770507812]])
pos_list_y.append([[-155.61624145507812, 199.73773193359375, 643.2987060546875], [-160.30960083007812, 187.8460235595703, 640.201416015625], [-164.43040466308594, 175.46456909179688, 634.7501220703125], [-168.87124633789062, 162.87734985351562, 629.3125], [-173.47308349609375, 150.8320770263672, 624.9931030273438], [-177.7498321533203, 138.4644012451172, 619.5834350585938], [-182.2976531982422, 126.3901596069336, 615.166748046875], [-186.84620666503906, 113.26399993896484, 609.5972900390625], [-191.50643920898438, 101.17784881591797, 605.2569580078125], [-195.8452606201172, 89.03768920898438, 600.0138549804688], [-200.28903198242188, 77.02104949951172, 595.5486450195312], [-204.65576171875, 64.81971740722656, 590.3125], [-209.1843719482422, 52.740257263183594, 585.6181640625], [-213.54714965820312, 40.704837799072266, 580.451416015625], [-218.22784423828125, 27.773698806762695, 574.8959350585938], [-223.15089416503906, 15.761789321899414, 570.8333129882812], [-227.58116149902344, 3.7671146392822266, 565.9375], [-231.98873901367188, -8.225497245788574, 560.986083984375], [-236.1595916748047, -20.248796463012695, 555.1389770507812], [-240.73147583007812, -32.1873779296875, 550.805419921875]])

def find_camera_rotation(pos_list_x, pos_list_y):

    pos_list_x = [x for x in pos_list_x if x[2] < 1000]
    pos_list_y = [y for y in pos_list_y if y[2] < 1000]

    pos_list = pos_list_x + pos_list_y
    plane_fit = fit.plane_best_fit(pos_list)
    # mat.data_show_3d(pos_list)

    #----------------Z rotation----------------
    line_fit_x = fit.line_best_fit(pos_list_x)
    plane_value = cal.get_plane_equation_from_point_normal_vector(plane_fit.vector, plane_fit.point)
    find_z_1 = (-plane_value[3] - 100*plane_value[0] - 100*plane_value[1])/plane_value[2]
    find_z_2 = (-plane_value[3] - 1*plane_value[0] - 100*plane_value[1])/plane_value[2]
    x_axis_direction_vector = np.array([100,100,find_z_1]) - np.array([1,100,find_z_2])
    rotate_angle_z = cal.get_angle_two_line_3d(line_fit_x.vector, x_axis_direction_vector)
    if line_fit_x.vector[1] < 0:
        rotate_angle_z = -rotate_angle_z
    if abs(rotate_angle_z) > 90:
        rotate_angle_z = -180+rotate_angle_z
    rotate_angle_z = -rotate_angle_z
    print('rotation Z angle = ', rotate_angle_z)
    # rotate_angle_z = -rotate_angle_z

    rotz_pos_list_x = []
    rotz_pos_list_y = []
    for pos_x in pos_list_x:
        new_pos_x = pos_x*cal.Rz(math.radians(rotate_angle_z))
        rotz_pos_list_x.append(new_pos_x.tolist()[0])
    for pos_y in pos_list_y:
        new_pos_y = pos_y*cal.Rz(math.radians(rotate_angle_z))
        rotz_pos_list_y.append(new_pos_y.tolist()[0])
    # rotz_line_fit_x = fit.line_best_fit(rotz_pos_list_x)
    # mat.data_show_3d(rotz_pos_list_x , pos_list_x)
    # mat.data_show_3d(rotz_pos_list_x + pos_list_x, rotz_pos_list_y + pos_list_y)
    # mat.data_show_3d(pos_list_x)

    #----------------X rotation----------------
    rotz_line_fit_y = fit.line_best_fit(rotz_pos_list_y)
    x_axis_direction_vector_z = fit.line_best_fit(rotz_pos_list_x)
    # current_plane_rotz_equation = fit.line_best_fit(rotz_pos_list_x + rotz_pos_list_y)
    y_axis_destination_direction_vector = cal.get_line_intersection_vector_from_two_planes([1,0,0],x_axis_direction_vector_z.vector)
    # rotate_angle_zx = cal.get_angle_two_line_3d(rotz_line_fit_y.vector, 
    #                                         y_axis_destination_direction_vector)

    rotate_angle_zx = cal.get_angle_two_line_3d(rotz_line_fit_y.vector, 
                                            [0,1,0])

    # rotate_angle_zx = cal.get_andg
    # rotate_angle_zx = -180+rotate_angle_zx                                            
    # rotate_angle_zx = cal.get_angle_two_line_3d(rotz_line_fit_y.vector, [0,1,0])
    if rotz_line_fit_y.vector[0] < 0:
        if rotz_line_fit_y.vector[1] < 0 or rotz_line_fit_y.vector[2] < 0:
            rotate_angle_zx = -rotate_angle_zx
        if abs(rotate_angle_zx) > 90:
            rotate_angle_zx = -rotate_angle_zx-180
    else:
        if abs(rotate_angle_zx) > 90:
            rotate_angle_zx = 180-rotate_angle_zx
        # else:
        #     if rotz_line_fit_y.vector[2] < 0:
        #         rotate_angle_zx = -rotate_angle_zx
    # if rotz_line_fit_y.vector[1]
    # rotate_angle_zx = 180 - rotate_angle_zx
    rotate_angle_zx = -rotate_angle_zx
    print('rotation X angle = ', rotate_angle_zx)

    rotzx_pos_list_x = []
    rotzx_pos_list_y = []
    for pos_x in rotz_pos_list_x:
        new_pos_x = pos_x*cal.Rx(math.radians(rotate_angle_zx))
        rotzx_pos_list_x.append(new_pos_x.tolist()[0])
    for pos_y in rotz_pos_list_y:
        new_pos_y = pos_y*cal.Rx(math.radians(rotate_angle_zx))
        rotzx_pos_list_y.append(new_pos_y.tolist()[0])
    mat.data_show_3d(rotzx_pos_list_x +rotz_pos_list_x ,rotzx_pos_list_y + rotz_pos_list_y)
    # mat.data_show_3d( rotzx_pos_list_y , rotz_pos_list_x )

    #----------------Y rotation----------------
    plane_fit = fit.plane_best_fit(rotzx_pos_list_x + rotzx_pos_list_y)
    rotate_angle_zxy = cal.get_angle_two_line_3d(plane_fit.vector, [0,0,1])
    if plane_fit.vector[0] < 0:
        rotate_angle_zxy = -rotate_angle_zxy
    # rotate_angle_zxy = -rotate_angle_zxy
    print('rotation Y angle = ',rotate_angle_zxy)

    rotzxy_pos_list_x = []
    rotzxy_pos_list_y = []
    for pos_x in rotzx_pos_list_x:
        new_pos_x = pos_x*cal.Ry(math.radians(rotate_angle_zxy))
        rotzxy_pos_list_x.append(new_pos_x.tolist()[0])
    for pos_y in rotzx_pos_list_y:
        new_pos_y = pos_y*cal.Ry(math.radians(rotate_angle_zxy))
        rotzxy_pos_list_y.append(new_pos_y.tolist()[0])

    # plane_fit = fit.plane_best_fit(rotzxy_pos_list_x + rotzxy_pos_list_y)
    # rotate_angle_zxy = cal.get_angle_two_line_3d(plane_fit.vector, [0,0,1])

    # mat.data_show_3d(rotzxy_pos_list_y + rotzxy_pos_list_x + rotzx_pos_list_x + rotzx_pos_list_y)
    mat.data_show_3d(rotzxy_pos_list_x, rotzxy_pos_list_y )
    return rotate_angle_z, rotate_angle_zx, rotate_angle_zxy
for index in range(len(pos_list_x)):
    pos_x = pos_list_x[index]
    pos_y = pos_list_y[index]
    find_camera_rotation(pos_x, pos_y)

# pos_x = pos_list_x[6]
# pos_y = pos_list_y[6]
# find_camera_rotation(pos_x, pos_y)
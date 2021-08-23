import geometry_calculation as cal
import numpy as np
x1 = [-169.5546201,   -91.92057201,  545.62606393]
x2 = [184.5633732,  -91.70316413, 549.10944798]
y1= [  -0.67450163, -122.51532226,  547.23790349]

y_gen = [(x1[0]+x2[0])/2,0,(x1[2]+x2[2])/2]

angle = cal.get_angle_two_line_3d(np.array(cal.get_plane_equation_from_points(x1,x2,y_gen))[:3].astype(int),
                                    [0,0,1])

print(angle)
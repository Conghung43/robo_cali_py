import os,sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cam_rotation_eye_hand as rot
import geometry_calculation as cal
import numpy as np
import math
import fit_skspatial as fit
import matplot_show as mat
# fit a fifth degree polynomial to the economic data
from numpy import arange
# from pandas import read_csv
from scipy.optimize import curve_fit
from matplotlib import pyplot
import numpy
from numpy import sin

x_new= numpy.array([-3.65040000e+02, -3.49830000e+02, -3.34620000e+02, -3.19410000e+02,
       -3.04200000e+02, -2.88990000e+02, -2.73780000e+02, -2.58570000e+02,
       -2.43360000e+02, -2.28150000e+02, -2.12940000e+02, -1.97730000e+02,
       -1.82520000e+02, -1.67310000e+02, -1.52100000e+02, -1.36890000e+02,
       -1.21680000e+02, -1.06470000e+02, -9.12600000e+01, -7.60500000e+01,
       -6.08400000e+01, -4.56300000e+01, -3.04200000e+01, -1.52100000e+01,
       -4.54747351e-13,  1.52100000e+01,  3.04200000e+01,  4.56300000e+01,
        6.08400000e+01,  7.60500000e+01,  9.12600000e+01,  1.06470000e+02,
        1.21680000e+02,  1.36890000e+02,  1.52100000e+02,  1.67310000e+02,
        1.82520000e+02,  1.97730000e+02,  2.12940000e+02,  2.28150000e+02])

y_downgrade = numpy.array([-12.536434052514586, -12.03911287369963, -12.009226582581995, 
                            -11.571856721827636, -10.777866885171086, -10.534453446525568, 
                            -9.445879964489542, -8.480782059208487, -7.276719644585917, 
                            -6.2161971600790835, -5.421575460732534, -4.5181037775833275, 
                            -3.576099536599422, -2.831492825448457, -2.030724405631439, -1.52842452572402, 
                            -0.9448004257273297, -0.5460975923980556, -0.21741788392684214, 0.0, 0.3710132536700783, 0.5738807300264099, 
                            0.8454012755645337, 1.1066907828604045, 1.3841938984874034, 1.765924550945705, 2.273210978233088, 
                            2.9701966934632296, 3.6531168112131454, 4.438315251876382, 5.147724983883926, 6.083337267187936, 6.776297964660806, 
                            7.8775380923382485, 8.27690989328368, 8.693977354402044, 9.62535953461812, 10.057790404274748, 10.312111140689431, 
                            10.201728366364328])

y_line = numpy.array([-15.71398139, -14.68504056, -13.70105411, -12.75997578,
       -11.8597593 , -10.9983584 , -10.17372682,  -9.38381828,
        -8.62658652,  -7.89998528,  -7.20196827,  -6.53048924,
        -5.88350192,  -5.25896004,  -4.65481733,  -4.06902752,
        -3.49954435,  -2.94432155,  -2.40131284,  -1.86847197,
        -1.34375267,  -0.82510866,  -0.31049368,   0.20213853,
         0.71483426,   1.22963975,   1.74860129,   2.27376514,
         2.80717757,   3.35088485,   3.90693323,   4.477369  ,
         5.06423842,   5.66958775,   6.29546327,   6.94391124,
         7.61697793,   8.31670961,   9.04515255,   9.80435301])



# define the true objective function
def objective_5(x, a, b, c, d, e, f):
	return (a * x) + (b * x**2) + (c * x**3) + (d * x**4) + (e * x**5) + f

def objective_3(x, a, b, c, d):
	return (a * x) + (b * x**2) + (c * x**3) + d 

def objective_2(x, b, c, a):
	return (b * x) + (c * x**2) + a  
def objective_1(x, c, b):
	return (c * x) + b  

def objective_sin(x, a, b, c, d):
	return a * sin(b - x) + c * x**2 + d

def find_error_equation():

    popt, _ = curve_fit(objective_5, x_new, y_downgrade)
    # summarize the parameter values
    a, b, c, d, e, f = popt
#     f = f+5
    y_line = objective_5(x_new, a, b, c, d, e, f)
    # y_line_2 = objective_2(x_new, 2*b, 3*c, a)

    # create a line plot for the mapping function
    pyplot.scatter(x_new, y_downgrade)
    # pyplot.plot(x_line, y_line, '--', color='red')
    # pyplot.plot(x_new, y_line_2, '--', color='red')
    pyplot.plot(x_new, y_line, '--', color='red')
    pyplot.show()
    return [a, b, c, d]
print(find_error_equation())
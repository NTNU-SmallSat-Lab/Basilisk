import numpy as np
import sys

from Basilisk.utilities import macros

''' 
A file to test the calculations done in the simulation and see if the coordinates obtained are coherent 
'''

if len(sys.argv) != 4:
    print("Usage: python script.py param1 param2 param3")
    sys.exit(1)

p1 = float(sys.argv[1])
p2 = float(sys.argv[2])
p3 = float(sys.argv[3])


def calc_L(l, L, i):
    res = -np.asin(np.tan(l*macros.D2R)/np.tan(i*macros.D2R))*macros.R2D
    return res

print(calc_L(p1,p2,p3))


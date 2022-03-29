#!/usr/bin/env python3
# File name: ctypestruct_testing.py
# Description: Used to confirm we can send arbitrary data back and forth from python to c++
# Date: 10/28/2021

# Based on this example: https://samuelstevens.me/writing/optimizing-python-code-with-ctypes

# other useful sources:
# https://stackoverflow.com/questions/11384015/python-ctypes-multi-dimensional-array
# https://stackoverflow.com/questions/17101845/python-ctypes-array-of-structs

# for dynamic data directly from numpy
# https://numpy.org/doc/stable/reference/routines.ctypeslib.html
# https://stackoverflow.com/questions/22425921/pass-a-2d-numpy-array-to-c-using-ctypes


import time
import rospkg

# Packages for C++ fitting
import ctypes

# class SEQUENCE(ctypes.Structure):
#     _fields_ = [('items', ctypes.POINTER(ctypes.c_char_p)),
#                 ('length', ctypes.c_int)]

class LINK(ctypes.Structure):
    _fields_ = [('name',ctypes.c_char_p),('R',(ctypes.c_float * 3) * 3)]

# def modelToCType():
    # return 0

class MODEL(ctypes.Structure):
    _fields_ = [('name', ctypes.c_char_p),('links',ctypes.POINTER(LINK))]
            
    
def main():
    rospack = rospkg.RosPack()
    root_dir = rospack.get_path('affordance_corrections')+'/../../'

    testlib = ctypes.CDLL(root_dir+'src/affordance_corrections/nodes/affordance_corrections/testcode/testctypestruct.so')
    testlib.passArguments.restype = ctypes.POINTER(MODEL)


    # Create the fake model
    name = "testmodel01"
    namebytes = bytes(name, 'utf-8')

    num_links = 2
    elems = (LINK * num_links)()
    LINK_ARRAY = ctypes.cast(elems,ctypes.POINTER(LINK))

    LINK_ARRAY[0].name = bytes('link0', 'utf-8')
    LINK_ARRAY[1].name = bytes('link1', 'utf-8')

    for i in range(3):
        for j in range(3):
            LINK_ARRAY[0].R[i][j] = 0.0
            LINK_ARRAY[1].R[i][j] = 1.0

    model = MODEL(namebytes,LINK_ARRAY)


    # Call C++ and get the updated model in return
    model2 = testlib.passArguments(ctypes.byref(model))[0]
    print(type(model2))

    print("From C++, I got",str(model2.name))
    
    
if __name__ == '__main__':
    main()

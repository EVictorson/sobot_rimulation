#!/usr/bin/env python

import sys
import numpy as np
import sympy as sp
#from mpmath import *
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# test that we are calculating the inertial matrix correctly

########### FIRST TEST ####################
"""
a1, a2, b11, b12, b21,b22 = sp.symbols("a1 a2 b11 b12 b22 b21")

A = sp.Matrix([[a1],[a2]])
B = sp.Matrix([[b11, b12], [b21, b22]])

print(B*A)
"""

####################


############### SECOND TEST ##################


l1, l2, a1, a2, theta1, theta2, ml1, ml2, mm1, mm2, Il1, Il2, Im1, Im2, kr1, kr2, = sp.symbols('l1 l2 a1 a2 theta1 theta2 ml1 ml2 mm1 mm2 Il1 Il2 Im1 Im2 kr1 kr2')


JP_L1 = sp.Matrix([[-1*l1*sp.sin(theta1), 0],[l1*sp.cos(theta1), 0],[0, 0]])
JP_L2 = sp.Matrix([[-1*a1*sp.sin(theta1)-l2*sp.sin(theta1+theta2), -l2*sp.sin(theta1+theta2)],[a1*sp.cos(theta1) + l2*sp.cos(theta1+theta2), l2*sp.cos(theta1+theta2)],[0, 0]])
JO_L1 = sp.Matrix([[0,0],[0,0],[1,0]])
JO_L2 = sp.Matrix([[0,0],[0,0],[1,1]])
JP_M1 = sp.Matrix([[0,0],[0,0],[0,0]])
JP_M2 = sp.Matrix([[-a1*sp.sin(theta1),0],[a1*sp.cos(theta1),0],[0,0]])
JO_M1 = sp.Matrix([[0,0],[0,0],[kr1,0]])
JO_M2 = sp.Matrix([[0,0],[0,0],[1,kr2]])

R0_1 = sp.Matrix([[sp.cos(theta1), -1*sp.sin(theta1), 0],[sp.sin(theta1), sp.cos(theta1),0],[0,0,1]])
R1_2 = sp.Matrix([[sp.cos(theta2), -1*sp.sin(theta2), 0],[sp.sin(theta2), sp.cos(theta2),0],[0,0,1]])
R0_2 = R0_1 * R1_2

term1 = ml1 * sp.Transpose(JP_L1) * JP_L1 + sp.Transpose(JO_L1) * R0_1 * Il1 * sp.Transpose(R0_1) * JO_L1
term2 = mm1 * sp.Transpose(JP_M1) * JP_M1 + sp.Transpose(JO_M1) * R0_1 * Im1 * sp.Transpose(R0_1) * JO_M1
term3 = ml2 * sp.Transpose(JP_L2) * JP_L2 + sp.Transpose(JO_L2) * R0_2 * Il2 * sp.Transpose(R0_2) * JO_L2
term4 = mm2 * sp.Transpose(JP_M2) * JP_M2 + sp.Transpose(JO_M2) * R0_2 * Im2 * sp.Transpose(R0_2) * JO_M2

B = term1 + term2 + term3 + term4

print(B)

#print(len(B))


#B11 = [Il1 + Il2 + Im1*kr1**2 + Im2 + ml1*(l1**2*sin(theta1)**2 + l1**2*cos(theta1)**2) + ml2*((-a1*sin(theta1) - l2*sin(theta1 + theta2))**2 + (a1*cos(theta1) + l2*cos(theta1 + theta2))**2) + mm2*(a1**2*sin(theta1)**2 + a1**2*cos(theta1)**2),
#B12 = Il2 + Im2*kr2 + ml2*(-l2*(-a1*sin(theta1) - l2*sin(theta1 + theta2))*sin(theta1 + theta2) + l2*(a1*cos(theta1) + l2*cos(theta1 + theta2))*cos(theta1 + theta2))]
#B21 = Il2 + Im2*kr2 + ml2*(-l2*(-a1*sin(theta1) - l2*sin(theta1 + theta2))*sin(theta1 + theta2) + l2*(a1*cos(theta1) + l2*cos(theta1 + theta2))*cos(theta1 + theta2)),
#B22 = Il2 + Im2*kr2**2 + ml2*(l2**2*sin(theta1 + theta2)**2 + l2**2*cos(theta1 + theta2)**2)]])

#c111 = 0.5 * sp.diff(B[0,0],theta1)
#print('B11 = ', B[0,0])
#print('c111 = ', c111)

b22 = Il2 + ml2*l2**2 + kr2**2*Im2
b12 = Il2 + ml2*(l2**2 + a1*l2*sp.cos(theta2)) + kr2*Im2


# TODO add method to symbolically calculate the christoffel symbols
c122 = sp.diff(b12, theta2) - 0.5*sp.diff(b22,theta1)
print('c122 = ', c122)



######## 3rd test #########
JP1_L1 = sp.Matrix([[-l1*sp.sin(theta1)],[l1*sp.cos(theta1)],[0]])
Go = sp.Matrix([[0],[-9.81],[0]])
JP2_L1 = sp.Matrix([[0],[0],[0]])
JP1_L2 = sp.Matrix([[-1*a1*sp.sin(theta1)-l2*sp.sin(theta1+theta2)],[a1*sp.cos(theta1)+l2*sp.cos(theta1+theta2)],[0]])
JP2_L2 = sp.Matrix([[-1*l2*sp.sin(theta1+theta2)],[l2*sp.cos(theta1+theta2)],[0]])
JP1_M1 = sp.Matrix([[0],[0],[0]])
JP2_M1 = sp.Matrix([[0],[0],[0]])
JP1_M2 = sp.Matrix([[-1*a1*sp.sin(theta1)],[a1*sp.cos(theta1)],[0]])
JP2_M2 = sp.Matrix([[0],[0],[0]])


g1 = ml1*sp.Transpose(Go)*JP1_L1 + mm1*sp.Transpose(Go)*JP1_M1 + ml2*sp.Transpose(Go)*JP1_L2 + mm2*sp.Transpose(Go)*JP1_M2

g2 = ml1*sp.Transpose(Go)*JP2_L1 + mm1*sp.Transpose(Go)*JP2_M1 + ml2*sp.Transpose(Go)*JP2_L2 + mm2*sp.Transpose(Go)*JP2_M2


print('G1 = ', g1)
print('G2 = ', g2)

# THESE EQUATIONS ALSO ADD UP!


# test the matrices are the same!

Il1 = 3
Il2 = 5
ml1 = 10
ml2 = 15
Im1 = 4
Im2 = 4
theta1 = 0.3
theta2 = 0.2
l1 = 3
l2 = 1
a1 = 1.5
a2 = 0.5
kr1 = 0.1
kr2 = 0.05

Ba = Il2 + Im2*kr2 + ml2*(-l2*(-a1*np.sin(theta1) - l2*np.sin(theta1 + theta2))*np.sin(theta1 + theta2) + l2*(a1*np.cos(theta1) + l2*np.cos(theta1 + theta2))*np.cos(theta1 + theta2))
Baa = Il2 + ml2*(l2**2 + a1*l2*np.cos(theta2)) + kr2*Im2

print(Ba)
print(Baa)

# Checks out, the B matrix I calculated is the same as the one they did!
#####################

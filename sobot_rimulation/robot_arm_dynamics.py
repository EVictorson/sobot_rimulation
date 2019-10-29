# ODE solver and system updates

import numpy as np
from scipy.integrate import odeint

class RobotDynamics:
  
  def __init__(self, save_state=True, config{}):
    # parameters of the dynamic model
    self.L1 = 0.5    # (m)
    self.L2 = 0.5    # (m)
    self.a1 = 1      # (m)
    self.a2 = 1      # (m)
    self.ML1 = 50    # (kg)
    self.ML2 = 50    # (kg)
    self.IL1 = 10    # (kg m^2)
    self.IL2 = 10    # (kg m^2)
    self.kr1 = 100   # (unitless)
    self.kr2 = 100   # (unitless)
    self.Mm1 = 5     # (kg)
    self.Mm2 = 5     # (kg)
    self.Im1 = 0.01  # (kg m^2)
    self.Im2 = 0.01  # (kg m^2)
    self.Fm1 = 0.01  # (N m s / rad)
    self.Fm2 = 0.01  # (N m s / rad)
    self.Ra1 = 10    # (ohm)
    self.Ra2 = 10    # (ohm)
    self.kt1 = 2     # (N m / A)
    self.kt2 = 2     # (N m / A)
    self.kv1 = 2     # (V s / rad)
    self.kv2 = 2     # (V s / rad)
    self.g = 9.807   # (m/s/s)  

"""
%% Dynamic model of generalized coordinates q1 and q2
% B(q) q_ddot + c(q,q_dot)*q_dot + F*q_dot + g(q) = T
% here we assume F (frictional forces) are negligible and drop this term
% resulting in:
%
% B(q) x q_ddot + c(q, q_dot) x q_dot + g(q) = T
%
% where B is our inertia matrix, C is the coriolis / centrifugal matrix,
% g is the gravity matrix, and T is the torque



% Inertia Matrix
b11 = IL1 + ML1*L1^2 + kr1^2*Im1 + IL2 + ML2*(a1^2 + L2^2 + 2*a1*L2*cos(q2)) ...
    + IM2 + Mm2 * a1^2;
b12 = IL2 + ML2 * (L2^2 + a1*L2*cos(q2)) + Kr2*Im2;
b21 = b12;
b22 = IL2 + Ml2*L2^2 + Kr2^2*Im2;

B = [b11 b12; b21 b22];

% Coriolis / centrifugal matrix

h = -ML2 * a1 * L2 * sin(q2);
c11 = h * theta_2_dot;
c12 = h * (q1_dot + q2_dot);
c21 = -1*h * q1_dot;
c22 = 0;

C = [c11 c12; c21 c22];

% Christoffel symbols 
% c111 = 
% c112 = 
% c121 = 
% c122 =
% c211 = 
% c212 = 
% c222 = 

% We can compute N as a means to verify the skew symmetry of C if desired
%N = B_dot - 2.*C;


% Gravity matrix
g1 = (Ml1*L1 + Mm2*a1 + ML2*a1)*g*cos(q1) + ML2*L2*g*cos(q1)*cos(q2);
g2 = ML2 * L2 * g * cos(q1)*cos(q2);
G = [g1 ; g2];


% state vectors

q = [q1; q2];
q_dot = [q1_dot; q2_dot];
q_ddot = [q1_ddot; q2_ddot];

T = B * q_ddot + C * q_dot + G;

%% Dynamic Inversion Control

% Proportional and Derivative gains:
Kp = [25 0; 0 25];  % = diag{wnq^2, ..., wnn^2}
Kd = [5 0; 0 5];    % = diag{2*xi_1*wn1, ..., 2* xi_n * wnn}

% with these gains we are effectively setting natural frequency = 5 rad/sec, 
% and damping ratio to 0.5.  I'm not thrilled about this damping ratio as
% it permits overshoot.


% actual control output to manipulator
% u = B x y + C x q_dot + G
% where y is virtual (outer loop) control

% [1] for each time step calculate q, q_dot, q_ddot
%
% [2] with the knowledge of q, q_dot, q_ddot calculate: B(q), C(q, q_dot), G(q)
%
% [3] calculate error terms: q_tilda, q_dot_tilda, q_ddot_tilda as:
%   q_tilda = qd - q;
%   q_dot_tilda = qd_dot - q_dot;
%   q_ddot_tilda = qd_ddot - q_ddot;
%
% [4] from the error terms:
%   y = -Kp x q - Kd x q_dot + qd_ddot + Kd x qd_dot + Kp x q_dot
%   which simplifies to:
%     = Kp x (qd - q) + Kd x (qd_dot - q_dot) + qd_ddot
%   y = Kp x q_tilda + Kd x q_dot_tilda + qd_ddot which is fed into the
%   inner loop.
%
% [5] With virtual control y, calculate actual control u as:
%   u = B x y + C x q_dot + G

%% Simulation approach
% 1)    Create goal point in task space and desired time to reach goal
% 2)    Perform inverse kinematics to figure out what joint space states
%       this corresponds to
% 3)    Use cubic polynomial trajectory generator in joint space
% 4)    Save output of trajectory to array to be fed as desired states to
%       controller
% 5)    Feed this goal trajectory to the controller
% 6)    Use a model that is slightly different from the controller model?


%% Point to point trajectory generation
% let's use simple cubic polynomial point-to-point trajectory generation in
% joint space.

% assume loop time of 1ms (1000 hz)
% (1) q(t) = a3 t ^3 + a2 t^2 + a1t + a0
% (2) q_dot(t) = 3a3 t^2 + 2a2 t + a1
% (3) q_ddot(t) = 6a3 t + 2a2

% solve for coefficients by noting that position and velocity at start and
% end are both 0.

% evaluate equations 1 and 2 above at q0, qf, q0_dot, qf_dot, and tf
% from p. 164


%% Denavit-Hartenberg Forward Kinematics 
% while we're doing this stuff we might as well show we know things about
% DH notation

% given the two-link planar arm being analyzed we have:
A0_1 = [cos(q1) -sin(q1) 0 a1*cos(q1);
        sin(q1) cos(q1) 0 a1*sin(q1);
        0 0 1 0;
        0 0 0 1];

A1_2 = [cos(q2) -sin(q2) 0 a1*cos(q2);
        sin(q2) cos(q2) 0 a1*sin(q2);
        0 0 1 0;
        0 0 0 1];
    
% The forward kinematics are then:    
T0_2 = A0_1 * A1_2;


%% Inverse Kinematics
% Let's 
"""

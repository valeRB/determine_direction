import math as m
import numpy as np
# multiplication between arrays is element by element multiplication
# multiplication between matrices is a normal matrix multiplication

# ------ THE MODEL ---------------
# --- x(k+1) = A * x(k) + B * u(k) + G * w(k)
# ---
# --- Measurement model
# --- z(k) = C * x(k+1) + D * v(k)
# ---
# --- Where:
# --- x(k) = [theta,
# ---         theta_dot]
# --- w(k) = Process noise
# --- v(k) = Measurement noise
# --- G is an identity matrix [2x2]
# --- D is an identity matrix [2x2]
# ---
# ------ THE KALMAN FILTER ------
# --- PREDICTION
# --- xhat(k+1|k) = A * xhat(k|k) + B * u(k)
# --- P(k+1|k) = A * P * A' + G * Q * G'
# ---
# --- UPDATE
# --- K(k+1) = P(k+1|k) * C' * inv(C * P(k+1|k) * C' + D * R * D')
# --- xhat(k+1|k+1) = xhat(k+1|k) + K(k+1) * (y(k+1) - C * xhat(k+1|k))
# --- P(k+1|k+1) = P(k+1|k) - K(k+1) * C * P(k+1|k)
# ----------------------------------

# Initialize model
dt = 0.001
A = np.matrix( ((1.0, dt), (0, 1)) )
B = np.matrix('0; 0')
C = np.matrix( ((1.0, 0), (0, 0)) )
eye_m = np.identity(2)
D = np.matrix(eye_m)
G = np.matrix(eye_m)
u = 0
wStd = 0.1
vStd = 10
# Initialize kalman filter

Q = np.matrix( ((m.pow(wStd,2),0),(0,m.pow(wStd,2))) )
R = np.matrix( ((m.pow(vStd,2),0),(0,m.pow(vStd,2))) ) 

def Kalman_Filter(m_theta, xhat_1, P_1):
##    print('m_theta', m_theta)
##    print('xhat_1', xhat_1)
##    print('P_1', P_1)
##    raw_input("PT 1")
    z = np.matrix( [[m_theta], [0]] ) 
##    print(z)
    # Prediction: 
##    raw_input("PT 2: PREDICTION")
    xhat = A * xhat_1 + B * u
    P = A * P_1 * np.transpose(A) + Q
##    print('xhat: ', xhat)
##    print('P: ', P)

    # Measurment Update:
    K = P * np.transpose(C) * np.linalg.inv(C * P * np.transpose(C) + R)
    xhat = xhat + K * (z - C * xhat)
    P = P - K * C * P
    return xhat, P
    
    
    

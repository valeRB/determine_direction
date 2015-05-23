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
# --- x(k) = [theta]
# --- w(k) = Process noise
# --- v(k) = Measurement noise
# --- A = 1
# --- B = 0
# --- u = 0 ; no input
# --- G = 1
# --- D = 1
# --- 
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
# ---
# ------ THE KALMAN FILTER EQUATIONS BECOME:
# --- PREDICTION:
# --- xhat(k) = xhat(k-1)
# --- P(k) = P(k-1) + Q
# ---
# --- UPDATE:
# --- K(t) = P(k) / (P(k) + R)
# --- xhat(k) = xhat(k) + K(t)*(z(t)-xhat(k))
# --- P(k) = (1-K(t))*P(k)
# ----------------------------------

# Initialize model
dt = 0.001
A = 1

wStd = 0.1 # Process noise
vStd = 20 # Measurement noise

Q = m.pow(wStd,2)
R = m.pow(vStd,2)

def Kalman_Filter(m_theta, xhat_1, P_1):
    z = m_theta
    # Prediction:
    xhat = A * xhat_1 
    P = P_1 + Q

    # Measurement Update:
    K = P/(P + R)
    xhat = xhat + K * (z - xhat)
    P = (1 - K) * P
    return xhat, P, K
    
    
    

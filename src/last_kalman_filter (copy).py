import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

def system_uncertainty():
    sigma_a_mat = np.array([ [0.5*(t**2)*sigma_a_x],
                             [0.5*(t**2)*sigma_a_y],
                             [0.5*(t**2)*sigma_a_d],
                             [t*sigma_a_x],
                             [t*sigma_a_y],
                             [t*sigma_a_d], ])
    Q_k = sigma_a_mat.dot(sigma_a_mat.T)
    return Q_k

def Gaussian_measurement():
    randnoise_x = np.random.normal(mu, rho_x)
    randnoise_y = np.random.normal(mu, rho_y)
    randnoise_d = np.random.normal(mu, rho_d)
    randnoise_v_x = np.random.normal(mu, rho_v_x)
    randnoise_v_y = np.random.normal(mu, rho_v_y)
    randnoise_v_d = np.random.normal(mu, rho_v_d)
    R = np.array([ [rho_x**2, 0, 0, 0, 0, 0],
                   [0, rho_y**2, 0, 0, 0, 0],
                   [0, 0, rho_d**2, 0, 0, 0],
                   [0, 0, 0, rho_v_x**2, 0, 0],
                   [0, 0, 0, 0, rho_v_y**2, 0],
                   [0, 0, 0, 0, 0, rho_v_d**2] ])
    V = np.array([ [randnoise_x],
                   [randnoise_y],
                   [randnoise_d],
                   [randnoise_v_x],
                   [randnoise_v_y],
                   [randnoise_v_d] ])
    return V, R

def prediction(X):
    randvar_a_x = np.random.normal(mu, sigma_a_x)
    randvar_a_y = np.random.normal(mu, sigma_a_y)
    randvar_a_z = np.random.normal(mu, sigma_a_d)
    F = np.array([ [1, 0, 0, t, 0, 0],
                   [0, 1, 0, 0, t, 0],
                   [0, 0, 1, 0, 0, t],
                   [0, 0, 0, 1, 0, 0],
                   [0, 0, 0, 0, 1, 0],
                   [0, 0, 0, 0, 0, 1] ])
    w = np.array([ [0.5*t**2*randvar_a_x],
                   [0.5*t**2*(-9.8+randvar_a_y)],
                   [0.5*t**2*randvar_a_z],
                   [t*randvar_a_x],
                   [t*randvar_a_y],
                   [t*randvar_a_z], ])
    X_pred = F.dot(X) + w
    return X_pred

def measurement(X_measurement, V):
    H = np.identity(6, dtype=float)
    Z = H.dot(X_measurement) + V
    return  Z

def prediction_system_uncertainty(P, Q_k):
    F = np.array([ [1, 0, 0, t, 0, 0],
                   [0, 1, 0, 0, t, 0],
                   [0, 0, 1, 0, 0, t],
                   [0, 0, 0, 1, 0, 0],
                   [0, 0, 0, 0, 1, 0],
                   [0, 0, 0, 0, 0, 1] ])
    P_p = np.diag(np.diag(F.dot(P).dot(F.T))) + Q_k
    return P_p

def kalman_filter(X):
    I = np.identity(6, dtype=float)
    H = np.identity(6, dtype=float)
    P = np.zeros((6,6))
    X_pred = prediction(X)  # Prediction steps
    print('X_pred %s' %X_pred)
    Q_k = np.diag(np.diag(system_uncertainty()))  # System uncertainty that from prediction steps
    print('Q_k %s' %Q_k)
    V, R = Gaussian_measurement()   # Noise Vector (V) and Noise Covariance Matrix (R)
    print('R %s' %R)
    Z = measurement(X, V)   # Observation Matrix
    print('Z %s' %Z)
    P_p = prediction_system_uncertainty(P, Q_k)
    S = H.dot(P_p).dot(H.T) + R
    K = P_p.dot(H.T).dot(inv(S))
    X = X_pred + K.dot(Z - H.dot(X_pred))
    P = (I - K.dot(H)).dot(P_p)
    return X, P

t = 1
mu = 0
sigma_a_x = 0.8
sigma_a_y = 0.8
sigma_a_d = 0.8
rho_x = 0.1
rho_y = 0.1
rho_d = 0.1
rho_v_x = 0.5
rho_v_y = 0.5
rho_v_d = 0.5
X = np.array([ [0],
               [0],
               [0],
               [0],
               [0],
               [0] ])
X_kal, P = kalman_filter(X)
print('X_kal %s' %(X_kal))
print('P %s' %(P))

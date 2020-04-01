#!/usr/bin/env python
# coding: utf-8

# In[1]:


import tensorflow as tf
import numpy as np
from scipy.integrate import odeint
from math import exp


# In[2]:


def canoSystem(tau,t):
    alpha_s = 4
    s = exp(-tau*alpha_s*t)
    return s


# In[ ]:



# In[3]:


def dmp(g,q,qd,tau,s,q0,W,Name = "DMP"):
    alpha = tf.constant(25,dtype=tf.float64)
    beta = alpha/4
    w,c,h = W
    n_gaussian = w.shape[0]
    with tf.name_scope(Name):
        w_tensor = tf.constant(w,dtype=tf.float64,name='w')
        c_tensor = tf.constant(c,dtype=tf.float64,name='c')
        h_tensor = tf.constant(h,dtype=tf.float64,name='h')

        with tf.name_scope('s'):
            s_tensor = s*tf.ones(n_gaussian,dtype=tf.float64)
        smc_pow = tf.pow(s_tensor-c_tensor,2)
        h_smc_pow = tf.math.multiply(smc_pow,(-h_tensor))
        with tf.name_scope('psi'):
            psi = tf.math.exp(h_smc_pow)
        sum_psi = tf.math.reduce_sum(psi,0)
        wpsi = tf.math.multiply(w_tensor,psi)
        wpsis = tf.math.reduce_sum(wpsi*s,0)
        with tf.name_scope('fs'):
            fs =wpsis/sum_psi
        qdd = alpha*(beta*(g-q)-tau*qd)+fs*(g-q0)
    return qdd


# In[4]:


g1 = tf.placeholder(tf.float64,name='g1')
q1 = tf.placeholder(tf.float64,name='q1')
qd1 = tf.placeholder(tf.float64,name='qd1')
q01 = tf.placeholder(tf.float64,name='q01')

g2 = tf.placeholder(tf.float64,name='g2')
q2 = tf.placeholder(tf.float64,name='q2')
qd2 = tf.placeholder(tf.float64,name='qd2')
q02 = tf.placeholder(tf.float64,name='q02')

g3 = tf.placeholder(tf.float64,name='g3')
q3 = tf.placeholder(tf.float64,name='q3')
qd3 = tf.placeholder(tf.float64,name='qd3')
q03 = tf.placeholder(tf.float64,name='q03')

g4 = tf.placeholder(tf.float64,name='g4')
q4 = tf.placeholder(tf.float64,name='q4')
qd4 = tf.placeholder(tf.float64,name='qd4')
q04 = tf.placeholder(tf.float64,name='q04')

g5 = tf.placeholder(tf.float64,name='g5')
q5 = tf.placeholder(tf.float64,name='q5')
qd5 = tf.placeholder(tf.float64,name='qd5')
q05 = tf.placeholder(tf.float64,name='q05')

g6 = tf.placeholder(tf.float64,name='g6')
q6 = tf.placeholder(tf.float64,name='q6')
qd6 = tf.placeholder(tf.float64,name='qd6')
q06 = tf.placeholder(tf.float64,name='q06')


s = tf.placeholder(tf.float64,name='s')
tau = tf.placeholder(tf.float64,name='tau')





# In[5]:


w1 = np.load('Weights/Joint1/w.npy')
w1 = np.reshape(w1,(len(w1),))
c1 = np.load('Weights/Joint1/c.npy')
c1 = np.reshape(c1,(len(w1),))
h1 = np.load('Weights/Joint1/h.npy')
h1 = np.reshape(h1,(len(w1),))
W1 = (w1,c1,h1)

w2 = np.load('Weights/Joint2/w.npy')
w2 = np.reshape(w2,(len(w2),))
c2 = np.load('Weights/Joint2/c.npy')
c2 = np.reshape(c2,(len(w1),))
h2 = np.load('Weights/Joint2/h.npy')
h2 = np.reshape(h2,(len(w2),))
W2 = (w2,c2,h2)

w3 = np.load('Weights/Joint3/w.npy')
w3 = np.reshape(w3,(len(w3),))
c3 = np.load('Weights/Joint3/c.npy')
c3 = np.reshape(c3,(len(w3),))
h3 = np.load('Weights/Joint3/h.npy')
h3 = np.reshape(h3,(len(w3),))
W3 = (w3,c3,h3)

w4 = np.load('Weights/Joint4/w.npy')
w4 = np.reshape(w4,(len(w4),))
c4 = np.load('Weights/Joint4/c.npy')
c4 = np.reshape(c4,(len(w4),))
h4 = np.load('Weights/Joint4/h.npy')
h4 = np.reshape(h4,(len(w4),))
W4 = (w4,c4,h4)

w5 = np.load('Weights/Joint5/w.npy')
w5 = np.reshape(w5,(len(w5),))
c5 = np.load('Weights/Joint5/c.npy')
c5 = np.reshape(c5,(len(w5),))
h5 = np.load('Weights/Joint5/h.npy')
h5 = np.reshape(h5,(len(w5),))
W5 = (w5,c5,h5)

w6 = np.load('Weights/Joint6/w.npy')
w6 = np.reshape(w6,(len(w6),))
c6 = np.load('Weights/Joint6/c.npy')
c6 = np.reshape(c6,(len(w6),))
h6 = np.load('Weights/Joint6/h.npy')
h6 = np.reshape(h6,(len(w6),))
W6 = (w6,c6,h6)




# In[6]:


dmp1 = dmp(g1,q1,qd1,tau,s,q01,W1,Name="DMP1")
dmp2 = dmp(g2,q2,qd2,tau,s,q02,W2,Name="DMP2")
dmp3 = dmp(g3,q3,qd3,tau,s,q03,W3,Name="DMP3")
dmp4 = dmp(g4,q4,qd4,tau,s,q04,W4,Name="DMP4")
dmp5 = dmp(g5,q5,qd5,tau,s,q05,W5,Name="DMP5")
dmp6 = dmp(g6,q6,qd6,tau,s,q06,W6,Name="DMP6")


# In[32]:


with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    # writer = tf.summary.FileWriter('graph',sess.graph)
#
    def dynamics(x,t,tau_v,g_v,q0,sess):
        s_v = canoSystem(tau_v,t)

        feeddict = {g1:g_v[0], g2:g_v[1], g3:g_v[2], g4:g_v[3], g5:g_v[4], g6:g_v[5],
        q1:x[0],q2:x[1], q3:x[2], q4:x[3], q5:x[4], q6:x[5], qd1:x[6], qd2:x[7],
        qd3:x[8],qd4:x[9],qd5:x[10], qd6:x[11], q01:q0[0], q02:q0[1], q03:q0[2], q04:q0[3],
        q05:q0[4], q06:q0[5], s:s_v, tau:tau_v}

        qdd1_v,qdd2_v,qdd3_v,qdd4_v,qdd5_v,qdd6_v = sess.run([dmp1,dmp2,dmp3,dmp4,dmp5,dmp6],feed_dict = feeddict)
        dx = [x[6],x[7],x[8],x[9],x[10],x[11],qdd1_v,qdd2_v,qdd3_v,qdd4_v,qdd5_v,qdd6_v]
        return dx

    t = np.linspace(0, 1.423553944, 50)
    tau_v = float(1)/1.423553944
    q0_v = [-0.0003235975848596695, -1.040771786366598, 1.6213598251342773, -0.34193402925600225, 1.5711277723312378, 3.141711950302124]

    v0 = [0,0,0,0,0,0]
    g_v = [-0.4201243559466761, -1.3455780188189905, 1.6121912002563477, -0.055014912282125294, 1.2821934223175049, 3.1416163444519043]
    x0 = []
    x0.extend(q0_v)
    x0.extend(v0)
    # print(q0_v)
    q = odeint(dynamics,x0,t,args=(tau_v,g_v,q0_v,sess))
    

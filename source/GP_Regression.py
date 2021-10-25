import scipy as sp
import numpy as np
import matplotlib.pyplot as plt

from scipy.stats import multivariate_normal


n = 3
x = np.array([0,2,3,5])
f = np.array([0,2,2,0])

m = 100 # numper of data points generated
xs = np.linspace(0, 5, m) # defined observations
mxs = np.zeros(m)
B = np.array([[1,2,3,4],[1,2,3,4],[1,2,3,4],[1,2,3,4]])
print(B)
A = np.array([[0.00929545, -0.00464772 ,0    ,0,  0  ,   0    ,       0       ,    0     ,      0       ,    0   ,       0    ,       0      ,     0       ,   0      ,     0    ,       0      ,     0    ,       0     ,      0      ,     0],
         [-0.00464772,  0.00929545, -0.00464772,           0   ,        0         ,  0    ,       0   ,        0      ,     0      ,     0     ,      0     ,      0   ,        0     ,      0     ,      0       ,    0      ,     0    ,       0  ,         0  ,         0],
          [0 -0.00464772,  0.00929545, -0.00464772 ,          0      ,    0    ,       0     ,      0       ,    0        ,   0        ,   0       ,    0       ,    0      ,     0       ,    0     ,      0      ,     0    ,       0  ,         0   ,        0],
          [0  ,         0, -0.00464772,  0.00929545, -0.00464772,           0    ,       0        ,   0     ,      0     ,      0      ,     0    ,       0   ,        0      ,     0       ,    0      ,     0        ,   0         ,  0      ,     0,           0],
          [0  ,         0 ,          0 ,-0.00464772 , 0.00929545 ,-0.00464772,           0 ,          0 ,          0 ,          0 ,          0 ,          0 ,          0 ,          0 ,          0,           0,           0 ,          0 ,          0 ,          0],
          [0  ,         0  ,         0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772 ,          0 ,          0  ,         0  ,         0  ,         0  ,         0  ,         0  ,         0 ,          0 ,          0  ,         0  ,         0  ,         0],
          [0  ,         0   ,        0   ,        0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772  ,         0   ,        0   ,        0   ,        0   ,        0   ,        0   ,        0  ,         0  ,         0   ,        0   ,        0   ,        0],
          [0   ,        0    ,       0    ,       0  ,         0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772    ,       0    ,       0    ,       0    ,       0    ,       0    ,       0   ,        0   ,        0    ,       0    ,       0    ,       0],
          [0    ,       0     ,      0     ,      0   ,        0   ,        0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772     ,      0     ,      0     ,      0     ,      0     ,      0    ,       0    ,       0     ,      0     ,      0     ,      0],
          [0     ,      0      ,     0      ,     0    ,       0    ,       0   ,        0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772      ,     0      ,     0      ,     0      ,     0     ,      0     ,      0      ,     0      ,     0      ,     0],
          [0      ,     0       ,    0       ,    0     ,      0     ,      0    ,       0  ,         0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772       ,    0       ,    0       ,    0      ,     0      ,     0       ,   0        ,   0        ,   0],
          [0       ,    0        ,   0        ,   0      ,     0      ,     0     ,      0   ,        0   ,        0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772        ,   0        ,   0       ,    0       ,    0         ,  0        ,   0        ,   0],
          [0        ,   0         ,  0         ,  0       ,    0       ,    0      ,     0    ,       0    ,      0    ,       0   ,        0 ,-0.00464772  ,0.00929545 ,-0.00464772          , 0         ,  0         ,  0         ,  0          , 0          , 0],
          [0   ,        0 ,          0 ,          0        ,   0        ,   0       ,    0     ,      0     ,      0    ,       0   ,        0 ,          0 ,-0.00464772 , 0.00929545 ,-0.00464772         ,  0  ,         0         ,  0          , 0          , 0],
          [0    ,       0  ,         0  ,         0         ,  0         ,  0        ,   0      ,     0      ,     0     ,      0    ,       0  ,         0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772  ,         0          , 0           ,0           ,0],
          [0     ,      0   ,        0   ,        0          , 0 ,          0         ,  0       ,    0       ,    0      ,     0     ,      0   ,        0   ,        0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772           ,0  ,         0 ,          0],
          [0      ,     0    ,       0    ,       0 ,          0  ,         0 ,          0        ,   0        ,   0       ,    0      ,     0    ,       0    ,       0   ,        0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772  ,         0 ,          0],
          [0       ,    0     ,      0     ,      0  ,         0   ,        0  ,         0  ,         0         ,  0        ,   0       ,    0     ,      0     ,      0    ,       0   ,        0  ,         0 ,-0.00464772  ,0.00929545 ,-0.00464772 ,          0],
          [0        ,   0      ,     0      ,     0   ,        0    ,       0   ,        0   ,        0          , 0         ,  0        ,   0      ,     0      ,     0     ,      0    ,       0   ,        0  ,         0 ,-0.00464772 , 0.00929545 ,-0.00464772],
          [0         ,  0       ,    0       ,    0    ,       0     ,      0    ,       0    ,       0           ,0          , 0         ,  0       ,    0       ,    0      ,     0     ,      0    ,       0   ,        0  ,         0 ,-0.00464772 , 0.00929545]])

#print(A)
l = np.sqrt(2)
sf2 = 1
def k(x,xp,l):
    return sf2*np.exp(-1/(2*l**2)*np.abs(x[:,None]-xp[:,None].T)**2) # squared exponential covariance matrix
Kss = k(xs,xs,l) # The larger the covariance the more similar, the lower the covariance the less similar
Ks = k(x,xs,l)
K = k(x,x,l)
print(K)
mu_post = (Ks.T@np.linalg.inv(K))@f
K_post = Kss - Ks.T@np.linalg.inv(K)@Ks
print(K)
plt.imshow(B)
plt.colorbar()
plt.show()

plt.imshow(Kss)
plt.colorbar()
plt.show()

# Function -- multivariate_normal -- does not inverse the matrix to be used.  It expects the function to e already inversed.

#K_post[0,1] = 2
#K_post[1,0] = 2 
s = 12
fs = multivariate_normal(mean=mu_post,cov=K_post, allow_singular=True).rvs(s).T
# have yet to understand what rvs() means but right now it just sets the number of samples drawn
plt.plot(xs,fs)
plt.scatter(x,f)
plt.show()

print(len(fs))


plt.plot(xs,mu_post, 'black')
plt.fill_between(xs,mu_post + 3*np.sqrt(np.diag(K_post)),mu_post - 3*np.sqrt(np.diag(K_post)), color = 'lightgray')
plt.fill_between(xs,mu_post + 2*np.sqrt(np.diag(K_post)),mu_post - 2*np.sqrt(np.diag(K_post)), color = 'darkgray')
plt.fill_between(xs,mu_post + 1*np.sqrt(np.diag(K_post)),mu_post - 1*np.sqrt(np.diag(K_post)), color = 'gray')
plt.scatter(x,f)
plt.show()

# Use the maximum log marginal likelihood function to determine the maximm values of the hyperarameters of your covariance matrix.
"""
nl = 101
cl = np.linspace(0.1,5,nl)
marg_lik= np.zeros(nl)

def neg_log_marg_lik(y,x,l,s2):
    K = k(x,x,l) + s2*np.eye(len(x))
    return 0.5*(y.T@np.linalg.inv(K)@y + np.log(np.linalg.det(K)) + len(x)*np.log(2*np.pi))

for i in range(nl):
    marg_lik[i] = -neg_log_marg_lik(f,x,l=cl[i], s2 = 0)

idx = np.argmax(marg_lik)
lmax = cl[idx]

plt.plot(cl, marg_lik)
plt.scatter(cl[idx],marg_lik[idx])
plt.show()

Kss = k(xs,xs,lmax)
Ks = k(x,xs,lmax)
K = k(x,x,lmax)

mu_post = (Ks.T@np.linalg.inv(K))@f
K_post = Kss - Ks.T@np.linalg.inv(K)@Ks


s = 25
fs = multivariate_normal(mean=mu_post,cov=K_post, allow_singular=True).rvs(s).T
plt.plot(xs,fs)
plt.scatter(x,f)
plt.show()
"""


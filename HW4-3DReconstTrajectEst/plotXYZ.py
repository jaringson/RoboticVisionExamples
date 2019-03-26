import numpy as np
import cv2
from matplotlib import pyplot as plt

import yaml


def estimate_trajectory(t,b,tNew,zT=0):
    ones = np.ones((t.shape))
    t2 = np.multiply(t, t)
    A = np.hstack((ones.T, t.T, t2.T))
    alpha = np.linalg.inv(A.T@A)@A.T@b
    newB = (alpha[0]+alpha[1]*tNew+alpha[2]*tNew**2).T
    zIntercept = alpha[0]+alpha[1]*zT+alpha[2]*zT**2
    return newB, np.roots(np.flip(alpha)), zIntercept


with open("all_xyz.yaml", 'r') as stream:
    data_loaded = yaml.load(stream)


leftXYZ = data_loaded['left3DPoints']
rightXYZ = data_loaded['right3DPoints']

leftCameraOffSet = np.array([10,29,20])
translationL2R = np.array([-2.0278140125496083e+01, -9.5316419399415889e-02,
       1.1853745758994180e-01])
rightCameraOffset = leftCameraOffSet+translationL2R

leftXYZ = np.array(leftXYZ).reshape((-1,3)) - leftCameraOffSet
rightXYZ = np.array(rightXYZ).reshape((-1,3))

num = 35 #leftXYZ.shape[0]
print(leftXYZ.shape)
# print(leftXYZ[:num,0])

t = np.atleast_2d(np.arange(0,num,1))
tNew = np.atleast_2d(np.arange(0,leftXYZ.shape[0]+20,1))


leftZNew, leftZRoots, _ = estimate_trajectory(t,leftXYZ[:num,2],tNew)
leftZRoot = np.min(leftZRoots)
if np.min(leftZRoots) < 0:
    leftZRoot = np.max(leftZRoots)

leftXNew, _, leftXZInter = estimate_trajectory(t,leftXYZ[:num,0],tNew,leftZRoot)
leftYNew, _, leftYZInter = estimate_trajectory(t,leftXYZ[:num,1],tNew,leftZRoot)



print("Z Roots Estimate:", leftZRoots, leftZRoot)
print("X Estimate at Z=0:", leftXZInter)
print("Y Estimate at Z=0:", leftYZInter)

#1st Plot
fig = plt.figure()
plt.subplots_adjust(hspace = 0.5, wspace=0.3)
ax = fig.add_subplot(311)
plt.plot(t.T,leftXYZ[:num,0],'b')
plt.plot(tNew.T,leftXNew,'r--')
plt.plot(leftZRoot,leftXZInter,'gx')
plt.ylabel('x (inches)')
ax.set_title('Left Camera 3D Estimates', loc='center')

ax = fig.add_subplot(312)
plt.plot(t.T,leftXYZ[:num,1])
plt.plot(tNew.T,leftYNew,'r--')
plt.plot(leftZRoot,leftYZInter,'gx')
plt.ylabel('y (inches)')

ax = fig.add_subplot(313)
plt.plot(t.T,leftXYZ[:num,2])
plt.plot(tNew.T,leftZNew,'r--')
plt.plot(leftZRoot,0,'gx')
plt.ylabel('z (inches)')
plt.xlabel('t')


#2nd Plot
fig2 = plt.figure()
ax = fig2.add_subplot(121)
plt.plot(leftXYZ[:num,0],leftXYZ[:num,2],'b')
plt.plot(leftXNew,leftZNew,'r--')
plt.plot(leftXZInter,0,'gx')
plt.xlabel('x (inches)')
plt.ylabel('z (inches)')

ax = fig2.add_subplot(122)
plt.plot(leftXYZ[:num,1],leftXYZ[:num,2],'b')
plt.plot(leftYNew,leftZNew,'r--')
plt.plot(leftYZInter,0,'gx')
plt.xlabel('y (inches)')

plt.show()

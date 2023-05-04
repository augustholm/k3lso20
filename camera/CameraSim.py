import numpy as np
from matplotlib import pyplot as plt
import cv2

x = 320
y = 240
floorLimit = 120
margin = 0.01

def getFrame():
    #Get frame from camera
    depthImage = np.loadtxt('depth_object.csv', delimiter=',')
    return depthImage

def calibration(image):
    floorV = np.zeros((y))
    #return floorV
    for i in range(floorLimit, y):
        floorV[i] = image[i, int(x/2)]
    return floorV

def findObstacle(image, floorV):
    obsVL = np.zeros((2, x * y))
    numObs = 0
    obsM = np.zeros((y,x))
    for i in range(0,x):
        for j in range(0,y):
            if (floorV[j] - margin) > image[j,i] > 0:
                obsM[j,i] = 1
                obsVL[0, numObs] = image[j, i]
                # obsVL[1,numObs] = 0 #Add angle to obstacle from camera - needs to be calculated
                numObs += 1

    obsV = obsVL[0:1, 0:numObs]
    return obsM, obsV

def saveCalibration(floorV):
    calFile = open("calibration.txt", 'w')
    for i in range(y):
        calFile.write(str(floorV[i]) + "\n")
    calFile.close()

def getCalibration():
    calFile = open("calibration.txt", 'r')
    floorV = calFile.read().split()
    for i in range(y):
        floorV[i] = float(floorV[i])
    return floorV

im = getFrame()

#floorVector = calibration(im)
#saveCalibration(floorVector)
#floorVector = getCalibration()
floorMetrix = np.loadtxt('depth_plane.csv', delimiter=',')
floorVector = floorMetrix[:,1]
print(floorVector)
plt.plot(floorVector)
plt.show()

obstacleIm, obstacleV = findObstacle(im, floorVector)
print(obstacleV)

depthColormap = cv2.applyColorMap(cv2.convertScaleAbs(im, alpha=30), cv2.COLORMAP_JET)
colorObstacle = cv2.applyColorMap(cv2.convertScaleAbs(obstacleIm, alpha=300), cv2.COLORMAP_JET)

images = np.hstack((depthColormap, colorObstacle))

cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RealSense', images)
cv2.waitKey(6000)
import pyrealsense2 as rs
import numpy as np
from matplotlib import pyplot as plt
import cv2

pipeline = rs.pipeline()
config = rs.config()

x = 640
y = 480
floorLimit = 240
margin = 10
numObs = -1

config.enable_stream(rs.stream.depth, x, y, rs.format.z16, 30)

pipeline.start(config)


def getFrame():
    #Wait for frames
    for i in range(0,8):
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            print("No frame")
            return

    depth_image = np.asanyarray(depth_frame.get_data())
    return depth_image

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', depth_colormap)
    cv2.waitKey(1)

def calibration(image):
    floorV = np.zeros((y))
    #return floorV
    for i in range(floorLimit, y):
        floorV[i] = image[i, int(x/2)]
    return floorV

def findObstacle(image, floorV):
    obsM = np.zeros((y,x))
    num = 0
    for i in range(0,x):
        for j in range(0,y):
            if (floorV[j] - margin) > image[j,i] > 0:
                obsM[j,i] = 1
                num += 1
    return obsM, num

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
floorVector = getCalibration()
plt.plot(floorVector)
plt.show()

obstacleIm, numObs = findObstacle(im, floorVector)
print(numObs)

depthColormap = cv2.applyColorMap(cv2.convertScaleAbs(im, alpha=0.03), cv2.COLORMAP_JET)
colorObstacle = cv2.applyColorMap(cv2.convertScaleAbs(obstacleIm, alpha=30), cv2.COLORMAP_JET)

images = np.hstack((depthColormap, colorObstacle))

cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RealSense', images)
cv2.waitKey(600)
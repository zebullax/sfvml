import cv2
import os
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

# Constant starting position for both players
player1StartPos = (300, 420)
player2StartPos = (980, 420)

def extractFramesFromMp4(filename, outputFolder = '.'):
    # MAke the output folder
    outputFrameFolder = os.path.join(outputFolder, filename.replace('.', '_'))
    os.makedirs(outputFrameFolder, exist_ok = True)
    
    vidcap = cv2.VideoCapture(filename)
    success,image = vidcap.read()
    count = 0
    framePerSecond = 24 # Actually get it
    success = True
    while success:
          success,image = vidcap.read()
          cv2.imwrite(os.path.join(outputFrameFolder, "frame%010d.jpg" % count), image)     # save frame as JPEG file, TODO hardcoded 10
          count += 1
          if (count % framePerSecond == 0):
              print ('.', end = '', flush = True)

def processFirstFrame(filename, outputFolder = '.'):
    firstFrameFilename = os.path.join(outputFolder, filename.replace('.', '_'))
    firstFrameFilename = os.path.join(firstFrameFilename, "frame%010d.jpg" % 0)
    img=mpimg.imread(firstFrameFilename)
    imgplot = plt.imshow(img)
    # Overlay some cross on top of it for visual check
    plt.plot(player1StartPos[0], player1StartPos[1], 'r+', markersize=20)
    plt.plot(player2StartPos[0], player2StartPos[1], 'g+', markersize=20)
    plt.show()

if __name__ == '__main__':
    #print("Extracting frame from ", sys.argv[1])
    #extractFramesFromMp4(sys.argv[1])
    print("Processing first frame")
    processFirstFrame(sys.argv[1])


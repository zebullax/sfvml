import cv2
import os
import sys

def videoToFrame(filename, outputFolder = '.'):
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
          cv2.imwrite(os.path.join(outputFrameFolder, "frame%d.jpg" % count), image)     # save frame as JPEG file
          count += 1
          if (count % framePerSecond == 0):
              print ('.', end = '', flush = True)

if __name__ == '__main__':
    print("Extracting frame from ", sys.argv[1])
    videoToFrame(sys.argv[1])

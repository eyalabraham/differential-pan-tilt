###############################################################################
# 
# camtest.py
#
#   Python OpenCV webcam test
#
#   May 2, 2020
#
###############################################################################

import sys
import numpy as np
import cv2
import time

###############################################################################
#
# main()
#
def main():
    """Webcam test and display webcam video until ESC key is pressed."""

    #
    # initialization
    #
    (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

    #
    # Initialize start time and frame count.
    # Initialize a reference start time for the CSV output trace
    #
    fps_text = '?? Fps'
    frame_count = 0
    start = time.time()
    ref_time = time.time()

    #
    # link event callback function
    #
    cv2.namedWindow('image', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)

    #
    # setup font for overlay text 
    #
    font = cv2.FONT_HERSHEY_SIMPLEX

    #
    # Open the capture device and print some
    # useful properties.
    # This tracker program will leave the default webcam frame size
    # that is 640x480 for the webcam I am using.
    #
    cap = cv2.VideoCapture(0)
    
    if cap.isOpened():
        #cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 320)
        #cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
        
        frameWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        frameHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        
        print(f'frame: width {frameWidth}, height {frameHeight}')
        
        frameCenterX = int(frameWidth/2)
        frameCenterY = int(frameHeight/2)
    else:
        sys.exit()

    #
    # frame capture and processing loop
    #
    while(True):
        #
        # capture a frame
        # cover to appropriate color space to improve detection
        # in different lighting conditions
        #
        cap_ok, frame = cap.read()
        if not cap_ok:
            break

        #
        # Calculate and display FPS.
        # Use the 10sec interval to also poll the NXT for battery level.
        #
        frame_count = frame_count + 1
        end = time.time()
        measure_interval = end - start
        if measure_interval > 10:
            fps = frame_count / measure_interval
            fps_text = f'{fps:.1f} Fps'
            frame_count = 0
            start = time.time()

        #
        # Display the resulting frame
        #
        cv2.putText(frame, fps_text, (1, 60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, 'press ESC to exit', (frameCenterX-20, frameCenterY), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow('image', frame)
              
        #
        #   key input mode/command
        #
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

    #
    # Release the capture when done
    #
    cap.release()
    cv2.destroyAllWindows()

#
# Startup
#
if __name__ == '__main__':
    main()

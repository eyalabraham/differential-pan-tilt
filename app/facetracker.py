###############################################################################
# 
# facetracker.py
#
#   Python OpenCV and differential pan-tilt program for identifying and tracking a face
#
#   The program requires a connected webcam and a differential pan-tilt system https://sites.google.com/site/eyalabraham/differential-pan-tilt .
#   The program has three operating modes:
#   'm' key Mark: enter this mode while holding an object in front of the webcam,
#       and use the mouse to select it with a rectangle.
#   's' key Show: this is a test mode that shows the tracked object. Use this mode
#       to test tracking on the display.
#   't' key Track: this mode activates the motors and sends power controls to them.
#       The Track mode will be halted (red bounding box) if the object is lost
#       Tracking will resume automatically when the tracker algorithm re-detects
#       the object. With the default selected tracker algorithm set to KCF
#       simply bringing the object back into the the view of the red bounding box.
#
#   May 8, 2020
#
###############################################################################

import sys
import time

import cv2

import ptcmd

###############################################################################
#
# main()
#
def main():
    """Control function that reads webcam, and tracks a marked object."""
    global x0, y0, x1, y1, drawing, mode, frame, bbox, tracker, tracker_initialized
    global MODE_MARK

    #
    # Initialization
    #
    (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
    
    MODE_TRACK = 0          # track an object
    MODE_SHOW = 1           # only show tracking markers on video

    tracker_initialized = False
    bbox = (0, 0, 0, 0)
    last_good_bbox = bbox
    
    mode = MODE_SHOW
    mode_text = 'Show'
    fps_text = '?? Fps'
    faces_text = 'Faces:'
    cvs_title_printed = False
    
    drawing = False         # true if mouse is pressed
    x0, y0 = -1, -1
    x1, y1 = -1, -1
    
    #
    # PID constants for pan and tilt PID controller.
    # These constants are tuned for the worm-gear mechanical setup
    #
    pan_P = 5.0
    pan_I = 0.0
    pan_D = 1.0
    
    tilt_P = 10.0
    tilt_I = 0.0
    tilt_D = 2.0

    print(' t - track\n s - display tracking marker only\n ESC - quit')

    #
    # Named display window with properties
    #
    cv2.namedWindow('image', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)

    #
    # Setup font for overlay text 
    #
    font = cv2.FONT_HERSHEY_SIMPLEX

    #
    # Open the capture device and print some useful properties.
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
    # Set up tracker.
    # 
    cascPath = '/home/eyal/data/projects/computer-vision/haarcascade_frontalface_alt.xml'
    faceCascade = cv2.CascadeClassifier(cascPath)

    #
    # Open pan-tilt controller
    #
    controller = ptcmd.PTCMD(baud=57600)
    #controller.home()

    #
    # Initialize start time and frame count.
    # Initialize a reference start time for the CSV output trace
    #
    frame_count = 0
    start = time.time()
    ref_time = time.time()

    #
    # Frame capture and tracking PID loop
    #
    while(True):
        #
        # Capture a frame
        #
        cap_ok, frame = cap.read()
        if not cap_ok:
            break

        #
        # Process frame and detect face
        #
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            #flags=cv2.cv.CV_HAAR_SCALE_IMAGE
            #flags=cv2.CV_HAAR_SCALE_IMAGE
        )

        #
        # Track a face or faces
        # Calculate center of mass of face (or faces) and also display
        # rectangles to identify detection.
        # Closer, larger faces, have more 'mass' and will draw the
        # center towards them
        #
        face_count = 0
        mass_sum = 0
        center_x = 0.0
        center_y = 0.0
        for (x, y, w, h) in faces:
            # Sum the mass, which is the detection area
            mass = w * h
            mass_sum = mass_sum + mass
            # Accumulate weigh sums for weighted average
            center_x = center_x + mass * (x + w / 2)
            center_y = center_y + mass * (y + h / 2)
            # Draw a rectangle around the detected face
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)
            face_count = face_count + 1

        faces_text = f'Faces: {face_count}'
        #
        # If the tracker found any face or faces, then:
        # - draw an arrow line from frame center to the center of tracking point
        # - calculate pan and tilt error from frame center
        #               
        if face_count > 0:
            object_x = int(center_x / mass_sum)
            object_y = int( center_y / mass_sum)

            err_pan_i = frameCenterX - object_x
            err_tilt_i = frameCenterY - object_y

            cv2.arrowedLine(frame, (frameCenterX, frameCenterY), (object_x, object_y), (0, 255, 0), 1)

            if mode == MODE_TRACK:
                #
                # First apply an exponential filter to the tracker position error.
                # info: https://en.wikipedia.org/wiki/Exponential_smoothing
                # Then do PID tracking for pan-tilt control.
                #
                err_pan = exp_filter_pan(err_pan_i)
                err_tilt = exp_filter_tilt(err_tilt_i)
                control_pan = pid_pan(err_pan, pan_P, pan_I, pan_D)
                control_tilt = pid_tilt(err_tilt, tilt_P, tilt_I, tilt_D)
                
                # Uncomment one of the following lines
                # in order to isolate pan or tilt for PID tuning/testing
                #control_pan = 0
                #control_tilt = 0           
                controller.run(control_pan, control_tilt)
                
                #
                # Print out some data in a CSV compatible format for graphing
                #
                #if not cvs_title_printed:
                #    print('rel_time,err_tilt_i,err_tilt,control_tilt,err_pan_i,err_pan,control_pan')
                #    cvs_title_printed = True
                #   
                #now = time.time() - ref_time 
                #print(f'{now:.2f},{err_tilt_i:.2f},{err_tilt:.2f},{control_tilt},{err_pan_i:.2f},{err_pan:.2f},{control_pan}')

        #
        # If tracking is lost for some reason then display a red cross
        # and stop the motors
        # TODO Smooth lapses in face detection that
        #      are sporadic and last less that TBD [mili-sec]
        #
        else:
            #controller.stop()
            controller.run(0, 0)
            
        #
        # Calculate and display FPS.
        #
        frame_count = frame_count + 1
        end = time.time()
        measure_interval = end - start
        if measure_interval > 10:
            fps = frame_count / measure_interval
            fps_text = '{:.2f} Fps'.format(fps)
            frame_count = 0
            start = time.time()

        #
        # Add text and other markers to the image
        #       
        cv2.putText(frame, mode_text, (1, 20), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, faces_text, (1, 40), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, fps_text, (1, 60), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        
        #
        # Display the resulting frame
        #
        cv2.imshow('image', frame)
              
        #
        #   key input mode/command
        #
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord('t'):
            mode_text = 'Track'
            mode = MODE_TRACK
        elif key == ord('s'):
            mode_text = 'Show'
            mode = MODE_SHOW
        else:
            pass

    #
    # When done, stop motors and release the capture.
    #
    controller.stop()
    controller.close()

    cap.release()
    cv2.destroyAllWindows()

###########################################################
#
# exp_filter_pan()
#
def exp_filter_pan(err_i, alpha = 0.1):
    """
    Calculate Exponential filter for Pan error and returns filtered value.

    param:  current error, filter's alpha (default 0.1)
    return: filtered error float value
    """

    filtered_err = (alpha * err_i) + ((1 - alpha) * exp_filter_pan.filtered_err_prev)
    exp_filter_pan.filtered_err_prev = filtered_err
    return filtered_err
#
exp_filter_pan.filtered_err_prev = 0.0

###########################################################
#
# exp_filter_tilt()
#
def exp_filter_tilt(err_i, alpha = 0.1):
    """
    Calculate Exponential filter for Tilt error and returns filtered value.

    param:  current error, filter's alpha (default 0.1)
    return: filtered error float value
    """

    filtered_err = (alpha * err_i) + ((1 - alpha) * exp_filter_tilt.filtered_err_prev)
    exp_filter_tilt.filtered_err_prev = filtered_err
    return filtered_err
#
exp_filter_tilt.filtered_err_prev = 0.0

###########################################################
#
# pid_pan()
#
def pid_pan(error, Kp = 0.0, Ki = 0.0, Kd = 0.0):
    """
    PID controller for pan direction error.
    Written as a function with an attribute.
    The function does its own time measurements between calls
    to get delta time in mSec for controller calculations.
    It accepts position error and calculates PID control output for Pan direction

    param:  Process error, Kp, Ki, Kd (PID constants) 
    return: Signed integer pan-rate output
    """

    e2 = cv2.getTickCount()
    delta_time = (e2 - pid_pan.e1) / cv2.getTickFrequency()
    pid_pan.e1 = e2
    pid_pan.integral = pid_pan.integral + (error * delta_time)
    derivative = (error - pid_pan.previous_error) / delta_time
    pid_pan.previous_error = error

    output = int(-1.0 * round((Kp * error) + (Ki * pid_pan.integral) + (Kd * derivative)))

    return output
#
pid_pan.previous_error = 0.0
pid_pan.integral = 0.0
pid_pan.e1 = cv2.getTickCount()

###########################################################
#
# pid_tilt()
#
def pid_tilt(error, Kp = 0.0, Ki = 0.0, Kd = 0.0):
    """
    PID controller for tilt direction error.
    Written as a function with an attribute.
    The function does its own time measurements between calls
    to get delta time in mSec for controller calculations.
    It accepts position error and calculates PID control output for tilt direction

    param:  Process error, Kp, Ki, Kd (PID constants) 
    return: Signed integer tilt-rate output
    """

    e2 = cv2.getTickCount()
    delta_time = (e2 - pid_tilt.e1) / cv2.getTickFrequency()
    pid_tilt.e1 = e2
    pid_tilt.integral = pid_tilt.integral + (error * delta_time)
    derivative = (error - pid_tilt.previous_error) / delta_time
    pid_tilt.previous_error = error

    output = int(round((Kp * error) + (Ki * pid_tilt.integral) + (Kd * derivative)))

    return output
#
pid_tilt.previous_error = 0.0
pid_tilt.integral = 0.0
pid_tilt.e1 = cv2.getTickCount()


#
# Startup
#
if __name__ == '__main__':
    main()

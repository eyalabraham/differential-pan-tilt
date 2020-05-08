###############################################################################
# 
# objtracker.py
#
#   Python OpenCV and differential pan-tilt program for tracking object with a webcam.
#   It is based on code from [https://www.learnopencv.com/object-tracking-using-opencv-cpp-python/]
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
#   May 4, 2020
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
    MODE_MARK = 2           # mark region color to track
    MODE_TRACK_HOLD = 3     # temporarily suspend tracking until object is recaptured

    tracker_initialized = False
    bbox = (0, 0, 0, 0)
    last_good_bbox = bbox
    
    mode = MODE_SHOW
    mode_text = 'Show'
    fps_text = '?? Fps'
    cvs_title_printed = False
    
    drawing = False         # true if mouse is pressed
    x0, y0 = -1, -1
    x1, y1 = -1, -1
    
    #
    # PID constants for pan and tilt PID controller.
    # These constants are tuned for the worm-gear mechanical setup
    #
    pan_P = 2.0
    pan_I = 0.0
    pan_D = 0.0
    
    tilt_P = 8.0
    tilt_I = 0.0
    tilt_D = 1.0

    print(' m - mark color region to track\n t - track\n s - display tracking marker only\n ESC - quit')

    #
    # Link event callback function
    #
    cv2.namedWindow('image', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('image', mark_rect)

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
    # Tracker algorithm is hard coded here to default tracker KCF.
    # 
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
    tracker_type = tracker_types[2]
 
    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN':
        tracker = cv2.TrackerGOTURN_create()

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
        # Operations on the captured image done here.
        # If marking a section on the frame for tracking
        # then only display the ROI selection
        #
        if mode == MODE_MARK:
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 1)

        #
        # If tracking or only displaying object tracking information
        # then draw the tracking markers on the frame before it is displayed.
        # Only do this if the tracker was initialized
        #
        elif tracker_initialized:
            #
            # Update the tracker with the newly acquired frame.
            #
            track_ok, bbox = tracker.update(frame)

            #
            # If the tracker update was successful, object still being tracked, then
            # update the previous bounding box position and proceed to:
            # - display the tracker bounding box
            # - an arrow line from frame center to the object center
            # - calculate pan and tilt error from frame center
            #
            if track_ok:
                last_good_bbox = bbox
                
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (255, 0, 0), 2)

                object_x = int(bbox[0] + bbox[2]/2)
                object_y = int(bbox[1] + bbox[3]/2)

                err_pan_i = frameCenterX - object_x
                err_tilt_i = frameCenterY - object_y

                cv2.arrowedLine(frame, (frameCenterX, frameCenterY), (object_x, object_y), (255, 0, 0), 2)

                if mode == MODE_TRACK_HOLD:
                    mode = MODE_TRACK
            
            #
            # If tracking is lost for some reason then use the last location
            # of the bounding box to mark that last location with a red box
            #
            else:
                p1 = (int(last_good_bbox[0]), int(last_good_bbox[1]))
                p2 = (int(last_good_bbox[0] + last_good_bbox[2]), int(last_good_bbox[1] + last_good_bbox[3]))
                cv2.rectangle(frame, p1, p2, (0, 0, 255), 1)

                if mode == MODE_TRACK:
                    mode = MODE_TRACK_HOLD

        #
        # Only when in tracking mode activate the motors,
        # and use PID calculations to control the pan-tilt device
        #
        if mode == MODE_TRACK and tracker_initialized:
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
            if not cvs_title_printed:
                print('rel_time,err_tilt_i,err_tilt,control_tilt,err_pan_i,err_pan,control_pan')
                cvs_title_printed = True
               
            now = time.time() - ref_time 
            print(f'{now:.2f},{err_tilt_i:.2f},{err_tilt:.2f},{control_tilt},{err_pan_i:.2f},{err_pan:.2f},{control_pan}')
            

        #
        # This section will turn motors off
        # when not in tracking mode. Note above code lines
        # force MODE_TRACK_HOLD if no objects exist or if more than one
        # object is detected. This state will shut motors off.
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
        if mode == MODE_TRACK_HOLD:
            cv2.putText(frame, mode_text, (1, 20), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, mode_text, (1, 20), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        cv2.putText(frame, tracker_type, (1, 40), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, fps_text, (1, 60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        
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
        elif key == ord('m') and not tracker_initialized:
            x0,y0  = -1,-1
            x1,y1  = -1,-1
            mode_text = 'Mark'
            mode = MODE_MARK
        elif key == ord('t'):
            mode_text = 'Track'
            if tracker_initialized:
                mode = MODE_TRACK
            else:
                mode = MODE_TRACK_HOLD
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

###########################################################
#
# mark_rect()
#
def mark_rect(event, x, y, flags, param):
    """
    Mouse callback that marks a frame region for tracker initialization.

    param:  event type, mouse coordinates and event parameters
    return: nothing, marks a frame region for tracker initialization
    """

    global x0, y0, x1, y1, drawing, mode, frame, bbox, tracker, tracker_initialized
    global MODE_MARK

    if mode != MODE_MARK:
        return
        
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        x0, y0 = x, y
        x1, y1 = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            x1, y1 = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

        #
        # Convert any start (x0,y0) and end (x1,y1) points to be
        # a top-left to bottom right pair.
        # Extract ROI and initialize the tracker
        #
        if x0 == x1 or y0 == y1 or x0 < 0 or x1 < 0 or y0 < 0 or y1 < 0:
            return
        if x0 > x1:
            x0, x1 = x1, x0
        if y0 > y1:
            y0, y1 = y1, y0

        bbox = (x0, y0, x1-x0, y1-y0)
        tracker_initialized = tracker.init(frame, bbox)
        print('tracker.init()', tracker_initialized)

#
# Startup
#
if __name__ == '__main__':
    main()

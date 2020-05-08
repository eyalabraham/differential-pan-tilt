# Differential pan-tilt

The original design came from [this video](https://www.youtube.com/watch?v=Su6O6155UJM). The designer published the 3D CAD files but they were not compatible with my CAD software so [I recreated the mechanical design](https://grabcad.com/library/differential-pan-tilt-1) with FreeCAD and adapted it to my TEAC stepper motors. The motors are driven by a pair of A4988 stepper motor drivers that are controlled by NEC V25 SBC I had from an older project. The controller board runs a drive+controller application that can be accessed through a serial link and managed through either a serial console (e.g. minicom) or commands issues by a remote program (I use a set of Python scripts for different applications). This setup allows the controller to maintain a simple set of command primitives that are use by higher level code to implement more complex applications such as integration with OpenCV for object tracking.  

Web page: (https://sites.google.com/site/eyalabraham/differential-pan-tilt)  
Mechanical design: (https://grabcad.com/library/differential-pan-tilt-1)  
Software: (https://github.com/eyalabraham/differential-pan-tilt)  
YouTube: (https://youtu.be/LF1JT0Pmddc)  

## System

- Controller and [driver software](controller/README.md)
- Python [applications and modules](app/README.md)


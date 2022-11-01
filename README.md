# StepFallDetector
The Arduino code of my fall detector device for the elderly, "STEP,"
###Description
Utilizes an MPU6050 accelerometer & gyroscope sensor, to sense the acceleration and the tilt of the device. The angle is found with a kalman filter. Should a spike in acceleration occur, the device checks whether the angle with respect to the horizontal has significantly changed in the subsequent seconds. This would mean that the person has fallen, since they're lying down, causing a change in the angle. Note that the device is worn on the belt, and the pelvis area in the body usually stays upright in normal movements. The double checking of both acceleration and angle reduces falls positives, as movements that induce acceleration (i.e. jumping) don't involve angle changes, while movements like lying in a bed only involve angle change. The only motion where an angle change follows a spike in acceleration is that of falling.


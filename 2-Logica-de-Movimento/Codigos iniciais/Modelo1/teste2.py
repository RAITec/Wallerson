import sim
from time import sleep as delay
import numpy as np
import cv2
import sys

print('Program started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

lSpeed = 0
rSpeed = 0

fw_ct = 0
ri_ct = 0
le_ct = 0

if (clientID != -1):
    print('Connected to remote API server')

else:
    sys.exit('Failed connecting to remote API server')

delay(1)

errorCode, left_motor_handle = sim.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

errorCode, camera_handle = sim.simxGetObjectHandle(
    clientID, 'cam1', sim.simx_opmode_oneshot_wait)
delay(1)
# errorCode, sensor1 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1', sim.simx_opmode_oneshot_wait)

# returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensor1, sim.simx_opmode_streaming)
returnCode, resolution, image = sim.simxGetVisionSensorImage(
    clientID, camera_handle, 0, sim.simx_opmode_streaming)
delay(1)

try:
    while (1):
        returnCode, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera_handle, 0, sim.simx_opmode_buffer)
        im = np.array(image, dtype=np.uint8)
        im.resize([resolution[0], resolution[1], 3])

        im = cv2.flip(im, 0)
        im = cv2.rotate(im, cv2.ROTATE_90_COUNTERCLOCKWISE)
        im = cv2.resize(im, (512, 512))
        im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)

        errorCode = sim.simxSetJointTargetVelocity(
            clientID, left_motor_handle, lSpeed, sim.simx_opmode_streaming)
        errorCode = sim.simxSetJointTargetVelocity(
            clientID, right_motor_handle, rSpeed, sim.simx_opmode_streaming)

        cv2.imshow("data", im)
        com = cv2.waitKey(1)
        if (com == ord('q')):
            break
        elif (com == ord('w')):
            lSpeed = 1.5
            rSpeed = 1.5
            cv2.imwrite(f"../train_data/forward/fw_{fw_ct}.jpg", im)
            fw_ct += 1
        elif (com == ord('a')):
            lSpeed = -1.0
            rSpeed = 1.5
            cv2.imwrite(f"../train_data/left/le_{le_ct}.jpg", im)
            le_ct += 1
        elif (com == ord('d')):
            lSpeed = 1.5
            rSpeed = -1.0
            cv2.imwrite(f"../train_data/right/ri_{ri_ct}.jpg", im)
            ri_ct += 1
        elif (com == ord('s')):
            lSpeed = -1.5
            rSpeed = -1.5
        else:
            lSpeed = 0
            rSpeed = 0
        com = 'o'

    cv2.destroyAllWindows()
except:
    cv2.destroyAllWindows()
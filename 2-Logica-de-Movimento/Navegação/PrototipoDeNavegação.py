import sim
import time
import numpy as np

def get_robot_position(clientID, robotHandle):
    returnCode, position = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    return position

def get_robot_orientation(clientID, robotHandle):
    returnCode, orientation = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    return orientation

def set_robot_position(clientID, robotHandle, position, orientation):
    returnCode = sim.simxSetObjectPosition(clientID, robotHandle, -1, position, sim.simx_opmode_oneshot_wait)
    returnCode = sim.simxSetObjectOrientation(clientID, robotHandle, -1, orientation, sim.simx_opmode_oneshot_wait)

def move_forward(clientID, l_wheel, r_wheel, speed):
    sim.simxSetJointTargetVelocity(clientID, l_wheel, speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, r_wheel, speed, sim.simx_opmode_oneshot)

def stop_robot(clientID, l_wheel, r_wheel):
    sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot)

def turn_left(clientID, l_wheel, r_wheel, speed):
    sim.simxSetJointTargetVelocity(clientID, l_wheel, -speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, r_wheel, speed, sim.simx_opmode_oneshot)

def turn_right(clientID, l_wheel, r_wheel, speed):
    sim.simxSetJointTargetVelocity(clientID, l_wheel, speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, r_wheel, -speed, sim.simx_opmode_oneshot)

def avoid_obstacles(clientID, l_wheel, r_wheel, sensors, speed, proximity_threshold, robotHandle, target_position):
    for sensor in sensors:
        returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensor, sim.simx_opmode_oneshot_wait)
        if detectionState:
            distance = np.linalg.norm(detectedPoint)
            if distance < proximity_threshold:
                stop_robot(clientID, l_wheel, r_wheel)
                current_position = get_robot_position(clientID, robotHandle)
                current_orientation = get_robot_orientation(clientID, robotHandle)[2]
                target_angle = calculate_direction(current_position, target_position)
                angle_difference = calculate_angle_difference(target_angle, current_orientation)

                if angle_difference > 0:
                    turn_left(clientID, l_wheel, r_wheel, speed * 0.4) # caso seja necessário, mude a velocidade de rotação de desvio de obstáculo
                else:
                    turn_right(clientID, l_wheel, r_wheel, speed * 0.4) # caso seja necessário, mude a velocidade de rotação de desvio de obstáculo

                time.sleep(0.6)
                move_forward(clientID, l_wheel, r_wheel, speed)
                return

def calculate_direction(current_position, target_position):
    direction = np.array(target_position) - np.array(current_position)
    angle = np.arctan2(direction[1], direction[0])
    return angle

def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def calculate_angle_difference(target_angle, current_angle):
    return normalize_angle(target_angle - current_angle)

def move_towards_target(clientID, robotHandle, l_wheel, r_wheel, speed, target_position):
    current_position = get_robot_position(clientID, robotHandle)
    current_orientation = get_robot_orientation(clientID, robotHandle)[2]
    target_angle = calculate_direction(current_position, target_position)
    angle_difference = calculate_angle_difference(target_angle, current_orientation)

    if abs(angle_difference) > 0.6:  #margem de erro para seguir até o alvo 

        if angle_difference > 0:
            turn_left(clientID, l_wheel, r_wheel, speed * 0.1)  # caso seja necessário, mude a velocidade de rotação de busca de orientção
        else:
            turn_right(clientID, l_wheel, r_wheel, speed * 0.1)  # caso seja necessário, mude a velocidade de rotação de busca de orientção
    else:
        move_forward(clientID, l_wheel, r_wheel, speed)

print('Program started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')

    robotname = 'Pioneer_p3dx'
    returnCode, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)
    returnCode, l_wheel = sim.simxGetObjectHandle(clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, r_wheel = sim.simxGetObjectHandle(clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)

    sensors = []
    for i in range(1, 17):
        returnCode, sensorHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(i), sim.simx_opmode_oneshot_wait)
        sensors.append(sensorHandle)

    initial_position = [-4.0, -4.0, 0.0] # posição inicial
    initial_orientation = [0.0, 0.0, 0.0] # orientação inicial 
    set_robot_position(clientID, robotHandle, initial_position, initial_orientation)

    waypoints = [ # definição da rota 
        [-4.0, 4.0, 0.0],  
        [-4.0, -4.0, 0.0],   
        [4.0, -4.0, 0.0],  
        [4.0, 4.0, 0.0],
        [4.0, -4.0, 0.0],
        [0.0, -4.0, 0.0],
        [0.0, 4.0, 0.0],
        [0.0, -4.0, 0.0],
        [-4.0, -4.0, 0.0]
    ]

    speed = 3.0  # Velocidade
    proximity_threshold = 0.5  # distancia para desviar de obstaculos

    try:
        for target_position in waypoints:
            while True:
                avoid_obstacles(clientID, l_wheel, r_wheel, sensors, speed, proximity_threshold, robotHandle, target_position)
                move_towards_target(clientID, robotHandle, l_wheel, r_wheel, speed, target_position)
                current_position = get_robot_position(clientID, robotHandle)
                distance_to_target = np.linalg.norm(np.array(target_position) - np.array(current_position))
                if distance_to_target < 0.5:  # margem de erro para chegar no alvo
                    stop_robot(clientID, l_wheel, r_wheel)
                    break
                time.sleep(0.1)
    except KeyboardInterrupt:
        stop_robot(clientID, l_wheel, r_wheel)

    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)

else:
    print('Failed to connect to remote API server')

print('Program ended')

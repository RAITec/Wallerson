import sim
from time import sleep as delay
import numpy as np
import cv2
import sys

print('Program started')
sim.simxFinish(-1)  # Fecha todas as conexões existentes
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')
else:
    sys.exit('Failed connecting to remote API server')

delay(1)

# Obtém os handles dos motores
errorCode, left_motor_handle = sim.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

# Obtém o handle da câmera
errorCode, camera_handle = sim.simxGetObjectHandle(
    clientID, 'cam1', sim.simx_opmode_oneshot_wait)

# Obtém os handles dos sensores ultrassônicos
errorCode, front_sensor_handle = sim.simxGetObjectHandle(
    clientID, 'sensor_front', sim.simx_opmode_oneshot_wait)
errorCode, right_sensor_handle = sim.simxGetObjectHandle(
    clientID, 'sensor_right', sim.simx_opmode_oneshot_wait)
errorCode, left_sensor_handle = sim.simxGetObjectHandle(
    clientID, 'sensor_left', sim.simx_opmode_oneshot_wait)

delay(1)

# Inicia a captura de imagens
returnCode, resolution, image = sim.simxGetVisionSensorImage(
    clientID, camera_handle, 0, sim.simx_opmode_streaming)
# Inicia a captura dos dados dos sensores ultrassônicos
returnCode, front_detectionState, front_detectedPoint, _, _ = sim.simxReadProximitySensor(
    clientID, front_sensor_handle, sim.simx_opmode_streaming)
returnCode, right_detectionState, right_detectedPoint, _, _ = sim.simxReadProximitySensor(
    clientID, right_sensor_handle, sim.simx_opmode_streaming)
returnCode, left_detectionState, left_detectedPoint, _, _ = sim.simxReadProximitySensor(
    clientID, left_sensor_handle, sim.simx_opmode_streaming)
delay(1)

# Função para girar o robô por um ângulo específico
def turn_angle(clientID, left_motor_handle, right_motor_handle, angle_deg):
    # Define a velocidade de rotação
    turn_speed = 1.0  # Velocidade de rotação das rodas
    turn_duration = abs(angle_deg) / 90  # Duração do giro para 90 graus
    
    if angle_deg > 0:
        lSpeed = turn_speed
        rSpeed = -turn_speed
    else:
        lSpeed = -turn_speed
        rSpeed = turn_speed

    # Define a velocidade das rodas para iniciar a rotação
    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, lSpeed, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_motor_handle, rSpeed, sim.simx_opmode_streaming)
    
    delay(turn_duration)  # Aguarda até completar o giro

    # Para as rodas após completar o giro
    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)

try:
    while True:
        # Captura de imagem da câmera
        returnCode, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera_handle, 0, sim.simx_opmode_buffer)
        
        if returnCode == sim.simx_return_ok:
            im = np.array(image, dtype=np.uint8)
            im.resize([resolution[1], resolution[0], 3])  # Corrige a ordem das dimensões
            im = cv2.flip(im, 0)
            im = cv2.rotate(im, cv2.ROTATE_90_COUNTERCLOCKWISE)
            im = cv2.resize(im, (512, 512))
            im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
            cv2.imshow("data", im)

        # Leitura dos sensores ultrassônicos
        returnCode, front_detectionState, front_detectedPoint, _, _ = sim.simxReadProximitySensor(
            clientID, front_sensor_handle, sim.simx_opmode_buffer)
        returnCode, right_detectionState, right_detectedPoint, _, _ = sim.simxReadProximitySensor(
            clientID, right_sensor_handle, sim.simx_opmode_buffer)
        returnCode, left_detectionState, left_detectedPoint, _, _ = sim.simxReadProximitySensor(
            clientID, left_sensor_handle, sim.simx_opmode_buffer)
        '''
        # Controle dos motores baseado nos sensores ultrassônicos
        if front_detectionState and np.linalg.norm(front_detectedPoint) < 0.2:  # Obstáculo a menos de 0.2m
            # Move para trás antes de girar
            #sim.simxSetJointTargetVelocity(clientID, left_motor_handle, -1.0, sim.simx_opmode_streaming)
            #sim.simxSetJointTargetVelocity(clientID, right_motor_handle, -1.0, sim.simx_opmode_streaming)
            #delay(0.5)
            
            # Decide para onde girar baseado nos sensores laterais
            if right_detectionState and (not left_detectionState or np.linalg.norm(right_detectedPoint) < np.linalg.norm(left_detectedPoint)):
                # Gira para a esquerda
                lSpeed = 1
                rSpeed = -1
            elif front_detectionState and np.linalg.norm(front_detectedPoint) < 0.2:
                lSpeed = -3
                rSpeed = 3
            else:
                # Gira para a direita
                lSpeed = 3
                rSpeed = -3
            delay(0.5)
        elif right_detectionState and np.linalg.norm(right_detectedPoint) < 0.21:  # Obstáculo à direita a menos de 0.2m
            lSpeed = -3.5
            rSpeed = 3.5
        elif left_detectionState and np.linalg.norm(left_detectedPoint) < 0.2:  # Obstáculo à esquerda a menos de 0.2m
            lSpeed = 3.5
            rSpeed = -3.5
        else:
            # Se não houver obstáculo, segue em frente
            lSpeed = 3.0
            rSpeed = 3.0
        '''
        lSeed = 0
        rSeed = 0
        if not front_detectionState:
            lSpeed = 3
            rSpeed = 3
            if np.linalg.norm(left_detectedPoint) < 0.25:
                sim.simxSetJointTargetVelocity(clientID, left_motor_handle, -2.0, sim.simx_opmode_streaming)
                sim.simxSetJointTargetVelocity(clientID, right_motor_handle, -2.0, sim.simx_opmode_streaming)
                delay(0.5)
                
        # Define as velocidades dos motores
        errorCode = sim.simxSetJointTargetVelocity(
            clientID, left_motor_handle, lSpeed, sim.simx_opmode_streaming)
        errorCode = sim.simxSetJointTargetVelocity(
            clientID, right_motor_handle, rSpeed, sim.simx_opmode_streaming)

        com = cv2.waitKey(1)
        if com == ord('q'):
            break

    cv2.destroyAllWindows()
except Exception as e:
    print(f"An error occurred: {e}")
    cv2.destroyAllWindows()

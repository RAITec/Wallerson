import sim
from time import sleep as delay
import numpy as np
import cv2
import sys

print('Program started')
sim.simxFinish(-1)  # Fecha todas as conexões existentes
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Conecta ao servidor remoto do CoppeliaSim

if clientID != -1:
    print('Connected to remote API server')  # Conexão bem-sucedida
else:
    sys.exit('Failed connecting to remote API server')  # Falha na conexão

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
errorCode, left_sensor_handle = sim.simxGetObjectHandle(
    clientID, 'sensor_left', sim.simx_opmode_oneshot_wait)
errorCode, right_sensor_handle = sim.simxGetObjectHandle(
    clientID, 'sensor_right', sim.simx_opmode_oneshot_wait)

delay(1)

# Inicia a captura de imagens
returnCode, resolution, image = sim.simxGetVisionSensorImage(
    clientID, camera_handle, 0, sim.simx_opmode_streaming)
# Inicia a captura dos dados dos sensores ultrassônicos
returnCode, front_detectionState, front_detectedPoint, _, _ = sim.simxReadProximitySensor(
    clientID, front_sensor_handle, sim.simx_opmode_streaming)
returnCode, left_detectionState, left_detectedPoint, _, _ = sim.simxReadProximitySensor(
    clientID, left_sensor_handle, sim.simx_opmode_streaming)
returnCode, right_detectionState, right_detectedPoint, _, _ = sim.simxReadProximitySensor(
    clientID, right_sensor_handle, sim.simx_opmode_streaming)
delay(1)

try:
    while True:
        # Captura de imagem da câmera
        returnCode, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera_handle, 0, sim.simx_opmode_buffer)
        
        if returnCode == sim.simx_return_ok:  # Verifica se a captura da imagem foi bem-sucedida.
            im = np.array(image, dtype=np.uint8)  # Converte a imagem em um array NumPy.
            im.resize([resolution[1], resolution[0], 3])  # Ajusta a forma da imagem.
            im = cv2.flip(im, 0)  # Inverte a imagem verticalmente.
            im = cv2.rotate(im, cv2.ROTATE_90_COUNTERCLOCKWISE)  # Rotaciona a imagem 90 graus no sentido anti-horário.
            im = cv2.resize(im, (512, 512))  # Redimensiona a imagem para 512x512 pixels.
            im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)  # Converte a imagem de RGB para BGR (formato usado pelo OpenCV).
            cv2.imshow("data", im)  # Exibe a imagem.

        # Leitura dos sensores ultrassônicos
        returnCode, front_detectionState, front_detectedPoint, _, _ = sim.simxReadProximitySensor(
            clientID, front_sensor_handle, sim.simx_opmode_buffer)
        returnCode, left_detectionState, left_detectedPoint, _, _ = sim.simxReadProximitySensor(
            clientID, left_sensor_handle, sim.simx_opmode_buffer)
        returnCode, right_detectionState, right_detectedPoint, _, _ = sim.simxReadProximitySensor(
            clientID, right_sensor_handle, sim.simx_opmode_buffer)

        if returnCode == sim.simx_return_ok:  # Verifica se a leitura dos sensores foi bem-sucedida.
            # Distâncias medidas pelos sensores
            front_distance = np.linalg.norm(front_detectedPoint) if front_detectionState else float('inf')
            left_distance = np.linalg.norm(left_detectedPoint) if left_detectionState else float('inf')
            right_distance = np.linalg.norm(right_detectedPoint) if right_detectionState else float('inf')

            # Controle dos motores baseado nos sensores ultrassônicos
            if front_distance < 0.5:  # Obstáculo à frente a menos de 0.5m
                if left_distance >= right_distance:  # Se há mais espaço à esquerda ou ambos são iguais
                    lSpeed = -0.25
                    rSpeed = 0.25
                else:  # Se há mais espaço à direita
                    lSpeed = 0.25
                    rSpeed = -0.25
                delay(1)  # Aguarda um tempo para a rotação

                # Após a rotação, verificar novamente os sensores laterais
                returnCode, left_detectionState, left_detectedPoint, _, _ = sim.simxReadProximitySensor(
                    clientID, left_sensor_handle, sim.simx_opmode_buffer)
                returnCode, right_detectionState, right_detectedPoint, _, _ = sim.simxReadProximitySensor(
                    clientID, right_sensor_handle, sim.simx_opmode_buffer)
                left_distance = np.linalg.norm(left_detectedPoint) if left_detectionState else float('inf')
                right_distance = np.linalg.norm(right_detectedPoint) if right_detectionState else float('inf')
                
                if left_distance < 0.3 or right_distance < 0.3:  # Se encontrar obstáculo lateral após rotação
                    lSpeed = -0.25
                    rSpeed = -0.25
                    delay(1)  # Move para trás para desviar do obstáculo

            elif left_distance < 0.3:  # Obstáculo à esquerda a menos de 0.3m
                lSpeed = 0.2
                rSpeed = 0.5
            elif right_distance < 0.3:  # Obstáculo à direita a menos de 0.3m
                lSpeed = 0.5
                rSpeed = 0.2
            else:  # Sem obstáculos próximos
                lSpeed = 0.5
                rSpeed = 0.5

            # Define a velocidade dos motores
            errorCode = sim.simxSetJointTargetVelocity(
                clientID, left_motor_handle, lSpeed, sim.simx_opmode_streaming)
            errorCode = sim.simxSetJointTargetVelocity(
                clientID, right_motor_handle, rSpeed, sim.simx_opmode_streaming)

        com = cv2.waitKey(1)  # Espera por 1 milissegundo por uma tecla pressionada.
        if com == ord('q'):  # Se a tecla 'q' for pressionada, sai do loop.
            break

    cv2.destroyAllWindows()  # Fecha todas as janelas do OpenCV.
except Exception as e:
    print(f"An error occurred: {e}")  # Se ocorrer uma exceção, imprime a mensagem de erro.
    cv2.destroyAllWindows()  # Fecha todas as janelas do OpenCV.

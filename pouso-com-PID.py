import time
import numpy as np
from djitellopy import Tello

# Constantes
kp = 1.5
ki = 0.2
kd = 0.5
dt = 0.1
max_vel = 0.3
max_error = 100
integral_max = 1000

# Inicialização do drone e do controlador PID
drone = Tello()
pid = [0, 0, 0]
prev_error = [0, 0, 0]
integral = [0, 0, 0]

# Função para controlar a descida do drone
def pid_controller(target_altitude):
    global prev_error, integral

    # Obtém a altura atual do drone
    current_altitude = drone.get_height()

    # Calcula o erro proporcional
    error = target_altitude - current_altitude

    # Calcula o termo integral
    integral[0] = integral[0] + error * dt
    integral[1] = integral[1] + integral[0] * dt
    integral[2] = integral[2] + integral[1] * dt

    # Limita o termo integral
    for i in range(3):
        if abs(integral[i]) > integral_max:
            integral[i] = np.sign(integral[i]) * integral_max

    # Calcula o termo derivativo
    derivative = [(error - prev_error[i]) / dt for i in range(3)]
    prev_error = [error, prev_error[0], prev_error[1]]

    # Calcula a saída do controlador PID
    pid[0] = kp * error + ki * integral[0] + kd * derivative[0]
    pid[1] = kp * error + ki * integral[1] + kd * derivative[1]
    pid[2] = kp * error + ki * integral[2] + kd * derivative[2]

    # Limita a velocidade do drone
    for i in range(3):
        if abs(pid[i]) > max_vel:
            pid[i] = np.sign(pid[i]) * max_vel

    # Move o drone
    drone.send_rc_control(int(pid[1]*100), int(pid[0]*100), int(pid[2]*100), 0)

# Conecta ao drone
drone.connect()
drone.streamon()

# Espera a conexão ser estabelecida
time.sleep(2)

# Inicia a decolagem do drone
drone.takeoff()

# Espera o drone estabilizar
time.sleep(2)

# Define a altura desejada para o pouso
target_altitude = 30

# Controla a descida do drone até atingir a altura desejada
while True:
    # Atualiza o controlador PID
    pid_controller(target_altitude)

    # Verifica se o drone chegou na altura desejada
    if abs(drone.get_height() - target_altitude) < 5:
        break

    # Espera um tempo antes de atualizar novamente o controlador
    time.sleep(dt)

# Pousa o drone
drone.land()

# Encerra a conexão com o drone
drone.streamoff()
drone.disconnect()

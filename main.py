import cv2
from djitellopy import Tello
import time

# Parâmetros do controlador PID
kp = 0.5
ki = 0.1
kd = 0.2
setpoint = 300  # altura de pouso desejada em centímetros

# Cria o objeto PIDController
pid = PIDController(kp, ki, kd, setpoint)

# Inicializa o drone Tello
drone = Tello()

# Conecta o drone Tello
drone.connect()

# Inicia a transmissão de vídeo do drone
drone.streamon()

# Define a largura e a altura do frame de imagem
width = 320
height = 240

# Define a área de interesse (ROI) para detecção da placa
roi_x = int(width / 3)
roi_y = int(height / 3)
roi_w = int(width / 3)
roi_h = int(height / 3)

# Inicia a captura de imagens do drone
drone.takeoff()
while True:
    # Captura um frame de imagem do drone
    frame = drone.get_frame_read().frame
    frame = cv2.resize(frame, (width, height))

    # Aplica a detecção de objetos ao frame
    object_detected = detect_object(frame, roi_x, roi_y, roi_w, roi_h)

    if object_detected:
        # A placa foi detectada, inicia o pouso controlado
        current_height = drone.get_height()
        while current_height > setpoint:
            # Atualiza o sinal de controle do PID
            output = pid.update(current_height)

            # Envia o sinal de controle para o drone
            drone.send_rc_control(0, 0, -output, 0)

            # Aguarda um curto período de tempo
            time.sleep(0.05)

            # Atualiza a altura atual do drone
            current_height = drone.get_height()

        # A altura de pouso foi atingida, inicia o pouso
        drone.land()
        break

    # Exibe o frame com a detecção de objetos para o usuário
    cv2.imshow("Object Detection", frame)

    # Verifica se o usuário pressionou a tecla 'q' para sair do loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Finaliza a conexão com o drone e encerra a janela de exibição da imagem
drone.streamoff()
drone.disconnect()
cv2.destroyAllWindows()

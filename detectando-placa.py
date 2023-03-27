def detect_object(frame, roi_x, roi_y, roi_w, roi_h):
    # Converte o frame para escala de cinza
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Define o classificador Haar Cascade
    cascade = cv2.CascadeClassifier("haarcascade_stop_sign.xml")

    # Detecta as regiões da imagem correspondentes à placa
    objects = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Verifica se há objetos detectados dentro da ROI definida
    for (x, y, w, h) in objects:
        if x > roi_x and y > roi_y and x + w < roi_x + roi_w and y + h < roi_y + roi_h:
            return True

    # Se nenhum objeto foi detectado dentro da ROI, retorna False
    return False

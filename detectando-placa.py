import cv2
import numpy as np

# Carregar a imagem da placa de pouso:
plate = cv2.imread('placa_de_pouso.jpg')

# Converter a imagem para escala de cinza e aplicar um filtro de suavização:
gray = cv2.cvtColor(plate, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (5, 5), 0)

# Definir os parâmetros do algoritmo de detecção de bordas Canny:
edges = cv2.Canny(gray, 50, 150, apertureSize=3)

# Aplicar uma transformada de Hough para detectar as linhas presentes na imagem:
lines = cv2.HoughLines(edges, 1, np.pi/180, 100)

# Desenhar as linhas detectadas na imagem original:
for rho, theta in lines[0]:
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    cv2.line(plate, (x1, y1), (x2, y2), (0, 0, 255), 2)

# Mostrar a imagem com as linhas detectadas:
cv2.imshow('plate', plate)
cv2.waitKey(0)
cv2.destroyAllWindows()
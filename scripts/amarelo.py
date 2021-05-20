
from sklearn.linear_model import LinearRegression
import rospy
import numpy as np
import math
import cv2
import time

cX = None
cY = None
ranges = None    

def center_of_mass_region(mask, x1, y1, x2, y2):
    # Para fins de desenho
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    clipped = mask[y1:y2, x1:x2]
    c = center_of_mass(clipped)
    c[0]+=x1
    c[1]+=y1
    crosshair(mask_bgr, c, 10, (0,0,255))
    cv2.rectangle(mask_bgr, (x1, y1), (x2, y2), (255,0,0),2,cv2.LINE_AA)
    return mask_bgr

def segmenta_linha_amarela(bgr):
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e retornar os segmentos amarelos do centro da pista em branco.
        Utiliza a função cv2.morphologyEx() para limpar ruidos na imagem
    """

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    img = hsv.copy()
    
    hsv1 = (25, 50, 50)
    hsv2 = (35, 255, 255)
    mask = cv2.inRange(img, hsv1, hsv2)
    
    
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
    mask_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel )
    mask_close = cv2.morphologyEx( mask_open, cv2.MORPH_CLOSE, kernel )

    return mask_close

def encontrar_contornos(mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca os contornos encontrados
    """

    contornos, arvore = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return contornos
   

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)

def center_of_mass(data):
    """ Retorna uma tupla (cx, cy) que desenha o centro de data, que pode ser contorno ou matriz"""
    M = cv2.moments(data)
    # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
    if M["m00"] == 0:
        m00 = 1
    else:
        m00 = M["m00"]

    cX = int(M["m10"] / m00)
    cY = int(M["m01"] / m00)
    return (int(cX), int(cY))

def encontrar_centro_dos_contornos(img, contornos):
    """Não mude ou renomeie esta função
        deve receber um contorno e retornar, respectivamente, a imagem com uma cruz no centro de cada segmento e o centro dele. formato: img, x, y
    """

    lista_centro_x = []
    lista_centro_y = []

    for i in contornos:
        centro_x, centro_y = center_of_mass(i)
        lista_centro_x.append(centro_x)
        lista_centro_y.append(centro_y)
        crosshair(img,(centro_x, centro_y), 5, (255,0,0))

    

    return img, lista_centro_x, lista_centro_y

def desenhar_linha_entre_pontos(img, X, Y, color):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e retornar uma imagem com uma linha entre os centros EM SEQUENCIA do mais proximo.
    """
    for i in range(1,len(X)):
        cv2.line(img,(X[i-1],Y[i-1]),(X[i],Y[i]),color,2)
    
    return img

def regressao_por_centro(img, x,y):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e estimar a melhor reta, utilizando o metodo preferir, que passa pelos centros. Retorne a imagem com a reta e os parametros da reta
        
        Dica: cv2.line(img,ponto1,ponto2,color,2) desenha uma linha que passe entre os pontos, mesmo que ponto1 e ponto2 não pertençam a imagem.
    """

    xr = np.array(x)
    yr = np.array(y)

    xr = xr.reshape(-1,1)
    yr = yr.reshape(-1,1)

    reg = LinearRegression()

    reg.fit(xr, yr)

    a, b = reg.coef_, reg.intercept_

    x1 = 100
    x2 = 10000
    y1 = int((x1*a + b))
    y2 = int((x2*a + b))
    ponto1 = (x1,y1)
    ponto2 = (x2,y2)
    color =  (255,0,0)

    cv2.line(img,ponto1,ponto2,color,2)

    return img, (a, b)

def calcular_angulo_com_vertical(img, lm):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e estimar a melhor reta, utilizando o metodo preferir, que passa pelos centros. Retorne a imagem com a reta.
        
        Dica: cv2.line(img,ponto1,ponto2,color,2) desenha uma linha que passe entre os pontos, mesmo que ponto1 e ponto2 não pertençam a imagem.
    """

    coef_angular = lm[0]

    angulo =  math.atan(coef_angular)

    graus = 90 + math.degrees(angulo)

    return graus


def fazTudo(mask):

    segmenta = segmenta_linha_amarela(mask)

    contornos = encontrar_contornos(segmenta)

    if len(contornos) < 2:
        return mask, None

    imagem, X, Y = encontrar_centro_dos_contornos(mask, contornos)

    imagem, lm= regressao_por_centro(imagem, X,Y)

    angulo = calcular_angulo_com_vertical(imagem, lm)

    return imagem,angulo
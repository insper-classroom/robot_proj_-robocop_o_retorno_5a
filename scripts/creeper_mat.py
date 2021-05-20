#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import amarelo
import ler_aruco

print("EXECUTE ANTES da 1.a vez: ")
print("wget https://github.com/Insper/robot21.1/raw/main/projeto/ros_projeto/scripts/MobileNetSSD_deploy.caffemodel")
print("PARA TER OS PESOS DA REDE NEURAL")

import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
ang = None



area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

ciano = [[np.array([75, 200, 100]), np.array([90, 255, 255])], [np.array([90, 200, 100]), np.array([105, 255, 255])]]
verde = [[np.array([45, 255, 255]), np.array([60, 255, 255])], [np.array([60, 255, 255]), np.array([75, 255, 255])]]
vermelho = [[np.array([0, 255, 255]), np.array([15, 255, 255])], [np.array([165, 255, 255]), np.array([180, 255, 255])]]

media = [10000,0]

distancia = 50

x = None
y = None

contador = 0
pula = 50

def recebe_odometria(data):
    global x
    global y
    global contador
    global angulo

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))
    angulo = angulos[2]

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1

def scaneou(dado):
	global distancia
	global ranges
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	ranges = np.array(dado.ranges).round(decimals=2)
	distancia = ranges[0]

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global resultados
    global ang
    global dist_aruco
    global ids
    global maior_contorno_area

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        # Esta logica do delay so' precisa ser usada com robo real e rede wifi 
        # serve para descartar imagens antigas
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        img_copy = temp_image.copy()
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        media, _, maior_contorno_area = identifica_cor(cv_image, ciano)
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
        cv_image_copy = cv_image.copy()
        img,ang = amarelo.fazTudo(cv_image_copy)
        img_aruco, dist_aruco, ids = ler_aruco.roda_aruco(img_copy)
        print(ang)
        cv2.imshow("cv_image", img_aruco)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

def identifica_cor(frame, cores):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cor_menor0 = cores[0][0]
    cor_maior0 = cores[0][1]
    cor_menor1 = cores[1][0]
    cor_maior1 = cores[1][1]
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor0, cor_maior0) + cv2.inRange(frame_hsv, cor_menor1, cor_maior1)

    centro = (frame.shape[1]//2, frame.shape[0]//2)

    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int( point[0] - length/2 ), point[1] ),  (int( point[0] + length/2 ), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], int(point[1] - length/2) ), (point[0], int( point[1] + length/2 ) ),color ,width, length)

    segmentado_cor = morpho_limpa(segmentado_cor)

    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [255,0,0], 1, 17)
    else:
        media = (0, 0)
    cv2.imshow('seg', segmentado_cor)
    cv2.waitKey(1)

    return media, centro, maior_contorno_area

def morpho_limpa(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,12))
    mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel )
    mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )   
    return mask

if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)


    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    try:
        # Inicializando - por default gira no sentido anti-horário
        velRoda = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
        velPara = Twist(Vector3(0,0,0), Vector3(0,0,0))
        velFrente = Twist(Vector3(0.2,0,0), Vector3(0,0,0))


        margin = 0

        state = 0

        stroll = False
        found = False
        find_pos = [0,0,0]
        para = False
        catch = 0

        while not rospy.is_shutdown():

            if state == 0:
                if ang is None:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                else:
                    if ang > 90:
                        if ang < 150:
                            vel = Twist(Vector3(0.4,0,0), Vector3(0,0,-0.1))
                        else:
                            vel = Twist(Vector3(0.4,0,0), Vector3(0,0,0))
                    else:
                        if ang > 30:
                            vel = Twist(Vector3(0.4,0,0), Vector3(0,0,0.1))
                        else:
                            vel = Twist(Vector3(0.4,0,0), Vector3(0,0,0))
            
            if state == 1:

                if ang is None:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
                else:
                    if ang > 90:
                        if ang < 150:
                            vel = Twist(Vector3(0.4,0,0), Vector3(0,0,0.1))
                        else:
                            vel = Twist(Vector3(0.4,0,0), Vector3(0,0,0))
                    else:
                        if ang > 30:
                            vel = Twist(Vector3(0.4,0,0), Vector3(0,0,-0.1))
                        else:
                            vel = Twist(Vector3(0.4,0,0), Vector3(0,0,0))
            
            
            if not stroll:
                if not found:
                    print(maior_contorno_area)
                    if maior_contorno_area > 200 and ids[0][0] == 11:
                        found = True
                        find_pos = [x, y, angulo]
                else:
                    if not para:
                        if maior_contorno_area > 50:
                            para = True
                        if (media[0] > centro[0]):
                            vel = Twist(Vector3(0.07,0,0), Vector3(0,0,-0.1))
                        elif (media[0] < centro[0]):
                            vel = Twist(Vector3(0.07,0,0), Vector3(0,0,0.1))
                    elif maior_contorno_area > 60:
                        vel = Twist(Vector3(-0.01,0,0), Vector3(0,0,0))
                    else:
                        if centro[0] - 5 < media[0] < centro[0] + 5:
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                            catch = 1
                        else:
                            if (media[0] > centro[0]):
                                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.05))
                            elif (media[0] < centro[0]):
                                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.05))
                if catch == 1:
                    print('a')
                    # codigo para a garra pegar
                elif catch == 2:
                    print('b')
                    # codigo para o robo voltar
                        

            
            velocidade_saida.publish(vel)

            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")



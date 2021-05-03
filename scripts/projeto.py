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
from sensor_msgs.msg import Image, CompressedImage
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



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global resultados
    global ang

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
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
        cv_image_copy = cv_image.copy()
        img,ang = amarelo.fazTudo(cv_image_copy)
        img_aruco = ler_aruco.roda_aruco(cv_image)
        print(ang)
        cv2.imshow("cv_image", img_aruco)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)


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
            velocidade_saida.publish(vel)

            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")



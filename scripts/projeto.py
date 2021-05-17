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
ids = []
rad = None

dist_aruco = None

angulo_inicial = 0



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

contador = 0
pula = 50
angulo = None

def angulo_q_roda(x,y, ang):
    angulo_trig = math.atan2(y,x)

    roda = (math.pi - ang) + angulo_trig

    return roda

def calcula_distancia(x,y):

    dist = math.sqrt(pow(x,2) + pow(y,2))

    return dist

def recebe_odometria(data):
    global x
    global y
    global contador
    global angulo
    global rad

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    rads = transformations.euler_from_quaternion(lista)
    rad = rads[2]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))
    angulo = angulos[2]

    contador = contador + 1

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global resultados
    global ang
    global dist_aruco
    global ids

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
        #print(ang)
        cv2.imshow("cv_image", img_aruco)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)


    #print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 

    try:

        margin = 0

        state = 0

        deu_volta = False

        distAruco = 70

        distAruco_2 = 110

        distAruco_1 = 65

        while not rospy.is_shutdown():

            if rad is not None:
                if rad < 0:
                    rad = rad + 2 * math.pi

            if state == 100:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.07))
                angulo_atual = angulo
                if angulo_atual < 0:
                    angulo_atual = angulo_atual + 180
                print(angulo_atual)
                if angulo_atual < 140 and angulo_atual > 100:
                    state = 0
                    print(state)
                    pos_x = x
                    pos_y = y
                    distAruco = distAruco_2
            

            if state == 50:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.5))

                x_conta = x - pos_x
                y_conta = y - pos_y

                rad_roda = angulo_q_roda(x_conta,y_conta,rad_inicial)
                if rad_roda < 0:
                    rad_roda = rad_roda + 2*math.pi
                if rad > rad_roda + rad_inicial + 0.6:
                    state = 1
                    print(state)
                    rospy.sleep(1)
            
            if state == 150:

                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.5))
               
                x_conta = x - pos_x
                y_conta = y - pos_y

                rad_roda = angulo_q_roda(x_conta,y_conta,rad_inicial) + 2*math.pi
                if rad_roda < 0:
                    rad_roda = rad_roda + 2*math.pi

                if rad -(rad_roda + rad_inicial) < -6.6:
                    state = 3
                    print(state)
                    rospy.sleep(1)

            if state == 200:
                if tempo2 - tempo < 3:
                    vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
                else:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.4))
                    angulo_atual = angulo
                    if angulo_atual < 0:
                        angulo_atual = angulo_atual + 180
                    if 100 < angulo_atual < 110:
                        state = 5
                        print(state)
                        pos_x = x
                        pos_y = y


            if state == 0:
                if ang is None:
                    vel = Twist(Vector3(0.06,0,0), Vector3(0,0,-0.07))
                else:
                    if ang > 90:
                        if ang < 150:
                            vel = Twist(Vector3(0.2,0,0), Vector3(0,0,-0.1))
                        else:
                            vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
                    else:
                        if ang > 35:
                            vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0.1))
                        else:
                            vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
                if dist_aruco is not None:
                    if dist_aruco < distAruco:
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                        state = ids[0][0]
                        print(state)
                        tempo = rospy.Time.to_sec(rospy.Time.now())
                        if rad < 0:
                            rad_inicial = rad + 2*math.pi
                        else:
                            rad_inicial = rad
                        

            
            if state == 1:

                delta_y = y-pos_y
                delta_x = x-pos_x

                rad_dir = math.atan2(delta_y, delta_x) + math.pi
                    
                if rad_dir < 0:
                    rad_dir+=math.pi*2
                    
                dist = calcula_distancia(delta_x, delta_y)
                if dist > 1:
                    if rad > rad_dir:
                        vel = Twist(Vector3(0.3,0,0), Vector3(0,0,-0.1))
                            
                    else:
                        vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0.1))
                
                else:
                    if rad > rad_dir:
                        vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.05))       
                    else:
                        vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.05))

                    if dist < 0.1:
                        state = 2
                        print(state)

            if state == 2:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                angulo_atual = angulo
                if angulo_atual < 0:
                    angulo_atual = angulo_atual + 180
                print(angulo_atual)
                if angulo_atual < 70:
                    state = 0
                
            
            if state == 3:

                delta_y = y-pos_y
                delta_x = x-pos_x

                rad_dir = math.atan2(delta_y, delta_x)
                    
                if rad_dir < 0:
                    rad_dir+=math.pi*2

                rad_dir -= math.pi
                    
                dist = calcula_distancia(delta_x, delta_y)
                if dist > 1:
                    if rad < rad_dir:
                        vel = Twist(Vector3(0.3,0,0), Vector3(0,0,0.1))
                            
                    else:
                        vel = Twist(Vector3(0.3,0,0), Vector3(0,0,-0.1))
                
                else:
                    if rad < rad_dir:
                        vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.05))       
                    else:
                        vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.05))

                    if dist < 0.1:
                        state = 4
                        print(state)

            if state == 4:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                angulo_atual = angulo
                print(angulo_atual)
                if 5 > angulo_atual > -5:
                    distAruco = distAruco_1
                    state = 0

            if state == 5:
                print(ang)
                if ang is None:
                    vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.2))
                else:
                    if ang > 90:
                        if ang < 160:
                            vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.07))
                        else:
                            vel = Twist(Vector3(0.25,0,0), Vector3(0,0,0))
                    else:
                        if ang > 20:
                            vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.07))
                        else:
                            vel = Twist(Vector3(0.25,0,0), Vector3(0,0,0))
                
                delta_y = y-pos_y
                delta_x = x-pos_x
                dist = calcula_distancia(delta_x, delta_y)
                
                if dist > 2:
                    deu_volta = True
                if deu_volta and dist < 0.5 and -0.1 < y < 0.1:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                    print("DEU A VOLTA")

            tempo2 = rospy.Time.to_sec(rospy.Time.now())

            velocidade_saida.publish(vel)

            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")



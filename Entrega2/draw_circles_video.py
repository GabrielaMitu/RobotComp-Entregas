#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__      = "Matheus Dib, Fabio de Miranda"


import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

from imutils import paths
import imutils
import math


# Cria o detector BRISK
brisk = cv2.BRISK_create()


# Configura o algoritmo de casamento de features que vê *como* o objeto que deve ser encontrado aparece na imagem
bf = cv2.BFMatcher(cv2.NORM_HAMMING)

# Define o mínimo de pontos similares
MINIMO_SEMELHANCAS = 18


def find_good_matches(descriptor_image1, frame_gray):
    """
        Recebe o descritor da imagem a procurar e um frame da cena, e devolve os keypoints e os good matches
    """
    des1 = descriptor_image1
    kp2, des2 = brisk.detectAndCompute(frame_gray,None)

    # Tenta fazer a melhor comparacao usando o algoritmo
    matches = bf.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    return kp2, good


if __name__ == "__main__":

    cap = cv2.VideoCapture(0)

    original_rgb = cv2.imread("InsperLogo.png")  # Imagem a procurar
    img_original = cv2.cvtColor(original_rgb, cv2.COLOR_BGR2GRAY)
    #original_rgb = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2RGB)


    # Encontra os pontos únicos (keypoints) nas duas imagems
    kp1, des1 = brisk.detectAndCompute(img_original ,None)


    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret == False:
            print("Problema para capturar o frame da câmera")
            continue

        # Our operations on the frame come here
        frame_rgb = frame #cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kp2, good_matches = find_good_matches(des1, gray)

        if len(good_matches) > MINIMO_SEMELHANCAS:
            img3 = cv2.drawMatches(original_rgb,kp1,frame_rgb,kp2, good_matches, None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            cv2.imshow('BRISK features', img3)
        else:
            cv2.imshow("BRISK features", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


# If you want to open a video, just change v2.VideoCapture(0) from 0 to the filename, just like below
#cap = cv2.VideoCapture('hall_box_battery.mp4')

# Parameters to use when opening the webcam.
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

lower = 0
upper = 1

print("Press q to QUIT")




# Returns an image containing the borders of the image
# sigma is how far from the median we are setting the thresholds
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged



while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # A gaussian blur to get rid of the noise in the image
    blur = cv2.GaussianBlur(gray,(5,5),0)
    #blur = gray
    # Detect the edges present in the image
    bordas = auto_canny(blur)

    circles = []

    out_circles = []

    # Obtains a version of the edges image where we can draw in color
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)

    # HoughCircles - detects circles using the Hough Method. For an explanation of
    # param1 and param2 please see an explanation here http://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/

    circles = None
    circles=cv2.HoughCircles(bordas,cv2.HOUGH_GRADIENT,2,70,param1=50,param2=40,minRadius=5,maxRadius=50)

    #Arrays de cores
    magenta_menor = np.array([166,  50,  50])
    magenta_maior = np.array([176, 255, 255])
    mask_coke_mag = cv2.inRange(hsv, magenta_menor, magenta_maior)

    ciano_menor = np.array([100, 50, 50])
    ciano_maior = np.array([110, 255, 255])
    mask_coke_cian = cv2.inRange(hsv, ciano_menor, ciano_maior)

    masks = mask_coke_mag + mask_coke_cian

    imagem = cv2.bitwise_or(frame, frame, mask=masks)

    #cv2.imshow("mascara", imagem)

    if circles is not None:        
        circles = np.uint16(np.around(circles)).astype("int")
        for i in circles[0,:]:
            print(i)
            # draw the outer circle
            cv2.circle(bordas_color,(i[0],i[1]),i[2],(0,255,0),2)

            # draw the center of the circle
            cv2.circle(bordas_color,(i[0],i[1]),i[2],(0,0,255),3)

            out_circles.append( ((i[0], i[1]) , i[2]) )

            if len(out_circles)>=2:
                x, y = out_circles[0][0]
                x2, y2 = out_circles[1][0]

                print(x, x2, y, y2)
                font = cv2.FONT_HERSHEY_SIMPLEX

                #distancia entre os dois circulos
                deltaX=(x-x2)**2
                deltaY=(y-y2)**2
                h=(deltaX+deltaY)**(0.5)
                #cv2.putText(imagem, "h: {}".format(h) ,(200,200), font, (0.5),(255,255,255),2,cv2.LINE_AA)

                H=14
                f=707

                #distancia entre folha e camera
                D=(H*f)/h

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(imagem, "Distancia: {}".format(D) ,(0,450), font, (0.5),(255,255,255),2,cv2.LINE_AA)

                #linha entre circulos

                cv2.line(imagem, (x,y), (x2,y2), (0, 255, 0), thickness=3, lineType=8)

                #angulo entre linha entre circulos e a horizontal
                Y=(y-y2)
                X=(x-x2)
                angulo=math.atan2(Y,X)
                angulo=math.degrees(angulo)

                #print(angulo)
                cv2.putText(bordas_color,"Angulo: {}".format(angulo),(0,400), font, (0.75),(255,255,255),2,cv2.LINE_AA)





    # cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(bordas_color,'Press q to quit',(0,50), font, 1,(255,255,255),2,cv2.LINE_AA)

    #More drawing functions @ http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

    #cv2.imshow('Detector de circulos',bordas_color)
    #cv2.imshow('Frame',frame)
 
 
    # Display the resulting frame
    tudo = bordas_color + imagem 

    cv2.imshow('Tudo', tudo)



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#  When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

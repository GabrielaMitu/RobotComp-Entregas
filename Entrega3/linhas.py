#!/usr/bin/python
# -*- coding: utf-8 -*-

__author__      = "Matheus Dib, Fabio de Miranda"

import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import math
import matplotlib.cm as cm
import numpy as np

# Parameters to use when opening the webcam.
cap = cv2.VideoCapture('LinhasSala.mp4')
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

lower = 0
upper = 1


def eq_reta(P1, P2):
    x1=P1[0]
    y1=P1[1]
    x2=P2[0]
    y2=P2[1]
    dx=x2-x1
    dy=y2-y1
    #coeficiente angular
    if dx != 0:
        m=dy/dx
    else:
        m=0
    #coeficiente linear
    h=y1-(m*x1)
    return (m,h)

def intersecao_retas(reta1,reta2):
    m1=reta1[0]
    m2=reta2[0]
    h1=reta1[1]
    h2=reta2[1]

    xi=int((h2-h1)/(m1-m2))
    yi= int((m1*xi)+h1)

    return (xi,yi)


def calcula_m(x2,x1,y2,y1):
    dx=(x2-x1)
    dy=(y2-y1)    
                
    if dx!=0:
        return dy/dx

def calcula_h(x,y,m):
    return y - m * x



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

def comp(a,b):
    if a[1] > b[1]:
        return -1
    elif a[1] == b[1]:
        return 0
    else:
        return 1



 
while(True):
    ret, frame = cap.read()

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    # A gaussian blur to get rid of the noise in the image
    blur = cv2.GaussianBlur(gray,(5,5),0)
    # Detect the edges present in the image
    bordas = auto_canny(blur)
    # Obtains a version of the edges image where we can draw in color
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)

    #mascaras para filtrar o branco
    masks = cv2.inRange(gray, 240, 255)
    nmask=cv2.bitwise_not(masks)


    imagem = cv2.bitwise_or(frame, frame, mask=masks)
    imgcon= cv2.bitwise_and(gray, gray, mask=nmask)
    
    imagem2 = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)

    #final=cv2.bitwise_or(imagem,imagem2)


    lines=[]
    lines = cv2.HoughLines(imagem2, 1, np.pi/180, 200)
    
    m1 = 0
    h1 = 0
    m2 = 0
    h2 = 0

    linha1 = False
    linha2 = False

    if lines is not None:
        for linha in lines:
            for r,theta in linha: 
                a = np.cos(theta) 
                b = np.sin(theta)  
                x0 = a*r 
                y0 = b*r 
                x1 = int(x0 + 1000*(-b))  
                y1 = int(y0 + 1000*(a))  
                x2 = int(x0 - 1000*(-b))  
                y2 = int(y0 - 1000*(a)) 
                
                m=calcula_m(x2,x1,y2,y1)

                if m < -0.3 and m > -3.3:
                    
                    if not linha1:
                        linha1=True
                        m1 = m
                        h1=calcula_h(x1,y1,m)
                        cv2.line(imagem,(x1,y1), (x2,y2), (0,255,0),2)

                elif m > 0.3 and m < 3.3:
                    if not linha2:
                        linha2 = True
                        m2 = m
                        h2 =calcula_h(x1,y1,m)
                        cv2.line(imagem,(x1,y1), (x2,y2), (0,255,0),2)


        if (m1-m2)!=0:    
            intersecao=intersecao_retas((m1,h1),(m2,h2))                
        cv2.circle(imagem,(intersecao[0],intersecao[1]), 15, (100,50,0), -1)


    cv2.imshow('Final', imagem)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


VideoCapture.release()
cv2.destroyAllWindows()




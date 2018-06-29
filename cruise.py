# -*- coding: utf-8 -*-
"""
Created on Thu Apr 12 21:25:25 2018

@author: Thinkpad
"""

import cv2
import numpy as np
from driver import driver

cap=cv2.VideoCapture(0)
d=driver()
d.setStatus(mode="speed")

#*****************************图像处理部分***********************************
#***************************************************************************

def ImgPro():
    #img = cv2.imread("F://testimages//road2.jpg", cv2.IMREAD_COLOR)  
    img=cap.read()
    cv2.imshow("color", img )
    dst=img.copy()

    size=np.shape(img)
    height=size[0]
    width=size[1]

    grayImg=cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
    #cv2.imshow("grayImg",grayImg)
    medianImg= cv2.medianBlur(grayImg,3)
    #cv2.imshow("medianImg",medianImg)
    ret,binary=cv2.threshold(medianImg,80,255,cv2.THRESH_BINARY_INV)
    #ret2,binary = cv2.threshold(medianImg,0,100,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    #cv2.imshow("binary",binary)
    kernel = np.ones((2,2),np.uint8)
    opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    #cv2.imshow("open",opening)

    #近处参考点用于控制当拍动作
    rec_sum=0
    rec_num=0
    for i in range(height-3,height-1) :
        for j in range(width-5):
            if (opening[i][j]==255 and opening[i][j+1]==255 and opening[i][j+2]==255 and opening[i][j+3]==255 and opening[i][j+4]==255):
                rec_num+=1
                rec_sum+=j
    ref=rec_sum/rec_num
    
    p1=(ref,0)
    p2=(ref,height)
    cv2.line(opening,p1,p2,(255))
    #cv2.imshow("opening",opening)
    
    #远处参考点用于预测前方路段形状
    rec_sum=0
    rec_num=0
    pre_ref=0
    for i in range(height-43,height-40) :
        for j in range(width-5):
            if (opening[i][j]==255 and opening[i][j+1]==255 and opening[i][j+2]==255 and opening[i][j+3]==255 and opening[i][j+4]==255):
                rec_num+=1
                rec_sum+=j
    pre_ref=rec_sum/rec_num
    p1=(pre_ref,0)
    p2=(pre_ref,height)
    cv2.line(opening,p1,p2,(255))
    cv2.imshow("opening",opening)
    
    #sign_detector
    flag=0
    
    
    
    if cv2.waitKey(1) & 0xFF==ord('p'):
        cv2.imwrite('test'+str(1)+'.jpg',img)
        
        
    cv2.waitKey(0)  
    cv2.destroyAllWindows()
    
    return ref,pre_ref,flag

#***************************main**********************************************
while(1):
    ref,pre_ref,flag=ImgPro()
    if (flag==1):
        d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
        break
    
    if (pre_ref<160 or pre_ref>480):
        sm=0.2
    else:
        sm=0.5
       
    if (ref<352 & ref>288):
        st=0
    else:
        st=-(320-ref)*0.02
        
    d.setStatus(motor = sm, servo = st)
    

        
    















   

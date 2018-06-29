# -*- coding: utf-8 -*-
"""
Created on Fri Apr 20 17:07:14 2018

@author: Thinkpad
"""

import cv2
import numpy as np
import math

#flags>0:返回3通道颜色，=0:返回灰度图像，<0:返回的图像带有透明度,alpha是灰度通道，记录透明度信息  
cap=cv2.VideoCapture(1)

while(1):
    ret,img=cap.read()
    if ret==True:
        #截图
        cv2.imshow("image",img)
        
        size=np.shape(img)
        height=size[0]
        width=size[1]
        
        x1=width/6
        x2=5*width/6
        y1=2*height/3
        y2=height
        dst=img[y1:y2,x1:x2]
        
        height=y2-y1
        width=x2-x1
        
        #取R通道
        #B = img[:, :, 0]
        #G = img[:, :, 1]
        R = dst[:, :, 2]

        #cv2.imshow("B",B)
        #cv2.imshow("G",G)
        cv2.imshow("R",R)
        
        #medianImg= cv2.medianBlur(R,3)
        #cv2.imshow("medianImg",medianImg)
        
        #双边滤波
        bilateralFilter = cv2.bilateralFilter(R,9,75,75)
        #cv2.imshow("bilateralFilter",bilateralFilter)
        #二值化
        ret,binary=cv2.threshold(bilateralFilter,100,255,cv2.THRESH_BINARY_INV)
        cv2.imshow("binary",binary)
        
        IM=binary.copy()
        '''
        rec_sum=0
        rec_num=0
        for j in range(width-5):
            if (IM[height-1][j]==255 & IM[height-1][j+1]==255 & IM[height-1][j+2]==255 & IM[height-1][j+3]==255 & IM[height-1][j+4]==255):
                rec_num+=1
                rec_sum+=j
        if(rec_num!=0):
            ref=rec_sum/rec_num
        
        p1=(ref,0)
        p2=(ref,height)
        cv2.line(IM,p1,p2,(255))
        #cv2.imshow("IM",IM)
        
        #远处参考点用于预测前方路段形状
        rec_sum=0
        rec_num=0
        pre_ref=0
        for j in range(width-5):
            if (IM[height-70][j]==255 & IM[height-70][j+1]==255 & IM[height-70][j+2]==255 & IM[height-70][j+3]==255 & IM[height-70][j+4]==255):
                rec_num+=1
                rec_sum+=j
        if(rec_num!=0):
            pre_ref=rec_sum/rec_num
        
        p1=(pre_ref,0)
        p2=(pre_ref,height)
        cv2.line(IM,p1,p2,(255))
        cv2.imshow("IM",IM)
        
        '''
        #近处参考点用于控制当拍动作
        rec_sum=0
        rec_num=0
        ref=0
        for i in range(height-3,height-1) :
            for j in range(width-5):
                if (IM[i][j]==255 & IM[i][j+1]==255 & IM[i][j+2]==255 & IM[i][j+3]==255 & IM[i][j+4]==255):
                    rec_num+=1
                    rec_sum+=j
        if(rec_num!=0):
            ref=rec_sum/rec_num
        
        p1=(ref,0)
        p2=(ref,height)
        cv2.line(IM,p1,p2,(255))
        #cv2.imshow("IM",IM)
        
        #远处参考点用于预测前方路段形状
        rec_sum=0
        rec_num=0
        pre_ref=0
        for i in range(height-70,height-67) :
            for j in range(width-5):
                if (IM[i][j]==255 & IM[i][j+1]==255 & IM[i][j+2]==255 & IM[i][j+3]==255 & IM[i][j+4]==255):
                    rec_num+=1
                    rec_sum+=j
        if(rec_num!=0):
            pre_ref=rec_sum/rec_num
        
        p1=(pre_ref,0)
        p2=(pre_ref,height)
        cv2.line(IM,p1,p2,(255))
        cv2.imshow("IM",IM)
          
        
        detref=ref-213
        theta=(math.atan((pre_ref-ref)/87))/np.pi
        
        #小车控制
        if (abs(theta)>0.16 or abs(detref)>160):
            sm=0.1
        else:
            sm=0.2
        
        st=-(theta+detref*0.001)
        if(st<-1):
            st=-1
        if(st>1):
            st=1
            
        print sm,st

        
        if cv2.waitKey(1) & 0xFF==ord('q'):
            break



cv2.waitKey(0)
cv2.destroyAllWindows()


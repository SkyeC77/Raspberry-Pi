# -*- coding: utf-8 -*-
"""
Created on Fri Jun 01 13:00:11 2018

@author: skye
"""

import cv2
import time
import numpy as np
import math
import glob
from driver import driver



d = driver()
d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
cap = cv2.VideoCapture(1)

#*************************前置摄像头校正**************************
def calibration(img):
    #前置摄像头内参数矩阵
    mtx = np.mat([[467.26107369, 0.0, 292.98887306], [0.0, 466.197742, 266.33596848], [0.0, 0.0, 1.0]])
    #畸变系数  distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
    dist = np.mat([-0.42720673,  0.2400467,   0.00188138,  -0.00255054,   -0.08276195])
    dst = cv2.undistort(img,mtx,dist,None,mtx)
    height_ = dst.shape[0]
    width_ = dst.shape[1]

    return dst, width_, height_

#********************选取红色区域**************************
def select_red(image):
    #lower = np.array([20,0,80])
    #upper = np.array([130,90,180])
    lower = np.array([10,0,150])
    upper = np.array([90,90,255])
    mask = cv2.inRange(image, lower, upper)

    return mask



#***********************滤波**********************************
def blur(image):
    kernel = 3
    #blurred = cv2.blur(image,(kernel,kernel))   #均值滤波
    #blurred = cv2.GaussianBlur(image,(kernel,kernel),0)  #高斯滤波
    #blurred = cv2.medianBlur(image,kernel)          #中值滤波去椒盐噪声
    blurred = cv2.bilateralFilter(image,kernel,5,5)

    return blurred

#*****************************图像处理部分******************************
def imageProcess(origin_img):
    '''
    # 判断摄像头是否打开
    if (cap.isOpened()):
        print('Open')
    else:
        print('摄像头未打开')

    ret,frame = cap.read()
    #image_size = cv2.GetSize(frame)
    '''

    flag = 0
    '''
    ret, frame = cap.read()
    while(ret == False):
        ret, frame = cap.read()
    '''
    global width, height
    undist_img, width, height = calibration(origin_img)
    img = blur(undist_img)
    
    #cv2.imshow('img',img)
    #get mask
    mask = select_red(img)

    result = cv2.bitwise_and(img, img, mask = mask)

    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    ret1, thresh1 = cv2.threshold(gray,5,255,cv2.THRESH_BINARY)

    #*******************得到完整连通域************************
    size = 10
    kernel = np.ones((size,size), dtype = np.uint8)
    new_img = cv2.erode(cv2.dilate(thresh1, kernel), kernel)

    #*******************边界集合****************************
    derp, contours, hierarchy = cv2.findContours(new_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(new_img,contours,-1,(0,0,255),3)

    #******************寻找最大连通域**********************

    areaArray = []
    
    if (len(contours) != 0):
        for i, c in enumerate(contours):
            area = cv2.contourArea(c)
            areaArray.append(area)
        sorteddata = sorted(zip(areaArray, contours), key=lambda x: x[0], reverse=True)
        bgst_cont = sorteddata[0][1]
        
        # Draw rect
        x, y, w, h = cv2.boundingRect(bgst_cont)
        
        cv2.rectangle(new_img, (x, y), (x + w, y + h), (255, 255, 255), 1)
        if w*h>100: flag=1
        else :flag=0
    else :
        flag = 0
        center = [0,0]
        x, y, w, h = [0, 0, 0, 0]

    return flag, x, y, w, h,new_img

#************************边界函数***************************
def boundary(a,upper_bound,lower_bound):
    if a > upper_bound:
        return upper_bound
    elif a < lower_bound:
        return lower_bound
    else:
        return a

#***********************pid参数初始化**************************
def pid_init():
    global error, error_, perror, perror_, ierror, derror
    global kp, ki, kd             #比例、积分、微分参数
    error_ = 0    #上一次的偏差
    perror_ = 0
    kp = 0.02
    ki = 0.001
    kd = 0.01

#*************************增量式PID值计算***********************
def cal_PID(tracking_width):
    global tracking_width_, tracking_height_
    #tracking_width_ = 200, tracking_height_ = 100
    global error, error_, perror, perror_, ierror, derror
    error = 200 - tracking_width
    perror = error - error_
    ierror = error
    derror = perror - perror_

    increment = kp*perror + ki*ierror + kd*derror

    return increment

#*************************主函数************************

if __name__ == '__main__':
    d.setStatus(mode = "speed")
    time.sleep(0.1)
    global error, error_, perror, perror_, ierror, derror
    global ret, frame
    # 判断摄像头是否打开
    if (cap.isOpened()):
        print 'Open'
    else:
        print '摄像头未打开'

    ret, frame = cap.read()
    while(ret == False):
        ret, frame = cap.read()
    flag_, x_, y_, tracking_width, tracking_height ,IM = imageProcess(frame)

    pid_init()
    
    i=0
    try:
        while flag_==0:
            flag_, x_, y_, tracking_width, tracking_height ,IM = imageProcess(frame)
            print " 1234"
            print frame[240,320]
        while(1):
            d.setStatus(mode = "speed")
            i=i+1

            ret, frame = cap.read()
            while(ret == False):
                ret, frame = cap.read()
            
            cv2.imshow('origin',frame)
            flag_, x_, y_, tracking_width, tracking_height,IM = imageProcess(frame)

            while flag_==0:
                flag_, x_, y_, tracking_width, tracking_height ,IM = imageProcess(frame)
            print flag_, x_, y_, tracking_width, tracking_height
            middle = width / 2
            tracking_middle = (2*x_ + tracking_width)/2
            delta_middle = tracking_middle - middle
            #width_ratio = tracking_width


            print "tracking_width",tracking_width
            print "delta_middle",delta_middle

            increment = cal_PID(tracking_width)
          
            #*******************转速与转向控制************************
            #物体不在画面中
            
            if x_ == 0 or x_ == width or flag_==0:
                print "out of bound!"
                d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")

            #小车跟随在物体正后方
            elif delta_middle < 20 and delta_middle > -20 :
                st = 0


                #根据窗口差进行控制
                if tracking_width <= 210 and tracking_width >= 190:
                    sm = 0.02
                    print "just follow"

                elif tracking_width < 190:
                    sm = boundary(0.02 + (200 - tracking_width)*0.002, 0.1, 0.02)
                    print "speed up"

                elif tracking_width > 210:
                    sm = boundary(0.02 + (200 - tracking_width)*0.001, 0.02, 0)
                    print "slow down"

            else :
                st = boundary((middle - tracking_middle)*0.003, 1, -1)
                print "middle", tracking_middle
                if(st > 0):
                    print "turn left!"
                else:
                    print "turn right!"

                if tracking_width <= 210 and tracking_width >= 190:
                    sm = 0.02
                elif(tracking_width > 210):
                    sm = boundary(0.02 + (200 - tracking_width)*0.002, 0.02, 0)
                elif(tracking_width < 190):
                    sm = boundary(0.02 + (200 - tracking_width)*0.002, 0.1, 0.02)
            '''          
            error_ = error
            perror_ = perror
            '''
            print sm, st
            d.setStatus(motor=sm, servo=st)
            
        

            cv2.imshow(" ",IM)
            if cv2.waitKey(1)  & 0xFF == ord('q'):
                break
            
            print "end"
    except KeyboardInterrupt:
        pass

d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
d.close()
del d
                                                  
cv2.waitKey(0)

cv2.destroyAllWindows()







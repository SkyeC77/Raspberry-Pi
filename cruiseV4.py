# -*- coding: utf-8 -*-
"""
Created on Sun Apr 29 12:30:07 2018

@author: Thinkpad
"""

import cv2
import numpy as np
from driver import driver
import math
import time
import argparse
d=driver()
d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")  
time.sleep(1)
mtx=np.mat([[ 335.89853515, 0,322.59797037],[0, 335.14127527,220.35461581],[ 0,0,1]])
dist=np.mat([[-3.32134083e-01,1.22371401e-01, 8.49838759e-04, -1.24911879e-04,-2.08547165e-02]])
def stopSign():
    ret,img=cap.read()
    while ret==False:
        ret,img=cap.read()
    white = [([180, 180, 180], [255,255,255])]
    blue =[([180, 60, 0], [255, 160, 120])]
    red = [([150,100,180], [220,170,255])]
    yellow = [([0,170,170], [80,255,255])]
    result=[0,0]
    #img_=img[50:300,:]
    gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #寻找圆
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,100,param1=100,param2=40,minRadius=20,maxRadius=70)  
    if circles is None:
        print "none"   
        return result
    else:           
        for circle in circles[0]:
            r=int(circle[2])
            print "r",r
            if r>20: 
                #关于颜色的检测
                x=int(circle[0])
                y=int(circle[1])
                sign=img[max(0,y-r*2):min(y+r*2,480),max(0,x-r*2):min(x+r*2,640)]     
                '''
                print x,y      
                sign=cv2.circle(sign,(x,y),r,(0,0,255),-1)
                cv2.imshow("sign",sign)
                '''
                #get mask
                #蓝
                for (lower, upper) in blue:
                    # 创建NumPy数组
                    lower = np.array(lower, dtype = "uint8")#颜色下限
                    upper = np.array(upper, dtype = "uint8")#颜色上限
                # 根据阈值找到对应颜色
                maskblue = cv2.inRange(sign, lower, upper)
                
                if maskblue is None:return result
                bluesum=sum(sum(maskblue/255))
                print "blue",bluesum
                
                #白
                for (lower, upper) in white:
                    lower = np.array(lower, dtype = "uint8")
                    upper = np.array(upper, dtype = "uint8")
                maskwhite = cv2.inRange(sign, lower, upper)
                if maskwhite is None:return result
                whitesum=sum(sum(maskwhite/255))
                print "white",whitesum
                '''
                #黄
                for (lower, upper) in yellow:
                    lower = np.array(lower, dtype = "uint8")
                    upper = np.array(upper, dtype = "uint8")
                maskyellow = cv2.inRange(sign, lower, upper)
                if maskyellow is None:return result
                yellowsum=sum(sum(maskyellow/255))
                '''
                '''
                #红
                for (lower, upper) in red:
                    lower = np.array(lower, dtype = "uint8")
                    upper = np.array(upper, dtype = "uint8")
                maskred = cv2.inRange(sign, lower, upper)
                if maskred is None:return result
                redsum=sum(sum(maskred/255))
                '''
                if whitesum>100 and bluesum>800:    
                    size = 15
                    kernel = np.ones((size, size), dtype=np.uint8)
                    img_close = cv2.erode(cv2.dilate(maskblue, kernel), kernel)
                    # 获取连通域img_close
                    derp, contours, hierarchy = cv2.findContours(img_close, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    # 最大连通域
                    areaArray = []
                    for i,c in enumerate(contours):
                        area = cv2.contourArea(c) 
                        areaArray.append(area) 
                    sorteddata = sorted(zip(areaArray, contours), key=lambda x: x[0], reverse=True) #x[0]->x[1]失败
                    bgst_cont = sorteddata[0][1] 
                    # Draw rect 
                    x, y, w, h = cv2.boundingRect(bgst_cont)
                    print "w,h",w*h
                    if w*h>1000 and bluesum<3000:
                        result=[1,w*h]
                return result
    return result


def cruiseIP():
    ret,img=cap.read()
    while ret==False:
        ret,img=cap.read()
    #*************************stopsign*******************************************

    #*************************图像处理*******************************************
    size=np.shape(img)
    height=size[0]
    width=size[1]
    
    img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    #img= cv2.medianBlur(img,5)
    ret,img=cv2.threshold(img,100,255,cv2.THRESH_BINARY_INV)
    IM=img.copy()
    #***********************近处参考点****************************************
    ref_unuseful=0
    pre_ref_unuseful=0
    ref_sum=0
    ref_num=0

    for i in range(height-3,height-1) :
        for j in range(width/6,5*width/6):
            if (img[i][j]==255 & img[i][j+1]==255 & img[i][j+2]==255 & img[i][j+3]==255 & img[i][j+4]==255):
                ref_num+=1
                ref_sum+=j
    if(ref_num!=0):
        ref=ref_sum/ref_num
    else:
        ref=0
    #**********检验近处参考点是否有利用价值**********
    if(ref!=0):
        i=ref
        c_left=0
        while(img[height-1][i]==255):
            i-=1
            if (i<106):
                break
            c_left+=1
            
        j=ref+1
        c_right=0
        while(img[height-1][j]==255):
            j+=1
            if(j>533):
                break
            c_right+=1
        if(c_left>30 or c_right>30 or c_left==0 or c_right==0):
            #处于十字路口或者中线不再路中央--失效
            ref_unuseful=1
            
    p1=(ref,0)
    p2=(ref,height)
    cv2.line(IM,p1,p2,(255))
    
    #*****************************远处参考点***************************************
    ref_sum=0
    ref_num=0
    
    for i in range(height-80,height-77) :
        for j in range(width/6,5*width/6):
            if (img[i][j]==255 & img[i][j+1]==255 & img[i][j+2]==255 & img[i][j+3]==255 & img[i][j+4]==255):
                ref_num+=1
                ref_sum+=j
    if(ref_num!=0):
        pre_ref=ref_sum/ref_num
    else:
        pre_ref=0
        
    #**************检验远处参考点是否有利用价值**************
    if(pre_ref!=0):
        i=pre_ref
        c_left=0
        while(img[height-40][i]==255):
            i-=1
            if (i<106):
                break
            c_left+=1
        j=pre_ref+1
        c_right=0
        while(img[height-40][j]==255):
            j+=1
            if(j>533):
                break
            c_right+=1
        if(c_left>30 or c_right>30 or c_left==0 or c_right==0):
            #处于十字路口或者中线不再路中央--失效
            pre_ref_unuseful=1
    
    p1=(pre_ref,0)
    p2=(pre_ref,height)  
    cv2.line(IM,p1,p2,(255))
   
    return ref,pre_ref,IM,ref_unuseful,pre_ref_unuseful       

def FuncImageP():
    nothing=0#没有读到东西
    park=0
    b_park=0
    center=[0,0]
    decelerate=0
    #********************读图像**********************************
    ret,img=cap.read()
    while(ret==False):
        ret,img=cap.read() 
    #********************校正**********************************              
    img = cv2.undistort(img,mtx,dist,None,mtx) 
    
    #********************提取车位**********************************
    
    #colour = [([80,60,60], [130,100,80])]         #blue1
    #colour = [([70,0,100], [150,60,180])]        #red2
    #colour = [([0,100,110], [50,180,180])]       #yellow3
    #colour = [([120,80,0], [200,160,100])]           #blue4
    #red = [([155,110,0], [220,160,40])]
    #5#
    #colour = [([30,0,80], [100,60,125])]        #red2
    #colour = [([10,70,80], [70,130,130])]  #yellow3
    #colour = [([60,60,0], [130,100,40])]           #blue4
    #colour = [([60,25,15], [110,75,70])]         #blue1

    #colour = [([30,0,80], [110,70,180])]        #red2
    #colour = [([20,110,110], [100,200,190])]  #yellow3
    colour = [([110,80,0], [190,160,80])]           #blue4
    #colour = [([60,25,15], [110,75,70])]         #blue1
    
    for (lower, upper) in colour:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
          
    mask = cv2.inRange(img, lower, upper)
        
    output = cv2.bitwise_and(img, img, mask = mask)
    
    gray=cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    gray=cv2.medianBlur(gray,5)
    ret,thresh=cv2.threshold(gray,5,255,cv2.THRESH_BINARY) 
    kernel = np.ones([15,15])
    img_close = cv2.erode(cv2.dilate(thresh, kernel), kernel)
    #********************减速判断**********************************
    for i in range(640):
        if(img_close[479][i]==255):
            decelerate=1
    #**************************************************************
    judgee=0
    for i in range(640):
        if(img_close[380][i]==255):
            judgee=1
    if (judgee==0):
        for i in range(640):
            if (img_close[479][i]==255):
                b_park=1
                
    #********************停车判断**********************************
    judge=0
    for i in range(640):
        if(img_close[420][i]==255):
            judge=1
    if (judge==0):
        for i in range(640):
            if (img_close[479][i]==255):
                park=1
                nothing=0
                center=[0,0]
                slopee=0
                return img_close,slopee,center,nothing,park,decelerate,b_park
    
    #********************边界集合**********************************
    derp, contours, hierarchy = cv2.findContours(img_close, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #********************没有提取到车位**********************************
    if len(contours)==0:
        nothing=1
        center=[0,0]
        slopee=0
        return img_close,slopee,center,nothing,park,decelerate,b_park

    areaArray=[]
    for i,item in enumerate(contours):
        area=cv2.contourArea(item)
        areaArray.append(area)
    sorteddata=sorted(zip(areaArray,contours),key=lambda x:x[0],reverse=True)
    bgst=sorteddata[0][1]
    x,y,w,h=cv2.boundingRect(bgst)
    sss=np.zeros([480,640],dtype=np.uint8)
    sss[y:y+h,x:x+w]=255
    img_close=cv2.add(img_close,np.zeros(np.shape(img_close),dtype=np.uint8),mask=sss)
    #********************重心**********************************
    M = cv2.moments(bgst)
    if (M["m00"]!=0):
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        nothing=1
        slopee=0
        center=[0,0]
        return img_close,slopee,center,nothing,park,decelerate,b_park
            
    
    #********************下底中点**********************************
    maxdistld=0
    maxdistrd=0

    global xrd,xld,yrd,yld
    xrd=320
    xld=320
    yrd=240
    yld=240
    for i in range (len(bgst)):
        index=bgst[i][0]
        x_tmp=index[0]
        y_tmp=index[1]
        if y_tmp>cY and x_tmp>cX:
            distance=(x_tmp-cX)*(x_tmp-cX)+(y_tmp-cY)*(y_tmp-cY)
            if distance>maxdistrd:
                maxdistrd=distance
                xrd=x_tmp
                yrd=y_tmp
        elif y_tmp>cY and x_tmp<cX:
            distance=(x_tmp-cX)*(x_tmp-cX)+(y_tmp-cY)*(y_tmp-cY)
            if distance>maxdistld:
                maxdistld=distance
                xld=x_tmp
                yld=y_tmp
       
    xdown=(xld+xrd)/2
    ydown=(yld+yrd)/2
    #xup=(xlu+xru)/2
    #yup=(ylu+yru)/2
    '''
    #********************停车判断**********************************
    if(yup>470):
        park=1
        slopee=0
        return img_close,slopee,center,nothing,park
    '''
    #********************斜率**********************************
    cv2.line(img_close,(cX, cY), (xdown,ydown) ,(0))
    if (cX-xdown==0):
        slopee=0
    else:
        slopee=-float(cY-ydown)/float(cX-xdown)
    center=[cX,cY]
    #print slopee,center,nothing,park
    return img_close,slopee,center,nothing,park,decelerate,b_park


if __name__ == '__main__':
    d.setStatus(mode="speed")
    try:
        cap=cv2.VideoCapture(1)
        distline=320
        nearline=320
        reff,pre_reff,lineIM,ref_un,pre_ref_un=cruiseIP()
        while (reff<220 or reff>420):
            reff,pre_reff,lineIM,ref_un,pre_ref_un=cruiseIP()
        
        st=0
        while(1):
            if_stop=stopSign()
            print if_stop
            #while(if_stop[1]<2000 and if_stop[0]==0):
                #if_stop=stopSign()
            if (if_stop[0]==0):
                #print if_stop
                reff,pre_reff,lineIM,ref_un,pre_ref_un=cruiseIP()
                if(ref_un!=1): nearline=reff
                if(pre_ref_un!=1): distline=reff
                if (nearline==0):
                    sm=0.08
                    if (st>0):
                        st=1
                    else:
                        st=-1
                else:
                    if distline==0:
                        if nearline>320:distline=580
                        else:distline=60
                    detref=nearline-320
                    theta=(math.atan((distline-nearline)/87))/np.pi
                    
                    if (abs(theta)>0.16 or abs(detref)>160):
                        sm=0.05
                    else:
                        sm=0.06
                
                    st=-(theta*0.4+detref*0.003)
                    if(st<-0.9):
                        st=-1
                    if(st>0.9):
                        st=1
                print sm,st
                d.setStatus(motor = sm, servo = st)
                
                cv2.imshow("IM",lineIM)
                if cv2.waitKey(1) & 0xFF==ord('q'):
                    break
            else:
                d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
                break
        print "cruise ending"
        #park
        
        cap=cv2.VideoCapture(0)
        d.setStatus(mode="speed")
        time.sleep(3)
        im,slope,cent,flag,ppark,dspeed,bpark=FuncImageP()

        while (flag==1):
            im,slope,cent,flag,ppark,dspeed,bpark=FuncImageP()
        limit2=310+abs(cent[0]-320)*0.22
        limit1=310-abs(cent[0]-320)*0.22
        print limit1,limit2
    
        while (ppark!=1):
            if (slope>2 or slope<-2 or slope==0):   
                print 3,slope
                st=(cent[0]-320)*0.016
                #sm=-0.04
                if st>0.9: st=0.9
                if st<-0.9: st=-0.9
                if (dspeed==1) :sm=-0.03
                else:sm=-0.05
                
            elif (slope<0.0 and cent[0]>limit1):     
                print '1_2',slope
                st=0.9
                if (dspeed==1) :sm=-0.03
                else:sm=-0.05
               
            elif (slope<0.0 and cent[0]<limit1):   
                print '1_1',slope
                if slope>0: st=(1.57-math.atan(slope))*1.2
                elif slope<0:st=-(1.57+math.atan(slope))*1.2
                elif slope==0:st=0
                #st=-(1.57+math.atan(slope))*0.8
                if st>0.9: st=0.9
                if st<-0.9: st=-0.9 
                if (dspeed==1) :sm=-0.03
                else:sm=-0.05
               
            elif (slope>0.0 and cent[0]<limit2):               
                print '2_2',slope
                st=-0.9
                if (dspeed==1) :sm=-0.03
                else:sm=-0.05
        
   
            elif (slope>0.0 and cent[0]>limit2): 
                print '2_1',slope
                if slope>0: st=(1.57-math.atan(slope))*1.2
                elif slope<0:st=-(1.57+math.atan(slope))*1.2
                elif slope==0:st=0
                #st=-(1.57+math.atan(slope))*0.8
                if st>0.9: st=0.9
                if st<-0.9: st=-0.9
                if (dspeed==1) :sm=-0.03
                else:sm=-0.05
                
            print sm,st
            d.setStatus(motor = sm, servo = st)
            im,slope,cent,flag,ppark,dspeed,bpark=FuncImageP()
            while (flag==1):
                im,slope,cent,flag,ppark,dspeed,bpark=FuncImageP()
            cv2.imshow("",im)       
            if cv2.waitKey(1) & 0xFF==ord('q'):
                break
            print st,slope,cent[0],ppark
    except KeyboardInterrupt:
        pass

d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")      
d.close()
del d
cv2.waitKey(0)
cv2.destroyAllWindows()

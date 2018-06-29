# -*- coding: utf-8 -*-
"""
Created on Mon Jun 11 17:43:05 2018

@author: Thinkpad
"""
import cv2
import numpy as np
import time
from driver import driver
import math
mtx=np.mat([[ 335.89853515, 0,322.59797037],[0, 335.14127527,220.35461581],[ 0,0,1]])
dist=np.mat([[-3.32134083e-01,1.22371401e-01, 8.49838759e-04, -1.24911879e-04,-2.08547165e-02]])

d=driver()
d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop") 
cap=cv2.VideoCapture(0)
left=0
right=0

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
    colour = [([60,25,15], [110,75,70])]         #blue1
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

#***************************运动控制部分*************************************
if __name__ == '__main__':
    d.setStatus(mode="speed")
    time.sleep(1)
    im,slope,cent,flag,ppark,dspeed,bpark=FuncImageP()

    while (flag==1):
        im,slope,cent,flag,ppark,dspeed,bpark=FuncImageP()
    print 11
    limit2=310+abs(cent[0]-320)*0.22
    limit1=310-abs(cent[0]-320)*0.22
    print limit1,limit2
    
    try:
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
            print st,slope,cent[0],ppark
            
            if cv2.waitKey(1) & 0xFF==ord('q'):
                break
    except KeyboardInterrupt:
        pass
    
d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")      
d.close()
del d

cv2.waitKey(0)
cv2.destroyAllWindows()    


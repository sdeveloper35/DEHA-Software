import cv2
import numpy as np
import math
import time


from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

from picamera.array import PiRGBArray
from picamera import PiCamera

distance_from_camera = 462  # kameranın hedefe olan uzaklığı (Cm olarak) 

Cx = 0
Cy = 0

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        cv2.putText(frame, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,1.0, (0, 0, 0), thickness=1)
        print(x,y)
        cv2.imshow('DETECTED CIRCLE', frame)

        RGB = frame
        hsv = cv2.cvtColor(RGB,cv2.COLOR_BGR2HSV)
        #print('hsv -> ',hsv[x,y])
        s0,s1,s2=cv2.split(hsv)
        #split() h, s ve v değerlerinin olduğu matrixler dönderiyor(her matrixte tüm pixellerin ayrıştırılmış
        #ayrı ayrı h s ve v değerleri var)(Ekrandaki tüm pixellerin)
        print("H:",s0[y][x],"   S:",s1[y][x],"    V:",s2[y][x])

target = False


def findRedContours():
    global vehicle
    global target
    global distance #uzaklığı global tanımladım
    global contour_area #alanı global değişken olarak tanımladım
    global ptin_contour
    global cX
    global cY
    # En büyük contouru seçiyoruz
    global one_pixel_horizontal
    global one_pixel_vertical
    if len(contours) != 0:
        c = max(contours, key=cv2.contourArea)  #maximum alana sahip contour

        contour_area = int(cv2.contourArea(c))
        if showCircleArea:
            print(contour_area)

        if contour_area >= 2000 and contour_area <= 180000:
            target = True

            Contour_Check = cv2.pointPolygonTest(c, (img_Center_X , img_Center_Y),False)

            #print(Contour_Check)
            if Contour_Check >= 0:
                ptin_contour = True
                contour_color = (0 , 255 , 0)
                #pool_font_color = (255 , 255 , 0)      #Havuz yazısının rengi tek renk olsun diye bu satırı çıkardım
            else:
                ptin_contour = False
                contour_color  = (255 , 0 , 0)
                #pool_font_color = (0 , 0 , 255)      #Havuz yazısının rengi tek renk olsun diye bu satırı çıkardım

            # Calculate Moments for each contour
            M = cv2.moments(c)

            if M["m00"] != 0:
                # Calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
                cv2.drawContours(image, [c], 0, contour_color, 3)
                cv2.arrowedLine(image, (320, 240), (cX, cY), (255, 255, 0), 2)
                cv2.putText(image,'Havuz', (cX+10, cY+10),cv2.FONT_HERSHEY_SIMPLEX, 1, pool_font_color, 2, cv2.LINE_AA)

                #Uzaklık Hesabı
                distx = (img_Center_X-cX)
                disty = (img_Center_Y-cY)

                #one_pixel_horizontal = ((math.tan(abs(vehicle.attitude.roll) + (cam_angle_x/2)) - math.tan(abs(vehicle.attitude.roll) - (cam_angle_x/2))) * distance_from_camera) / 640
                #one_pixel_vertical =  ((math.tan(abs(vehicle.attitude.pitch) + (cam_angle_y/2)) - math.tan(abs(vehicle.attitude.pitch) - (cam_angle_y/2))) * distance_from_camera) / 480

                global realdist_x
                global realdist_y
                realdist_x = (distx*one_pixel_horizontal)# + (math.tan(abs(vehicle.attitude.roll) - (cam_angle_x/2))*distance_from_camera)
                realdist_y = (disty*one_pixel_vertical)# + (math.tan(abs(vehicle.attitude.pitch) - (cam_angle_y/2))*distance_from_camera)

                #print('distance x : %d CM , distance y : %d CM'%((realdist_x),(realdist_y)))
                #print("attitude : {}".format(vehicle.attitude))
                print("pitch : {}".format(vehicle.attitude.pitch * math.pi / 180))
                
                #print("{2} {1}".format(3,4))
                distance = ((distx**2) + (disty**2))**0.5

        else:
            target = False
            ptin_contour = False

cap = cv2.VideoCapture(0)
upt = False
ptin_contour = False
circle_color = (0 , 255 , 0)
contour_color = (0 , 255 , 0)
pool_font_color = (255 , 255 , 0)
Use_Circle_Check = False
Aim_Length = 40
total_Mean_Check = False
hsv_Mean_limit = 60
showCircleArea = False
ShowDistance = False
distance = 0
dy = 0



frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

size = (frame_width, frame_height)
print(size)

cam_angle_x = 1.0856
cam_angle_y = 0.8517


realdist_x = 0
realdist_y = 0

one_pixel_horizontal = ((math.tan(cam_angle_x/2)) * distance_from_camera) / 320
one_pixel_vertical = ((math.tan(cam_angle_y/2)) * distance_from_camera) / 240
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

connection_address = '/dev/ttyACM0'  # kontrol et
baud_rate = 115200
print("\nConnecting to vehicle on: " + connection_address + " with baud rate: " + str(baud_rate))
vehicle = connect(connection_address, wait_ready=True, baud=baud_rate)



for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    
    print("pitch : {}".format(vehicle.attitude.pitch * 180 / math.pi))
    image = frame.array

    #İmage Size
    #dimentions = frame.shape    #shape[0] --> resmin boyu ---------- shape[1] --> Resmin eni --------- shape[2] ---> resimdeki channel sayısı
    #print(dimentions[0],dimentions[1])
    #boy,en = dimentions[0],dimentions[1]
    img_Center_X = int(320)
    img_Center_Y = int(240)


    #HSV'ye dönüştür
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    lower_red = np.array([150, 50, 50])
    upper_red = np.array([200, 255, 255])

    #Thresholding
    red_hue_range = cv2.inRange(hsv, lower_red, upper_red)

    #Threshold ile ayrılan resmi tekrar maskele
    res = cv2.bitwise_and(image, image, mask=red_hue_range)

    #3x3 blurla (Kullanılmıyor)
    gray_blurred1 = cv2.blur(red_hue_range,(3,3))

    #kırmızı ayıkladığın yuvarlağı gray yap
    gray_blurred2 = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

    #3x3 blue uygula (üstteki gereksiz)
    gray_blurred = cv2.blur(gray_blurred2,(10,10))


    #kenarları bul - KULLANMADIN
    canny_edge = cv2.Canny(red_hue_range,50,240)
    #--------------------------------------------------

    #detected_circles = cv2.HoughCircles(image=gray_blurred,method=cv2.HOUGH_GRADIENT,dp=1,minDist=600,param1=50,param2=30,minRadius=50,maxRadius=150)

    contours, hierarchy = cv2.findContours(red_hue_range, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    # resmin hsv değerlerinin ortalamasını alma
    hsv2 = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
    # Disassemble for each channel
    h, s, v = cv2.split(hsv2)
    mean1 = h.mean()
    mean2 = s.mean()
    mean3 = v.mean()
    #---print('H:% .1f S:% .1f V:% .1f' % (mean1, mean2, mean3))
    # ek (toplam ortalama)
    total_mean = (mean1 + mean2 + mean3) / 3

    if total_mean <= hsv_Mean_limit:
        total_Mean_Check = True
    else:
        total_Mean_Check = False

    if Use_Circle_Check:
        if total_mean <= hsv_Mean_limit:
            total_Mean_Check = True
            #draw circles that are detected--------------------------------------
            if detected_circles is not None:
                target = True
                upt=True
                #convert the circle parameters a, b and r to integers
                detected_circles = np.uint16(np.around(detected_circles))

                for pt in detected_circles[0, :]:
                    a, b, r = pt[0], pt[1], pt[2]

                    #Draw the circumference of the circle
                    cv2.circle(image, (a,b), r, circle_color,3)

                    #Draw a small circle to show th ecenter
                    cv2.circle(image, (a,b), 1, (0,0,255),3)
            #Belirlenen daireleri çizme buraya kadar---------------------------- (Altta contour muhabbeti var)
                    cv2.putText(image,'Havuz', (a+10, b+10),cv2.FONT_HERSHEY_SIMPLEX, 1, pool_font_color, 2, cv2.LINE_AA)
                    #distance entegresi---------------------(ekle daha eklemedim)

            else:
                target = False
                upt = False
                findRedContours()

        else:
            findRedContours()
            total_Mean_Check = False

    else:
        findRedContours()

    #Dairenin içinde mi?
    if upt:
        distance = (((a-img_Center_X)**2)+((b-img_Center_Y)**2))**0.5
        if abs(distance) < r:
            circle_color = (0,255,0)
        else:
            circle_color = (255,0,0)
            cv2.arrowedLine(image,(img_Center_X , img_Center_Y) , (a,b) , (255,255,0) , 2)

    #hsv ortalamalarını yazdır------------------------------
    image = cv2.putText(image,'H:% .1f S:% .1f V:% .1f  Tot: % .1f' % (mean1, mean2, mean3,total_mean),(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2,cv2.LINE_AA   )

    if showCircleArea: #Daire alanını yazdır (Contour için - normal daireyi eklemedim)
        cv2.putText(image, 'Kirmizi Alan : %d' % contour_area, (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    if ShowDistance:    #Merkeze uzaklığı yazdır (bu da contour için)
        cv2.putText(image, 'Distance : %.1f x : %.1f Cm y : %.1f Cm' % (distance,realdist_x,realdist_y), (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2, cv2.LINE_AA)

    #cv2.putText(image, 'Y : %.1f' % dy, (450, 100) #çıkar
                #cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)


    #Kameranın pist tesğit edip etmediği
    #if total_Mean_Check:
     #   if target:
      #      image = cv2.putText(image, 'Pist Tespit Edildi' , (50, 75),
       #                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
       # else:
        #    image = cv2.putText(image, 'Pist Tespit Edilemedi', (50, 75),
         #                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)


    # Merkez noktası ekle (Drone Aim)
    cv2.rectangle(image, (img_Center_X - int(Aim_Length / 2), img_Center_Y - int(Aim_Length / 2)),
                  (img_Center_X + int(Aim_Length / 2), img_Center_Y + int(Aim_Length / 2)), (0, 255, 255), 1)
    # cv2.circle(image , ( img_Center_X , img_Center_Y ) , int(Aim_Length/2) , (50,255,255) , 2)
    # cv2.line(image , ( img_Center_X , img_Center_Y - int(Aim_Length/2) ) , ( img_Center_X , img_Center_Y + int(Aim_Length/2) ) , (50,255,255) , 2)
    # cv2.line(image , ( img_Center_X - int(Aim_Length/2) , img_Center_Y ) , ( img_Center_X + int(Aim_Length/2) , img_Center_Y ) , (50,255,255) , 2)

    #cv2.namedWindow('DETECTED CIRCLE',cv2.WINDOW_NORMAL)    #Window oluştur (Bu satır olmadan sadece imshow() ile de yapılabilir)
    #cv2.resizeWindow('DETECTED CIRCLE',1280,920)           #Windowun size'ını değiştir

    #video.write(image)     #Şuan video kaydetmek istemediğim için kullanmıyorum
    """
    cv2.imshow('DETECTED CIRCLE',image)
    cv2.imshow('Gray',gray_blurred)
    cv2.imshow('MASK', res)
    cv2.imshow('CANNY',canny_edge)
    cv2.imshow('RedHue',red_hue_range)
    cv2.setMouseCallback('DETECTED CIRCLE', on_EVENT_LBUTTONDOWN)
    #cv2.waitKey(0)
    key=cv2.waitKey(10)
    if key & 0xFF == ord('q'):
        break
    if key & 0xFF == ord('p'):
        while(True):
            if cv2.waitKey(10) & 0xFF == ord('s'):
                break
    if key & 0xFF == ord('a'):
        if showCircleArea:
            showCircleArea=False
        else:
            showCircleArea=True
    if key & 0xFF == ord('d'):
        if ShowDistance:
            ShowDistance=False
        else:
            ShowDistance=True
    """
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
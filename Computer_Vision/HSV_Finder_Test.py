import cv2
import numpy as np

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        #cv2.putText(frame, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,1.0, (0, 0, 0), thickness=1)
        print(x,y)
        #cv2.imshow('DETECTED CIRCLE', frame)

        RGB = frame
        hsv = cv2.cvtColor(RGB,cv2.COLOR_BGR2HSV)
        #print('hsv -> ',hsv[x,y])
        s0,s1,s2=cv2.split(hsv)
        #split() h, s ve v değerlerinin olduğu matrixler dönderiyor(her matrixte tüm pixellerin ayrıştırılmış
        #ayrı ayrı h s ve v değerleri var)(Ekrandaki tüm pixellerin)
        print("H:",s0[y][x],"   S:",s1[y][x],"    V:",s2[y][x])#y ve xlerin yeri ilk y sonra x olacak!!!

FindHSV = False

def KareBul(frame):
    original = frame.copy()
    image =cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

cap = cv2.VideoCapture(0)
upt = False
circle_color = (0 , 255 , 0)
while(True):
    global frame
    ret,frame = cap.read()
    if FindHSV:
        x = 320
        y = 240
        RGB = frame
        hsv = cv2.cvtColor(RGB,cv2.COLOR_BGR2HSV)
        #print('hsv -> ',hsv[x,y])
        s0,s1,s2=cv2.split(hsv)
        #split() h, s ve v değerlerinin olduğu matrixler dönderiyor(her matrixte tüm pixellerin ayrıştırılmış
        #ayrı ayrı h s ve v değerleri var)(Ekrandaki tüm pixellerin)
        print("H:",s0[y][x],"   S:",s1[y][x],"    V:",s2[y][x])#y ve xlerin yeri ilk y sonra x olacak!!!
        FindHSV = False
    #Merkez noktası ekle
    #cv2.rectangle(frame, (300, 220), (340, 260), (0, 255, 255),1)

    #HSV'ye dönüştür
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    #lower_red = np.array([150, 50, 50])
    #upper_red = np.array([200, 255, 255])

    lower_red = np.array([170, 104, 90])
    upper_red = np.array([180, 175, 210])

    #Thresholding
    red_hue_range = cv2.inRange(hsv, lower_red, upper_red)

    #Threshold ile ayrılan resmi tekrar maskele
    res = cv2.bitwise_and(frame, frame, mask=red_hue_range)

    #Deneme-----------------------------
    kernel = np.ones((5, 5), 'uint8')
    dilate_img = cv2.dilate(red_hue_range, kernel, iterations=1)
    closing = cv2.morphologyEx(red_hue_range, cv2.MORPH_CLOSE, kernel)
    res_gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
    image = cv2.GaussianBlur(res_gray,(5,5),1)



    #3x3 blurla (Kullanılmıyor)
    gray_blurred1 = cv2.blur(red_hue_range,(3,3))

    #kırmızı ayıkladığın yuvarlağı gray yap
    gray_blurred2 = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

    #3x3 blue uygula (üstteki gereksiz)
    gray_blurred = cv2.blur(gray_blurred2,(10,10))


    #kenarları bul - KULLANMADIN
    canny_edge = cv2.Canny(gray_blurred,50,240)
    #--------------------------------------------------
    # resmin hsv değerlerinin ortalamasını alma
    hsv2 = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
    # Disassemble for each channel
    h, s, v = cv2.split(hsv2)
    mean1 = h.mean()
    mean2 = s.mean()
    mean3 = v.mean()
    #print('H:% .1f S:% .1f V:% .1f' % (mean1, mean2, mean3))
    # ek (toplam ortalama)
    total_mean = (mean1 + mean2 + mean3) / 3
    #hsv ortalamalarını yazdır
    frame = cv2.putText(frame,'H:% .1f S:% .1f V:% .1f  Tot: % .1f' % (mean1, mean2, mean3,total_mean),(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2,cv2.LINE_AA   )
    approval = input('Take HSV (Y) or (N) : ')
    if approval == 'y':
        FindHSV = True
        
"""
    cv2.imshow('DETECTED CIRCLE',frame)
    cv2.imshow('Gray',gray_blurred)
    cv2.imshow('MASK', res)
    cv2.imshow('CANNY',canny_edge)
    cv2.imshow('RedHue',red_hue_range)
    cv2.imshow('HSV',hsv)
    cv2.imshow('gaussian', image)
    cv2.imshow('Closing',closing)
    cv2.setMouseCallback('DETECTED CIRCLE', on_EVENT_LBUTTONDOWN)
    #cv2.waitKey(0)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
"""
import numpy as np
import cv2
import math

def intersection(rho1, theta1, rho2, theta2):
    a = np.cos(theta1)
    b = np.sin(theta1)
    c = np.cos(theta2)
    d = np.sin(theta2)
    det = a*d - b*c
    try:
        y = (d*rho1 - b*rho2) / det
        x = (-c*rho1 + a*rho2) / det
    except:
        pass
    return y, x

def draw(I,points,h):
    max = 0
    y=0
    for i in range(len(points) - 1):
        if points[i][2] > max:
            if points[i][1]<h/2 +50 and points[i][1]> h/2-50:
                max = points[i][2]
                #x = points[i][0]
                y = points[i][1]

    cv2.line(I, (0, int(y)), (640, int(y)), (0, 0, 255), 5)

def RANSAC(x,y,rho,theta):
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    return np.linalg.norm(np.cross([x - x1, y - y1], [x2 - x1, y2 - y1])) / np.linalg.norm([x1 - x2, y1 - y2])


cap = cv2.VideoCapture('high.avi')
w = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)) # width of the frame
h = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))
fourcc = cv2.cv.CV_FOURCC(*'XVID')

ret1, Iv1 = cap.read()
ret2, Iv2 = cap.read()
ret3, Iv3 = cap.read()

out = cv2.VideoWriter('result1.avi', fourcc, 30.0, (w, h))

while True:
    
    
    ret, K = cap.read()
    Iv3 = Iv2
    Iv2 = Iv1
    Iv1 = K
    I =  K  + Iv2

    if ret == False:  # end of video (perhaps)
        break

    G = cv2.cvtColor(I,cv2.COLOR_BGR2GRAY) # -> grayscale

    E = cv2.Canny(G,100,200) # find the edges

    min_votes = 100 # mininum votes to be considered a line
    L = cv2.HoughLines(E,1,np.pi/180,min_votes)

    filtered=[]
    lines = []
    points=[]
    count=0

    for itr in range(2*L[0].shape[0]):
        # print L[0].shape[0]
        lines= L[0,np.random.choice(L[0].shape[0],size=2,replace=False)]

        rho1, theta1 = lines[0]
        rho2, theta2 = lines[1]
        if ((0.3<theta1<1.4) or (1.6<theta1<2)) and ((0.3<theta2<1.4) or (1.6<theta2<2)) :
            
            x,y = intersection(rho1, theta1, rho2, theta2)
            
            if x > 1 and y > 1:
                if not math.isnan(x) or not math.isnan(y):
                    if not math.isinf(y) or not math.isinf(x):
                        for i in range(0,L[0].shape[0]):
                            rho, theta =L[0,i]
    
                            distance = RANSAC(x,y,rho,theta)
                            if distance < 5:
                                count = count + 1
    
                        points.append([x, y, count])
                        count = 0

    draw(K,points,h)

    cv2.imshow("Horizion Line",K)
    cv2.imshow("canny",E)
    # out.write(I)
    key = chr(cv2.waitKey(1) & 0xFF)
    if (key == 'q'):
        break
import numpy as np
import cv2
from matplotlib import pyplot as plt
import math



def detect_features(img, kp1, des1, frame, frame_g):
    MIN_MATCH_COUNT = 60

    # find the keypoints and descriptors with SURF in each image

    kp2, des2 = surf.detectAndCompute(frame_g,None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    # Configura o algoritmo de casamento de features
    try: flann = cv2.FlannBasedMatcher(index_params, search_params)
    except: pass
    # Tenta fazer a melhor comparacao usando o algoritmo
    matches = flann.knnMatch(des1,des2,k=2)
    centro = (frame.shape[1]//2, frame.shape[0]//2)
    #print(frame.shape[0])
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    print("%d/%d matches are found" % (len(good), MIN_MATCH_COUNT))
    
    if len(good) > MIN_MATCH_COUNT:
        # Separa os bons matches na origem e no destino
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)


        # Tenta achar uma trasformacao composta de rotacao, translacao e escala que situe uma imagem na outra
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img.shape[0],img.shape[1]

        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

        # Transforma os pontos da imagem origem para onde estao na imagem destino
        dst = cv2.perspectiveTransform(pts,M)
        # Desenha as linhas
        frame = cv2.polylines(frame,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        x=0
        y=0
        for i in range(0, len(np.int32(dst)-1)):
            x += np.int32(dst)[i][0][0]
            y += np.int32(dst)[i][0][1]
        media = (x // 4, y // 4)
        #Pcentro=cv2.circle(frame,centro,10,150,3, cv2.LINE_AA)
        #Pcentro=cv2.circle(frame,media,10,255,3, cv2.LINE_AA)
        x1 = np.int32(dst)[0][0][0]
        y1 = np.int32(dst)[0][0][1]

        x2 = np.int32(dst)[1][0][0]
        y2 = np.int32(dst)[1][0][1]

        x3 = np.int32(dst)[2][0][0]
        y3 = np.int32(dst)[2][0][1]


        xdist1 = x1 - x2
        ydist1 = y1 - y2

        xdist2 = x1 - x3
        ydist2 = y1 - y3


        lado1 = math.sqrt((xdist1)**2 + (ydist1)**2)
        lado2 = math.sqrt((xdist2)**2 + (ydist2)**2)

        area = lado1 * lado2
        #print("matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
        #print(area)
    else:
        matchesMask = None
        area = 0
        media = (0,0)

    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                    singlePointColor = None,
                    matchesMask = matchesMask, # draw only inliers
                    flags = 2)
    #frame=drawMatches(img,kp1,frame_g,kp2,good[:20])
    cv2.imshow('original',frame)
    cv2.waitKey(1)
    return media, centro, area


if __name__=="__main__":
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    surf = cv2.xfeatures2d.SURF_create(hessianThreshold = 3000)
    imagem = cv2.imread("powerpuff-girls.png")  #Arquivo
    kp1, des1 = surf.detectAndCompute(imagem, None)

    while(True):
        ret, frame = cap.read()
        frame = cv2.imread("powerpuff-girls.png")  #Migue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        media, centro, area = detect_features(imagem, kp1, des1, frame, gray)

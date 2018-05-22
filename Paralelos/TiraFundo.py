import cv2
from matplotlib import pyplot as plt
import numpy as np
import time as t

def auto_canny(image, sigma = 0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

# Exemplo para um frame

Fundo = cv2.imread("FrenteSea.jpg")
CObjeto = cv2.imread("FundoSea.jpg")

diferenca = cv2.subtract(CObjeto,  Fundo)
diferenca2 = cv2.subtract(Fundo,  CObjeto)

or_img = cv2.bitwise_or(diferenca, diferenca2)
not_img = cv2.bitwise_not(or_img)

blur = cv2.GaussianBlur(not_img,(19,19),0)
bordas = auto_canny(blur)

cv2.imwrite("NFundoSea.jpg",not_img)

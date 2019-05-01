import numpy as np
import cv2 as cv

THRESHOLD = 90

def onTopOfWhite(image):
    image = cv.cvtColor(image,cv.COLOR_BGR2HSV)
    l,a,b = cv.split(image)
    aux = a.mean()
    #cv.imshow('result1',l)
    #cv.imshow('result2',a)
    #cv.imshow('result3',b)
    #cv.waitKey()
    if(aux < THRESHOLD):
        return True
    else:
        return False

if __name__ == "__main__":
    name = input("input image name: ")
    img = cv.imread(name, -1)
    #captura da imagem pode ser feita por aqui pela camera ao inves de ler imagem, funciona do mesmo jeito

    #cv.namedWindow('orig',cv.WINDOW_NORMAL)
    #cv.imshow('orig',a)
    result = onTopOfWhite(img)
    if(result):
        print("big white area detected")
        pass
    else:
        print("not on top of white area")
        pass
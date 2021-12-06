"""
Structure light 3D scanner

@TODO:
    * Довършване на генериране на шаблони
        - Преглед примери от examples
    * Въртене на маса
    * Проектиране на проектор
    * Калибриране на камера
    * Калибриране на проектор
    * !!!СКАНИРАНЕ!!! = проектиране, въртене меса, заснемане, обработка
"""
import cv2
import numpy as np
import structuredlight as sl
import math

def main():
    width  = 640
    height = 480

    gray = sl.Gray()

    # Generate and Decode x-coord
    # Generate
    imlist_posi_pat = gray.generate((width, height))
    # imlist_nega_paat = sl.invert(imlist_posi_pat)
    # for img in imlist_posi_pat:
    #     cv2.imshow('image',img)
    #     cv2.waitKey(0)
    # for img in imlist_nega_paat:
    #     cv2.imshow('image',img)
    #     cv2.waitKey(0)
    #
    # cv2.destroyAllWindows()

if __name__=="__main__":
    width  = 16
    height = 16
    patternCnt = int(math.log2(16))+1
    print(np.fromfunction(lambda x,y: (y/(width/pow(2,x)))%2, (patternCnt,width), dtype=int).astype(np.uint8))#(y/(width%(x+1)))%2
    # print(list(imgs.transpose(2, 0, 1)))

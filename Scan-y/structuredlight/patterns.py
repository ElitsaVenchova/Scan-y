import numpy as np
import math

"""
    Генериране на шаблони за сканиране
"""
class Patterns:

    WHITE = 0
    BINARY = 1
    STRIPE = 2
    GRAY_CODE = 3
    PHASE_SHIFTING = 4

    # Генериране на шаблон според подадения код
    def genetare(self, patternCode, dsize):
        if patternCode == self.WHITE:
            return self.white(dsize)
        elif patternCode == self.BINARY:
            return self.binary(dsize)
        elif patternCode == self.STRIPE:
            return self.stripe(dsize)
        elif patternCode == self.GRAY_CODE:
            return self.gray(dsize)
        elif patternCode == self.PHASE_SHIFTING:
            return self.phaseShifting(dsize)
        else:
            raise ValueError('Bad pattern code!')

    """
        Единичен шаблон бял - за пълно осветяване на сцената
    """
    def white(self, dsize):
        width, height = dsize
        patternCnt = 1
        imgMatr = (255/2)*np.fromfunction(lambda x,y: 1, (patternCnt,width), dtype=float)
        return self.addHeight(imgMatr, height)

    """
        Двойчен шаблон
    """
    def binary(self, dsize):
        width, height = dsize
        patternCnt = int(math.log2(width))+1
        #<<Горното>> = width/pow(2,x) - през колко трябва да се сменят 0/1;
        #(y/(width/pow(2,x)))%2 - ако y/<<Горното>> е четно, то 0, иначе 1
        imgMatr = 255*np.fromfunction(lambda x,y: (y/(width/pow(2,x)))%2, (patternCnt,width), dtype=int).astype(np.uint8)#uint8 e [0,255]
        whitePattern = self.white(dsize)
        pattern = self.addHeight(imgMatr, height)
        return np.append(whitePattern, pattern)

    """
        Gray code шаблон
    """
    def gray(self, dsize):
        width, height = dsize
        patternCnt = int(math.log2(width))+1
        #<<Горното>> = width/pow(2,x) - през колко трябва да се сменят 0/1;
        #<<Горното2>>=(<<Горното>>+1)/2-в този шаблон редът е- чббччббчч(binary-чбчбчб). Връща по двойки index 0=0, ind 1 и 2=2(от (1+1)/2=2 и (2+1)/2=2),ind 3 и 4=3
        #<<Горното2>>%2 - за четни двойки 0, иначе 1
        imgMatr = 255*np.fromfunction(lambda x,y: (((y/(width/pow(2,x)))+1)/2)%2, (patternCnt,width), dtype=int).astype(np.uint8)#uint8 e [0,255]
        #добавяне един бял шаблон в началото за пълно осветяване на сцената
        whitePattern = self.white(dsize)
        pattern = self.addHeight(imgMatr, height)
        return np.append(whitePattern, pattern)

    """
        Stripe шаблон
    """
    def stripe(self, dsize):
        width, height = dsize
        # #шаблони е ширината(за всяка линия по един)
        #само по една линия на всеки шаблон отдясно на ляво
        imgMatr = 255*np.fromfunction(lambda x,y: x==y, (width,width), dtype=int).astype(np.uint8)#uint8 e [0,255]
        #добавяне един бял шаблон в началото за пълно осветяване на сцената
        whitePattern = self.white(dsize)
        pattern = self.addHeight(imgMatr, height)
        return np.append(whitePattern, pattern)

    """
        Отместване на фазата
    """
    def phaseShifting(self, dsize):
        width, height = dsize
        patternCnt = 3
        shiftStep = 2*np.pi/patternCnt #2pi/3 (това са точно 3 стъпки и четвъртата е 2pi или 0)
        freq = np.pi/16 #честота. Колкото е по-голямо, толква по ситно, иначе по-дълги вълни
        # <<step>>=x*2*np.pi/3 = всеки следващ шаблон е с отместване 2pi/3.
        # <<func>> = 1+np.cos(freq*y+<<step>>) - 1+, защото cos връща от -1 до 1 и така се неутрализира.
        # 127*,защото <<func>> връща стойности от 0 до 2, а на нас ни трябват от 0 до 255
        imgMatr = (255/2)*np.fromfunction(lambda x,y: 1+np.cos(freq*y+(x*shiftStep)), (patternCnt,width), dtype=float)
        #добавяне един бял шаблон в началото за пълно осветяване на сцената
        whitePattern = self.white(dsize)
        pattern = self.addHeight(imgMatr, height)
        return np.append(whitePattern, pattern)

    # всеки ред imgMatr съдържа шаблон, който трябва да се размножи по редовете до height
    def addHeight(self, imgMatr, height):
        x, y = imgMatr.shape
        imgMatr = np.tile(imgMatr, height) # Повтаря се всеки ред от матрицата #пъти за височината(пр. h=2 и [[1,2],[3,4]]-> [[1,2,1,2],[3,4,3,4]])
        imgMatr = np.reshape(imgMatr,(x,height,y)) # всеки ред се d1 се превръща в dHeight(пр от горе -> [[[1,2],[1,2]],[[3,4],[3,4]]])
        return np.array(imgMatr,dtype=np.uint8) # връща се array, за да може да се прожектира от cv2.imshow

    # обръщане на черно-бял шаблон. Черното става бяла и обратното
    def invert(self, imlist):
        # imlist съдържа стойности 0 и 255 => всяко 0 става 255-0=255 и всяко 255 става 255-255=0
        return [255-img for img in imlist]

    # Транспониране на шаблоните(от вертикални в хоризонстални раета), за да се засеме по y остта
    def transpose(self, imlist):
        # img.T транспонира матрицата
        return [ img.T for img in imlist]

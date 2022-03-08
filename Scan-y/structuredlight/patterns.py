import numpy as np
import math
import cv2 as cv

"""
    Генериране на шаблони за сканиране
"""
class Patterns:

    WHITE = 0
    BLACK = 1
    OPENCV_GRAY_CODE = 2
    MANUAL_GRAY_CODE = 3
    PHASE_SHIFTING = 4
    GRAY_CODE_AND_PHASE_SHIFTING = 5
    BINARY = 6
    STRIPE = 7
    CHESS_BOARD = 8

    WHITE_PATTERN = "White"
    BLACK_PATTERN = "Black"
    GRAY_CODE_PATTERN = "Gray"
    PHASE_PATTERN = "Phase"
    IMAGE_PATTERN = "Img"
    INV_PATTERN = "Inv"
    TRANS_PATTERN = "Trans"
    TRANS_INV_PATTERN = "TransInv"
    CHESS_BOARD_PATTERN = "Chessboard"

    # Генериране на шаблон според подадения код
    def genetare(self, patternCode, pSize, chessboardSize=None):
        if patternCode == self.WHITE:
            return self.white(pSize)
        elif patternCode == self.BLACK:
            return self.black(pSize)
        elif patternCode == self.OPENCV_GRAY_CODE:
            patterns = self.white(pSize)
            patterns.update(self.black(pSize))
            patterns.update(self.opencvGray(pSize))
            return patterns
        elif patternCode == self.OPENCV_GRAY_CODE:
            return self.manualGray(pSize)
        elif patternCode == self.PHASE_SHIFTING:
            return self.phaseShifting(pSize)
        elif patternCode == self.GRAY_CODE_AND_PHASE_SHIFTING:
            patterns = self.white(pSize)
            patterns.update(self.black(pSize))
            patterns.update(self.manualGray(pSize))
            patterns.update(self.phaseShifting(pSize))
            return patterns
        elif patternCode == self.BINARY:
            return self.binary(pSize)
        elif patternCode == self.STRIPE:
            return self.stripe(pSize)
        elif patternCode == self.CHESS_BOARD:
            return self.chessboard(pSize,chessboardSize)
        else:
            raise ValueError('Bad pattern code!')

    """
        Единичен шаблон бял - за пълно осветяване на сцената
    """
    def white(self, pSize):
        height, width = pSize
        imgMatr = 255*np.zeros((1,height, width), np.uint8)
        return {self.WHITE_PATTERN:imgMatr}

    """
        Единичен шаблон черен
    """
    def black(self, pSize):
        height, width = pSize
        imgMatr = 255*np.ones((1,height, width), np.uint8)
        return {self.BLACK_PATTERN:imgMatr}

    """
        Gray code шаблон генериран от opencv библиотеката
    """
    def opencvGray(self, pSize):
        height, width = pSize
        graycode = cv.structured_light_GrayCodePattern.create(width,height)
        imgMatr = graycode.generate()[1]

        return {self.GRAY_CODE_PATTERN:imgMatr}

    """
        Собствена имплементация на Gray code шаблон
    """
    def manualGray(self, pSize):
        height, width = pSize
        # patternCnt = int(math.log2(width))+1
        # #<<Горното>> = width/pow(2,x) - през колко трябва да се сменят 0/1;
        # #<<Горното2>> = (<<Горното>>+1)/2-в този шаблон редът е- чббччббчч(binary-чбчбчб). Връща по двойки index 0=0, ind 1 и 2=2(от (1+1)/2=2 и (2+1)/2=2),ind 3 и 4=3
        # #<<Горното2>>%2 - за четни двойки 0, иначе 1
        imgMatr = 255*np.fromfunction(lambda x,y: (((y/(width/pow(2,x)))+1)/2)%2, (patternCnt,width), dtype=int).astype(np.uint8)#uint8 e [0,255]
        imgMatrTrans = 255*np.fromfunction(lambda x,y: (((y/(height/pow(2,x)))+1)/2)%2, (patternCnt,height), dtype=int).astype(np.uint8)#uint8 e [0,255]

        return self.multiply(imgMatr,imgMatrTrans,pSize)

    """
        Отместване на фазата
    """
    def phaseShifting(self, pSize):
        height, width = pSize
        patternCnt = 3
        shiftStep = 2*np.pi/patternCnt #2pi/3 (това са точно 3 стъпки и четвъртата е 2pi или 0)
        freq = np.pi/16 #честота. Колкото е по-голямо, толква по ситно, иначе по-дълги вълни
        # <<step>>=x*2*np.pi/3 = всеки следващ шаблон е с отместване 2pi/3.
        # <<func>> = 1+np.cos(freq*y+<<step>>) - 1+, защото cos връща от -1 до 1 и така се неутрализира.
        # 127*,защото <<func>> връща стойности от 0 до 2, а на нас ни трябват от 0 до 255
        imgMatr = (255/2)*np.fromfunction(lambda x,y: 1+np.cos(freq*y+(x*shiftStep)), (patternCnt,width), dtype=float)

        imgMatrTrans = (255/2)*np.fromfunction(lambda x,y: 1+np.cos(freq*y+(x*shiftStep)), (patternCnt,height), dtype=float)
        imgMatrTrans = self.addHeight(imgMatrTrans, pSize[1])

        return {self.PHASE_PATTERN:                    self.addHeight(imgMatr, height),
                self.PHASE_PATTERN+self.TRANS_PATTERN: self.transpose(imgMatrTrans)}

    """
        Двойчен шаблон
    """
    def binary(self, pSize):
        height, width = pSize
        patternCnt = int(math.log2(width))+1
        #<<Горното>> = width/pow(2,x) - през колко трябва да се сменят 0/1;
        #(y/(width/pow(2,x)))%2 - ако y/<<Горното>> е четно, то 0, иначе 1
        imgMatr = 255*np.fromfunction(lambda x,y: (y/(width/pow(2,x)))%2, (patternCnt,width), dtype=int).astype(np.uint8)#uint8 e [0,255]
        imgMatrTrans = 255*np.fromfunction(lambda x,y: (y/(height/pow(2,x)))%2, (patternCnt,height), dtype=int).astype(np.uint8)#uint8 e [0,255]

        return self.multiply(imgMatr,imgMatrTrans,pSize)

    """
        Stripe шаблон
    """
    def stripe(self, pSize):
        height, width = pSize
        # #шаблони е ширината(за всяка линия по един)
        #само по една линия на всеки шаблон отдясно на ляво
        imgMatr = 255*np.fromfunction(lambda x,y: x==y, (width,width), dtype=int).astype(np.uint8)#uint8 e [0,255]
        imgMatrTrans = 255*np.fromfunction(lambda x,y: x==y, (width,height), dtype=int).astype(np.uint8)#uint8 e [0,255]

        return self.multiply(imgMatr,imgMatrTrans,pSize)

    """
        Шахматна дъска за калибриране на проектора
        pSize - размери на проекториа
        chessboardSize - бр. пресичания на черен и черен квадрат по диагонал
    """
    def chessboard(self, pSize, chessboardSize):
        height, width = pSize
        rowsCnt,colsCnt = (chessboardSize[0]+1,chessboardSize[1]+1)#бр. квадрати по ширина и дължина, за да удовлетовори chessboardSize
        squareSize = int(min(width/rowsCnt, height/colsCnt))-10
        rightPad,bottomPad = (width-rowsCnt*squareSize,height-colsCnt*squareSize) # оставащо празно пространсвто в дясно и долу

        # (*) (x/squareSize)%2 - Редуват се черно/бяло през squareSize по x
        # (#) (y/squareSize)%2 - Редуват се черно/бяло през squareSize по x
        # (*)!=(#) - xor на двете стойности за получаване на квадрати
        imgMatr = 255*np.fromfunction(lambda y,x: ((x/squareSize).astype(int)%2!=(y/squareSize).astype(int)%2), (height,width), dtype=float).astype(np.uint8)
        # Центриране на дъската за естетичност
        imgMatr[-bottomPad:,:] = 255 # излишните квадрати отдолу стават бели
        imgMatr[:,-rightPad:] = 255 # излишните квадрати отдясно стават бели
        imgMatr = np.roll(imgMatr, int(bottomPad/2), axis=0) #прехвърлят се пикселите отдолу->горе, за да се получи еднаква дупка от двете страни
        imgMatr = np.roll(imgMatr, int(rightPad/2), axis=1) #прехвърлят се пикселите от дясно->в ляво, за да се получи еднаква дупка от двете страни
        imgMatr = np.reshape(imgMatr,(1,height,width))

        return {self.CHESS_BOARD_PATTERN: imgMatr}

    # Размножава шаблините - шаблони, транспонирани, обърнати, транспонирани и обърнати
    # pSize = (width, height)
    def multiply(self, pattern, patternTrans, pSize):
        pattern = self.addHeight(pattern, pSize[0])
        patternInv = self.invert(pattern)

        patternTrans = self.addHeight(patternTrans, pSize[1])
        patternTrans = self.transpose(patternTrans)
        patternTransInv = self.invert(patternTrans)

        patterns = {
            self.IMAGE_PATTERN: pattern,
            self.INV_PATTERN: patternInv,
            self.TRANS_PATTERN: patternTrans,
            self.TRANS_INV_PATTERN: patternTransInv}
        return patterns

    # всеки ред imgMatr съдържа шаблон, който трябва да се размножи по редовете до height
    def addHeight(self, imgMatr, height):
        x, y = imgMatr.shape
        imgMatr = np.tile(imgMatr, height) # Повтаря се всеки ред от матрицата #пъти за височината(пр. h=2,бр.шабломи=2 и [[1,2],[3,4]]-> [[1,2,1,2],[3,4,3,4]])
        imgMatr = np.reshape(imgMatr,(x,height,y)) # всеки ред се d1 се превръща в dHeight(пр. от горе -> [[[1,2],[1,2]],[[3,4],[3,4]]])
        return np.array(imgMatr,dtype=np.uint8) # връща се array, за да може да се прожектира от cv.imshow

    # обръщане на черно-бял шаблон. Черното става бяла и обратното
    def invert(self, imlist):
        # imlist съдържа стойности 0 и 255 => всяко 0 става 255-0=255 и всяко 255 става 255-255=0
        return [255-img for img in imlist]

    # Транспониране на шаблоните(от вертикални в хоризонстални раета), за да се засеме по y остта
    def transpose(self, pattern):
        # img.T транспонира матрицата
        return [ img.T for img in pattern]

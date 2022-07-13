import cv2 as cv
import numpy as np
from .patterns import Patterns
from natsort import natsorted
import glob
import math
import open3d as o3d

from .processPointClouds import ProcessPointClouds

"""
    Рекоструиране на обекта от изображения в Point clound и Mesh
"""
class Reconstruct3D:

    def __init__(self,cameraPi):
        self.cameraPi = cameraPi
        self.pSize = self.cameraPi.stereoCalibrationRes['pShape'] # Размер прожекцията
        self.cSize = self.cameraPi.stereoCalibrationRes['cShape'] # Размер камера

        self.processPointClouds = ProcessPointClouds(self.pSize) # облак от точки

        self.mask = np.zeros(self.cSize, np.uint8)# маска кои пиксели стават за обработване.След инициализацията има стойност False

        self.colorImg = np.zeros((self.cSize[0],self.cSize[1],3), np.int)
        self.grayCodeMap = np.zeros((self.cSize[0],self.cSize[1], 3), np.float32)#връзката на координатите на пикселите на снимката и шаблона GrayCode

    """
        Основната функция за реконструкция.
        Обхождат се сканиранията на различните гледни точки, прави се Point Cloud за тях,
        обединяват се всички point clounds в общ 360 градусово изображение
        Прави се point mesh
    """
    def reconstruct(self, dir, patternCode, stepsCnt, stepSize, theshold):

        for scan_no in range(0, stepsCnt, stepSize):#[70]:#
            self.__init__(self.cameraPi)#Класът се инициализира отново, за да се изчистят вече запазените данни от предходното сканиране
            self.manualMapGrayCode(dir, patternCode, scan_no, theshold) # Ръчно мапиране на шаблоните. Работи за GrayCode и Binary(не е тествано)
            # Тези долу са само резервни варианти, които не работят или не са оптимални, но могат да се използват в бъдеще
            # # # self.autoMapGrayCode(dir, scan_no) # Автоматично мапиране от OpenCV на GrayCode шаблони, но трябва да е заснето с шаблоните на OpenCV
            # # #
            # # # self.filterGrayCode() # smoothing filter. Премахване на артефакти и изглаждане граници заден-преден фон, допълнително заглажда рязките разлики в дълбочината. НЕ РАБОТИ МНОГО ДОБРЕ И Е СПРЯНО!
            # # # pointCloud = self.processPointClouds.genPointCloud(self.cameraPi.stereoCalibrationRes["disparityToDepthMatrix"]) # преобразуване на disparity в дълбочина.

            # Запазване на облака от точки като xyzrbg файл
            # self.processPointClouds.savePointCloud(dir, self.grayCodeMap, self.mask, self.colorImg, scan_no)
            # Зареждане на облака от точки от xyzrbg файл
        self.processPointClouds.process(dir, stepsCnt, stepSize)

    # Автоматично мапиране от OpenCV на GrayCode шаблони, но трябва да е заснето с шаблоните на OpenCV
    def autoMapGrayCode(self, dir, scan_no):
        BLACK_THRESHOLD = 20#праг на черното
        WHITE_THRESHOLD = 4#праг на бялото
        self.colorImg = self.cameraPi.loadPatternImages(dir, Patterns.WHITE_PATTERN, cv.IMREAD_COLOR, scan_no)[0]

        white = self.cameraPi.loadPatternImages(dir, Patterns.WHITE_PATTERN, scan_no)[0] # Напълно осветено изображение
        black = self.cameraPi.loadPatternImages(dir, Patterns.BLACK_PATTERN, scan_no)[0] # Тъмно изображение
        grayCodeImgs = self.cameraPi.loadPatternImages(dir, Patterns.OPENCV_GRAY_CODE, scan_no) # GrayCode изображенията

        graycode = cv.structured_light_GrayCodePattern.create(self.pSize[0], self.pSize[1]) # Шаблоните
        graycode.setBlackThreshold(BLACK_THRESHOLD) # задаване на праг на бялото
        graycode.setWhiteThreshold(WHITE_THRESHOLD) # задаване на праг на черното
        yerr, nerr = 0,0 # за DEBUG - за колко пиксела има успешно съвпадение
        for y in range(self.cSize[0]):
            for x in range(self.cSize[1]):
                if (y>=white.shape[0] or x>=white.shape[1] or
                    y>=black.shape[0] or x>=black.shape[1] or
                    int(white[y, x]) - int(black[y, x]) <= BLACK_THRESHOLD): #т.е. пиксела е черен и не може да се обработи(255-0 трябва да дава бяло, а не черно)
                    continue
                err, proj_pix = graycode.getProjPixel(grayCodeImgs, x, y)#за (x,y) от снимките, връща (x,y) от шаблоните
                if err:
                    yerr+=1
                if not err:
                    nerr+=1
                    dist = math.sqrt(pow(y-proj_pix[0],2) + pow(x-proj_pix[1],2))#разстоянието между ъточките в едната и др. коорд.
                    self.grayCodeMap[y, x, :] = np.array([proj_pix[0],proj_pix[1],dist])#записва съответствята на координатите между снимките и шаблоните
                    #маркира се като бяло, т.е. има съвпадение
                    self.mask[y, x] = 255
        print(yerr,nerr)#1 256 929-63 764

    # Ръчно мапиране на шаблоните. Работи за GrayCode и Binary(не е тествано)
    # Горното не работи много вярно
    def manualMapGrayCode(self, dir, patternCode, scan_no, theshold):
        colorImgAll = self.cameraPi.loadPatternImages(dir, Patterns.INV_PATTERN, scan_no, cv.IMREAD_COLOR, 0)[0] #Първият шаблон от Inverse е изцяло бял

        print('Pattern code: ', patternCode, '/ scan_no: ', scan_no)
        patternImgs = Patterns().genetare(patternCode, self.cSize,Patterns().binaryCodePattCnt(self.pSize)) # Шаблоните
        # Съдържа изображенията в 4-те различни варианта
        grayCodeImgs = {
            Patterns.IMAGE_PATTERN: self.cameraPi.loadPatternImages(dir, Patterns.IMAGE_PATTERN, scan_no),
            Patterns.INV_PATTERN: self.cameraPi.loadPatternImages(dir, Patterns.INV_PATTERN, scan_no),
            Patterns.TRANS_PATTERN: self.cameraPi.loadPatternImages(dir, Patterns.TRANS_PATTERN, scan_no),
            Patterns.TRANS_INV_PATTERN: self.cameraPi.loadPatternImages(dir, Patterns.TRANS_INV_PATTERN, scan_no)}

        yerr = 0 # За debug - колко намерени пиксела има
        white = grayCodeImgs[Patterns.INV_PATTERN][0] # Напълно осветено изображение #Първият шаблон от Inverse е изцяло бял
        black = grayCodeImgs[Patterns.IMAGE_PATTERN][0] # Тъмно изображение #Първият шаблон от Img е изцяло черен

        # Съдържа измерените стойности за пикселите на изображенията в 4-те различни варианта
        tempGrayCodeMap = {
            Patterns.IMAGE_PATTERN: np.zeros(self.cSize, np.uint16),
            Patterns.INV_PATTERN: np.zeros(self.cSize, np.uint16),
            Patterns.TRANS_PATTERN: np.zeros(self.cSize, np.uint16),
            Patterns.TRANS_INV_PATTERN: np.zeros(self.cSize, np.uint16)}

        # Съдържа измерените стойности за пикселите на шаблоните в 4-те различни варианта
        tempPattGrayCodeMap = {
            Patterns.IMAGE_PATTERN: np.zeros(self.cSize, np.uint16),
            Patterns.INV_PATTERN: np.zeros(self.cSize, np.uint16),
            Patterns.TRANS_PATTERN: np.zeros(self.cSize, np.uint16),
            Patterns.TRANS_INV_PATTERN: np.zeros(self.cSize, np.uint16)}

        for pattType in patternImgs: #итериране по видовете шаблони
            print(pattType)
            if len(grayCodeImgs[pattType]) != len(patternImgs[pattType]):
                raise ValueError('Number of patterns and images is not equal for pattern {0}({1},{2})!'.format(pattType,len(patternImgs[pattType]),len(grayCodeImgs[pattType])))
            # Създава степените, на които трябва да се повдигне съответното изображение. Първото е на степен 0, второто е на степен 1 и т.н.
            # np.mgrid[:4,:3,:2] = [[[0,0,0],[0,0,0]],[[1,1,1],[1,1,1]],[[2,2,2],[2,2,2]],[[3,3,3],[3,3,3]]]
            ind = np.mgrid[:len(grayCodeImgs[pattType]),:self.cSize[0],:self.cSize[1]][0]
            # ((white+black)/2) + theshold - средно аритметично на интензитета на пиксела, когато е осветен и когато не е + побавена стойност.
            # чрез проба грешка, беше установено, че theshold=20 връща най-добри резултати.
            msk = grayCodeImgs[pattType]>((white+black)/2)+theshold #white-30# макса дали съответния индекс трябва да се използва за повдигане с основа 2
            # msk = np.array([self.morphologyEx(255*m) for m in msk.astype(np.uint16)]).astype(np.bool_)# Засега е по-лошо. създава плавна граница на обекта и изчиства страничния бял шум от задния план

            # Подвига се 2 на степен индексите и при умножаването по msk се взимат само стойностите с true. Резултатът се сумира по ос 0,
            # т.е. сумира се съответния пиксел за всички изображения в шаблона
            tempGrayCodeMap[pattType] = np.sum(np.power(2, ind)*msk, axis=0)

            # Аналогично за шаблоните като тук сравнението е директно с 255 цвят на пиксела
            indPatt = np.mgrid[:len(patternImgs[pattType]),:self.cSize[0],:self.cSize[1]][0]
            mskPatt = patternImgs[pattType] == 255
            tempPattGrayCodeMap[pattType] = np.sum(np.power(2, indPatt)*mskPatt, axis=0)

        # DEBUG
        # np.savetxt('Img{0}.txt'.format(scan_no), x, fmt='%i', delimiter='\t')
        # cv.imwrite('Img{0}{1}.jpg'.format(scan_no,'txt'), msk)

        # Генериране на индекси на изображенията [[0,0],[0,1],[0,1],[1,0],[1,1],...]
        indImg = np.mgrid[:tempGrayCodeMap[Patterns.IMAGE_PATTERN].shape[0],:tempGrayCodeMap[Patterns.IMAGE_PATTERN].shape[1]]
        # Stack-ване на масивите на изображенията в едно общо със 6 стойности на ред - x,y, IMAGE_PATTERN value, INV_PATTERN value, ...
        stackImgs = np.dstack((indImg[0],indImg[1],
                        tempGrayCodeMap[Patterns.IMAGE_PATTERN],tempGrayCodeMap[Patterns.INV_PATTERN],
                        tempGrayCodeMap[Patterns.TRANS_PATTERN],tempGrayCodeMap[Patterns.TRANS_INV_PATTERN]))
        # Филтриране - Сборът на стойността в пискела(Image) и обратния(Inverse) шаблона трябва да е = 2^(броя шаблони)-1
        # Аналогичното важи и за транспонираните шаблони. Използва се оператор умножаване <*>, защото дава грешка при използването на <and>
        stackImgs = stackImgs[((stackImgs[:,:,2] + stackImgs[:,:,3] == pow(2,len(grayCodeImgs[Patterns.IMAGE_PATTERN]))-1) *
            (stackImgs[:,:,4] + stackImgs[:,:,5]  == pow(2,len(grayCodeImgs[Patterns.TRANS_PATTERN]))-1)),:]#Някак от 3D става 2D масив, но точно това ми трябва

        # cv.imwrite('Img{0}{1}.jpg'.format(scan_no,Patterns.IMAGE_PATTERN), tempGrayCodeMap[Patterns.IMAGE_PATTERN])
        # cv.imwrite('Img{0}{1}.jpg'.format(scan_no,Patterns.INV_PATTERN), tempGrayCodeMap[Patterns.INV_PATTERN])
        # cv.imwrite('Img{0}{1}.jpg'.format(scan_no,Patterns.TRANS_PATTERN), tempGrayCodeMap[Patterns.TRANS_PATTERN])
        # cv.imwrite('Img{0}{1}.jpg'.format(scan_no,Patterns.TRANS_INV_PATTERN), tempGrayCodeMap[Patterns.TRANS_INV_PATTERN])

        # Генериране на индекси на шаблоните [[0,0],[0,1],[0,1],[1,0],[1,1],...]
        indPatt = np.mgrid[:tempPattGrayCodeMap[Patterns.IMAGE_PATTERN].shape[0],:tempPattGrayCodeMap[Patterns.IMAGE_PATTERN].shape[1]]
        # Stack-ване на масивите на шаблоните в едно общо със 6 стойности на ред - x,y, IMAGE_PATTERN value, INV_PATTERN value, ...
        stackPatts = np.dstack((indPatt[0],indPatt[1],
                tempPattGrayCodeMap[Patterns.IMAGE_PATTERN],tempPattGrayCodeMap[Patterns.INV_PATTERN],
                tempPattGrayCodeMap[Patterns.TRANS_PATTERN],tempPattGrayCodeMap[Patterns.TRANS_INV_PATTERN]))
        stackPatts = stackPatts.reshape(stackPatts.shape[0]*stackPatts.shape[1],stackPatts.shape[2]) # Преформатиране на масива в редове, т.е. (cSize[0]*cSize[1],6)

        #Неуспешен опит да се направи декартово произведение на изображенията и шаблоните. Ако се намери начин, то долното въртене на цикли ще стане бързо и на един ред
        # # mapImgsPatts = np.array([[x0, y0] for x0 in reshapeStackImgs for y0 in reshapeStackPatts])
        # # mapImgsPatts = np.array(list(itertools.product(*[stackImgs,stackPatts])))

        for patPix in stackPatts:
            for pix in stackImgs[(patPix[[2,4]]==stackImgs[:,[2,4]]).all(axis=1)]:#филтрират се само тези с еднакъв код от вертикален и хоризонтален шаблон
                if (pix[2] == patPix[2] and pix[4] == patPix[4]):

                    #разстоянието между точките в изборажението и шаблона
                    #добавя се +1 на всички координати, за да може при умножението по коефициентите за разликата на изображенияна,
                    #да се получи вярна стойност. Иначе за y=1 => 1*1.2 = 1.2, а това реално е втория пиксел и трябва да бъде 2*1.2 = 2.4
                    dist = math.sqrt(pow((pix[0]+1)-(patPix[0]+1),2)
                            + pow((pix[1]+1)-(patPix[1]+1),2))
                    # За пискела не е намирано съвпадение или намереното разстояние е минимално
                    if self.mask[int(patPix[0]), int(patPix[1])] == 0 or dist < self.grayCodeMap[int(patPix[0]), int(patPix[1]), 2]:
                        #записва съответствята на координатите между снимките и шаблоните
                        self.grayCodeMap[int(patPix[0]), int(patPix[1]), :] = np.array([patPix[0], patPix[1],dist])
                        #маркира се като бяло, т.е. има съвпадение
                        self.mask[int(patPix[0]), int(patPix[1])] = 255
                        self.colorImg[int(patPix[0])][int(patPix[1])] = colorImgAll[pix[0]][pix[1]]#Записване на цвета на съвпадащия пиксел
                        yerr += 1

        print(yerr)#1 256 929/117 821
        cv.imwrite('Img{0}{1}.jpg'.format(scan_no,'Mask'), self.mask)

    # smoothing filter
    # Премахване на артефакти и изглаждане граници заден-преден фон, допълнително заглажда рязките разлики в дълбочината. НЕ РАБОТИ МНОГО ДОБРЕ И Е СПРЯНО!
    def filterGrayCode(self):
        FILTER = 1# Размер на филтъта. 1-3x3 прозорец
        #Трансгормация Dilation(създава плавна граница на обекта) последвана от Erosion(изчиства страничния бял шум от задния план).
        #Използва се за премахване на малки черни дупки в предния план.
        # създава плавна граница на обекта и изчиства страничния бял шум от задния план
        ext_mask = cv.morphologyEx(self.mask, cv.MORPH_CLOSE,
                                    np.ones((FILTER*2+1, FILTER*2+1)))

        for y in range(self.cSize[0]):
            for x in range(self.cSize[1]):
                if self.mask[y, x] == 0 and ext_mask[y, x] != 0:
                    sum_x = 0
                    sum_y = 0
                    cnt = 0
                    for dy in range(-FILTER, FILTER+1):#[-1,0,1]
                        for dx in range(-FILTER, FILTER+1):#[-1,0,1]
                            #Взимат се съседни пиксели
                            ty = y + dy
                            tx = x + dx
                            if ((dy != 0 or dx != 0)# поне едното не е нулева координата
                                    and ty >= 0 and ty < self.pSize[0] #ty не е извън координатите на камерата
                                    and tx >= 0 and tx < self.pSize[1] #tx не е извън координатите на камерата
                                    and self.mask[ty, tx] != 0):#mask[ty, tx] не е черен пиксел, т.е. обработваме го
                                sum_y += self.grayCodeMap[ty, tx, 0]#Сумират се координатите на съседните изображения
                                sum_x += self.grayCodeMap[ty, tx, 1]#Сумират се координатите на съседните изображения
                                cnt += 1
                    if cnt != 0:
                        #така новата GC карта орговяря на инфомрацията в новата маска ext_mask
                        self.grayCodeMap[y, x, 0] = np.round(sum_y/cnt)# x е средно аритметично на x от съседните пиксели
                        self.grayCodeMap[y, x, 1] = np.round(sum_x/cnt)# y е средно аритметично на x от съседните пиксели
                        self.grayCodeMap[y, x, 2] = math.sqrt(pow(y-self.grayCodeMap[y, x, 0],2) + pow(x-self.grayCodeMap[y, x, 1],2))
                        print(self.grayCodeMap[y, x, 2])

            self.mask = ext_mask

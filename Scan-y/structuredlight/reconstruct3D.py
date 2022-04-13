import cv2 as cv
import numpy as np
from .patterns import Patterns
from natsort import natsorted
import glob
import math

from matplotlib import pyplot as plt
# import open3d as o3d

"""
    Рекоструиране на обекта от изображения в Point clound и Mesh
"""
class Reconstruct3D:

    FILTER = 1# Размер на филтъта. 1-3x3 прозорец
    BLACK_THRESHOLD = 20#праг на черното
    WHITE_THRESHOLD = 4#праг на бялото

    BLACK_N_WHITE = 0
    COLOR = 1

    FILE_NAME = 'pointCloud.ply'

    def __init__(self,cameraPi):
        self.cameraPi = cameraPi

        self.pSize = self.cameraPi.stereoCalibrationRes['pShape'] # Размер прожекцията
        self.cSize = self.cameraPi.stereoCalibrationRes['cShape'] # Размер камера

        self.mask = np.zeros(self.cSize, np.uint8)# маска кои пиксели стават за обработване.След инициализацията има стойност False

        self.colorImg = np.zeros((self.cSize[0],self.cSize[1],3), np.float32)
        self.grayCodeMap = np.zeros((self.cSize[0],self.cSize[1], 3), np.float32)#връзката на координатите на пикселите на снимката и шаблона GrayCode
        self.pointCloud = np.zeros(self.cSize, np.float32)

    def reconstruct(self, dir, patternCode):
        # TODO: Да се направи ръчно мапиране, защото това не работи много добре.
        # Идеята е да се взима средно аритметично на пиксела между white и black, което да е граница.
        # След това стойностите ще се определят с мнозинство между Img, ImgInv, ImgTrans и ImgTransInv
        self.manualMapGrayCode(dir, patternCode)
        # self.autoMapGrayCode(dir)

        self.filterGrayCode()
        # @TODO: Тук има неуспешни опити да се направи реконструкция.
        self.prespectiveTransform(self.cameraPi.stereoCalibrationRes["disparityToDepthMatrix"])

        self.savePointCloud(dir)

        #pcd = o3d.io.read_point_cloud(dir + '/'+self.FILE_NAME)
        #print(pcd)
#         #http://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html
#             mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
#             o3d.io.write_triangle_mesh("copy_of_knot.ply", mesh)
#         Multi view matching - http://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html
#         Color map optimisation - http://www.open3d.org/docs/release/tutorial/pipelines/color_map_optimization.html

    #Прочита изображенията за определен шаблон
    def readImages(self, dir, patternCode, readType,img_no='?'):
        # Зареждат се всички изображения, които отговорят на шаблона
        # img_no - ?(точно един символ),*(0 или повече символи),конретно чисто(зарежда изображението с конкретен номер)
        imgsNames = natsorted(glob.glob('./{0}/image10{1}{2}.jpg'.format(dir,patternCode,img_no)))
        imgs = [] # np.empty([2, 2]
        for fname in imgsNames:
            img = None
            if readType == self.BLACK_N_WHITE:#От цветно изображението се прехвърля в черно бяло
                img = self.cameraPi.loadImage(fname,cv.IMREAD_GRAYSCALE)
            else:# иначе цветно
                img = cv.imread(fname)
            imgs.append(img)
        return np.array(imgs)

    def autoMapGrayCode(self, dir):
        self.colorImg = self.readImages(dir, Patterns.WHITE_PATTERN, self.COLOR)[0]

        white = self.readImages(dir, Patterns.WHITE_PATTERN, self.BLACK_N_WHITE)[0]
        black = self.readImages(dir, Patterns.BLACK_PATTERN, self.BLACK_N_WHITE)[0]
        grayCodeImgs = self.readImages(dir, Patterns.OPENCV_GRAY_CODE, self.BLACK_N_WHITE)

        graycode = cv.structured_light_GrayCodePattern.create(self.pSize[0], self.pSize[1])
        graycode.setBlackThreshold(self.BLACK_THRESHOLD)
        graycode.setWhiteThreshold(self.WHITE_THRESHOLD)
        yerr, nerr = 0,0
        for y in range(self.cSize[0]):
            for x in range(self.cSize[1]):
                if (y>=white.shape[0] or x>=white.shape[1] or
                    y>=black.shape[0] or x>=black.shape[1] or
                    int(white[y, x]) - int(black[y, x]) <= self.BLACK_THRESHOLD): #т.е. пиксела е черен и не може да се обработи(255-0 трябва да дава бяло, а не черно)
                    continue
                err, proj_pix = graycode.getProjPixel(grayCodeImgs, x, y)#за (x,y) от снимките, връща (x,y) от шаблоните
                if err:
                    yerr+=1
                if not err:
                    nerr+=1
                    dist = math.sqrt(pow(y-proj_pix[0],2) + pow(x-proj_pix[1],2))#разстоянието между ъточките в едната и др. коорд.
                    self.grayCodeMap[y, x, :] = np.array([proj_pix[0],proj_pix[1],dist])#записва съответствята на координатите между снимките и шаблоните
                    #мареика се като бяло, т.е. има съвпадение
                    self.mask[y, x] = 255
        print(yerr,nerr)#1 256 929-63 764

    def manualMapGrayCode(self, dir, patternCode):
        self.colorImg = self.readImages(dir, Patterns.INV_PATTERN, self.COLOR, '0')[0] #Първият шаблон от Inverse е изцяло бял

        yerr, nerr = 0,0 # За debug - колко намерени пиксела има и колко пиксела нямат успешна сума 15.
        white = self.readImages(dir, Patterns.INV_PATTERN, self.BLACK_N_WHITE)[0] # Напълно осветено изображение #Първият шаблон от Inverse е изцяло бял
        black = self.readImages(dir, Patterns.IMAGE_PATTERN, self.BLACK_N_WHITE)[0] # Тъмно изображение #Първият шаблон от Img е изцяло черен

        print('Pattern code: ', patternCode)
        patternImgs = Patterns().genetare(patternCode, self.pSize) # Шаблоните

        # Съдържа изображенията в 4-те различни варианта
        grayCodeImgs = {
            Patterns.IMAGE_PATTERN: self.readImages(dir, Patterns.IMAGE_PATTERN, self.BLACK_N_WHITE),
            Patterns.INV_PATTERN: self.readImages(dir, Patterns.INV_PATTERN, self.BLACK_N_WHITE),
            Patterns.TRANS_PATTERN: self.readImages(dir, Patterns.TRANS_PATTERN, self.BLACK_N_WHITE),
            Patterns.TRANS_INV_PATTERN: self.readImages(dir, Patterns.TRANS_INV_PATTERN, self.BLACK_N_WHITE)}

        # Съдържа измерените стойности за пикселите на изображенията в 4-те различни варианта
        tempGrayCodeMap = {
            Patterns.IMAGE_PATTERN: np.zeros(self.cSize, np.uint16),
            Patterns.INV_PATTERN: np.zeros(self.cSize, np.uint16),
            Patterns.TRANS_PATTERN: np.zeros(self.cSize, np.uint16),
            Patterns.TRANS_INV_PATTERN: np.zeros(self.cSize, np.uint16)}

        # Съдържа измерените стойности за пикселите на шаблоните в 4-те различни варианта
        tempPattGrayCodeMap = {
            Patterns.IMAGE_PATTERN: np.zeros(self.pSize, np.uint16),
            Patterns.INV_PATTERN: np.zeros(self.pSize, np.uint16),
            Patterns.TRANS_PATTERN: np.zeros(self.pSize, np.uint16),
            Patterns.TRANS_INV_PATTERN: np.zeros(self.pSize, np.uint16)}

        for pattType in patternImgs: #итериране по видовете шаблони
            print(pattType)
            if len(grayCodeImgs[pattType]) != len(patternImgs[pattType]):
                raise ValueError('Number of patterns and images is not equal for pattern ',pattType,'(',len(grayCodeImgs[pattType]),len(patternImgs[pattType]),')!')
            # Не изглежда елегантно, но създава
            # <#>np.arange(0,4) - генерира степените за повдихане при основа 2. [0,1,2,3]
            # <!>np.expand_dims(np.expand_dims(<#>, axis=1), axis=1) - Два пъти обавя измерение в масива: [[0],[1],[2],....] и втория път в [[[0]],[[1]],[[2]],[[3]]]
            # <!>.repeat(3, axis=2).repeat(2, axis=1) - Два пъти копира съдържаниет ов масива(един в най-вътрешното и след това по-горното измерени)
                # [[[0,0,0]],[[1,1,1]],[[2,2,2]],[[3,3,3]]] и втория път копира единичните масиви [[[0,0,0],[0,0,0]],[[1,1,1],[1,1,1]],[[2,2,2],[2,2,2]],[[3,3,3],[3,3,3]]]
            ind = np.expand_dims(np.expand_dims(np.arange(0,len(grayCodeImgs[pattType])), axis=1), axis=1).repeat(self.cSize[1], axis=2).repeat(self.cSize[0], axis=1)
            # (white[y,x]+black[y,x])/2 - средно аритметично на цвета на пиксела, когато е оцветен и когато не е.
            msk = grayCodeImgs[pattType]>((white+black)/2) + 20 # макса дали съответния индекс трябва да се използва за повдигане с основа 2
            for i in range(len(msk)):
                msk[i] = cv.morphologyEx((255*msk[i]).astype(np.uint8), cv.MORPH_CLOSE,
                                            np.ones((self.FILTER*2+1, self.FILTER*2+1)))

            # Подвига се 2 на степен индексите и при умножаването по msk се взимат само стойностите с true. Резултатът се сумира по ос 0,
            # т.е. сумира се съответния пиксел за всички изображения в шаблона
            tempGrayCodeMap[pattType] = np.sum(np.power(2, ind)*msk, axis=0)
            # Аналогично за шаблоните като тук размерът се взима от pSize и сравнението е директно с 255 цвят на пиксела
            indPatt = np.expand_dims(np.expand_dims(np.arange(0,len(patternImgs[pattType])), axis=1), axis=1).repeat(self.pSize[1], axis=2).repeat(self.pSize[0], axis=1)
            mskPatt = patternImgs[pattType] == 255
            tempPattGrayCodeMap[pattType] = np.sum(np.power(2, indPatt)*mskPatt, axis=0)

            # #Не беше толкова бавна реализация, но горното работи впъти по-добре.
            # for y in range(self.cSize[0]): #итериране по ширината на изображението
            #     for x in range(self.cSize[1]): #итериране по височината на изображението
            #         ind = np.arange(0,len(grayCodeImgs[pattType]))# степените за повдихане при основа 2.
            #         msk = grayCodeImgs[pattType][:,y,x]>(white[y,x]+black[y,x])/2 # макса дали съответния индекс трябва да се използва за повдигане с основа 2
            #         tempGrayCodeMap[pattType][y,x] = np.sum(np.power(2, ind)*msk)

            # #!!! СТАРА И МНОГО БАВНА ВЕРСИЯ. МНОГО ПО-БЪРЗО Е КАТО НЕ СЕ ИТЕРИРАТ ВСИЧКИ ПИСКЕЛИ ЕДИН ПО ЕДИН ЗА ВСЯКО ОТДЕЛНО ИЗОБРАЖЕНИЕ
            # for i in range(len(grayCodeImgs[pattType])): #итериране по изображенията
            #     print('image: ', i)
            #     for y in range(self.cSize[0]): #итериране по ширината на изображението
            #         for x in range(self.cSize[1]): #итериране по височината на изображението
            #             # ако пиксела е бял(по-голямо от средно аритметичното между черното и бялото), то се добавя 2^i към mapping-а в първата колона.
            #             # (white[y,x]+black[y,x])/2 - средно аритметично на цвета на пиксела, когато е оцветен и когато не е.
            #             if grayCodeImgs[pattType][i][y][x] > (white[y,x]+black[y,x])/2:
            #                 tempGrayCodeMap[pattType][y,x] += pow(2,i)
            #             # Проекторът е с по-малка резолюция от камерата и затова се проверява дали няма да се излезе от масива.
            #             # ако пиксела е бял, то се добавя 2^i към mapping-а във вторака колона.
            #             if y < self.pSize[0] and x < self.pSize[1] and patternImgs[pattType][i][y][x] == 255:
            #                 tempPattGrayCodeMap[pattType][y,x] += pow(2,i)

        # np.savetxt('Img{0}.txt'.format(i), x, fmt='%i', delimiter='\t')
        # cv.imwrite('Img{0}.jpg'.format(i), img)
        for cY in range(self.cSize[0]):
            for cX in range(self.cSize[1]):
                # Сборът на стойността в пискела и обратния(Inverse) шаблона трябва да е = 2^(броя шаблони)-1
                # Аналогично за транспонираните шаблони
                if (tempGrayCodeMap[Patterns.IMAGE_PATTERN][cY,cX] + tempGrayCodeMap[Patterns.INV_PATTERN][cY,cX] != pow(2,len(grayCodeImgs[pattType]))-1 or
                    tempGrayCodeMap[Patterns.TRANS_PATTERN][cY,cX] + tempGrayCodeMap[Patterns.TRANS_INV_PATTERN][cY,cX] != pow(2,len(grayCodeImgs[pattType]))-1):
                    nerr += 1
                    continue
                for pY in range(self.pSize[0]):
                    for pX in range(self.pSize[1]):
                        # Трябва стойността на пиксела от изображението да съответства на този на шаблона и аналогично да има съответствие при транспонирания шаблон.=.
                        # Обратните(inverse) шаблони са само контролни и не е необходимо да се търси съвпадение при тях.
                        if (tempGrayCodeMap[Patterns.IMAGE_PATTERN][cY,cX] == tempPattGrayCodeMap[Patterns.IMAGE_PATTERN][pY,pX] and
                           tempGrayCodeMap[Patterns.TRANS_PATTERN][cY,cX] == tempPattGrayCodeMap[Patterns.TRANS_PATTERN][pY,pX]):
                            dist = math.sqrt(pow(cY-pY,2) + pow(cX-pX,2))#разстоянието между точките в изборажението и шаблона
                            # @TODO: Да се направи Lagrange Interpolation https://aikiddie.wordpress.com/2017/05/24/depth-sensing-stereo-image/
                            # За пискела не е намирано съвпадение или намереното разстояние е минимално
                            if self.mask[cY, cX] == 0 or dist < self.grayCodeMap[cY, cX, 2]:
                                self.grayCodeMap[cY, cX, :] = np.array([cY,cX,dist])#записва съответствята на координатите между снимките и шаблоните
                                #мареика се като бяло, т.е. има съвпадение
                                self.mask[cY, cX] = 255
                                yerr += 1

        print(yerr,nerr)#1 256 929-63 764/ 79 540-2 052 088

    # smoothing filter
    def filterGrayCode(self):
        #Трансгормация Dilation(създава плавна граница на обекта) последвана от Erosion(изчиства страничния бял шум от задния план).
        #Използва се за премахване на малки черни дупки в предния план.
        ext_mask = cv.morphologyEx(self.mask, cv.MORPH_CLOSE,
                                    np.ones((self.FILTER*2+1, self.FILTER*2+1)))#Прилага се филтър 3x3
        for y in range(self.cSize[0]):
            for x in range(self.cSize[1]):
                if self.mask[y, x] == 0 and ext_mask[y, x] != 0:
                    sum_x = 0
                    sum_y = 0
                    cnt = 0
                    for dy in range(-self.FILTER, self.FILTER+1):#[-1,0,1]
                        for dx in range(-self.FILTER, self.FILTER+1):#[-1,0,1]
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

    def prespectiveTransform(self,q_matrix):
        self.pointCloud = cv.perspectiveTransform(self.grayCodeMap, q_matrix) # за по-редки съвпадащи точки
        # self.pointCloud = cv.reprojectImageTo3D(???, q_matrix) # за по-гъсти съвпадащи точки

    def savePointCloud(self,dir):
        with open(dir + '/'+self.FILE_NAME,'w') as fid:
            fid.write('ply\n')
            fid.write('format ascii 1.0\n')
            fid.write('element vertex %d\n'%np.count_nonzero(self.mask))
            fid.write('property float x\n')
            fid.write('property float y\n')
            fid.write('property float z\n')
            fid.write('property float red\n')
            fid.write('property float green\n')
            fid.write('property float blue\n')
            fid.write('end_header\n')

            # Write 3D points to .ply file
            for y in range(0, self.cSize[0]):
              for x in range(0, self.cSize[1]):
                  # @TODO: Има разминаване между броя на редовете в маската и тези от лога.
                  # Ако долу филтрирам допълнително pointCloud!=[0,0,0] се получава бройката от лога. Горе трябва да се разследва!
                  if self.mask[y][x] != 0:#
                      fid.write("{0} {1} {2} {3} {4} {5}\n".format(x,y,self.grayCodeMap[y][x][2],#self.pointCloud[y][x][2],#
                                self.colorImg[y][x][2],self.colorImg[y][x][1],self.colorImg[y][x][0]))#150,0,0))#
        print(self.FILE_NAME)

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
        self.colorImg = self.readImages(dir, Patterns.WHITE_PATTERN, self.COLOR)[0]

        # TODO: Да се направи ръчно мапиране, защото това не работи много добре.
        # Идеята е да се взима средно аритметично на пиксела между white и black, което да е граница.
        # След това стойностите ще се определят с мнозинство между Img, ImgInv, ImgTrans и ImgTransInv
        self.manualMapGrayCode(dir, patternCode)
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
    def readImages(self, dir, patternCode, readType):
        print('./{0}/image70{1}*.jpg'.format(dir,patternCode))
        imgsNames = natsorted(glob.glob('./{0}/image70{1}*.jpg'.format(dir,patternCode)))
        imgs = []
        for fname in imgsNames:
            img = None
            if readType == self.BLACK_N_WHITE:#От цветно изображението се прехвърля в черно бяло
                img = self.cameraPi.loadImage(fname,cv.IMREAD_GRAYSCALE)
            else:# иначе цветно
                img = cv.imread(fname)
            imgs.append(img)
        return imgs

    def autoMapGrayCode(self, dir):
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
        yerr, nerr = 0,0 # За debug - колко намерени пиксела има и колко пиксела нямат успешна сума 15.
        white = self.readImages(dir, Patterns.WHITE_PATTERN, self.BLACK_N_WHITE)[0] # Напълно осветено изображение
        black = self.readImages(dir, Patterns.BLACK_PATTERN, self.BLACK_N_WHITE)[0] # Тъмно изображение

        print(patternCode)
        patternImgs = Patterns().genetare(patternCode, self.pSize) # Шаблоните

        # Съдържа изображенията в 4-те различни варианта
        grayCodeImgs = {
            Patterns.IMAGE_PATTERN: self.readImages(dir, Patterns.IMAGE_PATTERN, self.BLACK_N_WHITE),
            Patterns.INV_PATTERN: self.readImages(dir, Patterns.INV_PATTERN, self.BLACK_N_WHITE),
            Patterns.TRANS_PATTERN: self.readImages(dir, Patterns.TRANS_PATTERN, self.BLACK_N_WHITE),
            Patterns.TRANS_INV_PATTERN: self.readImages(dir, Patterns.TRANS_INV_PATTERN, self.BLACK_N_WHITE)}

        # Съдържа измерените стойности за пикселите на изображенията в 4-те различни варианта
        # 3-тото измерение има две стойности - една от изображенията и една от шаблоните.
        tempGrayCodeMap = {
            Patterns.IMAGE_PATTERN: np.zeros((self.cSize[0],self.cSize[1]), np.float32),
            Patterns.INV_PATTERN: np.zeros((self.cSize[0],self.cSize[1]), np.float32),
            Patterns.TRANS_PATTERN: np.zeros((self.cSize[0],self.cSize[1]), np.float32),
            Patterns.TRANS_INV_PATTERN: np.zeros((self.cSize[0],self.cSize[1]), np.float32)}

        # Съдържа измерените стойности за пикселите на шаблоните в 4-те различни варианта
        tempPattGrayCodeMap = {
            Patterns.IMAGE_PATTERN: np.zeros((self.cSize[0],self.cSize[1]), np.float32),
            Patterns.INV_PATTERN: np.zeros((self.cSize[0],self.cSize[1]), np.float32),
            Patterns.TRANS_PATTERN: np.zeros((self.cSize[0],self.cSize[1]), np.float32),
            Patterns.TRANS_INV_PATTERN: np.zeros((self.cSize[0],self.cSize[1]), np.float32)}

        for pattType in patternImgs:
            if len(grayCodeImgs[pattType]) != len(patternImgs[pattType]):
                raise ValueError('Number of patterns and images is not equal!')
            for i in range(len(grayCodeImgs[pattType])):
                for y in range(self.cSize[0]):
                    for x in range(self.cSize[1]):
                        whiteThreshold = white[y,x] #Цвета на пиксела, когато е осветен
                        blackThreshold = black[y,x] #Цвета на пиксела, когато не е осветен
                        grayCodePixel = grayCodeImgs[pattType][y,x] #Цвета на пиксела за конкретния шаблон
                        # ако пиксела е бял(по-голямо от средно аритметичното между черното и бялото), то се добавя 2^i към mapping-а в първата колона.
                        if grayCodePixel > (whiteThreshold+blackThreshold)/2:
                            tempGrayCodeMap[pattType][y,x] += pow(2,i)
                        # Проекторът е с по-малка резолюция от камерата и затова се проверява дали няма да се излезе от масива.
                        # ако пиксела е бял, то се добавя 2^i към mapping-а във вторака колона.
                        if y < self.pSize[0] and x < self.pSize[1] and patternImgs[pattType][y,x] == 255:
                            tempPattGrayCodeMap[pattType][y,x] += pow(2,i)

        for cY in range(self.cSize[0]):
            for cX in range(self.cSize[1]):
                # Сбора на стойността в пискела и обратния(Inverse) шаблона трябва да е 15. Аналогично и са транспонираните шаблони.
                if (tempGrayCodeMap[Patterns.IMAGE_PATTERN] + tempGrayCodeMap[Patterns.INV_PATTERN] != 15 or
                    tempGrayCodeMap[Patterns.TRANS_PATTERN] + tempGrayCodeMap[Patterns.TRANS_INV_PATTERN] != 15):
                    nerr += 1
                    continue
                for pY in range(self.pSize[0]):
                    for pX in range(self.pSize[1]):
                        # Трябва стойността на пиксела от изображението да съответства на този на шаблона и аналогично да има съответствие при транспонирания шаблон.=.
                        # Обратните(inverse) шаблони са само контролни и не е необходимо да се търси съвпадение при тях.
                        if (tempGrayCodeMap[Patterns.IMAGE_PATTERN] == tempPattGrayCodeMap[Patterns.IMAGE_PATTERN] and
                           tempGrayCodeMap[Patterns.TRANS_PATTERN] == tempPattGrayCodeMap[Patterns.TRANS_PATTERN]):
                            dist = math.sqrt(pow(cY-pY,2) + pow(cX-pX,2))#разстоянието между точките в изборажението и шаблона
                            # @TODO: Да се направи Lagrange Interpolation https://aikiddie.wordpress.com/2017/05/24/depth-sensing-stereo-image/
                            self.grayCodeMap[y, x, :] = np.array([cY,cX,dist])#записва съответствята на координатите между снимките и шаблоните
                            #мареика се като бяло, т.е. има съвпадение
                            self.mask[cY, cX] = 255
                            yerr += 1

        print(yerr,nerr)#1 256 929-63 764

    # smoothing filter
    def filterGrayCode(self):
        #Трансгормация Dilation(създава плавна граница на обекта) последвана от Erosion(изчиства страничния бял шум от задния план).
        #Използва се за премахване на малки черни дупки в предния план.
        ext_mask = cv.morphologyEx(self.mask, cv.MORPH_CLOSE,
                                    np.ones((self.FILTER*2+1, self.FILTER*2+1)))#Прилага се филтър 3x3
        for y in range(self.pSize[0]):
            for x in range(self.pSize[1]):
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
                  if self.mask[y][x] != 0 and not np.array_equal(self.pointCloud[y][x],[0,0,0]):#
                      fid.write("{0} {1} {2} {3} {4} {5}\n".format(x,y,self.pointCloud[y][x][2],
                                self.whiteImg[0][y][x][2],self.whiteImg[0][y][x][1],self.whiteImg[0][y][x][0]))#150,0,0))#
        print(self.FILE_NAME)

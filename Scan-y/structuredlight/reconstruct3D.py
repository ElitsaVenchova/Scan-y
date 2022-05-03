import cv2 as cv
import numpy as np
from .patterns import Patterns
from natsort import natsorted
import glob
import math
import open3d as o3d

"""
    Рекоструиране на обекта от изображения в Point clound и Mesh
"""
class Reconstruct3D:



    BLACK_N_WHITE = 0 #Флаг, изображението да се прочете като черно бяло
    COLOR = 1 # Флаг, изображението да се прочете като цветно

    FILE_NAME = 'pointCloud'

    def __init__(self,cameraPi):
        self.cameraPi = cameraPi

        self.pSize = self.cameraPi.stereoCalibrationRes['pShape'] # Размер прожекцията
        self.cSize = self.cameraPi.stereoCalibrationRes['cShape'] # Размер камера

        self.mask = np.zeros(self.cSize, np.uint8)# маска кои пиксели стават за обработване.След инициализацията има стойност False

        self.colorImg = np.zeros((self.cSize[0],self.cSize[1],3), np.float32)
        self.grayCodeMap = np.zeros((self.cSize[0],self.cSize[1], 3), np.float32)#връзката на координатите на пикселите на снимката и шаблона GrayCode
        self.pointCloud = np.zeros(self.cSize, np.float32) # крайния резултат. Облак от точки.

        # Коефицинти с разликата спрямо по-голямото изображение.
        # Използва се при изичсляването на разстоянието, за да може да е пропорционално
        self.cCoef = (max(self.pSize[0]/self.cSize[0],1), max(self.pSize[1]/self.cSize[1],1))
        self.pCoef = (max(self.cSize[0]/self.pSize[0],1), max(self.cSize[1]/self.pSize[1],1))
        self.fullPointClound = np.array([]) # open3d point cloud

    """
        Основната функция за реконструкция.
        Обхождат се сканиранията на различните гледни точки, прави се Point Cloud за тях,
        обединяват се всички point clounds в общ 360 градусово изображение
        Прави се point mesh
    """
    def reconstruct(self, dir, patternCode, stepsCnt, stepSize):

        for scan_no in [10]:#range(0, stepsCnt, stepSize):#
            self.__init__(self.cameraPi)#Класът се инициализира отново, за да се изчистят вече запазените данни от предходното сканиране
            # self.manualMapGrayCode(dir, patternCode, scan_no) # Ръчно мапиране на шаблоните. Работи за GrayCode и Binary(не е тествано)
            # Тези долу са само резервни варианти, които не работят или не са оптимални, но могат да се използват в бъдеще
            # # # self.autoMapGrayCode(dir) # Автоматично мапиране от OpenCV на GrayCode шаблони, но трябва да е заснето с шаблоните на OpenCV
            # # #
            # # # self.filterGrayCode() # smoothing filter. Премахване на артефакти и изглаждане граници заден-преден фон, допълнително заглажда рязките разлики в дълбочината. НЕ РАБОТИ МНОГО ДОБРЕ И Е СПРЯНО!
            # # # self.genPointCloud(self.cameraPi.stereoCalibrationRes["disparityToDepthMatrix"]) # преобразуване на disparity в дълбочина.

            # self.savePointCloud(dir, scan_no) # Запазване на облака от точки като xyzrbg файл
            self.loadPointCloud(dir, scan_no) # Зареждане на облака от точки от xyzrbg файл

    #Прочита изображенията за определен шаблон
    def readImages(self, dir, patternCode,  readType, scan_no = '', img_no='?'):
        # Зареждат се всички изображения, които отговорят на шаблона
        # img_no - ?(точно един символ),*(0 или повече символи),конретно чисто(зарежда изображението с конкретен номер)
        imgsNames = natsorted(glob.glob('./{0}/image{1}{2}{3}.jpg'.format(dir,scan_no,patternCode,img_no)))
        imgs = [] # масив със заредените изображения
        img = None
        for fname in imgsNames:
            if readType == self.BLACK_N_WHITE:#Изображението се прочита черно бяло
                img = self.cameraPi.loadImage(fname,cv.IMREAD_GRAYSCALE)
            else:# иначе цветно
                img = self.cameraPi.loadImage(fname)
            imgs.append(img)
        return np.array(imgs)

    # Автоматично мапиране от OpenCV на GrayCode шаблони, но трябва да е заснето с шаблоните на OpenCV
    def autoMapGrayCode(self, dir):
        BLACK_THRESHOLD = 20#праг на черното
        WHITE_THRESHOLD = 4#праг на бялото
        self.colorImg = self.readImages(dir, Patterns.WHITE_PATTERN, self.COLOR)[0]

        white = self.readImages(dir, Patterns.WHITE_PATTERN, self.BLACK_N_WHITE)[0] # Напълно осветено изображение
        black = self.readImages(dir, Patterns.BLACK_PATTERN, self.BLACK_N_WHITE)[0] # Тъмно изображение
        grayCodeImgs = self.readImages(dir, Patterns.OPENCV_GRAY_CODE, self.BLACK_N_WHITE) # GrayCode изображенията

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
    def manualMapGrayCode(self, dir, patternCode, scan_no):
        self.colorImg = self.readImages(dir, Patterns.INV_PATTERN, self.COLOR, scan_no, 0)[0] #Първият шаблон от Inverse е изцяло бял

        print('Pattern code: ', patternCode, '/ scan_no: ', scan_no)
        patternImgs = Patterns().genetare(patternCode, self.pSize) # Шаблоните

        # Съдържа изображенията в 4-те различни варианта
        grayCodeImgs = {
            Patterns.IMAGE_PATTERN: self.readImages(dir, Patterns.IMAGE_PATTERN, self.BLACK_N_WHITE, scan_no),
            Patterns.INV_PATTERN: self.readImages(dir, Patterns.INV_PATTERN, self.BLACK_N_WHITE, scan_no),
            Patterns.TRANS_PATTERN: self.readImages(dir, Patterns.TRANS_PATTERN, self.BLACK_N_WHITE, scan_no),
            Patterns.TRANS_INV_PATTERN: self.readImages(dir, Patterns.TRANS_INV_PATTERN, self.BLACK_N_WHITE, scan_no)}

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
            Patterns.IMAGE_PATTERN: np.zeros(self.pSize, np.uint16),
            Patterns.INV_PATTERN: np.zeros(self.pSize, np.uint16),
            Patterns.TRANS_PATTERN: np.zeros(self.pSize, np.uint16),
            Patterns.TRANS_INV_PATTERN: np.zeros(self.pSize, np.uint16)}

        for pattType in patternImgs: #итериране по видовете шаблони
            print(pattType)
            if len(grayCodeImgs[pattType]) != len(patternImgs[pattType]):
                raise ValueError('Number of patterns and images is not equal for pattern ',pattType,'(',len(grayCodeImgs[pattType]),len(patternImgs[pattType]),')!')
            # Създава степените, на които трябва да се повдигне съответното изображение. Първото е на степен 0, второто е на степен 1 и т.н.
            # np.mgrid[:4,:3,:2] = [[[0,0,0],[0,0,0]],[[1,1,1],[1,1,1]],[[2,2,2],[2,2,2]],[[3,3,3],[3,3,3]]]
            ind = np.mgrid[:len(grayCodeImgs[pattType]),:self.cSize[0],:self.cSize[1]][0]
            # (white[y,x]+black[y,x])/2 - средно аритметично на цвета на пиксела, когато е оцветен и когато не е.
            msk = grayCodeImgs[pattType]>((white+black)/2) + 25 #white-30# макса дали съответния индекс трябва да се използва за повдигане с основа 2
            # msk = np.array([self.morphologyEx(255*m) for m in msk.astype(np.uint16)]).astype(np.bool_)# Засега е по-лошо. създава плавна граница на обекта и изчиства страничния бял шум от задния план

            # Подвига се 2 на степен индексите и при умножаването по msk се взимат само стойностите с true. Резултатът се сумира по ос 0,
            # т.е. сумира се съответния пиксел за всички изображения в шаблона
            tempGrayCodeMap[pattType] = np.sum(np.power(2, ind)*msk, axis=0)

            # Аналогично за шаблоните като тук размерът се взима от pSize и сравнението е директно с 255 цвят на пиксела
            indPatt = np.mgrid[:len(patternImgs[pattType]),:self.pSize[0],:self.pSize[1]][0]
            mskPatt = patternImgs[pattType] == 255
            tempPattGrayCodeMap[pattType] = np.sum(np.power(2, indPatt)*mskPatt, axis=0)

        # DEBUG
        # [675:1030,745:1145]
        # np.savetxt('Img{0}.txt'.format(scan_no), x, fmt='%i', delimiter='\t')
        # cv.imwrite('Img{0}{1}.jpg'.format(scan_no,'txt'), img)

        # Генериране на индекси на изображенията [[0,0],[0,1],[0,1],[1,0],[1,1],...]
        indImg = np.mgrid[:tempGrayCodeMap[Patterns.IMAGE_PATTERN].shape[0],:tempGrayCodeMap[Patterns.IMAGE_PATTERN].shape[1]]
        # Stack-ване на масивите на изображенията в едно общо със 6 стойности на ред - x,y, IMAGE_PATTERN value, INV_PATTERN value, ...
        stackImgs = np.dstack((indImg[0],indImg[1],
                        tempGrayCodeMap[Patterns.IMAGE_PATTERN],tempGrayCodeMap[Patterns.INV_PATTERN],
                        tempGrayCodeMap[Patterns.TRANS_PATTERN],tempGrayCodeMap[Patterns.TRANS_INV_PATTERN]))
        # Филтриране - Сборът на стойността в пискела(Image) и обратния(Inverse) шаблона трябва да е = 2^(броя шаблони)-1
        # Аналогичното важи и за транспонираните шаблони. Използва се оператор умножаване <*>, защото дава грешка при използването на <and>
        stackImgs = stackImgs[((stackImgs[:,:,2] + stackImgs[:,:,3] == pow(2,len(grayCodeImgs[Patterns.IMAGE_PATTERN]))-1) *
            (stackImgs[:,:,4] + stackImgs[:,:,5]  == pow(2,len(grayCodeImgs[Patterns.IMAGE_PATTERN]))-1)),:]#Някак от 3D става 2D масив, но точно това ми трябва

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

        for i,pix in enumerate(stackImgs):
            for patPix in stackPatts[(pix[[2,4]]==stackPatts[:,[2,4]]).all(axis=1)]:
                if (pix[2] == patPix[2] and pix[4] == patPix[4]):

                    #разстоянието между точките в изборажението и шаблона
                    #добавя се +1 на всички координати, за да може при умножението по коефициентите за разликата на изображенияна,
                    #да се получи вярна стойност. Иначе за y=1 => 1*1.2 = 1.2, а това реално е втория пиксел и трябва да бъде 2*1.2 = 2.4
                    dist = math.sqrt(pow((pix[0]+1)-(patPix[0]+1),2)
                            + pow((pix[1]+1)-(patPix[1]+1),2))
                    # math.sqrt(pow((pix[0]+1)*self.cCoef[0]-(patPix[0]+1)*self.pCoef[0],2)
                    #         + pow((pix[1]+1)*self.cCoef[1]-(patPix[1]+1)*self.pCoef[1],2))
                    # За пискела не е намирано съвпадение или намереното разстояние е минимално
                    if self.mask[int(pix[0]), int(pix[1])] == 0 or dist < self.grayCodeMap[int(pix[0]), int(pix[1]), 2]:
                        self.grayCodeMap[int(pix[0]), int(pix[1]), :] = np.array([pix[0], pix[1],dist])#записва съответствята на координатите между снимките и шаблоните
                        #маркира се като бяло, т.е. има съвпадение
                        self.mask[int(pix[0]), int(pix[1])] = 255
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

    """
     Генeрира облака от точки от матрицата на Q и мапинга на разликата между пикселите в едното и другото изображение (x-x')
    """
    def genPointCloud(self,q_matrix):
        self.pointCloud = cv.perspectiveTransform(self.grayCodeMap, q_matrix) # за по-редки съвпадащи точки
        # self.pointCloud = cv.reprojectImageTo3D(???, q_matrix) # за по-гъсти съвпадащи точки

    # Запазване на облака от точки като xyzrbg файл
    def savePointCloud(self,dir, scan_no):
        fileFullName = '{0}/{1}{2}.ply'.format(dir,self.FILE_NAME,scan_no)
        with open(fileFullName,'w') as fid:
            fid.write('ply\n')
            fid.write('format ascii 1.0\n')
            fid.write('element vertex %d\n'%np.count_nonzero(self.mask))#[675:1030,745:1145]
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
                  if self.mask[y][x] != 0 :#and y>=675 and y<=1030 and x>=745 and x<=1145#Записват се само редовете записани като валидни
                      fid.write("{0} {1} {2} {3} {4} {5}\n".format(x,y,self.grayCodeMap[y][x][2],#self.pointCloud[y][x][2],#
                                self.colorImg[y][x][2],self.colorImg[y][x][1],self.colorImg[y][x][0]))#150,0,0))# .astype(np.float) / 255.0
        print(fileFullName)

    # Зареждане на облака от точки от xyzrbg файл
    # dir - директорията, в която са запазени point clound да всяка гледна точка
    # scan_no - номер на сканиране. Ако не е подадено, зарежда всички гледни точки
    def loadPointCloud(self, dir, scan_no):
        for scan_no2 in [0,30]:#range(0, 200, 10):#range(0, 50, 10):#
            fileFullName = '{0}/{1}{2}.ply'.format(dir,self.FILE_NAME,scan_no2)
            pcd = o3d.io.read_point_cloud(fileFullName)
            # points = np.asarray(pcd.points)
            # pcd = pcd.select_by_index(np.where(points[:,1] > 600)[0])#700
            # points = np.asarray(pcd.points)
            # pcd = pcd.select_by_index(np.where(points[:,1] < 1000)[0])
            # points = np.asarray(pcd.points)
            # pcd = pcd.select_by_index(np.where(points[:,2] < 1000)[0])
            # downpcd = pcd.voxel_down_sample(voxel_size=0.05) # down sample на входните точки, за да не са твърде много и да се обработва по-лесно.
            R = pcd.get_rotation_matrix_from_xyz((0, -np.deg2rad(1.8*scan_no2), 0))
            pcd = pcd.rotate(R, center=(885,1000,295))
# 885,1000,295
            self.fullPointClound = np.append(self.fullPointClound,pcd)

        # За тест може да се визуализира пълния или down sample облак от точки
        self.visualizationPointCloud(self.fullPointClound)#downpcd # Визуализиране на облака от точки

    # Визуализиране на облака от точки
    def visualizationPointCloud(self, pcd):
        o3d.visualization.draw_geometries(pcd)

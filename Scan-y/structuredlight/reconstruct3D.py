+import cv2 as cv
import numpy as np
from .patterns import Patterns
import open3d as o3d

"""
    Рекоструиране на обекта от изображения в Point clound и Mesh
"""
class Reconstruct3D:

    FILTER = 1# Размер на филтъта. 1-3x3 прозорец
    BLACKTHR = 20#праг на черното
    WHITETHR = 4#праг на бялото

    BLACK_N_WHITE = 0
    COLOR = 1

    self.FILE_NAME = 'pointCloud.ply'

    def __init__(self,pSize):
        self.piCamera = piCamera
        self.calibrationResCam = self.cameraPi.readCalibrationResult(self.piCamera.CALIBRATION_DIR)

        self.pSize = pSize # Размер прожекцията
        self.cSize = self.calibrationResCam['shape'] # Размер камера

        self.mask = np.zeros(self.cSize, np.bool_)# маска кои пиксели стават за обработване.След инициализацията има стойност False

        self.whiteImg = np.zeros((self.cSize[0],self.cSize[1],3), np.float32)
        self.grayCodeMap = np.zeros((self.cSize[0],self.cSize[1], 2), np.int16)#връзката на координатите на пикселите на снимката и шаблона GrayCode
        self.perspectiveTransformMap = np.zeros((3,3), np.float32)
        self.pointCloud = np.zeros(cSize, np.float32)

    def reconstruct(self, dir, cameraPi):
        self.whiteImg = self.readImages(dir, Patterns.WHITE_PATTERN, self.COLOR)

        self.mapGrayCode(dir)
        self.filterGrayCode()
        self.prespectiveTransform()

        self.getPointCloud()
        self.savePointCloud(dir)

        pcd = o3d.io.read_point_cloud("../../test_data/fragment.pcd")
        #http://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
            o3d.io.write_triangle_mesh("copy_of_knot.ply", mesh)
        Multi view matching - http://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html
        Color map optimisation - http://www.open3d.org/docs/release/tutorial/pipelines/color_map_optimization.html

    def mapGrayCode(self, dir):
        white = self.readImages(dir, Patterns.WHITE_PATTERN, self.BLACK_N_WHITE)
        black = self.readImages(dir, Patterns.BLACK_PATTERN, self.BLACK_N_WHITE)
        grayCodeImgs = self.readImages(dir, Patterns.GRAY_CODE, self.BLACK_N_WHITE)

        graycode = cv2.structured_light_GrayCodePattern.create(gc_width, gc_height)
        graycode.setBlackThreshold(BLACKTHR)
        graycode.setWhiteThreshold(WHITETHR)
        for y in range(self.pSize[0]):
            for x in range(self.pSize[1]):
                if int(white[y, x]) - int(black[y, x]) <= BLACKTHR: #т.е. пиксела е черен и не може да се обработи(255-0 трябва да дава бяло, а не черно)
                    continue
                err, proj_pix = graycode.getProjPixel(grayCodeImgs, x, y)#за (x,y) от снимките, връща (x,y) от шаблоните
                if not err:
                    self.grayCodeMap[y, x, :] = np.array(proj_pix)#записва съответствята на координатите между снимките и шаблоните
                    #мареика се като бяло, т.е. има съвпадение
                    self.mask[y, x] = True

    # smoothing filter
    def filterGrayCode(self):
        #Трансгормация Dilation(създава плавна граница на обекта) последвана от Erosion(изчиства страничния бял шум от задния план).
        #Използва се за премахване на малки черни дупки в предния план.
        #   255*self.mask - от Bool масив прави масив от 0 или 255
        ext_mask = cv.morphologyEx(255*self.mask, cv.MORPH_CLOSE,
                                    np.ones((self.FILTER*2+1, self.FILTER*2+1)))#Прилага се филтър 3x3
        for y in range(self.pSize[0]):
            for x in range(self.pSize[1]):
                if self.mask[y, x] == False and ext_mask[y, x] != 0:
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
                                sum_x += self.grayCodeMap[ty, tx, 0]#Сумират се координатите на съседните изображения
                                sum_y += self.grayCodeMap[ty, tx, 1]#Сумират се координатите на съседните изображения
                                cnt += 1
                    if cnt != 0:
                        #така новата GC карта орговяря на инфомрацията в новата маска ext_mask
                        self.grayCodeMap[y, x, 0] = np.round(sum_x/cnt)# x е средно аритметично на x от съседните пиксели
                        self.grayCodeMap[y, x, 1] = np.round(sum_y/cnt)# y е средно аритметично на x от съседните пиксели

            self.mask = ext_mask==255#ext_mask е масив от 0 или 255. Целият израз връща True за стойностите 255 и False за останалите

    def prespectiveTransform(self):
        h = np.arange(self.cSize[0])
        w = np.arange(self.cSize[1])
        imgMap = np.transpose([np.repeat(w, len(h)),np.tile(h, len(w))])
        imgMap = imgMap.reshape(w,h,2)
        self.perspectiveTransformMap = cv.getPerspectiveTransform(imgMap, self.grayCodeMap)

    def getPointCloud(self):
        disparityMap = np.zeros((self.cSize[0],self.cSize[1],3), np.float32)
        for y in range(self.cSize[0]):
            for x in range(self.cSize[1]):
                disparityMap[y,x] = np.sqrt(np.power((y-grayCodeMap[y,x,1]),2) + np.power((z-grayCodeMap[y,x,1]),2))

        self.pointCloud = cv.reprojectImageTo3D(
                         disparityMap,
                         self.perspectiveTransformMap)

    def savePointCloud(self,dir):
        with open(dir + '/'+self.FILE_NAME,'w') as fid:
            fid.write('ply\n')
            fid.write('format ascii 1.0\n')
            fid.write('element vertex %d\n'%self.mask.sum())
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
                  if (not self.mask[y][x]):#
                      fid.write("{0} {1} {2} {3} {4} {5}\n".format(x,y,self.pointCloud[y][x],
                                self.whiteImg[y][x][2],self.whiteImg[y][x][1],self.whiteImg[y][x][0]))

    #Прочита изображенията за определен шаблон
    def readGrayCodeImgs(self, dir, patternCode, readType):
        imgsNames = natsorted(glob.glob('./{0}/image0{1}*.jpg'.format(dir,patternCode)))
        imgs = []
        for fname in imgsNames:
            img = None
            if readType == 0:#От цветно изображението се прехвърля в черно бяло
                img = cv.imread(fname, cv.IMREAD_GRAYSCALE)
            else:# иначе цветно
                img = cv.imread(fname)
            img = self.piCamera.undistortImage(img, self.calibrationResCam)#прилагане на калибрирането на камерата
            imgs.append(img)
        return imgs

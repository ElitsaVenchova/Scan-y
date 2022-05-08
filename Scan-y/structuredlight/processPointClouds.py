import cv2 as cv
import numpy as np
from .patterns import Patterns
from natsort import natsorted
import glob
import math
import open3d as o3d

"""
    Обработка на данните за облаците от точки и mesh.
    Запазване, зареждане, обработка, съединаване на Point clound и преобразуване в Mesh.
"""
class ProcessPointClouds:

    FILE_NAME = 'pointCloud'

    # Съдържа 360 градуса сканиране
    fullPointClound = np.array([]) # open3d point cloud

    def __init__(self, cSize):
        self.cSize = cSize # Размер камера

    def process(self, dir, stepsCnt, stepSize):
        for scan_no in [70]:#range(0, stepsCnt, stepSize):#
            self.loadPointCloud(dir, scan_no)

    """
     Генeрира облака от точки от матрицата на Q и мапинга на разликата между пикселите в едното и другото изображение (x-x')
    """
    def genPointCloud(self, grayCodeMap, q_matrix):
        pointCloud = cv.perspectiveTransform(grayCodeMap, q_matrix) # за по-редки съвпадащи точки
        # pointCloud = cv.reprojectImageTo3D(???, q_matrix) # за по-гъсти съвпадащи точки
        return pointCloud

    # Запазване на облака от точки като xyzrbg файл
    def savePointCloud(self, dir, grayCodeMap, mask, colorImg, cSize, scan_no):
        fileFullName = '{0}/{1}{2}.ply'.format(dir,self.FILE_NAME,scan_no)
        with open(fileFullName,'w') as fid:
            fid.write('ply\n')
            fid.write('format ascii 1.0\n')
            fid.write('element vertex %d\n'%np.count_nonzero(mask))#[675:1030,745:1145]
            fid.write('property double x\n')
            fid.write('property double y\n')
            fid.write('property double z\n')
            fid.write('property uchar red\n')
            fid.write('property uchar green\n')
            fid.write('property uchar blue\n')
            fid.write('end_header\n')

            # Write 3D points to .ply file
            for y in range(0, self.cSize[0]):
              for x in range(0, self.cSize[1]):
                  if mask[y][x] != 0 :#and y>=675 and y<=1030 and x>=745 and x<=1145#Записват се само редовете записани като валидни
                      fid.write("{0} {1} {2} {3} {4} {5}\n".format(x,y,grayCodeMap[y][x][2],#self.pointCloud[y][x][2],#
                                colorImg[y][x][2],colorImg[y][x][1],colorImg[y][x][0]))#150,0,0))# .astype(np.float) / 255.0
        print(fileFullName)

    # Зареждане на облака от точки от xyzrbg файл
    # dir - директорията, в която са запазени point clound да всяка гледна точка
    # scan_no - номер на сканиране. Ако не е подадено, зарежда всички гледни точки
    def loadPointCloud(self, dir, scan_no):
        rotCenter=(750, 935, 413)#800, 935, 283#(ширина,височина,дълбочина)#Центърът, около който се върти платформата

        for scan_no2 in [30,70]:#range(0, 200, 10):#
            fileFullName = '{0}/{1}{2}.ply'.format(dir,self.FILE_NAME,scan_no2)
            pcd = o3d.io.read_point_cloud(fileFullName)
            # downpcd = pcd.voxel_down_sample(voxel_size=0.05) # down sample на входните точки, за да не са твърде много и да се обработва по-лесно.
            # pcd = self.removeBackground(pcd)#махане на задния фон и максимална част от въртящата платформа
            # pcd = pcd.translate((-350, 100, -870))
            # Всекя гледна точка е завътряна на 1.8*броя стъпки.(1.8. са градусите на завъртане за една стъпка)
            R = pcd.get_rotation_matrix_from_xyz((0, -np.deg2rad(1.8*scan_no2),  -np.deg2rad(2.5)))
            pcd = pcd.rotate(R, center=rotCenter)

            o3d.io.write_point_cloud("copy_of_fragment.ply",pcd, True)

            self.fullPointClound = np.append(self.fullPointClound,pcd)

        self.drawBoxesAndCenter(pcd,rotCenter)
        self.visualizationPointCloud(self.fullPointClound)#downpcd # Визуализиране на облака от точки

    # Визуализиране на облака от точки
    def visualizationPointCloud(self, pcd):
        o3d.visualization.draw_geometries(pcd)

    def removeBackground(self, pcd):
        rotCenter=(800, 935, 283)#(ширина,височина,дълбочина)
        R = pcd.get_rotation_matrix_from_xyz((np.deg2rad(0), np.deg2rad(0), np.deg2rad(2.5)))#Последното върти все едно сминката и е 0, защото си е ОК
        pcd = pcd.rotate(R, center=rotCenter)#Завъртане малко на основата, за да се махне максимално въртящата платформа
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, 0, 0), max_bound=(self.cSize[1], self.cSize[0]-155, 283))#(ширина,височина,дълбочина)
        pcd = pcd.crop(bbox)#махане на задния фон и максимална част от въртящата платформа
        #връщане на първоначалната ориентация, за да може да се приложи начисто резултатът от сканирането.
        R = pcd.get_rotation_matrix_from_xyz((np.deg2rad(0), np.deg2rad(0), -np.deg2rad(2.5)))#Последното върти все едно сминката и е 0, защото си е ОК
        pcd = pcd.rotate(R, center=rotCenter)#Завъртане малко на основата, за да се махне максимално въртящата платформа
        return pcd

    def triangle_pcd(self,rotCenter):
        '''
         Defines the point cloud of a triangle
        :return:
        '''
        center = np.array(rotCenter)
        triangle_points = np.array([center-5,center, center+5], dtype=np.float32)
        lines = [[0, 1], [1, 2], [2, 0]]  # Right leg
        colors = [[0, 0, 1] for i in range(len(lines))]  # Default blue
        #  Define the three corners of a triangle
        point_pcd = o3d.geometry.PointCloud()  #  Defining point clouds
        point_pcd.points = o3d.utility.Vector3dVector(triangle_points)

        #  Define the triangle, three connecting lines
        line_pcd = o3d.geometry.LineSet()
        line_pcd.lines = o3d.utility.Vector2iVector(lines)
        line_pcd.colors = o3d.utility.Vector3dVector(colors)
        line_pcd.points = o3d.utility.Vector3dVector(triangle_points)

        return line_pcd, point_pcd

    # За DEBUG - Визуализиране на кутии за ориентиране(на обекта и на координатната система). Рисува центъра
    def drawBoxesAndCenter(self, pcd, rotCenter):
        # Визуализиране на кутии за ориентиране
        aabb = pcd.get_axis_aligned_bounding_box()
        aabb.color = (1, 0, 0)
        obb = pcd.get_oriented_bounding_box()
        obb.color = (0, 1, 0)
        line_pcd, point_pcd = self.triangle_pcd(rotCenter)
        o3d.visualization.draw_geometries([pcd,line_pcd,point_pcd,aabb,obb])

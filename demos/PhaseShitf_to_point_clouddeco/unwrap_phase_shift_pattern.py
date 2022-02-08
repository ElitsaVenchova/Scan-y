import cv2 as cv
import numpy as np
import struct

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
import csv
import math

# https://www.instructables.com/Structured-Light-3D-Scanning/ към него https://code.google.com/archive/p/structured-light/downloads
# като е гледано ThreePhase-1-win.zip от Dec 29, 2009
class PhaseShiftDecode:
    phase1Image, phase2Image, phase3Image = (None, None, None)
    inputHeight, inputWidth = (None, None)
    phase = None # фазово обвоване(phase wrap)
    mask = None # премахва се шума
    process = None
    colors = None # цветовете на пикселите. По-долу се взима най-осветения
    # списък с точките за обработка.
    # Използва се, за да се приравни фазата към съм съседните пискели като се започва от среда и после се разширява спираловидно.
    toProcess = []
    # 3d координатит на точките
    pointClound = None

    def decode(self):
      self.loadImages()
      self.inputHeight, self.inputWidth = self.phase1Image.shape[:2]
      self.phase = np.zeros((self.inputHeight,self.inputWidth), np.float32)
      self.mask = np.zeros((self.inputHeight,self.inputWidth), np.bool_)
      self.process = np.zeros((self.inputHeight,self.inputWidth), np.bool_)
      self.colors = np.zeros((self.inputHeight,self.inputWidth,3), np.float32)
      self.pointCloud = np.zeros((self.inputHeight,self.inputWidth), np.float32)

      self.phaseWrap()
      self.phaseUnwrap()
      self.calcDepth()
      self.writeCsv('pointCloud.csv')
      # self.plot()
      self.write_pointcloud("res.ply")

    def loadImages(self):
        self.phase1Image = cv.imread('./img/phase1.jpg')
        self.phase2Image = cv.imread('./img/phase2.jpg')
        self.phase3Image = cv.imread('./img/phase3.jpg')

    def phaseWrap(self):
        noiseThreshold = 0.1

        sqrt3 = np.sqrt(3)
        for y in range(0, self.inputHeight):
          for x in range(0, self.inputWidth):
              color1 = self.phase1Image[y][x]
              color2 = self.phase2Image[y][x]
              color3 = self.phase3Image[y][x]

              phase1 = float(color1[0]/255) #color1[0] - връща син цвят
              phase2 = float(color2[0]/255)
              phase3 = float(color3[0]/255)

              phaseSum = phase1 + phase2 + phase3
              phaseRange = max(phase1, phase2, phase3) - min(phase1, phase2, phase3)

              # ignore noise
              gamma = phaseRange / phaseSum
              self.mask[y][x] = gamma < noiseThreshold
              self.process[y][x] = not self.mask[y][x]

              # this equation can be found in Song Zhang's
              # "Recent progresses on real-time 3D shape measurement..."
              # and it is the "bottleneck" of the algorithm
              # it can be sped up with a look up table, which has the benefit
              # of allowing for simultaneous gamma correction.
              self.phase[y][x] = np.arctan2(sqrt3 * (phase1 - phase3), 2 * phase2 - phase1 - phase3) / (np.pi*2)

              #build color based on the lightest channels from all three images
              #
              # От RGB цвете преобразуваме в YUV и по-точно само Y(яркост), за се вземе най-осветения пиксел
              # Y' = 0.299*R + 0.587*G + 0.144*B
              Ycolor1 = 0.299*color1[2] + 0.587*color1[1] + 0.144*color1[0]
              Ycolor2 = 0.299*color2[2] + 0.587*color2[1] + 0.144*color2[0]
              Ycolor3 = 0.299*color3[2] + 0.587*color3[1] + 0.144*color3[0]
              self.colors[y][x] = color1 if Ycolor1 >= Ycolor2 and Ycolor1 >= Ycolor3 else (color2 if Ycolor2 >= Ycolor3 else color3)

    def phaseUnwrap(self):
        startX = int(self.inputWidth / 2)
        startY = int(self.inputHeight / 2)

        self.toProcess.append((startX, startY))
        self.process[startX][startY] = False # фазата на средния пиксел за взима за приравняване и съответно тя не се приравнява

        # while self.toProcess is not empty
        while (self.toProcess):
            xy = self.toProcess.pop(0)
            x = xy[0]
            y = xy[1]
            r = self.phase[y][x]

            if (y > 0):
                self.phaseUnwrapWithBase(r, x, y - 1)
            if (y < self.inputHeight - 1):
                self.phaseUnwrapWithBase(r, x, y + 1)
            if (x > 0):
                self.phaseUnwrapWithBase(r, x - 1, y)
            if (x < self.inputWidth - 1):
                self.phaseUnwrapWithBase(r, x + 1, y)

    def phaseUnwrapWithBase(self, basePhase, x, y):
        if (self.process[y][x]):
            diff = self.phase[y][x] - (basePhase - int(basePhase))
            if (diff > 0.5):
                diff-=1
            if (diff < -0.5):
                diff+=1
            # фазата се подранява със съседния пиксел.
            # мисля, че промяната не е по-голяма от [-0.5,0.5]
            self.phase[y][x] = basePhase + diff
            self.process[y][x] = False
            self.toProcess.append((x, y))

    def calcDepth(self):
        zscale = 130
        zskew = 24
        for y in range(0, self.inputHeight):
          planephase = 0.5 - (y - (self.inputHeight / 2)) / zskew
          for x in range(0, self.inputWidth):
            if (not self.mask[y][x]):
                self.pointCloud[y][x] = (self.phase[y][x] - planephase) * zscale

    # https://gist.github.com/Shreeyak/9a4948891541cb32b501d058db227fff
    def write_pointcloud(self, filename):

        """ creates a .pkl file of the point clouds generated
        """
        # Write header of .ply file
        # with open(filename,'wb') as fid:
        #     fid.write(bytes('ply\n', 'utf-8'))
        #     fid.write(bytes('format binary_little_endian 1.0\n', 'utf-8'))
        #     r = self.inputHeight*self.inputWidth
        #     fid.write(bytes('element vertex %d\n'%r, 'utf-8'))
        #     fid.write(bytes('property float x\n', 'utf-8'))
        #     fid.write(bytes('property float y\n', 'utf-8'))
        #     fid.write(bytes('property float z\n', 'utf-8'))
        #     # fid.write(bytes('property float red\n', 'utf-8'))
        #     # fid.write(bytes('property float green\n', 'utf-8'))
        #     # fid.write(bytes('property float blue\n', 'utf-8'))
        #     fid.write(bytes('end_header\n', 'utf-8'))
        #
        #     # Write 3D points to .ply file
        #     for y in range(0, self.inputHeight):
        #       for x in range(0, self.inputWidth):
        #           fid.write(bytearray(struct.pack("iiffff",y,x,self.pointCloud[y][x],
        #                                           self.colors[y][x][0],self.colors[y][x][1],self.colors[y][x][2])))

        with open(filename,'w') as fid:
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
            for y in range(0, self.inputHeight):
              for x in range(0, self.inputWidth):
                  if (not self.mask[y][x] and self.pointCloud[y][x]<500 and self.pointCloud[y][x]>-500):#
                      fid.write("{0} {1} {2} {3} {4} {5}\n".format(x,y,self.pointCloud[y][x]/(2/np.pi),
                                self.colors[y][x][0],self.colors[y][x][1],self.colors[y][x][2]))

    def plot(self):
        mesh = np.array(np.meshgrid(np.arange(0,self.inputWidth), np.arange(0,self.inputHeight)))
        combinations = mesh.T.reshape(-1, 2)
        msk = np.invert(self.mask.flatten())
        x = combinations[:,1] #self.pointCloud[:, 0]
        x = x[msk]
        y = combinations[:,0] #self.pointCloud[:, 1]
        y = y[msk]
        z = self.pointCloud.flatten()#self.pointCloud[:, 2]
        z = z[msk]
        print(self.pointCloud.shape)
        print(x.shape)
        print(y.shape)
        print(z.shape)

        fig = plt.figure(figsize=(12,7))
        ax = fig.add_subplot(projection='3d')
        img = ax.scatter(x, y, z, c=z, cmap=plt.hot())
        fig.colorbar(img)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # fig = plt.figure(figsize=(8, 8))
        # ax = fig.add_subplot(111, projection='3d')
        #
        # ax.scatter(x, y, z, 1)
        plt.show()

    def writeCsv(self,filename):
        header = ['x', 'y', 'z']

        with open(filename, 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)

            for y in range(0, self.inputHeight):
              for x in range(0, self.inputWidth):
                  if (not self.mask[y][x]):
                       writer.writerow((x,y,self.pointCloud[y][x]))

if __name__=="__main__":
    phaseShift = PhaseShiftDecode()
    phaseShift.decode()

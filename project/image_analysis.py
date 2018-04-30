
import numpy as np
import cv2
import time
from numpy.matlib import matrix, identity, dot, transpose, array
from numpy.linalg import norm
from math import cos, sin
from operator import add, sub

class ImageAnalysis(object):

    def __init__(self):
        self.penWidth = 14
        self.lowerColor = (90, 190, 190)
        self.upperColor = (120, 255, 255)
        self.averageBrightness = 164601. * 4
        self.averageIntensity = 161428.

    def analyzeImages(self, imageTop, imageBot):
        self.penPositionInImage(imageTop)
        if self.penFound:
            self.penInTopImage = True
            return
        self.penPositionInImage(imageBot)
        if self.penFound:
            self.penInTopImage = False
            return


    # return true if pen found
    def penPositionInImage(self, image):
        border = self.calcBorderChains(image)
        if (len(border) != 2):
            self.penFound = False
            return
        self.penFound = True

        self.center1 = self.getCenterOfBorder(border[0])
        self.center2 = self.getCenterOfBorder(border[1])
        center = (self.center1[0] + self.center2[0]) / 2, (self.center1[1] + self.center2[1]) / 2

        self.borderPoint1 = self.getMostOrthogonalBorderPoint(self.center1, self.center2, border[0])
        self.width1 = 2 * np.linalg.norm(map(sub, self.borderPoint1, self.center1))
        self.borderPoint2 = self.getMostOrthogonalBorderPoint(self.center2, self.center1, border[1])
        self.width2 = 2 * np.linalg.norm(map(sub, self.borderPoint2, self.center2))
        width = (self.width1 + self.width2) / 2

        self.calculateKoordinatesOfPenInImage(center, width)

    def calculateKoordinatesOfPenInImage(self, center, width):
        anglePerPixel = ((60.97 / 640) / 180) * np.pi # winkel pro pixel

        theta = center[1] * anglePerPixel - (47.64 / 180) * np.pi / 2 + np.pi / 2 # hoehe im bild, winkel von oben
        phi = center[0] * anglePerPixel - (60.97 / 180) * np.pi / 2  # seitliche verschiebung im bild, winkel von Mitte des Bildes
        r = (self.penWidth / 2) / np.tan(anglePerPixel * width / 2)  # do the calculation (trigonometry)

        x = r * np.sin(theta) * np.cos(phi) # to cartesian coordinates
        y = - r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)

        # print (theta, phi, r)
        # print (x,y,z)
        self.penPosition = (x,y,z)

    def getCenterOfBorder(self, border):
        sumx = sum(border[:,0,0])
        sumy = sum(border[:,0,1])
        length = len(border[:,0,0])
        return sumx / length, sumy / length

    def getMostOrthogonalBorderPoint(self, center1, center2, border1):
        center2Normed = map(sub, center2, center1)
        center2Normed = center2Normed / np.linalg.norm(center2Normed)
        self.penDirection = center2Normed

        orthogonality = [None] * len(border1[:])
        for index, borderPoint in enumerate(border1[:,0]):
            borderPointNormed = map(sub, borderPoint, center1)
            borderPointNormed = borderPointNormed / np.linalg.norm(borderPointNormed)
            orthogonality[index] = ((center2Normed[0] * borderPointNormed[0]) + (center2Normed[1] * borderPointNormed[1]))**2
        index = orthogonality.index(min(orthogonality))
        return border1[index][0]

    #torso-headyaw       0,     0, 126.5
    #headyaw-headpitch   0,     0, 0
    #headpitch-camTop    58.71, 0, 63.64    tilted down 0.0209 [1.2]
    #headpitch-camBot    50.71, 0, 17,74    tilted down 0.6929 [39.7]
    def calculatePenToBody(self, headYaw, headPitch):
        transformToHeadyaw = self.calculateLocalTransformationMatrix(headYaw, [0,0,126.5], [0,0,1])
        transformToHeadpitch = self.calculateLocalTransformationMatrix(headPitch, [0,0,0], [0,1,0])
        if self.penInTopImage:
            transformToCam = self.calculateLocalTransformationMatrix(0.0209, [58.71, 0, 63.64], [0,1,0])
            #-0.0209
        else:
            transformToCam = self.calculateLocalTransformationMatrix(0.6929, [50.71, 0, 17.74], [0, 1, 0])
            #-0.6929

        transformToImage = self.calculateLocalTransformationMatrix(0, self.penPosition, [0, 1, 0])


        #M = transformToHeadyaw
        #print M[0, 3], M[1, 3], M[2,3]
        #M = dot(transformToHeadyaw, transformToHeadpitch)
        #print M[0, 3], M[1, 3], M[2,3]
        #M = dot(dot(transformToHeadyaw, transformToHeadpitch), transformToCam)
        #print M[0, 3], M[1, 3], M[2,3]
        #M = dot(dot(dot(transformToHeadyaw, transformToHeadpitch), transformToCam), transformToImage)
        #print M[0, 3], M[1, 3], M[2,3]
        M = dot(dot(dot(transformToHeadyaw, transformToHeadpitch), transformToCam), transformToImage)
        return M[0, 3], M[1, 3], M[2,3]

    def calculateLocalTransformationMatrix(self, angle, transform, direction):
        u = direction / norm(direction)
        x = u[0]
        y = u[1]
        z = u[2]
        c = cos(angle)
        s = sin(angle)
        return matrix(
            [[x*x*(1-c)+c,   x*y*(1-c)-z*s, x*z*(1-c)+y*s, transform[0]],
             [y*x*(1-c)+z*s, y*y*(1-c)+c,   y*z*(1-c)-x*s, transform[1]],
             [z*x*(1-c)-y*s, z*y*(1-c)+x*s, z*z*(1-c)+c,   transform[2]],
             [0,             0,             0,             1]])


    def calcBorderChains(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        sumBrightness = float(sum(sum(hsv[:,:,1])))
        sumIntensity = float(sum(sum(hsv[:,:,2])))
        for pix in hsv[:,:,1]:
            pix += np.uint8(((self.averageBrightness - sumBrightness) / (480.*640.)))
        for pix in hsv[:,:,2]:
            pix += np.uint8(((self.averageIntensity - sumIntensity) / (480.*640.)))

        mask = cv2.inRange(hsv, self.lowerColor, self.upperColor)
        self.mask = mask
        mask = cv2.erode(mask, None, iterations=1) # less white
        mask = cv2.dilate(mask, None, iterations=10) # more white
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for i in range(0, len(cnts)):
            hull = cv2.convexHull(cnts[i])
            cv2.fillPoly(mask, pts= [hull], color = (255, 255, 255))

            #pt1 = hull[0]
            #for j in range(1, len(hull)):
            #    cv2.line(mask, (pt1[0][0], pt1[0][1]), (hull[j][0][0], hull[j][0][1]), (255,255,255), 1)
            #    pt1 = hull[j]
        mask = cv2.erode(mask, None, iterations = 9)

        self.maskCalculated = mask
        self.original = image

        return cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

    def plot(self):
        if self.penFound:
            if self.penInTopImage:
                print 'FOUND-TOP: '
                print 'Position in Image:  ' + str(self.penPosition)
                print 'Position from Body: ' + str(self.calculatePenToBody(0, 0))
            else:
                print 'FOUND-BOT: '
                print 'Position in Image:  ' + str(self.penPosition)
                print 'Position from Body: ' + str(self.calculatePenToBody(0, 0))
        else:
            print 'nothing found'

    #TODO plot calculated values
    def plotMask(self, imTop, imBot):
        img = array([range(960*1920*3)], dtype=np.uint8).reshape((960,1920,3))

        self.penPositionInImage(imTop)
        img[0:480,0:640,:] = imTop
        img[0:480,640:1280,0] = self.mask
        img[0:480,640:1280,1] = self.mask
        img[0:480,640:1280,2] = self.mask
        #img[0:480, 640:1280, :] = cv2.cvtColor(self.mask, cv2.COLOR_HSV2RGB)

        img[0:480,1280:,0] = self.maskCalculated
        img[0:480,1280:,1] = self.maskCalculated
        img[0:480,1280:,2] = self.maskCalculated
        if self.penFound:
            self.drawPenPositionToImage(img, (0, 0))
            self.drawPenPositionToImage(img, (640, 0))
            self.drawPenPositionToImage(img, (1280, 0))

        self.penPositionInImage(imBot)
        img[480:,0:640,:] = imBot
        img[480:,640:1280,0] = self.mask
        img[480:,640:1280,1] = self.mask
        img[480:,640:1280,2] = self.mask
        #img[480:, 640:1280, :] = cv2.cvtColor(self.mask, cv2.COLOR_HSV2RGB)

        img[480:,1280:,0] = self.maskCalculated
        img[480:,1280:,1] = self.maskCalculated
        img[480:,1280:,2] = self.maskCalculated
        if self.penFound:
            self.drawPenPositionToImage(img, (0, 480))
            self.drawPenPositionToImage(img, (640, 480))
            self.drawPenPositionToImage(img, (1280, 480))
        cv2.imshow("robot vision", img)
        cv2.waitKey(50)

    def drawPenPositionToImage(self, img, translation):
        (x1, y1) = map(add, self.center1, translation)
        (x2, y2) = map(add, self.center2, translation)
        (x3, y3) = map(add, self.borderPoint1, translation)
        (x4, y4) = map(add, self.borderPoint2, translation)

        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.circle(img, (x1, y1), int(self.width1 / 2), (255, 0, 0), 2)
        cv2.circle(img, (x2, y2), int(self.width2 / 2), (255, 0, 0), 2)

        cv2.circle(img, (x3, y3), 2, (0, 0, 255), 2)
        cv2.circle(img, (x4, y4), 2, (0, 0, 255), 2)


if __name__ == '__main__':
    Imana = ImageAnalysis()
    #for i in range(0, 1):
    #    bot = cv2.imread('/home/daniel/programming-humanoid-robot-in-python/project/testimg/outfile' + str(i) + 'Top.png', cv2.IMREAD_UNCHANGED)
    #    top = cv2.imread('/home/daniel/programming-humanoid-robot-in-python/project/testimg/outfile' + str(i) + 'Bot.png', cv2.IMREAD_UNCHANGED)
    #    bot = cv2.cvtColor(bot, cv2.COLOR_RGB2HSV)
    #    top = cv2.cvtColor(top, cv2.COLOR_RGB2HSV)
    #    print bot
    #    print '---'
    #    print bot[:,:,1]
    #    sumBrightness = sum(sum(bot[:,:,1])) + sum(sum(top[:,:,1]))
    #    sumIntensity = sum(sum(bot[:,:,2])) + sum(sum(top[:,:,2]))
    #    print sumBrightness
    #    print sumIntensity


    for i in range(1, 10):
        bot = cv2.imread('/home/daniel/programming-humanoid-robot-in-python/project/testimg/outfile' + str(i) + 'Top.png', cv2.IMREAD_UNCHANGED)
        top = cv2.imread('/home/daniel/programming-humanoid-robot-in-python/project/testimg/outfile' + str(i) + 'Bot.png', cv2.IMREAD_UNCHANGED)

        Imana.analyzeImages(top, bot)
        Imana.plot()
        Imana.plotMask(top, bot)
        cv2.waitKey()
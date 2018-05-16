
import numpy as np
import cv2
from numpy.matlib import matrix, dot, array
from numpy.linalg import norm
from math import cos, sin
from operator import add, sub

class ImageAnalysis(object):

    def __init__(self):
        # width of pen in mm
        self.pen_width = 14

        # lower and upper threshold for the color
        self.lower_color = (5, 40, 30)
        self.upper_color = (30, 255, 255)

        # average hsv - values in test - images
        self.average_h = 81.7006626491
        self.average_s = 83.959792866
        self.average_v = 221.986922586

    # gets bot and top image of the same frame
    # sets variable self.pen_in_top_image as expected
    def analyze_images(self, image_top, image_bot):
        self.pen_in_image(image_top)
        if self.pen_found:
            self.pen_in_top_image = True
            return
        self.pen_in_image(image_bot)
        if self.pen_found:
            self.pen_in_top_image = False
            return

    # calculates border chain and calls border_to_pen_position if pen found
    # returns true if pen is found
    # sets variable self.pen_found if pen is found
    def pen_in_image(self, image):
        border = self.image_to_contoures(image)
        if len(border) != 2:
            self.pen_found = False
            return
        self.pen_found = True
        self.border_to_pen_position(border)

    # calculate the width and center of the pen for given borders
    def border_to_pen_position(self, border):
        self.center_1 = self.get_center_of_border(border[0])
        self.center_2 = self.get_center_of_border(border[1])
        center = (self.center_1[0] + self.center_2[0]) / 2, (self.center_1[1] + self.center_2[1]) / 2

        self.border_point_1 = self.get_most_orthogonal_border_point(self.center_1, self.center_2, border[0])
        self.width_1 = 2 * np.linalg.norm(map(sub, self.border_point_1, self.center_1))
        self.border_point_2 = self.get_most_orthogonal_border_point(self.center_2, self.center_1, border[1])
        self.width_2 = 2 * np.linalg.norm(map(sub, self.border_point_2, self.center_2))
        width = (self.width_1 + self.width_2) / 2

        self.coordinates_of_pen_in_image(center, width)

    # transforms width and center of pen to catesian coordinates
    def coordinates_of_pen_in_image(self, center, width):
        angle_per_pixel = ((60.97 / 640) / 180) * np.pi # winkel pro pixel

        theta = center[1] * angle_per_pixel - (47.64 / 180) * np.pi / 2 + np.pi / 2  # hoehe im bild, winkel von oben
        phi = center[0] * angle_per_pixel - (60.97 / 180) * np.pi / 2  # seitliche verschiebung im bild, winkel von Mitte des Bildes
        r = (self.pen_width / 2) / np.tan(angle_per_pixel * width / 2)  # do the calculation (trigonometry)

        x = r * np.sin(theta) * np.cos(phi)  # to cartesian coordinates
        y = - r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)

        self.pen_position = (x, y, z)

    # gets border points and calculates center of them
    def get_center_of_border(self, border):
        sum_x = sum(border[:, 0, 0])
        sum_y = sum(border[:, 0, 1])
        length = len(border[:, 0, 0])
        return sum_x / length, sum_y / length

    # calculate the border point that is closest to the line
    # through center_2 and orthogonal to the connection of center_1 and center_2
    def get_most_orthogonal_border_point(self, center_1, center_2, border_1):
        center_2_normed = map(sub, center_2, center_1)
        center_2_normed = center_2_normed / np.linalg.norm(center_2_normed)
        self.pen_direction = center_2_normed

        orthogonality = [None] * len(border_1[:])
        for index, border_point_ in enumerate(border_1[:,0]):
            border_point_normed = map(sub, border_point_, center_1)
            border_point_normed = border_point_normed / np.linalg.norm(border_point_normed)
            orthogonality[index] = ((center_2_normed[0] * border_point_normed[0]) + (center_2_normed[1] * border_point_normed[1]))**2
        index = orthogonality.index(min(orthogonality))
        return border_1[index][0]

    # use pen position in image, head_yaw and head_pitch to calculate the pen position in the
    # coordinate system based at NAO's chest
    def pen_to_body_transformation(self, head_yaw, head_pitch):
        transform_to_head_yaw = self.local_transformation_matrix(head_yaw, [0, 0, 126.5], [0, 0, 1])
        transform_to_head_pitch = self.local_transformation_matrix(head_pitch, [0, 0, 0], [0, 1, 0])
        if self.pen_in_top_image:
            transform_to_cam = self.local_transformation_matrix(0.0209, [58.71, 0, 63.64], [0, 1, 0])
        else:
            transform_to_cam = self.local_transformation_matrix(0.6929, [50.71, 0, 17.74], [0, 1, 0])
        transform_to_image = self.local_transformation_matrix(0, self.pen_position, [0, 1, 0])

        M = dot(dot(dot(transform_to_head_yaw, transform_to_head_pitch), transform_to_cam), transform_to_image)
        return M[0, 3], M[1, 3], M[2,3]

    # calculate quaternion-matrix
    def local_transformation_matrix(self, angle, transform, direction):
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

    # extract the contours of the two orange endings of the pen.
    def image_to_contoures(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(hsv)
        sum_s = np.matrix(s).mean()
        sum_v = np.matrix(v).mean()
        hsv = self.calibrate_brightness_and_intensity(hsv, int(float(sum_s) - float(sum_s)), int(float(self.average_v) - float(sum_v)))
        self.the_original = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        self.mask = mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=10)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for i in range(0, len(cnts)):
            hull = cv2.convexHull(cnts[i])
            cv2.fillPoly(mask, pts=[hull], color=(255, 255, 255))

        mask = cv2.erode(mask, None, iterations = 8)

        self.mask_calculated = mask
        self.original = image

        return cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

    # plot information on the current image analysis to console
    def plot(self):
        if self.pen_found:
            if self.pen_in_top_image:
                print('FOUND-TOP: ')
                print('Position in Image:  ' + str(self.pen_position))
                print('Position from Body: ' + str(self.pen_to_body_transformation(0, 0)))
            else:
                print('FOUND-BOT: ')
                print('Position in Image:  ' + str(self.pen_position))
                print('Position from Body: ' + str(self.pen_to_body_transformation(0, 0)))
        else:
            print('nothing found')

    # show information on the current image analysis as pictures.
    # CAUTION: class variables like self.pen_position might differ after a call to this method
    def plot_images(self, im_top, im_bot):
        img = array([range(960*1920*3)], dtype=np.uint8).reshape((960, 1920, 3))

        self.pen_in_image(im_top)
        img[0:480, 0:640, :] = self.the_original
        img[0:480, 640:1280, 0] = self.mask
        img[0:480, 640:1280, 1] = self.mask
        img[0:480, 640:1280, 2] = self.mask
        #img[0:480,  640:1280, :] = cv2.cvtColor(self.mask, cv2.COLOR_HSV2RGB)

        img[0:480, 1280:, 0] = self.mask_calculated
        img[0:480, 1280:, 1] = self.mask_calculated
        img[0:480, 1280:, 2] = self.mask_calculated
        if self.pen_found:
            self.draw_pen_position_to_image(img, (0, 0))
            self.draw_pen_position_to_image(img, (640, 0))
            self.draw_pen_position_to_image(img, (1280, 0))

        self.pen_in_image(im_bot)
        img[480:, 0:640, :] = self.the_original
        img[480:, 640:1280, 0] = self.mask
        img[480:, 640:1280, 1] = self.mask
        img[480:, 640:1280, 2] = self.mask
        #img[480:, 640:1280, :] = cv2.cvtColor(self.mask, cv2.COLOR_HSV2RGB)

        img[480:, 1280:, 0] = self.mask_calculated
        img[480:, 1280:, 1] = self.mask_calculated
        img[480:, 1280:, 2] = self.mask_calculated
        if self.pen_found:
            self.draw_pen_position_to_image(img, (0, 480))
            self.draw_pen_position_to_image(img, (640, 480))
            self.draw_pen_position_to_image(img, (1280, 480))
        cv2.imshow("robot vision", img)
        #cv2.waitKey(50)

    def draw_pen_position_to_image(self, img, translation):
        (x1, y1) = map(add, self.center_1, translation)
        (x2, y2) = map(add, self.center_2, translation)
        (x3, y3) = map(add, self.border_point_1, translation)
        (x4, y4) = map(add, self.border_point_2, translation)

        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.circle(img, (x1, y1), int(self.width_1 / 2), (255, 0, 0), 2)
        cv2.circle(img, (x2, y2), int(self.width_2 / 2), (255, 0, 0), 2)

        cv2.circle(img, (x3, y3), 2, (0, 0, 255), 2)
        cv2.circle(img, (x4, y4), 2, (0, 0, 255), 2)

    # calibrate the average brightness and intensity to match the average values of the test images
    def calibrate_brightness_and_intensity(self, hsv_img, s_change, v_change):
        h, s, v = cv2.split(hsv_img)
        if v_change >= 0:
            lim = 255 - v_change
            v[v > lim] = 255
            v[v <= lim] += v_change
        else:
            lim = 0 - v_change
            v[v < lim] = 0
            v[v >= lim] -= -v_change
        if s_change >= 0:
            lim = 255 - s_change
            s[s > lim] = 255
            s[s <= lim] += s_change
        else:
            lim = 0 - s_change
            s[s < lim] = 0
            s[s >= lim] -= -s_change

        return cv2.merge((h, s, v))

    def calculate_average_hsv_values_of_test_images(self):
        h_array = []
        s_array = []
        v_array = []
        for i in range(0, 38):
            bot = cv2.imread('/home/daniel/programming-humanoid-robot-in-python/project/testimg/outfile' + str(i) + 'Top.png', cv2.IMREAD_UNCHANGED)
            top = cv2.imread('/home/daniel/programming-humanoid-robot-in-python/project/testimg/outfile' + str(i) + 'Bot.png', cv2.IMREAD_UNCHANGED)

            h, s, v = cv2.split(cv2.cvtColor(bot, cv2.COLOR_BGR2HSV))
            h_array.append(np.matrix(h).mean())
            s_array.append(np.matrix(s).mean())
            v_array.append(np.matrix(v).mean())

            h, s, v = cv2.split(cv2.cvtColor(top, cv2.COLOR_BGR2HSV))
            h_array.append(np.matrix(h).mean())
            s_array.append(np.matrix(s).mean())
            v_array.append(np.matrix(v).mean())

        print(np.matrix(h_array).mean())
        print(np.matrix(s_array).mean())
        print(np.matrix(v_array).mean())

# analyze all test images
if __name__ == '__main__':
    Imana = ImageAnalysis()
    #Imana.calculate_average_hsv_values_of_test_images()

    for i in range(0, 38):
        bot = cv2.imread('/home/daniel/programming-humanoid-robot-in-python/project/testimg/outfile' + str(i) + 'Top.png', cv2.IMREAD_UNCHANGED)
        top = cv2.imread('/home/daniel/programming-humanoid-robot-in-python/project/testimg/outfile' + str(i) + 'Bot.png', cv2.IMREAD_UNCHANGED)

        Imana.analyze_images(top, bot)
        Imana.plot()
        Imana.plot_mask(top, bot)
        cv2.waitKey()
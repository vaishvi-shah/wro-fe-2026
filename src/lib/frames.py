import cv2
import time

class Frame:
    def __init__(self, img, x1, x2, y1, y2,low,high, second_low=None, second_high=None):
        self.img = img
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.low = low
        self.high = high
        self.second_low = second_low
        self.second_high = second_high
        self.frame = 0
        self.mask = 0
        self.hsv = 0
        self.frame_gaussed = 0
        self.contours = None
        self.last_seen = time.time()
        self.last_seen_timer = 2
        self.line_counter1 = 0
        self.line_counter2 = 0
        self.update(img)

    def update(self, img):
        cv2.rectangle(img, (self.x1, self.y1), (self.x2, self.y2), (255, 255, 255), 1)
        self.frame = img[self.y1:self.y2, self.x1:self.x2]
        self.frame_gaussed = cv2.GaussianBlur(self.frame, (1, 1), cv2.BORDER_DEFAULT)  # blurring the image
        self.hsv = cv2.cvtColor(self.frame_gaussed, cv2.COLOR_BGR2HSV)


    def find_contours(self, is_red=False, colour=(0,0,255), colour2=(0,255,0)):
        self.mask = cv2.inRange(self.hsv, self.low[0], self.high[0])
        contours2 = None        
        if is_red:
            mask1 = cv2.inRange(self.hsv, self.low[1], self.high[1])
            self.mask = cv2.bitwise_or(self.mask, mask1)
        self.contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if self.contours:
            cv2.drawContours(self.frame, self.contours, -1, colour, 2)
        if self.second_low is not None and self.second_high is not None:
            mask2 = cv2.inRange(self.hsv, self.second_low[0], self.second_high[0])
            contours2, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            if contours2:
                cv2.drawContours(self.frame, contours2, -1, colour2, 2)
            return self.contours, contours2
        return self.contours
        
    def add_lines(self, colour=1):
        if time.time() - self.last_seen > self.last_seen_timer:
            if colour == 1:
                self.line_counter1 += 1
            elif colour == 2:
                self.line_counter2 += 1
            self.last_seen = time.time()
            
    def get_line_count(self, number=1):
        if number == 1:
            return self.line_counter1
        elif number == 2:
            return self.line_counter2
        return 0

    def get_areas(self, contours = 0, contours2 = 0):
        if contours != 0:
            areas = [cv2.contourArea(cnt) for cnt in contours]
            contours1 = sum(areas)
        if contours2 != 0:
            areas2 = [cv2.contourArea(cnt) for cnt in contours2]
            contours2 = sum(areas2)
        biggest_contour = max(contours1, contours2)
        if biggest_contour == 0:
            colour = None
        elif biggest_contour == contours1:
            colour = 1
        elif biggest_contour == contours2:
            colour = 2
        return biggest_contour,colour


import cv2
import time


class Frame:
    def __init__(self, img, x1, x2, y1, y2, colour_range):
        """
        colour_range: FLAT list of [low, high] np.array pairs, one pair per
        colour, e.g.
            black_range  = [[low, high]]                  # 1 colour
            multi_range  = blue_range + orange_range       # 2 colours (concat!)
            red_range    = [[low1, high1], [low2, high2]]  # 1 colour, 2 ranges (is_red=True)
        """
        self.img = img
        self.x1, self.x2, self.y1, self.y2 = x1, x2, y1, y2

        self.low = []
        self.high = []
        for i, pair in enumerate(colour_range):
            if len(pair) != 2:
                raise ValueError(
                    f"colour_range[{i}] must be [low, high], got {pair!r} "
                    "(check you didn't double-nest a range list)"
                )
            self.low.append(pair[0])
            self.high.append(pair[1])

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
        self.frame_gaussed = cv2.GaussianBlur(self.frame, (1, 1), cv2.BORDER_DEFAULT)
        self.hsv = cv2.cvtColor(self.frame_gaussed, cv2.COLOR_BGR2HSV)

    def find_contours(self, is_red=False, colour=(0, 0, 255), colour2=(0, 255, 0)):
        """
        Primary colour = index 0 (or indices 0+1 merged if is_red=True, same
        as before). Every remaining colour in self.low/self.high is treated
        as an "other" colour and returned as a list, in order.

        colour2 can be a single BGR tuple (all extra colours drawn the same)
        or a list of tuples, one per extra colour.
        """
        self.mask = cv2.inRange(self.hsv, self.low[0], self.high[0])
        start_idx = 1
        if is_red:
            mask1 = cv2.inRange(self.hsv, self.low[1], self.high[1])
            self.mask = cv2.bitwise_or(self.mask, mask1)
            start_idx = 2

        self.contours, _ = cv2.findContours(
            self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
        )
        if self.contours:
            cv2.drawContours(self.frame, self.contours, -1, colour, 2)

        draw_colours = colour2 if isinstance(colour2, list) else [colour2]

        other_contours = []
        for j, x in enumerate(range(start_idx, len(self.low))):
            mask2 = cv2.inRange(self.hsv, self.low[x], self.high[x])
            contours2, _ = cv2.findContours(
                mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
            )
            other_contours.append(contours2)
            if contours2:
                cv2.drawContours(
                    self.frame, contours2, -1, draw_colours[j % len(draw_colours)], 2
                )

        return self.contours, other_contours

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

    def get_areas(self, contours=(), contours2=()):
        area1 = sum(cv2.contourArea(cnt) for cnt in contours) if contours else 0
        area2 = sum(cv2.contourArea(cnt) for cnt in contours2) if contours2 else 0

        biggest = max(area1, area2)
        if biggest == 0:
            colour = None
        elif biggest == area1:
            colour = 1
        else:
            colour = 2
        return biggest, colour

import cv2
import time


class Frame:
    def __init__(self, img, x1, x2, y1, y2, colour_range):
        """
        colour_range: list of colour GROUPS. Each group is a list of one or
        more [low, high] np.array range pairs that all represent the SAME
        colour (multiple ranges per group are OR'd together — this is how
        hue-wraparound colours like red are handled, no special-casing needed).

        Examples:
            black_range = [[low, high]]                 # 1 range
            colour_range=[black_range]                   # 1 group, 1 colour

            colour_range=[blue_range, orange_range]      # 2 groups, 2 colours

            red_range = red1_range + red2_range           # merge two ranges
            colour_range=[red_range, green_range, black_range]  # 3 groups
        """
        self.img = img
        self.x1, self.x2, self.y1, self.y2 = x1, x2, y1, y2

        self.groups = []
        for gi, group in enumerate(colour_range):
            pairs = []
            for ri, pair in enumerate(group):
                if len(pair) != 2:
                    raise ValueError(
                        f"colour_range[{gi}][{ri}] must be [low, high], got "
                        f"{pair!r} — check your range isn't double/under-nested"
                    )
                pairs.append((pair[0], pair[1]))
            self.groups.append(pairs)

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
        self.frame = img[self.y1:self.y2, self.x1:self.x2]
        self.frame_gaussed = cv2.GaussianBlur(self.frame, (1, 1), cv2.BORDER_DEFAULT)
        self.hsv = cv2.cvtColor(self.frame_gaussed, cv2.COLOR_BGR2HSV)

    def draw_roi(self, img):
        """
        Draws this ROI's debug border onto img. Deliberately separate from
        update(): update() used to draw this border directly onto the shared
        capture frame, which -- since multiple Frames crop from that same
        array -- baked one Frame's border line into pixels another Frame
        (e.g. a wider obstacle-detection ROI containing this one) would crop
        and colour-mask afterward, slicing anything that straddled the
        border into two separate contours. Call this only after every
        Frame's update()/detection has run for the current loop iteration.
        """
        cv2.rectangle(img, (self.x1, self.y1), (self.x2, self.y2), (255, 255, 255), 1)

    def find_contours(self):
        """
        Finds contours for EVERY colour group given at init, in order.
        Each group's ranges are OR'd into one mask before finding contours
        (this is what makes red's two hue ranges act as a single colour).

        All detected contours are drawn in yellow.

        Returns:
            - a single contours list if there's only 1 group (e.g. left/right walls)
            - a tuple of contours lists, one per group, if there are 2+ groups
            (so `a, b = frame.find_contours()` / `a, b, c = ...` just works)
        """
        results = []

        for gi, pairs in enumerate(self.groups):
            mask = None

            for low, high in pairs:
                m = cv2.inRange(self.hsv, low, high)
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            if gi == 0:
                self.mask = mask
                self.contours = contours

            if contours:
                cv2.drawContours(self.frame, contours, -1, (0, 255, 255), 1)  # Yellow (BGR)

            results.append(contours)

        return results[0] if len(results) == 1 else tuple(results)

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

    def scan_ahead(self, img):
        """
        Looks at what's currently in this ROI and records how many obstacles are
        ahead (0, 1, or 2), each one's colour, and its position. Assumes group 0
        is RED and group 1 is GREEN (this Frame's obstacle-colour setup). Meant to
        be called only at startup and right after a turn completes -- NOT every
        loop iteration -- so the result stays fixed while avoidance/wall-following
        logic keeps grabbing frames where an obstacle can drop in and out of view.
        """
        self.update(img)
        red_contours, green_contours = self.find_contours()

        found = (
            [(c, "RED") for c in red_contours if cv2.contourArea(c) > 5] +
            [(c, "GREEN") for c in green_contours if cv2.contourArea(c) > 5]
        )
        found.sort(key=lambda item: item[0][:, :, 1].max(), reverse=True)

        obstacles = []
        for c, colour in found[:2]:
            pts = c.reshape(-1, 2)
            if colour == "GREEN":
                rel_x, rel_y = pts[pts[:, 0].argmin()]
            else:
                rel_x, rel_y = pts[pts[:, 0].argmax()]
            obstacles.append({
                "colour": colour,
                "x": int(rel_x) + self.x1,
                "y": int(rel_y) + self.y1,
            })

        print(f"AHEAD SCAN: {len(obstacles)} obstacle(s) -- {obstacles}")
        return obstacles

    def get_areas(self, *contour_sets):
        """
        Accepts however many contour sets you have (one per colour group)
        and returns (biggest_total_area, index_of_biggest) where index is
        1-based (1 = first set passed in, 2 = second, ...), matching the
        old 1=blue/2=orange, 1=red/2=green/3=black conventions.
        Returns (0, None) if everything is empty/zero.
        """
        areas = [sum(cv2.contourArea(c) for c in contours) if contours else 0
                  for contours in contour_sets]

        if not areas:
            return 0, None

        biggest = max(areas)
        colour = None if biggest == 0 else areas.index(biggest) + 1
        return biggest, colour

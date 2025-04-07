class DetectedObject:
    def __init__(self, class_label, confidence, x_min, y_min, x_max, y_max):
        self.class_label = class_label
        self.confidence = confidence
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max

    @property
    def center(self):
        center_x = (self.x_min + self.x_max) / 2
        center_y = (self.y_min + self.y_max) / 2
        return (center_x, center_y)

    @property
    def width(self):
        return self.x_max - self.x_min

    @property
    def height(self):
        return self.y_max - self.y_min

    @property
    def area(self):
        return self.width * self.height

    def __str__(self):
        return (f"Class: {self.class_label}, Confidence: {self.confidence:.2f}, "
                f"BBox: [x_min: {self.x_min}, y_min: {self.y_min}, x_max: {self.x_max}, y_max: {self.y_max}], "
                f"Center: {self.center}, Area: {self.area}")

    def send_movement_command(self):
        return
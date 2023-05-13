class Rect:
    def __init__(self, x, y, w, h):
        self.x, self.y, self.w, self.h = x, y, w, h

    def move(self, x, y):
        self.x, self.y = self.x + x, self.y + y

    def scale(self, factor, dx, dy):
        self.w, self.h = self.w * factor, self.h * factor
        self.x, self.y = self.x + (1 - factor) * dx, self.y + (1 - factor) * dy

    @property
    def center(self):
        return self.x + self.w * .5, self.y + self.h * .5

    @center.setter
    def center(self, value):
        self.x, self.y = value[0] - self.w * .5, value[1] - self.h * .5

    def clamp(self, other):
        c = other.center
        if self.w > other.w:
            self.x = c[0] - self.w * .5
        else:
            self.x = min(max(self.x, other.x), other.x + other.w - self.w)
        if self.h > other.h:
            self.y = c[1] - self.h * .5
        else:
            self.y = min(max(self.y, other.y), other.y + other.h - self.h)

    def __repr__(self):
        return f'<Rect : {self.x:.03f}, {self.y:.03f}, {self.w:.03f}, {self.h:.03f}>'

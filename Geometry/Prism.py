from Geometry.Rectangle import Rectangle

class Prism(Rectangle):

    def __init__(self, x, y, width, height, alt):
        Rectangle.__init__(self, x, y, width, height)
        self.alt = alt

    @classmethod
    def init_with_rect(cls, rect, alt):
        return Prism(rect.x, rect.y, rect.width, rect.height, alt)

    def center_flat(self):
        return super.center()

    def center(self):
        xy_center = super.center()
        return numpy.array([xy_center[0], xy_center[1], self.alt/2.0])

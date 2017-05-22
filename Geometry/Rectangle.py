import numpy

class Rectangle:
    '''where x and y are in the top left corner'''
    def __init__(self, x, y, width, height):
        self.x = x
        self.y  = y
        self.width = width
        self.height = height

    def shift_border(self, border):
        self.x -= border
        self.y += border
        self.width += border * 2
        self.height += border * 2

    def center(self):
        return numpy.array([self.x+self.width/2.0, self.y-self.height/2.0])

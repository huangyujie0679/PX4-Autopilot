import math


class Straight:

    def update(self, t):

        x = 50

        y = 0

        z = 10

        return x, y, z



class Circle:

    def update(self, t):

        r = 10

        w = 0.2

        x = 50

        y = r * math.sin(w * t)

        z = 10

        return x, y, z

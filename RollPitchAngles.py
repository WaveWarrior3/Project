from math import atan

# h1 is height of left actuator, h2 is height of right actuator, d is distance between actuators, l is distance from hitch to actuators (all floats)
def rollAngle(h1, h2, d):
    return atan((h1-h2)/d)


def PitchAngle(h1, h2, l):
    return atan((h1+h2)/(2*l))
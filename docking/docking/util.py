import math

def is_between(val, min, max):
    return min < val and val < max

def is_between_symm(val, cap):
    return is_between(val, -abs(cap), abs(cap))
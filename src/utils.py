import math

def norm(vector):
    return math.sqrt(sum(component**2 for component in vector))

def clip(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def subtract(v1, v2):
    return[
        v1_component - v2_component
        for v1_component, v2_component in zip(v1,v2)
    ]
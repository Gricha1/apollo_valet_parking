import math
import numpy as np
import matplotlib.pylab as plt

def separatingAxes(a, axes):
    for i in range(len(a)):
        current = a[i]
        next = a[(i + 1) % len(a)]
        edge = np.array(next) - np.array(current) 
        new_edge = edge / (np.sqrt(np.sum(edge ** 2)) + 1e-6)
        # print(f"new_edge : {new_edge}")
        axes.append([-new_edge[1], new_edge[0]])

def project(a, axis):
    maxProj = -math.inf
    minProj = math.inf
    for v in a:
        proj = np.dot(axis, v)
        if proj < minProj:
            minProj = proj
        if proj > maxProj:
            maxProj = proj
    
    return minProj, maxProj

def intersect(a, b):
    axes = []
    separatingAxes(a, axes)
    separatingAxes(b, axes)
    for axis in axes:
        aMinProj, aMaxProj, bMinProj, bMaxProj = 0., 0., 0., 0.
        aMinProj, aMaxProj = project(a, axis)
        bMinProj, bMaxProj = project(b, axis)
        if (aMinProj > bMaxProj) or (bMinProj > aMaxProj):
            return False 
    return True

def intersectPoint(point, polygon, epsilon=1e-4):
    result = False
    i = 0
    j = len(polygon) - 1
    for i in range(len(polygon)):
        if ((polygon[i][1] > point[1]) != (polygon[j][1] > point[1]) and (point[0] < (polygon[j][0]\
            - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1] + epsilon) + polygon[i][0])):
            result = not result
        j = i
        i += 1

    return result

# a = [[0, 0], [2, 0], [2, 2], [0, 2]]
# b = [[1, 1], [3, 0], [3, 5], [1, 6]]
# c = [[0, 3], [2, 3], [2, 6], [0, 7]]

# print("intersect(a, b): ", intersect(a, b))
# print("intersect(b, c): ", intersect(b, c))
# print("intersect(a, c): ", intersect(a, c))

# points = [[0, 0], [0.5, 0.5], [1.5, 1.5], [-1, 0], [1, 5]]
# for point in points:
#     print("point:", point, " ", intersectPoint(point, a))
#     plt.plot(point[0], point[1], "rH")
# plt.plot([a[(i + 1) % len(a)][0] for i in range(len(a) + 1)], [a[(i + 1) % len(a)][1] for i in range(len(a) + 1)])
# plt.plot([b[(i + 1) % len(b)][0] for i in range(len(b) + 1)], [b[(i + 1) % len(b)][1] for i in range(len(b) + 1)])
# plt.plot([c[(i + 1) % len(c)][0] for i in range(len(c) + 1)], [c[(i + 1) % len(c)][1] for i in range(len(c) + 1)])
# plt.show()
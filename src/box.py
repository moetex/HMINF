import numpy as np

class Box:
    def __init__(self, vertices):
        self.vertices = np.array(vertices, dtype=float)
        if self.vertices.shape != (8, 3):
            raise ValueError("vertices must be an 8x3 array")

    @property
    def min(self):
        return np.min(self.vertices, axis=0)

    @property
    def max(self):
        return np.max(self.vertices, axis=0)

    @property
    def center(self):
        return np.mean(self.vertices, axis=0)

    @property
    def size(self):
        return self.max - self.min

    def check_collision(self, position, radius):
        offsets = np.zeros(3)
        for i in range(3):
            if position[i] + radius > self.max[i]:
                offsets[i] = self.max[i] - (position[i] + radius)
            elif position[i] - radius < self.min[i]:
                offsets[i] = self.min[i] - (position[i] - radius)
        return offsets



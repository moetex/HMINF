import numpy as np


class Quaternion:

    @staticmethod
    def normalize(q: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(q)
        if n < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return q / n

    @staticmethod
    def mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:  # Quaternionen multiplikation
        # (w,x,y,z)
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ], dtype=float)

    @staticmethod
    def from_omega(omega: np.ndarray, dt: float) -> np.ndarray:
        # kleine winkel Quaternion aus Winkelgeschwindigkeit
        ang = np.linalg.norm(omega)
        if ang < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        axis = omega / ang
        half = 0.5 * ang * dt
        return np.array([np.cos(half), *(np.sin(half) * axis)], dtype=float)

    @staticmethod
    def to_R(q: np.ndarray) -> np.ndarray:
        # quat = (w,x,y,z)
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]
        ], dtype=float)


class Transform:
    @staticmethod
    def pose(x: np.ndarray, q: np.ndarray) -> np.ndarray:
        T = np.eye(4, dtype=float)
        T[:3, :3] = Quaternion.to_R(q)
        T[:3, 3] = x
        return T
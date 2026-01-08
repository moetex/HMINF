import numpy as np
from src.model.colliders import ColliderHandle
from src.model.rigidBody import RigidBody

from distance3d import colliders


class BoxWorld:
    def __init__(self, half_size: float, wall_thickness: float, wall_restitution=0.9):
        self.half = float(half_size)
        self.t = float(wall_thickness)
        self.wall_restitution = float(wall_restitution)
        self.walls: list[RigidBody] = self._make_walls()

    def _make_walls(self) -> list[RigidBody]:
        hs = self.half
        t = self.t
        big = 2 * hs + 2 * t

        def wall_at(x, size):
            handle = ColliderHandle(lambda T, s=np.array(size, float): colliders.Box(T, s))
            return RigidBody(
                mass=0.0, inertia_body=np.eye(3),
                x=x, q=[1, 0, 0, 0],
                v=[0, 0, 0], w=[0, 0, 0],
                restitution=self.wall_restitution,
                collider=handle,
                visual=None
            )

        walls = []
        # +X, -X
        walls.append(wall_at([hs + t / 2, 0, 0],  [t, big, big]))
        walls.append(wall_at([-hs - t / 2, 0, 0],  [t, big, big]))

        # +Y, -Y
        walls.append(wall_at([0, hs + t / 2, 0],  [big, t, big]))
        walls.append(wall_at([0, -hs - t / 2, 0],  [big, t, big]))

        # +Z, -Z
        walls.append(wall_at([0, 0, hs + t / 2],  [big, big, t]))
        walls.append(wall_at([0, 0, -hs - t / 2],  [big, big, t]))

        return walls
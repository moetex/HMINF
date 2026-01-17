import numpy as np
from vpython import color

from src.model.colliders import ColliderHandle
from src.model.rigidBody import RigidBody

from distance3d import colliders

from src.view.visualsVPython import VpythonBoxVisual


class BoxWorld:
    def __init__(self, half_size: float, wall_thickness: float, wall_material):
        self.half = float(half_size)
        self.t = float(wall_thickness)
        self.wall_material = wall_material
        self.walls: list[RigidBody] = self._make_walls()


    def _static_wall(self, center_xyz, size_xyz):
        size_xyz = np.asarray(size_xyz, dtype=float)
        collider_handle = ColliderHandle(lambda T, s=size_xyz: colliders.Box(T, s))

        wall = RigidBody(
            material=self.wall_material,
            shape="box",
            size_xyz_m=size_xyz,  #
            is_static=True,
            x=center_xyz,
            q=[1, 0, 0, 0],
            v=[0, 0, 0],
            w=[0, 0, 0],
            collider=collider_handle,
            visual=None
        )
        return wall


    def _make_walls(self) -> list[RigidBody]:
        hs = self.half
        t = self.t
        big = 2 * hs + 2 * t    # Außenmaß der Wand-Boxen


        walls = []
        # +X, -X
        walls.append(self._static_wall([hs + t / 2, 0, 0],  [t, big, big]))
        walls.append(self._static_wall([-hs - t / 2, 0, 0],  [t, big, big]))

        # +Y, -Y
        walls.append(self._static_wall([0, hs + t / 2, 0],  [big, t, big]))
        walls.append(self._static_wall([0, -hs - t / 2, 0],  [big, t, big]))

        # +Z, -Z
        walls.append(self._static_wall([0, 0, hs + t / 2],  [big, big, t]))
        walls.append(self._static_wall([0, 0, -hs - t / 2],  [big, big, t]))

        return walls
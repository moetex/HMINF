import numpy as np

from vpython import (
    canvas, vector, vertex, triangle, rate, color,
    box as vp_box, curve
)


from src.model import stlMesh
from src.res.MathHelpers import Quaternion


class VpythonTriangleMeshView:
    """Renders a triangle mesh (V, F). Updates by applying R,p to base vertices."""
    def __init__(self, mesh: stlMesh, mesh_color=color.gray(0.75)):
        self.base = mesh.V.copy()
        self.faces = mesh.F
        self.vtx = [vertex(pos=vector(*p), color=mesh_color) for p in self.base]
        for (i, j, k) in self.faces:
            triangle(v0=self.vtx[i], v1=self.vtx[j], v2=self.vtx[k])

    def sync(self, x: np.ndarray, q: np.ndarray):
        R = Quaternion.to_R(q)
        for i in range(self.base.shape[0]):
            p = R @ self.base[i] + x
            self.vtx[i].pos = vector(p[0], p[1], p[2])


class VpythonBoxVisual:
    def __init__(self, size_xyz: np.ndarray, color_=color.orange, opacity=0.9):
        self.size = np.asarray(size_xyz, dtype=float)
        self.obj = vp_box(
            pos=vector(0, 0, 0),
            size=vector(self.size[0], self.size[1], self.size[2]),
            color=color_,
            opacity=opacity
        )

    def sync(self, x: np.ndarray, q: np.ndarray):
        R = Quaternion.to_R(q)
        self.obj.pos = vector(x[0], x[1], x[2])

        ax = R[:, 0]  # Lokale +X Achse
        self.obj.axis = vector(ax[0], ax[1], ax[2]) * float(self.size[0])

        up = R[:, 1]
        self.obj.up = vector(up[0], up[1], up[2])

class BoxWireframe:
    @staticmethod
    def draw(half_size: float):
        hs = float(half_size)
        corners = [
            vector(-hs, -hs, -hs), vector(hs, -hs, -hs),
            vector(hs, hs, -hs), vector(-hs, hs, -hs),
            vector(-hs, -hs, hs), vector(hs, -hs, hs),
            vector(hs, hs, hs), vector(-hs, hs, hs),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        for i, j in edges:
            curve(pos=[corners[i], corners[j]], radius=hs * 0.003, color=color.gray(0.7))

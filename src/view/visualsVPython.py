import numpy as np

from vpython import (
    canvas, vector, vertex, triangle, sphere, arrow, rate, color,
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

class ContactDebugView:
    """
    Zeichnet Kontaktpunkte + Normalen als Vektor
    """
    def __init__(self, max_points=64, point_radius=0.006, normal_scale=0.08):
        self.max_points = int(max_points)
        self.point_radius = float(point_radius)
        self.normal_scale = float(normal_scale)
        self.enable = True

        self._points = []
        self._normals = []
        for _ in range(self.max_points):
            s = sphere(radius=self.point_radius, color=color.red, visible=False)
            a = arrow(color=color.cyan, schaftwidth=self.point_radius*0.8, visible=False)
            self._points.append(s)
            self._normals.append(a)

        self._count = 0

    def set_visible(self, visible: bool):
        self.enable = bool(visible)
        if not self.enable:
            self.clear()

    def clear(self):
        for i in range(self._count):
            self._points[i].visible = False
            self._normals[i].visible = False
        self._count = 0

    def add(self, p_world, n_world):
        if self._count >= self.max_points:
            return

        p = np.asarray(p_world, dtype=float)
        n = np.asarray(n_world, dtype=float)
        nn = np.linalg.norm(n)
        if nn < 1e-12:
            return
        n = n / nn

        s = self._points[self._count]
        a = self._normals[self._count]

        s.pos = vector(p[0], p[1], p[2])
        s.visible = True

        a.pos = vector(p[0], p[1], p[2])
        a.axis = vector(n[0], n[1], n[2]) * self.normal_scale
        #a.visible = True
        a.visible = False

        self._count += 1

    def reset(self):
        self.clear()


class BodyFrameDebugView:
    def __init__(self, axis_len=0.08, show_v_dir=True, show_w_dir=True, v_len=0.06, w_len=0.05, min_dir_mag=1e-6, show_trail=True, trail_max=400):
        self.axis_len = float(axis_len)

        self.show_v_dir = bool(show_v_dir)
        self.show_w_dir = bool(show_w_dir)
        self.v_len = float(v_len)
        self.w_len = float(w_len)
        self.min_dir_mag = float(min_dir_mag)

        #self.enable = True
        self.gizmo_enable = True
        self.trail_enable = bool(show_trail)

        # Lokale Achsen
        self.ax_x = arrow(color=color.red, schaftwidth=0.006)
        self.ax_y = arrow(color=color.green, schaftwidth=0.006)
        self.ax_z = arrow(color=color.blue, schaftwidth=0.006)

        self.vel = arrow(color=color.orange, schaftwidth=0.006, visible=self.show_v_dir)
        self.omg = arrow(color=color.magenta, schaftwidth=0.006, visible=self.show_w_dir)

        # Trail
        #self.show_trail = bool(show_trail)
        self.trail_max = int(trail_max)
        self._trail_points = []

        self.trail = curve(color=color.gray(0.4)) if show_trail else None

        self.set_trail_visible(True)
        self.set_trail_visible(self.trail_enable)


    def set_gizmo_visible(self, visible: bool):
        self.gizmo_enable = bool(visible)
        self.ax_x.visible = self.gizmo_enable
        self.ax_y.visible = self.gizmo_enable
        self.ax_z.visible = self.gizmo_enable
        self.vel.visible = self.gizmo_enable and self.show_v_dir
        self.omg.visible = self.gizmo_enable and self.show_w_dir
        #if self.trail is not None:
        #    self.trail.visible = self.enable and self.show_trail

    def set_trail_visible(self, visible: bool):
        self.trail_enable = bool(visible)
        if self.trail is not None:
            self.trail.visible = self.trail_enable

    def reset_trail(self):
        self._trail_points.clear()
        if self.trail is not None:
            self.trail.visible = False
            self.trail = curve(color=color.gray(0.4))
            self.trail.visible = self.trail_enable



    def _rebuild_trail(self):
        if self.trail is None:
            return

        self.trail.visible = False
        self.trail = curve(color=color.gray(0.4))
        for p in self._trail_points:
            self.trail.append(p)

    def _set_fixed_arrows(self, arr, origin, vec_np, fixed_len):
        mag = float(np.linalg.norm(vec_np))
        if mag < self.min_dir_mag:
            arr.visible = False
            return
        d = vec_np / mag
        arr.pos = origin
        arr.axis = vector(d[0], d[1], d[2]) * float(fixed_len)
        arr.visible = True

    def sync(self, body):
        #if not self.enable:
        #    return

        x = np.asarray(body.x, dtype=float)
        R = body.R()
        p = vector(x[0], x[1], x[2])

        if self.gizmo_enable:
            ex = R[:, 0]; ey = R[:, 1]; ez = R[:, 2]
            self.ax_x.pos = p; self.ax_x.axis = vector(ex[0], ex[1], ex[2]) * self.axis_len
            self.ax_y.pos = p; self.ax_y.axis = vector(ey[0], ey[1], ey[2]) * self.axis_len
            self.ax_z.pos = p; self.ax_z.axis = vector(ez[0], ez[1], ez[2]) * self.axis_len

            if self.show_v_dir:
                v = np.asarray(body.v, float)
                self._set_fixed_arrows(self.vel, p, v, self.v_len)

            if self.show_w_dir:
                w = np.asarray(body.w, float)
                self._set_fixed_arrows(self.omg, p, w, self.w_len)

        #v = np.asarray(body.v, dtype=float)
        #w = np.asarray(body.w, dtype=float)
        #self.vel.pos = p; self.vel.axis = vector(v[0], v[1], v[2]) * self.vec_scale_v
        #self.omg.pos = p; self.omg.axis = vector(w[0], w[1], w[2]) * self.vec_scale_w

        # Trail
        if self.trail is not None and self.trail_enable:
            self._trail_points.append(p)
            if len(self._trail_points) > self.trail_max:
                self._trail_points.pop(0)

                # rebuild
                self.trail.visible = False
                self.trail = curve(color=color.gray(0.4))
                #self._rebuild_trail()
                for pt in self._trail_points:
                    self.trail.append(pt)
                self.trail.visible = True
            else:
                self.trail.append(p)



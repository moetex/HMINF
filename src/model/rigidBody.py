import numpy as np

from src.model.colliders import ColliderHandle
from src.res.MathHelpers import Quaternion
from src.res.MathHelpers import Transform


class RigidBody:
    """Vollständiger Starrkörper"""

    def __init__(self, mass, inertia_body,
                 x, q, v, w,
                 restitution,
                 collider: ColliderHandle,
                 visual=None):
        self.mass = float(mass)
        self.inv_mass = 0.0 if self.mass <= 0 else 1.0 / self.mass

        # inertia in BODY frame (3x3), and inverse
        self.Ib = np.array(inertia_body, dtype=float)
        if self.inv_mass == 0:
            self.inv_Ib = np.zeros((3, 3), dtype=float)
        else:
            self.inv_Ib = np.linalg.inv(self.Ib)

        self.x = np.array(x, dtype=float)
        self.q = Quaternion.normalize(np.array(q, dtype=float))
        self.v = np.array(v, dtype=float)
        self.w = np.array(w, dtype=float)

        self.restitution = float(restitution)

        #self.collider_factory = collider_factory
        #self.collider = None

        self.collider_handle = collider

        self.visual = visual
        self.sync()

    def R(self):
        return Quaternion.to_R(self.q)

    def inv_I_world(self):
        R = self.R()
        return R @ self.inv_Ib @ R.T

    def pose(self) -> np.ndarray:
        return Transform.pose(self.x, self.q)

    def sync(self):
        self.collider_handle.sync(self.pose())
        if self.visual is not None:
            self.visual.sync(self.x, self.q)

    #def sync_collider(self):
    #    # collider neu bauen
    #    self.collider = self.collider_factory(pose_from_xq(self.x, self.q))
    #    if self.visual_sync:
    #        self.visual_sync(self)

    def step(self, dt):
        self.x += self.v * dt

        dq = Quaternion.from_omega(self.w, dt)
        self.q = Quaternion.normalize(Quaternion.mul(dq, self.q))
        self.sync()

    def vel_at_point(self, p_world):
        r = p_world - self.x
        return self.v + np.cross(self.w, r)


def inertia_box(mass, size_xyz):
    sx, sy, sz = map(float, size_xyz)
    Ixx = (mass / 12.0) * (sy * sy + sz * sz)
    Iyy = (mass / 12.0) * (sx * sx + sz * sz)
    Izz = (mass / 12.0) * (sx * sx + sy * sy)
    return np.diag([Ixx, Iyy, Izz])
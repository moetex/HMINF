import numpy as np
from distance3d import mpr

from src.model.rigidBody import RigidBody


class Contact:
    def __init__(self, depth: float, normal: np.ndarray, point: np.ndarray):
        self.depth = depth
        self.normal = np.asarray(normal, dtype=float)
        self.point = np.asarray(point, dtype=float)

class CollisionDetector:
    @staticmethod
    def detect(a: RigidBody, b: RigidBody) -> Contact:
        inter, depth, pen_dir, contact_pos = mpr.mpr_penetration(
            a.collider_handle.collider,
            b.collider_handle.collider
        )
        if not inter or depth is None or pen_dir is None or contact_pos is None:
            return None
        return Contact(depth=depth, normal=pen_dir, point=contact_pos)

class ImpulseSolver:
    def __init__(self, slop=1e-4, baumgarte=0.2):
        self.slop = slop
        self.baumgarte = baumgarte

    def resolve(self, a: RigidBody, b: RigidBody, contact: Contact):
        n = contact.normal.copy()
        nn = np.linalg.norm(n)
        if nn < 1e-12:
            return
        n /= nn

        # --- positional correction ---
        pen = max(0.0, contact.depth - self.slop)
        inv_mass_sum = a.inv_mass + b.inv_mass
        if pen > 0.0 and inv_mass_sum > 0.0:
            corr = (self.baumgarte * pen / inv_mass_sum) * n
            a.x -= a.inv_mass * corr
            b.x += b.inv_mass * corr
            a.sync()
            b.sync()

        # --- impulse at contact point ---
        p = contact.point
        ra = p - a.x
        rb = p - b.x

        va = a.vel_at_point(p)
        vb = b.vel_at_point(p)
        vrel = vb - va
        vn = float(np.dot(vrel, n))
        if vn >= 0.0:
            return

        e = min(a.restitution, b.restitution)

        invIa = a.inv_I_world()
        invIb = b.inv_I_world()

        ra_x_n = np.cross(ra, n)
        rb_x_n = np.cross(rb, n)

        ang_term = np.dot(n, np.cross(invIa @ ra_x_n, ra) + np.cross(invIb @ rb_x_n, rb))
        denom = a.inv_mass + b.inv_mass + ang_term
        if denom < 1e-12:
            return

        j = -(1.0 + e) * vn / denom
        J = j * n

        # linear
        if a.inv_mass > 0:
            a.v -= a.inv_mass * J
        if b.inv_mass > 0:
            b.v += b.inv_mass * J

        # angular
        if a.inv_mass > 0:
            a.w -= invIa @ np.cross(ra, J)
        if b.inv_mass > 0:
            b.w += invIb @ np.cross(rb, J)


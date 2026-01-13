import numpy as np
from distance3d import mpr

from src.model.Material import Material, PairLookup
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
        self.pair_lookup = PairLookup

    def resolve(self, a: RigidBody, b: RigidBody, contact: Contact):
        n = contact.normal.copy()
        nn = np.linalg.norm(n)
        if nn < 1e-12:
            return
        n /= nn

        # Korrektur, damit Positionskorrektur und Impuls nicht falsch herum wirken
        if np.dot(n, (b.x - a.x)) < 0.0:
            n = -n


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

        #e = min(a.restitution, b.restitution)

        # --- Materialpaar-Eigenschaften ---
        props = self.pair_lookup.get(a. material, b.material)
        mu_s = props.mu_s
        mu_k = props.mu_k
        e = float(props.e)

        invIa = a.inv_I_world()
        invIb = b.inv_I_world()

        # --- Normaler Impuls ---
        ra_x_n = np.cross(ra, n)
        rb_x_n = np.cross(rb, n)

        ang_term_n = np.dot(n, np.cross(invIa @ ra_x_n, ra) + np.cross(invIb @ rb_x_n, rb))
        denom_n = a.inv_mass + b.inv_mass + ang_term_n
        if denom_n < 1e-12:
            return

        jn = -(1.0 + e) * vn / denom_n
        Jn = jn * n

        # Wende normalen Impuls an
        # linear und angular
        if a.inv_mass > 0:
            a.v -= a.inv_mass * Jn
            a.w -= invIa @ np.cross(ra, Jn)
        if b.inv_mass > 0:
            b.v += b.inv_mass * Jn
            b.w += invIb @ np.cross(rb, Jn)

        # --- Reibungsimpuls (Haft- und Gleitreibung) ---
        va = a.vel_at_point(p)
        vb = b.vel_at_point(p)
        vrel = vb - va

        vt = vrel - np.dot(vrel, n) * n
        vt_norm = np.linalg.norm(vt)
        if vt_norm > 1e-10:
            return

        t = vt / vt_norm

        ra_x_t = np.cross(ra, t)
        rb_x_t = np.cross(rb, t)

        ang_term_t = np.dot(t, np.cross(invIa @ ra_x_t, ra) + np.cross(invIb @ rb_x_t, rb))
        denom_t = a.inv_mass + b.inv_mass + ang_term_t
        if denom_t < 1e-12:
            return

        jt_unc = -np.dot(vrel, t) / denom_t

        jt_max_static = mu_s * t

        if abs(jt_unc) <= jt_max_static:
            # Haftreibung
            jt = jt_unc
        else:
            # Gleitreibung
            jt = -mu_k * jn * np.sign(np.dot(vrel, t))

        Jt = jt * t

        # Wende Reibungsimpuls an
        if a.inv_mass > 0:
            a.v -= a.inv_mass * Jt
            a.w -= invIa @ np.cross(ra, Jt)
        if b.inv_mass > 0:
            b.v += b.inv_mass * Jt
            b.w += invIb @ np.cross(rb, Jt)


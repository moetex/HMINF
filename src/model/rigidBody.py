import numpy as np
import trimesh

from src.model.material import Material
from src.model.colliders import ColliderHandle
from src.res.MathHelpers import Quaternion
from src.res.MathHelpers import Transform





class RigidBody:
    """
    Vollständiger Starrkörper
    Translation:
        - x # Position im Raum (x, y, z)
        - v # Geschwindigkeit
    Rotation:
        - q # Quaternion im Raum (w, x, y, z)
        - w # Winkel-Geschwindigkeit

    Mit Material:
        - Masse aus Dichte * Volumen
    """

    def __init__(self,
                 *,
                 material: Material,
                 shape: str,
                 # shape parms:
                 size_xyz_m: np.ndarray | None = None,
                 mesh_V_m: np.ndarray | None = None,
                 mesh_F: np.ndarray | None = None,
                 x, q, v, w,
                 collider: ColliderHandle,
                 visual=None,
                 is_static: bool = False
    ):
        self.material = material
        self.shape = shape

        # Geometrie-Parameter speichern (für UI/Debug)
        self.size_xyz_m = None
        self.mesh_V_m = None
        self.mesh_F = None

        if shape == "box":
            if size_xyz_m is None:
                raise ValueError("shape='box' requires size_xyz_m")
            self.size_xyz_m = np.array(size_xyz_m, dtype=float, copy=True)

        elif shape == "mesh":
            if mesh_V_m is None or mesh_F is None:
                raise ValueError("shape='mesh' requires mesh_V_m and mesh_F")
            self.mesh_V_m = np.array(mesh_V_m, dtype=float, copy=True)
            self.mesh_F = np.array(mesh_F, dtype=np.int32, copy=True)

        #self.mass = float(mass)
        #self.inv_mass = 0.0 if self.mass <= 0 else 1.0 / self.mass

        # inertia in BODY frame (3x3), and inverse
        #self.Ib = np.array(inertia_body, dtype=float)
        #if self.inv_mass == 0:
        #    self.inv_Ib = np.zeros((3, 3), dtype=float)
        #else:
        #    self.inv_Ib = np.linalg.inv(self.Ib)

        self.x = np.array(x, dtype=float)
        self.q = Quaternion.normalize(np.array(q, dtype=float))
        self.v = np.array(v, dtype=float)
        self.w = np.array(w, dtype=float)

        #self.restitution = float(restitution)

        #self.collider_factory = collider_factory
        #self.collider = None

        self.collider_handle = collider

        self.visual = visual


        # Geometry: Volume, Mass, inertia_body
        if is_static:
            self.mass = 0.0
            self.inv_mass = 0.0
            self.Ib = np.eye(3, dtype=float)
            self.inv_Ib = np.eye(3, dtype=float)
        else:
            volume_m3, Ib = self._compute_mass_and_inertia(
                material=material,
                shape=shape,
                size_xyz_m=size_xyz_m,
                mesh_V_m=mesh_V_m,
                mesh_F=mesh_F
            )
            self.volume_m3 = volume_m3
            self.mass = float(material.density * volume_m3)
            self.inv_mass = 0.0 if self.mass <= 0.0 else 1.0 / self.mass
            self.Ib = Ib
            self.inv_Ib = np.linalg.inv(Ib)


        self.sync()

    def set_state(self, x, q, v, w):
        self.x = np.array(x, dtype=float, copy=True)
        self.q = Quaternion.normalize(np.array(q, dtype=float, copy=True))
        self.v = np.array(v, dtype=float, copy=True)
        self.w = np.array(w, dtype=float, copy=True)
        self.sync()

    """
    # Mathe Helfer-Methoden
    """
    def R(self):
        return Quaternion.to_R(self.q)

    def inv_I_world(self):
        R = self.R()
        return R @ self.inv_Ib @ R.T

    def pose(self) -> np.ndarray:
        return Transform.pose(self.x, self.q)

    def sync(self, sync_visual: bool = True):
        self.collider_handle.sync(self.pose())
        if self.visual and self.visual is not None:
            self.visual.sync(self.x, self.q)

    #def sync_collider(self):
    #    # collider neu bauen
    #    self.collider = self.collider_factory(pose_from_xq(self.x, self.q))
    #    if self.visual_sync:
    #        self.visual_sync(self)

    def step(self, dt, sync_visual: bool = True):
        self.x += self.v * dt

        dq = Quaternion.from_omega(self.w, dt)
        self.q = Quaternion.normalize(Quaternion.mul(dq, self.q))
        self.sync(sync_visual=sync_visual)

    def vel_at_point(self, p_world):
        r = p_world - self.x
        return self.v + np.cross(self.w, r)


    """
    # Mass & inertia
    """
    @staticmethod
    def _compute_mass_and_inertia(
            *,
            material: Material,
            shape: str,
            size_xyz_m: np.ndarray | None = None,
            mesh_V_m: np.ndarray | None = None,
            mesh_F: np.ndarray | None
    ) -> tuple[float, np.ndarray]:

        rho = float(material.density)

        if shape == "box":
            if size_xyz_m is None:
                raise ValueError("shape='box' requires size_xyz_m")
            sx, sy, sz = map(float, size_xyz_m)
            volume = sx * sy * sz
            mass = rho * volume

            Ixx = (mass / 12.0) * (sy * sy + sz * sz)
            Iyy = (mass / 12.0) * (sx * sx + sz * sz)
            Izz = (mass / 12.0) * (sx * sx + sy * sy)
            Ib = np.diag([Ixx, Iyy, Izz])
            return float(volume), Ib

        elif shape == "mesh":
            if mesh_V_m is None or mesh_F is None:
                raise ValueError("shape='mesh' requires mesh_V_m ans mesh_F")

            tm = trimesh.Trimesh(vertices=np.asarray(mesh_V_m, float),
                                 faces=np.asarray(mesh_F, np.int32),
                                 process=True)     #.convex_hull

            # Volumen ist nur gültig, wenn das Mesh geschlossen ist
            mp = tm.mass_properties
            volume = float(mp["volume"])
            if not np.isfinite(volume) or abs(volume) < 1e-12:
                raise ValueError("Mesh Volumen ist ~0 oder ungültig. "
                                 "Versichere dich, dass das Mesh vollständig geschlossen ist.")

            mass = rho * volume
            Ib = (rho * mp["inertia"]).astype(float)
            return float(volume), Ib

        raise ValueError(f"Unknown shape {shape!r}")




def inertia_box(mass, size_xyz):
    sx, sy, sz = map(float, size_xyz)
    Ixx = (mass / 12.0) * (sy * sy + sz * sz)
    Iyy = (mass / 12.0) * (sx * sx + sz * sz)
    Izz = (mass / 12.0) * (sx * sx + sy * sy)
    return np.diag([Ixx, Iyy, Izz])
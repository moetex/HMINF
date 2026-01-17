import numpy as np
import logging
import trimesh

from src.util.config import SimConfig
from src.model.boxWorld import BoxWorld
from src.model.material import MaterialLibrary
from src.model.collision import CollisionDetector, ImpulseSolver
from src.model.colliders import ColliderHandle
from src.model.rigidBody import RigidBody
from src.model.stlMesh import STLMesh

from distance3d import colliders

class SimulationCore:
    def __init__(self, stl_path: str, cfg: SimConfig, logger: logging.Logger):
        self.cfg = cfg
        self.log = logger

        self.world = BoxWorld(
            half_size=cfg.container_size / 2.0,
            wall_thickness=cfg.wall_thickness,
            wall_material=MaterialLibrary.HOLZ
        )

        # --- Mesh laden: Physik = ConvexHull, Visual separat in UI ---
        mesh_raw = STLMesh.load(stl_path).centered()
        mesh_phys = mesh_raw.convex_hull()
        V_m = mesh_phys.V * 1e-3  # mm -> m

        mesh_collider = ColliderHandle(lambda T: colliders.MeshGraph(T, V_m, mesh_phys.F))
        self.body_mesh = RigidBody(
            material=MaterialLibrary.HOLZ,
            shape="mesh",
            mesh_V_m=V_m,
            mesh_F=mesh_phys.F,
            x=[-0.10, 0, 0],
            q=[1, 0, 0, 0],
            v=[0.50, 0.06, 0.18],
            w=[1.2, 0.4, 0.9],
            collider=mesh_collider,
            visual=None
        )

        cube_size = np.array([0.05, 0.05, 0.05], dtype=float)
        cube_collider = ColliderHandle(lambda T: colliders.Box(T, cube_size))
        self.body_cube = RigidBody(
            material=MaterialLibrary.STAHL,
            shape="box",
            size_xyz_m=cube_size,
            x=[0.10, 0.0, 0.0],
            q=[1, 0, 0, 0],
            v=[-0.86, -0.08, 0.10],
            w=[0.7, 1.6, 0.3],
            collider=cube_collider,
            visual=None
        )

        # Initialzustände (immer Copies!)
        self._init_mesh = (self.body_mesh.x.copy(), self.body_mesh.q.copy(), self.body_mesh.v.copy(), self.body_mesh.w.copy())
        self._init_cube = (self.body_cube.x.copy(), self.body_cube.q.copy(), self.body_cube.v.copy(), self.body_cube.w.copy())

        self.detector = CollisionDetector()
        self.solver = ImpulseSolver(slop=1e-4, baumgarte=0.2)

        self.contacts_this_frame: list[tuple[np.ndarray, np.ndarray]] = []

    def reset(self):
        x, q, v, w = self._init_mesh
        self.body_mesh.set_state(x.copy(), q.copy(), v.copy(), w.copy())

        x, q, v, w = self._init_cube
        self.body_cube.set_state(x.copy(), q.copy(), v.copy(), w.copy())

        self.contacts_this_frame = []
        self.log.info("Reset durchgeführt.")

    def _store_contact(self, c):
        self.contacts_this_frame.append((np.asarray(c.point, float), np.asarray(c.normal, float)))

    def _solve_contacts_once(self):
        # body vs walls
        for w in self.world.walls:
            c = self.detector.detect(self.body_mesh, w)
            if c:
                self._store_contact(c)
                self.solver.resolve(self.body_mesh, w, c)

            c = self.detector.detect(self.body_cube, w)
            if c:
                self._store_contact(c)
                self.solver.resolve(self.body_cube, w, c)

        # body vs body
        c = self.detector.detect(self.body_mesh, self.body_cube)
        if c:
            self._store_contact(c)
            self.solver.resolve(self.body_mesh, self.body_cube, c)

    def step_physics(self):
        self.contacts_this_frame = []

        # Integrate (nur Collider sync, keine Visuals)
        self.body_mesh.step(self.cfg.dt, sync_visual=False)
        self.body_cube.step(self.cfg.dt, sync_visual=False)

        for _ in range(self.cfg.solver_iters):
            self._solve_contacts_once()

        # künstliche Dämpfung
        if self.cfg.damping != 1.0:
            for b in (self.body_mesh, self.body_cube):
                b.v *= self.cfg.damping
                b.w *= self.cfg.damping

        # Instabilität erkennen
        for name, b in (("mesh", self.body_mesh), ("cube", self.body_cube)):
            if not np.isfinite(b.x).all() or not np.isfinite(b.v).all() or not np.isfinite(b.q).all():
                self.log.error("Numerische Instabilität (%s): x/v/q enthält NaN/Inf", name)

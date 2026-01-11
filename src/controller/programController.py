import numpy as np
from kiwisolver import Solver

import src.model.rigidBody as rigidBody
from src.model.boxWorld import BoxWorld
from src.model.colliders import ColliderHandle
from src.model.collision import CollisionDetector, ImpulseSolver
from src.model.rigidBody import RigidBody, inertia_box

from src.model.stlMesh import STLMesh
from src.view.visualsVPython import BoxWireframe, VpythonTriangleMeshView, VpythonBoxVisual

from distance3d import colliders
from vpython import (
    canvas, vector, vertex, triangle, rate, color,
    box as vp_box, curve
)



class Simulation:
    def __init__(self, stl_path: str):

        self.dt = 1.0 / 120.0
        self.solver_iters = 8
        self.damping = 0.9995

        self.container_size = 120.0
        self.world = BoxWorld(half_size=self.container_size / 2.0, wall_thickness=2.0)


        canvas(title="STL Mesh + Cube in a closed Box", width=1100, height=800, background=color.white)
        BoxWireframe.draw(self.world.half)

        mesh_vis = STLMesh.load(stl_path).centered().scaled_to_radius(18.0)
        mesh_phys = STLMesh.load(stl_path).convex_hull().centered().scaled_to_radius(18.0)

        mesh_visual = VpythonTriangleMeshView(mesh_vis, mesh_color=color.gray(0.7))
        cube_size = np.array([20.0, 20.0, 20.0], dtype=float)
        cube_visual = VpythonBoxVisual(cube_size, color_=color.orange, opacity=0.9)

        mesh_mass = 2.0
        mesh_I = rigidBody.inertia_box(mesh_mass, mesh_phys.extents())

        mesh_collider = ColliderHandle(lambda T: colliders.MeshGraph(T, mesh_phys.V, mesh_phys.F))
        self.body_mesh = RigidBody(
            mass=mesh_mass, inertia_body=mesh_I,
            x=[-20, 0, 0], q=[1, 0, 0, 0],
            v=[30, 6, 18], w=[1.2, 0.4, 0.9],
            restitution=0.85,
            collider=mesh_collider,
            visual=mesh_visual
        )


        cube_mass = 1.5
        cube_I = inertia_box(cube_mass, cube_size)
        cube_collider = ColliderHandle(lambda T: colliders.Box(T, cube_size))
        self.body_cube = RigidBody(
            mass=cube_mass, inertia_body=cube_I,
            x=[20, 0, 0], q=[1, 0, 0, 0],
            v=[-26, -8, 20], w=[0.7, 1.6, 0.3],
            restitution=0.85,
            collider=cube_collider,
            visual=cube_visual
        )

        self.detector = CollisionDetector()
        self.solver = ImpulseSolver(slop=1e-4, baumgarte=0.2)

    def _solve_contacts(self):
        # body vs box
        for w in self.world.walls:
            c = self.detector.detect(self.body_mesh, w)
            if c: self.solver.resolve(self.body_mesh, w, c)

            c = self.detector.detect(self.body_cube, w)
            if c: self.solver.resolve(self.body_cube, w, c)

        # body vs body
        c = self.detector.detect(self.body_mesh, self.body_cube)
        if c: self.solver.resolve(self.body_mesh, self.body_cube, c)

    def step(self):
        self.body_mesh.step(self.dt)
        self.body_cube.step(self.dt)

        for _ in range(self.solver_iters):
            self._solve_contacts()

        for b in (self.body_mesh, self.body_cube):
            b.v *= self.damping
            b.w *= self.damping

    def run(self):
        while True:
            rate(int(1.0 / self.dt))
            self.step()












from operator import truediv

import numpy as np
from kiwisolver import Solver

import src.model.rigidBody as rigidBody
from src.model.Material import Material, MaterialLibrary
from src.model.boxWorld import BoxWorld
from src.model.colliders import ColliderHandle
from src.model.collision import CollisionDetector, ImpulseSolver
from src.model.rigidBody import RigidBody, inertia_box

from src.model.stlMesh import STLMesh
from src.view.visualsVPython import BoxWireframe, VpythonTriangleMeshView, VpythonBoxVisual, ContactDebugView, \
    BodyFrameDebugView

from distance3d import colliders
from vpython import (
    canvas, vector, vertex, triangle, rate, color,
    box as vp_box, curve, color, button, slider, wtext, scene
)



class Simulation:
    def __init__(self, stl_path: str):

        self.dt = 1.0 / 240.0   # 120
        self.solver_iters = 10    #8
        self.damping = 1.0 #0.9995

        self.container_size = 0.3   # 80cm
        self.wall_thickness = 0.01   # 1cm
        self.world = BoxWorld(half_size=self.container_size / 2.0,
                              wall_thickness=self.wall_thickness,
                              wall_material=MaterialLibrary.HOLZ)

        self.running = True
        self.show_settings = False

        scene = canvas(title="STL Mesh + Cube in a closed Box", width=1100, height=800, background=color.white)
        #scene.center = vector(0, 0, 0)
        scene.range = 0.25

        BoxWireframe.draw(self.world.half)

        scene.append_to_caption("\n")
        button(text="Pause", bind=self.toggle_run)
        scene.append_to_caption("\t")
        wtext(text="\t")

        scene.append_to_caption("   ")
        button(text="Reset", bind=self.reset)
        scene.append_to_caption("\t")
        wtext(text="\t")

        scene.append_to_caption("\n")
        button(text="Settings", bind=self.toggle_settings)
        scene.append_to_caption("\n")

        self.damping_text = wtext(text="Damping: ")
        self.damping_label = wtext(text=f"{self.damping:.4f}")
        self.damping_slider = slider(
            min=0.95, max=1.0, value=self.damping,
            step=0.0001, bind=self.set_damping
        )

        self.damping_text.visible = False
        self.damping_label.visible = False
        self.damping_slider.visible = False

        # Anfangs verstecken
        self.damping_label.visible = False
        self.damping_slider.visible = False

        mesh_raw = STLMesh.load(stl_path).centered()    # in mm
        mesh_phys = mesh_raw.convex_hull()
        V_phys_m = mesh_phys.V * 1e-3                 # mm -> m
        mesh_vis_m = STLMesh(mesh_raw.V * 1e-3, mesh_raw.F)

        #mesh_vis = STLMesh.load(stl_path).centered().scaled_to_radius(18.0) #18.0 Visualisierung frei skalierbar
        #mesh_phys = STLMesh.load(stl_path).convex_hull().centered() #.scaled_to_radius(24.0)  #18.0
        V_m = mesh_phys.V * 1e-3    # mm -> m


        #mesh_mass = 2.0
        #mesh_I = rigidBody.inertia_box(mesh_mass, mesh_phys.extents())

        mesh_collider = ColliderHandle(lambda T: colliders.MeshGraph(T, V_m, mesh_phys.F))
        mesh_visual = VpythonTriangleMeshView(mesh_vis_m, mesh_color=color.orange)
        self.body_mesh = RigidBody(
            material=MaterialLibrary.HOLZ,
            shape="mesh",
            mesh_V_m=V_m,
            mesh_F=mesh_phys.F,
            x=[-0.10, 0, 0],    # meter ; [-20, 0, 0]
            q=[1, 0, 0, 0],
            v=[0.50, 0.06, 0.18],      # m/s ; [30, 6, 18]
            w=[1.2, 0.4, 0.9],  # rad/s ; [1.2, 0.4, 0.9]
            collider=mesh_collider,
            visual=mesh_visual
        )


        #cube_mass = 1.5
        #cube_I = inertia_box(cube_mass, cube_size)
        cube_size = np.array([0.05, 0.05, 0.05], dtype=float)
        cube_collider = ColliderHandle(lambda T: colliders.Box(T, cube_size))
        cube_visual = VpythonBoxVisual(cube_size, color_=color.gray(0.7), opacity=0.9)
        self.body_cube = RigidBody(
            material=MaterialLibrary.STAHL,
            shape="box",
            size_xyz_m=cube_size,    # 2cm
            x=[0.10, 0.0, 0.0],
            q=[1, 0, 0, 0],
            v=[-0.86, -0.08, 0.10],
            w=[0.7, 1.6, 0.3],
            collider=cube_collider,
            visual=cube_visual
        )

        self.detector = CollisionDetector()
        self.solver = ImpulseSolver(slop=1e-4, baumgarte=0.2)


        self.contact_view = ContactDebugView(max_points=64, point_radius=0.004, normal_scale=0.06)
        self.mesh_frame_view = BodyFrameDebugView(axis_len=0.06, show_v_dir=True, show_w_dir=True, v_len=0.05, w_len=0.06, show_trail=True)
        self.cube_frame_view = BodyFrameDebugView(axis_len=0.06, show_v_dir=True, show_w_dir=True, v_len=0.05, w_len=0.06, show_trail=True)

        self._contacts_this_frame = []


    def _store_contacts(self, c):
        p = np.asarray(c.point, float)
        n = np.asarray(c.normal, float)
        self._contacts_this_frame.append((p, n))


    def _solve_contacts(self):
        # body vs box
        for w in self.world.walls:
            c = self.detector.detect(self.body_mesh, w)
            if c:
                self._store_contacts(c)
                self.solver.resolve(self.body_mesh, w, c)

            c = self.detector.detect(self.body_cube, w)
            if c:
                self._store_contacts(c)
                self.solver.resolve(self.body_cube, w, c)

        # body vs body
        c = self.detector.detect(self.body_mesh, self.body_cube)
        if c:
            self._store_contacts(c)
            self.solver.resolve(self.body_mesh, self.body_cube, c)

    def step(self):
        self._contacts_this_frame.clear()

        self.body_mesh.step(self.dt)
        self.body_cube.step(self.dt)

        for _ in range(self.solver_iters):
            self._solve_contacts()

        for b in (self.body_mesh, self.body_cube):
            b.v *= self.damping
            b.w *= self.damping

        self.contact_view.clear()
        for p, n in self._contacts_this_frame[: self.contact_view.max_points]:
            self.contact_view.add(p, n)

        self.mesh_frame_view.sync(self.body_mesh)
        self.cube_frame_view.sync(self.body_cube)

    def run(self):
        while True:
            rate(int(1.0 / self.dt))
            if self.running:
                self.step()

    def toggle_run(self, b):
        self.running = not self.running
        b.text = "Pause" if self.running else "Resume"

    def reset(self, _=None):
        # Mesh
        self.body_mesh.x[:] = [-20, 0, 0]
        self.body_mesh.q[:] = [1, 0, 0, 0]
        self.body_mesh.v[:] = [30, 6, 18]
        self.body_mesh.w[:] = [1.2, 0.4, 0.9]

        # Cube
        self.body_cube.x[:] = [20, 0, 0]
        self.body_cube.q[:] = [1, 0, 0, 0]
        self.body_cube.v[:] = [-26, -8, 20]
        self.body_cube.w[:] = [0.7, 1.6, 0.3]

    def toggle_settings(self, _=None):
        self.show_settings = not self.show_settings

        self.damping_text.visible = self.show_settings
        self.damping_label.visible = self.show_settings
        self.damping_slider.visible = self.show_settings

    def set_damping(self, s):
        self.damping = s.value
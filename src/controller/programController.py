import numpy as np
from kiwisolver import Solver
import trimesh

import src.model.rigidBody as rigidBody
from src.model.material import Material, MaterialLibrary
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
    box as vp_box, curve, color, button, slider, wtext, scene, checkbox
)



class Simulation:
    def set_mesh_speed(self, s):
        # Nach dem ersten Start gesperrt
        if self.has_started:
            s.value = self.mesh_speed_factor
            return

        self.mesh_speed_factor = float(s.value)
        self.mesh_speed_label.text = f"{self.mesh_speed_factor:.2f}  "

        x0, q0, v0, w0 = self._base_init_mesh
        new_v = v0 * self.mesh_speed_factor

        # Aktuellen Body updaten (vor Start direkt sichtbar)
        self.body_mesh.v = new_v.copy()

        # Reset-Zielwerte updaten (damit Reset wieder korrekt ist)
        self._init_mesh = (x0.copy(), q0.copy(), new_v.copy(), w0.copy())

        # Pfeile/Debug sofort aktualisieren
        self.mesh_frame_view.sync(self.body_mesh)

    def set_cube_speed(self, s):
        if self.has_started:
            s.value = self.cube_speed_factor
            return

        self.cube_speed_factor = float(s.value)
        self.cube_speed_label.text = f"{self.cube_speed_factor:.2f}  "

        x0, q0, v0, w0 = self._base_init_cube
        new_v = v0 * self.cube_speed_factor

        self.body_cube.v = new_v.copy()
        self._init_cube = (x0.copy(), q0.copy(), new_v.copy(), w0.copy())

        self.cube_frame_view.sync(self.body_cube)

    def __init__(self, stl_path: str):

        self.dt = 1.0 / 240.0   # 120
        self.solver_iters = 10    #8
        self.damping = 1.0 #0.9995

        self.container_size = 0.3   # 80cm
        self.wall_thickness = 0.01   # 1cm
        self.world = BoxWorld(half_size=self.container_size / 2.0,
                              wall_thickness=self.wall_thickness,
                              wall_material=MaterialLibrary.HOLZ)

        self.running = False  # startet NICHT automatisch
        self.has_started = False  # damit Button zuerst "Start" zeigt
        self.show_settings = False
        self.mesh_speed_factor = 1.0
        self.cube_speed_factor = 1.0

        scene = canvas(title="STL Mesh + Cube in a closed Box", width=1100, height=800, background=color.white)
        #scene.center = vector(0, 0, 0)
        scene.range = 0.25

        scene.append_to_caption("\n")
        self.run_btn = button(text="Start", bind=self.toggle_run)
        #scene.append_to_caption("\t")
        #wtext(text="\t")

        scene.append_to_caption("   ")
        button(text="Reset", bind=lambda _: self.reset())
        #scene.append_to_caption("\t")
        #wtext(text="\t")

        scene.append_to_caption("   ")
        button(text="Settings", bind=lambda _: self.toggle_settings())
        scene.append_to_caption("   ")

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

        scene.append_to_caption("Mesh Speed: ")
        self.mesh_speed_label = wtext(text=f"{self.mesh_speed_factor:.2f} ")
        self.mesh_speed_slider = slider(
            min=0.0, max=2.0, value=self.mesh_speed_factor,
            step=0.01, bind=self.set_mesh_speed
        )

        scene.append_to_caption("\nCube Speed: ")
        self.cube_speed_label = wtext(text=f"{self.cube_speed_factor:.2f}  ")
        self.cube_speed_slider = slider(
            min=0.0, max=2.0, value=self.cube_speed_factor,
            step=0.01, bind=self.set_cube_speed
        )

        scene.append_to_caption("\n")

        def set_mesh_speed(self, s):
            # Nach dem ersten Start gesperrt
            if self.has_started:
                s.value = self.mesh_speed_factor
                return

            self.mesh_speed_factor = float(s.value)
            self.mesh_speed_label.text = f"{self.mesh_speed_factor:.2f}  "

            x0, q0, v0, w0 = self._base_init_mesh
            new_v = v0 * self.mesh_speed_factor

            # Aktuellen Body updaten (vor Start direkt sichtbar)
            self.body_mesh.v = new_v.copy()

            # Reset-Zielwerte updaten (damit Reset wieder korrekt ist)
            self._init_mesh = (x0.copy(), q0.copy(), new_v.copy(), w0.copy())

            # Pfeile/Debug sofort aktualisieren
            self.mesh_frame_view.sync(self.body_mesh)

        def set_cube_speed(self, s):
            if self.has_started:
                s.value = self.cube_speed_factor
                return

            self.cube_speed_factor = float(s.value)
            self.cube_speed_label.text = f"{self.cube_speed_factor:.2f}  "

            x0, q0, v0, w0 = self._base_init_cube
            new_v = v0 * self.cube_speed_factor

            self.body_cube.v = new_v.copy()
            self._init_cube = (x0.copy(), q0.copy(), new_v.copy(), w0.copy())

            self.cube_frame_view.sync(self.body_cube)





        #scene.append_to_caption("\n")
        #button(text="Reset", bind=lambda _: self.reset())
        #scene.append_to_caption("   ")


        # Checkboxen
        scene.append_to_caption("\n")
        cb_arrows = checkbox(text="Freiheitsgrade", checked=True, bind=lambda c: self.toggle_Arrows(c.checked))
        scene.append_to_caption("   ")
        cb_contacts = checkbox(text="Kontaktpunkte", checked=True, bind=lambda c: self.toggle_contacts(c.checked))
        scene.append_to_caption("   ")
        cb_trails = checkbox(text="Kurve/Trail", checked=True, bind=lambda c: self.toggle_trail(c.checked))
        scene.append_to_caption("   ")



        BoxWireframe.draw(self.world.half)

        mesh_raw = STLMesh.load(stl_path).centered()    # in mm
        tm_vis = trimesh.Trimesh(vertices=mesh_raw.V, faces=mesh_raw.F, process=False)
        tm_vis_s = tm_vis.simplify_quadric_decimation(face_count=3000)  # 3000 Faces als Startwert
        mesh_vis_m = STLMesh(tm_vis_s.vertices * 1e-3, tm_vis_s.faces)

        mesh_phys = mesh_raw.convex_hull()
        V_m = mesh_phys.V * 1e-3                 # mm -> m

        #mesh_vis = STLMesh.load(stl_path).centered().scaled_to_radius(18.0) #18.0 Visualisierung frei skalierbar
        #mesh_phys = STLMesh.load(stl_path).convex_hull().centered() #.scaled_to_radius(24.0)  #18.0
        #V_m = mesh_phys.V * 1e-3    # mm -> m


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


        self.mesh_speed_factor = 1.0
        self.cube_speed_factor = 1.0



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
        self._init_cube = (self.body_cube.x.copy(), self.body_cube.q.copy(), self.body_cube.v.copy(), self.body_cube.w.copy())
        self._init_mesh = (self.body_mesh.x.copy(), self.body_mesh.q.copy(), self.body_mesh.v.copy(),
                           self.body_mesh.w.copy())

        # --- Speed-Slider Basiswerte (werden nur VOR dem Start verändert) ---
        self._base_init_mesh = (self._init_mesh[0].copy(), self._init_mesh[1].copy(), self._init_mesh[2].copy(),
                                self._init_mesh[3].copy())
        self._base_init_cube = (self._init_cube[0].copy(), self._init_cube[1].copy(), self._init_cube[2].copy(),
                                self._init_cube[3].copy())

        self.detector = CollisionDetector()
        self.solver = ImpulseSolver(slop=1e-4, baumgarte=0.2)


        self.contact_view = ContactDebugView(max_points=64, point_radius=0.006, normal_scale=0.04)  # 0.004, 0.06
        self.mesh_frame_view = BodyFrameDebugView(axis_len=0.06, show_v_dir=True, show_w_dir=True, v_len=0.05, w_len=0.06, show_trail=True)
        self.cube_frame_view = BodyFrameDebugView(axis_len=0.06, show_v_dir=True, show_w_dir=True, v_len=0.05, w_len=0.06, show_trail=True)

        self._contacts_this_frame = []

        # Initialer Debug-Sync: sonst haben VPython-arrows Default-Größe (zu groß),
        # bis der erste Simulation-step läuft.
        self.mesh_frame_view.sync(self.body_mesh)
        self.cube_frame_view.sync(self.body_cube)

        # optional: Kontaktanzeige initial leeren
        self.contact_view.clear()


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

    def step(self, sync_visual: bool = True):
        self._contacts_this_frame = []
        #self._contacts_this_frame.clear()

        self.body_mesh.step(self.dt, sync_visual=sync_visual)
        self.body_cube.step(self.dt, sync_visual=sync_visual)

        for _ in range(self.solver_iters):
            self._solve_contacts()

        for b in (self.body_mesh, self.body_cube):
            b.v *= self.damping
            b.w *= self.damping

        # Kontaktpunkte anzeigen
        for p, n in self._contacts_this_frame[: self.contact_view.max_points]:
            self.contact_view.add(p, n)

    def _draw_debug(self):
        # Kontaktpunkte anzeigen
        self.contact_view.clear()
        for p, n in self._contacts_this_frame[: self.contact_view.max_points]:
            self.contact_view.add(p, n)

        # Freiheitsgrade
        self.mesh_frame_view.sync(self.body_mesh)
        self.cube_frame_view.sync(self.body_cube)

    def run(self):
        render_hz = 60
        substeps = int(round((1.0 / render_hz) / self.dt))  # dt=1/240 => 4

        while True:
            rate(render_hz)
            if self.running:
                for _ in range(substeps):
                    self.step(sync_visual=False)  # Physik + Collider, aber ohne Mesh-Vertex-Updates
                # EINMAL pro Frame Visuals + Debug zeichnen:
                self.body_mesh.sync(sync_visual=True)
                self.body_cube.sync(sync_visual=True)
                self._draw_debug()

    def toggle_run(self, b):
        # Erstes Starten
        if not self.has_started:
            self.has_started = True
            self.running = True
            b.text = "Pause"

            # Slider ab jetzt sperren
            self._set_speed_sliders_enabled(False)
            return

        # Danach normal togglen
        self.running = not self.running
        b.text = "Pause" if self.running else "Resume"


    def reset(self, _=None):
        # Mesh
        x, q, v, w = self._init_mesh
        self.body_mesh.set_state(x, q, v, w)
        #self.body_mesh.x[:] = [-20, 0, 0]
        #self.body_mesh.q[:] = [1, 0, 0, 0]
        #self.body_mesh.v[:] = [30, 6, 18]
        #self.body_mesh.w[:] = [1.2, 0.4, 0.9]

        # Cube
        x, q, v, w = self._init_cube
        self.body_cube.set_state(x, q, v, w)
        #self.body_cube.x[:] = [20, 0, 0]
        #self.body_cube.q[:] = [1, 0, 0, 0]
        #self.body_cube.v[:] = [-26, -8, 20]
        #self.body_cube.w[:] = [0.7, 1.6, 0.3]

        # Anzeigen
        if hasattr(self, 'contact_view'):
            self.contact_view.reset()
        if hasattr(self, 'mesh_frame_view'):
            self.mesh_frame_view.reset_trail()
        if hasattr(self, 'cube_frame_view'):
            self.cube_frame_view.reset_trail()

        self._contacts_this_frame = []
        if hasattr(self, "contact_view"):
            self.contact_view.clear()

        if hasattr(self, "mesh_frame_view"):
            self.mesh_frame_view.sync(self.body_mesh)
        if hasattr(self, "cube_frame_view"):
            self.cube_frame_view.sync(self.body_cube)

        self.running = False
        self.has_started = False
        if hasattr(self, "run_btn"):
            self.run_btn.text = "Start"

        # Speed-Slider wieder freigeben
        self._set_speed_sliders_enabled(True)

        # Slider-UI auf die aktuellen Faktoren zurücksetzen (optional, aber sauber)
        if hasattr(self, "mesh_speed_slider"):
            self.mesh_speed_slider.value = self.mesh_speed_factor
        if hasattr(self, "cube_speed_slider"):
            self.cube_speed_slider.value = self.cube_speed_factor

    def toggle_settings(self, _=None):
        self.show_settings = not self.show_settings

        self.damping_text.visible = self.show_settings
        self.damping_label.visible = self.show_settings
        self.damping_slider.visible = self.show_settings

    def _set_speed_sliders_enabled(self, enabled: bool):
        # VPython unterstützt meist .disabled; falls nicht, guardet zusätzlich die Callback-Funktion
        for s in (getattr(self, "mesh_speed_slider", None), getattr(self, "cube_speed_slider", None)):
            if s is None:
                continue
            try:
                s.disabled = not enabled
            except Exception:
                pass

    # Slider
    def set_damping(self, s):
        self.damping = s.value


    # Checkboxen
    def toggle_Arrows(self, checked: bool):
        self.mesh_frame_view.set_gizmo_visible(checked)
        self.cube_frame_view.set_gizmo_visible(checked)

    def toggle_contacts(self, checken: bool):
        self.contact_view.set_visible(checken)

    def toggle_trail(self, checked: bool):
        self.mesh_frame_view.set_trail_visible(checked)
        self.cube_frame_view.set_trail_visible(checked)
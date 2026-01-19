import numpy as np
import trimesh

from vpython import canvas, color, rate, button, slider, wtext, checkbox

from src.util.config import SimConfig
from src.model.stlMesh import STLMesh
from src.view.visualsVPython import BoxWireframe, VpythonTriangleMeshView, VpythonBoxVisual, ContactDebugView, BodyFrameDebugView

class VpythonUI:
    def __init__(self, sim, stl_path: str, cfg: SimConfig):
        self.sim = sim
        self.cfg = cfg

        self.running = False
        self.has_started = False

        # UI Scene
        self.scene = canvas(title="Rigid Body Collision", width=1100, height=800, background=color.white)
        self.scene.range = 0.25

        # Buttons
        self.scene.append_to_caption("\n")
        self.run_btn = button(text="Start", bind=self.toggle_run)
        self.scene.append_to_caption("   ")
        button(text="Reset", bind=self.reset)
        self.scene.append_to_caption("   ")

        # Speed Slider (vor Start)
        self.scene.append_to_caption("\nMesh Speed: ")
        self.mesh_speed = 1.0
        self.mesh_speed_label = wtext(text=f"{self.mesh_speed:.2f}  ")
        self.mesh_speed_slider = slider(min=0.0, max=2.0, value=1.0, step=0.01, bind=self.set_mesh_speed)

        self.scene.append_to_caption("\nCube Speed: ")
        self.cube_speed = 1.0
        self.cube_speed_label = wtext(text=f"{self.cube_speed:.2f}  ")
        self.cube_speed_slider = slider(min=0.0, max=2.0, value=1.0, step=0.01, bind=self.set_cube_speed)
        self.scene.append_to_caption("\n")

        # Debug toggles
        cb_frames = checkbox(text="Freiheitsgrade", checked=cfg.debug_frames, bind=lambda c: self.toggle_frames(c.checked))
        self.scene.append_to_caption("   ")
        cb_contacts = checkbox(text="Kontaktpunkte", checked=cfg.debug_contacts, bind=lambda c: self.toggle_contacts(c.checked))
        self.scene.append_to_caption("   ")
        cb_trails = checkbox(text="Trail", checked=cfg.debug_trails, bind=lambda c: self.toggle_trails(c.checked))
        self.scene.append_to_caption("\n")

        # World wireframe
        BoxWireframe.draw(self.sim.world.half)

        # Visual Mesh (decimated)
        mesh_raw = STLMesh.load(stl_path).centered()

        # Decimation optional
        mesh_vis = mesh_raw
        if cfg.visual_target_faces and cfg.visual_target_faces > 0:
            tm = trimesh.Trimesh(vertices=mesh_raw.V, faces=mesh_raw.F, process=False)
            try:
                tm_s = tm.simplify_quadric_decimation(target_count=int(cfg.visual_target_faces))
                mesh_vis = STLMesh(tm_s.vertices, tm_s.faces)
            except Exception:
                # fallback: Original
                mesh_vis = mesh_raw

        mesh_vis_m = STLMesh(mesh_vis.V * 1e-3, mesh_vis.F)
        self.mesh_visual = VpythonTriangleMeshView(mesh_vis_m, mesh_color=color.orange)
        self.sim.body_mesh.visual = self.mesh_visual

        cube_size = self.sim.body_cube.size_xyz_m
        self.cube_visual = VpythonBoxVisual(cube_size, color_=color.gray(0.7), opacity=0.9)
        self.sim.body_cube.visual = self.cube_visual

        # Debug visuals
        self.contact_view = ContactDebugView(max_points=64, point_radius=0.006, normal_scale=0.04)
        self.mesh_frame_view = BodyFrameDebugView(axis_len=0.06, show_v_dir=True, show_w_dir=True, v_len=0.05, w_len=0.06, show_trail=True)
        self.cube_frame_view = BodyFrameDebugView(axis_len=0.06, show_v_dir=True, show_w_dir=True, v_len=0.05, w_len=0.06, show_trail=True)

        # Initial sync (damit keine riesigen Default-Pfeile)
        self.sim.body_mesh.sync(sync_visual=True)
        self.sim.body_cube.sync(sync_visual=True)
        self.mesh_frame_view.sync(self.sim.body_mesh)
        self.cube_frame_view.sync(self.sim.body_cube)
        self.contact_view.clear()

        # Basisgeschwindigkeiten für Slider (Reset-sicher)
        self._base_mesh_v = self.sim._init_mesh[2].copy()
        self._base_cube_v = self.sim._init_cube[2].copy()

        self.toggle_frames(cfg.debug_frames)
        self.toggle_contacts(cfg.debug_contacts)
        self.toggle_trails(cfg.debug_trails)

    def _set_speed_sliders_enabled(self, enabled: bool):
        for s in (self.mesh_speed_slider, self.cube_speed_slider):
            try:
                s.disabled = not enabled
            except Exception:
                pass

    def set_mesh_speed(self, s):
        if self.has_started:
            s.value = self.mesh_speed
            return
        self.mesh_speed = float(s.value)
        self.mesh_speed_label.text = f"{self.mesh_speed:.2f}  "
        new_v = self._base_mesh_v * self.mesh_speed
        x, q, _, w = self.sim._init_mesh
        self.sim._init_mesh = (x.copy(), q.copy(), new_v.copy(), w.copy())
        self.sim.body_mesh.v = new_v.copy()
        self.mesh_frame_view.sync(self.sim.body_mesh)

    def set_cube_speed(self, s):
        if self.has_started:
            s.value = self.cube_speed
            return
        self.cube_speed = float(s.value)
        self.cube_speed_label.text = f"{self.cube_speed:.2f}  "
        new_v = self._base_cube_v * self.cube_speed
        x, q, _, w = self.sim._init_cube
        self.sim._init_cube = (x.copy(), q.copy(), new_v.copy(), w.copy())
        self.sim.body_cube.v = new_v.copy()
        self.cube_frame_view.sync(self.sim.body_cube)

    def toggle_run(self, b):
        if not self.has_started:
            self.has_started = True
            self.running = True
            b.text = "Pause"
            self._set_speed_sliders_enabled(False)
            return

        self.running = not self.running
        b.text = "Pause" if self.running else "Resume"

    def reset(self, _=None):
        self.running = False
        self.has_started = False
        self.run_btn.text = "Start"
        self._set_speed_sliders_enabled(True)

        self.sim.reset()

        # Debug zurücksetzen
        self.contact_view.clear()
        self.mesh_frame_view.reset_trail()
        self.cube_frame_view.reset_trail()
        self.mesh_frame_view.sync(self.sim.body_mesh)
        self.cube_frame_view.sync(self.sim.body_cube)

        # Visual sync
        self.sim.body_mesh.sync(sync_visual=True)
        self.sim.body_cube.sync(sync_visual=True)

    def toggle_frames(self, on: bool):
        self.mesh_frame_view.set_gizmo_visible(on)
        self.cube_frame_view.set_gizmo_visible(on)

    def toggle_contacts(self, on: bool):
        self.contact_view.set_visible(on)

    def toggle_trails(self, on: bool):
        self.mesh_frame_view.set_trail_visible(on)
        self.cube_frame_view.set_trail_visible(on)

    def _draw_debug(self):
        self.contact_view.clear()
        for p, n in self.sim.contacts_this_frame[: self.contact_view.max_points]:
            self.contact_view.add(p, n)
        self.mesh_frame_view.sync(self.sim.body_mesh)
        self.cube_frame_view.sync(self.sim.body_cube)

    def run(self):
        substeps = int(round((1.0 / self.cfg.render_hz) / self.cfg.dt))
        while True:
            rate(self.cfg.render_hz)
            if self.running:
                for _ in range(substeps):
                    self.sim.step_physics()
                # Visuals einmal pro Frame
                self.sim.body_mesh.sync(sync_visual=True)
                self.sim.body_cube.sync(sync_visual=True)
                self._draw_debug()

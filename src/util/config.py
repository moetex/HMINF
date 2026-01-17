from dataclasses import dataclass

@dataclass
class SimConfig:
    dt: float = 1.0 / 240.0
    solver_iters: int = 8
    damping: float = 1.0

    container_size: float = 0.30
    wall_thickness: float = 0.01

    render_hz: int = 60  # UI-Framerate
    debug_contacts: bool = True
    debug_frames: bool = True
    debug_trails: bool = True

    # STL Visual-Decimation (optional)
    visual_target_faces: int = 3000

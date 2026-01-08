import numpy as np
import trimesh


class STLMesh:
    def __init__(self, vertices: np.ndarray, faces: np.ndarray):
        self.V = np.asarray(vertices, dtype=float)
        self.F = np.asarray(faces, dtype=np.int32)

    @staticmethod
    def load(path: str):
        tm = trimesh.load(path, force="mesh", process=True)
        if isinstance(tm, trimesh.Scene):
            tm = trimesh.util.concatenate(tuple(tm.dump()))
        if tm.faces is None or len(tm.faces) == 0:
            raise ValueError("STL/Mesh enthält keine Faces")
        return STLMesh(tm.vertices, tm.faces)


    def convex_hull(self):
        tm = trimesh.Trimesh(vertices=self.V, faces=self.F, process=True)
        hull = tm.convex_hull
        return STLMesh(hull.vertices, hull.faces)

    def centered(self):
        V = self.V - self.V.mean(axis=0)
        return STLMesh(V, self.F)

    def scaled_to_radius(self, radius: float):
        V = self.V.copy()
        r = np.max(np.linalg.norm(V, axis=1))
        if r > 1e-12:
            V *= (float(radius) / r)
        return STLMesh(V, self.F)

    def extents(self) -> np.ndarray:
        return self.V.max(axis=0) - self.V.min(axis=0)
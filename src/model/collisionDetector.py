
import numpy as np
import trimesh
from distance3d import colliders, gjk

def load_stl_as_mesh(stl_path: str,
                     mesh2origin: np.ndarray | None = None,
                     scale: float = 1.0,
                     make_convex: bool = False) -> colliders.MeshGraph:
    #mesh = trimesh.load_mesh(file_obj = trimesh.util.wrap_as_stream(stl_path), file_type = "stl")
    mesh = trimesh.load_mesh(stl_path)

    # Falls ein Scene-Objekt zurückkommt (mehrere Geometrien)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))

    if make_convex:
        mesh = mesh.convex_hull

    vertices = np.asarray(mesh.vertices, dtype=float) * scale
    triangles = np.asarray(mesh.faces, dtype=np.int32)

    if mesh2origin is None:
        mesh2origin = mesh.eye(4)

    return colliders.MeshGraph(mesh2origin, vertices, triangles)


def detectCollision(body1_path: str, body2_path: str):
    c1 = load_stl_as_mesh(body1_path, mesh2origin=np.eye(4), scale= 1.0, make_convex=False)
    c2 = load_stl_as_mesh(body2_path, mesh2origin=np.eye(4), scale= 1.0, make_convex=False)

    #print("GJK intersect:", gjk.gjk_intersection(c1, c2, tolerance=1e-10))

    #dist, p1, p2, _ = gjk.gjk(c1, c2)

    #print("distance:", dist)
    #print("closest on c1:", p1)
    #print("closest on c2:", p2)

    return gjk.gjk_intersection(c1, c2)








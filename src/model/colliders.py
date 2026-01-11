import numpy as np

class ColliderHandle:

    def __init__(self, factory):
        self._factory = factory
        self._collider = None

    @property
    def collider(self):
        return self._collider

    def sync(self, pose: np.ndarray):
        if self._collider is None:
            self._collider = self._factory(pose)
            return
        # Try update pose, fallback to rebuild
        if hasattr(self._collider, "update_pos"):
            try:
                self._collider.update_pose(pose)
                return
            except NotImplementedError:
                pass
            except Exception as e:
                pass
        self._collider = self._factory(pose)
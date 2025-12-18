from __future__ import annotations
from contextlib import contextmanager
from typing import Final, NamedTuple, final,Generator

import math
import numpy as np
import pyray
from pyray import *

from box import Box


class Context(NamedTuple):
    screen_width: int
    screen_height: int

    def begin(self) -> None:
        pyray.begin_drawing()

    def end(self) -> None:
        pyray.end_drawing()

    def get_frame_time(self) -> float:
        return pyray.get_frame_time()

    def should_close(self) -> bool:
        return pyray.window_should_close()

    def is_key_down(self, key: int) -> bool:
        return pyray.is_key_down(key)

    def is_key_pressed(self, key: int) -> bool:
        return pyray.is_key_pressed(key)


@contextmanager
def create_context(
    width: int = 1280,
    height: int = 720,
    title: str = "HMINF",
    target_fps: int = 60,
) -> Generator[Context, None, None]:

    pyray.init_window(width, height, title)
    pyray.set_target_fps(target_fps)

    try:
        yield Context(
            screen_width=width,
            screen_height=height,
        )
    finally:
        pyray.close_window()



BOX_LENGTH = 60
box = Box(
    np.array([
        [-BOX_LENGTH, 0, -BOX_LENGTH],
        [BOX_LENGTH, 0, -BOX_LENGTH],
        [BOX_LENGTH, BOX_LENGTH, -BOX_LENGTH],
        [-BOX_LENGTH, BOX_LENGTH, -BOX_LENGTH],
        [-BOX_LENGTH, 0, BOX_LENGTH],
        [BOX_LENGTH, 0, BOX_LENGTH],
        [BOX_LENGTH, BOX_LENGTH, BOX_LENGTH],
        [-BOX_LENGTH, BOX_LENGTH, BOX_LENGTH]
    ])
)


camera = Camera3D(
    [BOX_LENGTH * 3] * 3,
    [0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    45.0,
    0
)

def load_model():
    # https://free3d.com/3d-model/-oz-beer-bottle-v2--175362.html
    model = pyray.load_model("assets/bottle.obj")
    model.transform = pyray.matrix_rotate_x(-math.pi / 2)
    return model

def main():

    position = np.array([0.0, 2.0, 0.0])
    velocity = np.array([0.5, 0.3, 0.7])
    radius = 2


    with create_context() as context:
        model = load_model()
        while not context.should_close():
            position += velocity
            offset = box.check_collision(position, radius)
            for i in range(3):
                if offset[i] != 0:
                    position[i] += offset[i]
                    velocity[i] *= -1

            context.begin()
            clear_background(WHITE)

            begin_mode_3d(camera)
            draw_model(model, Vector3(20, 33, 11), 2, GREEN)
            draw_cube_wires(Vector3(*box.center), *box.size, BLUE)
            #draw_sphere(Vector3(*position), radius, RED)
            end_mode_3d()

            draw_text("HMINF", context.screen_width // 2 - 40, 40, 22, GREEN)
            context.end()
        pyray.unload_model(model)


if __name__ == '__main__':
    main()



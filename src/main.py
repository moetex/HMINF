from __future__ import annotations
from contextlib import contextmanager
from typing import Final, NamedTuple, final,Generator

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



BOX_LENGTH = 20
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



def main():

    position = np.array([0.0, 2.0, 0.0])
    velocity = np.array([0.5, 0.3, 0.7])
    radius = 2


    with create_context() as context:
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
            draw_cube_wires(Vector3(*box.center), *box.size, BLUE)
            draw_sphere(Vector3(*position), radius, RED)
            end_mode_3d()

            draw_text("HMINF", int(context.screen_width / 2), 40, 22, GREEN)
            context.end()

if __name__ == '__main__':
    main()



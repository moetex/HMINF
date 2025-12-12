from __future__ import annotations
from contextlib import contextmanager
from typing import Final, NamedTuple, final,Generator

import pyray

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


def main():
    with create_context() as context:
        while not context.should_close():
            context.begin()
            clear_background(WHITE)
            draw_text("HMINF", int(context.screen_width / 2), 40, 120, GREEN)
            context.end()

if __name__ == '__main__':
    main()



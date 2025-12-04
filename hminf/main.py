from __future__ import annotations

import pygame
from contextlib import contextmanager
from typing import Final, NamedTuple, final


@final
class PyGameContext(NamedTuple):
    screen: pygame.Surface
    clock: pygame.time.Clock

    def flip(self):
        pygame.display.flip()

    def get_events(self):
        return pygame.event.get()

    def get_pressed_keys(self):
        return pygame.key.get_pressed()

    def tick(self, fps: int):
        return self.clock.tick(fps)

@contextmanager
def create_game(width: int = 800, height: int = 600, title: str = "HMINF"):
    try:
        pygame.init()
        pygame.display.set_caption(title)

        screen: Final = pygame.display.set_mode((width, height))
        clock: Final = pygame.time.Clock()

        ctx = PyGameContext(
            screen=screen,
            clock=clock,
        )
        yield ctx

    finally:
        pygame.quit()



def main():
    with create_game() as context:
        running = True
        while running:
            for event in context.get_events():
                if event.type == pygame.QUIT:
                    running = False
            context.screen.fill("green")
            context.flip()
            context.tick(60)

if __name__ == '__main__':
    main()



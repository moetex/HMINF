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
def create_game(width: int = 800, height: int = 800, title: str = "HMINF"):

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
    crcl_x = 100
    crcl_y = 200
    crcl_radius = 30
    box_width = 700
    box_height = 700
    box_left_border = 50
    box_upper_border = 50
    crcl_speed_x = 2
    crcl_speed_y = 2
    with create_game() as context:
        running = True
        while running:
            for event in context.get_events():
                if event.type == pygame.QUIT:
                    running = False
                    print("Spiel durch Benutzer beendet")
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_m:
                        print("Taste gedrückt")

            context.screen.fill("black")
            pygame.draw.rect(context.screen, "red", [50, 50, box_height, box_width], 3)
            pygame.draw.circle(context.screen, "white", [crcl_x, crcl_y], crcl_radius)
            if crcl_x >= box_width:
                crcl_speed_x = -crcl_speed_x
            if crcl_x-crcl_radius <= box_left_border:
                crcl_speed_x = -crcl_speed_x

            if crcl_y >= box_height:
                crcl_speed_y = -crcl_speed_y
            if crcl_y-crcl_radius <= box_upper_border:
                crcl_speed_y = -crcl_speed_y
            crcl_x += crcl_speed_x
            crcl_y += crcl_speed_y
            #crcl_radius += 0.05


            context.flip()
            context.tick(60)

if __name__ == '__main__':
    main()



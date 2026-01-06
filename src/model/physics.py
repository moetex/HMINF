

import numpy as np
from typing import List

from .collisionDetector import detectCollision
from .rigid_body import RigidBody

class Physics:

    def __init__(self, box_size: float = 10.0, gravity: np.ndarray = None):
        self.box_size = box_size
        self.gravity = gravity if gravity is not None else np.zeros(3)
        self.bodies: List[RigidBody] = []
        self.collision_detector = detectCollision
        self.time = 0.0

    def add_body(self, body: RigidBody):
        self.bodies.append(body)

    def remove_body(self, body: RigidBody):
        if body in self.bodies:
            self.bodies.remove(body)

    def step(self, dt: float):

        # Erkenne Kollision
        #contact = self.collision_detector(self.bodies)
        contact = self.collision_detector("../res/Cube.stl", "../res/Cube2.stl")


    def _resolve_collision(self, contact: Contact):
        """
        Löst eine Kollision zwischen zei Körpern
        :param contact: Kollisionsinformationen
        :return:
        """

        # Berechne kombinierte Materialeigenschaften



        # Berechne Reibungskoeffizient



        # Relativgeschwindigkeit am Kontaktpunkt



        # Geschwindigkeit entlang der Normalen



        # Objekte bewegen sich voneinander weg



        # Berechne Impuls



        # Normalenimpuls



        # Reibungsimpuls




        # Positionskorrektur (verhindert Überlappung)

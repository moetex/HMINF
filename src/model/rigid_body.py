"""Starrer Körper"""
import numpy as np
from typing import Optional
from .material import Material
from .geometry import Geometry

class RigidBody:

    _id_counter = 0

    def __init__(self, geometry: Geometry, material: Material,
                 position: np.ndarray,
                 velocity: np.ndarray = None,
                 angular_velocity: np.ndarray = None,
                 rotation: np.ndarray = None):

        """
        Initialisiert einen starreb Körper

        :param geometry: Geometrie des Körpers
        :param material: Material des Körpers
        :param position: Position des Schwerpunktes
        :param velocity: lineare Geschwindigkeit
        :param angular_velocity: Winkelgeschwindigkeit
        :param rotation: Rotationsmatrix (3x3)
        """
        self.id = RigidBody._id_counter
        RigidBody._id_counter += 1

        self.geometry = geometry
        self.material = material

        # Berechne physikalische Eigenschaften
        self.mass = geometry.compute_mass(material.density)
        self.inv_mass = 1.0 / self.mass if self.mass > 0 else 0.0

        self.inertia_tensor_local = geometry.compute_inertia_tensor(self.mass)
        self.inv_inertia_tensor_local = np.linalg.inv(self.inertia_tensor_local)

        # Kinematische Eigenschaften
        self.position = np.array(position, dtype=float)
        self.velocity = (np.array(velocity, dtype=float)
                         if velocity is not None else np.zeros(3))
        self.angular_velocity = (np.array(angular_velocity, dtype=float)
                                 if angular_velocity is not None else np.zeros(3))

        # Orientierung (Rotationsmatrix)
        self.rotation = (np.array(rotation, dtype=float) if rotation is not None else np.eye(3))

        # Kräfte und Momente
        self.force = np.zeros(3)
        self.torque = np.zeros(3)

        # Flags
        self._is_static = False     # Statische Körper bewegen sich nicht

    @property
    def inertia_tensor_world(self) -> np.ndarray:
        """Berechnet den Trägheitstensor in Weltkoordinaten"""
        R = self.rotation
        return R @ self.inertia_tensor_local @ R.T

    @property
    def inv_inertia_tensor_world(self) -> np.ndarray:
        """Berechnet den inversen Trägheitstensor in Weltkoordinaten"""
        R = self.rotation
        return R @ self.inv_inertia_tensor_local @ R.T

    # Kräfte und Momente
    def apply_force(self, force: np.ndarray, point: Optional[np.ndarray] = None):
        """
        Wendet eine Kraft auf den Körper an

        :param force: Kraftvektor
        :param point: Angriffspunkt (Weltkoordinaten). Wenn None, wirkt am Schwerpunkt
        :return:
        """
        if self._is_static:
            return

        self.force += force
        if point is not None:
            # Berechne Drehmoment: τ = r × F
            r = point - self.position
            self.torque = np.cross(r, force)

    # Impulsübertragung bei Kollision
    def apply_impulse(self, impulse: np.ndarray, point: Optional[np.ndarray] = None):
        """
        Wendet einen Impuls auf den Körper an

        Geschwindigkeit: v(t+Δt) += p * m^(-1)
        Drehimpuls: ∆L = r×p
        Winkelgeschwindigkeit: ∆ω = I^(-1)∙(r×p)

        :param impulse: Impulsvektor
        :param point: Angriffspunkt (Weltkoordinaten). Wenn None, wirkt am Schwerpunkt
        :return:
        """
        if self._is_static:
            return

        self.velocity += impulse * self.inv_mass

        if point is not None:
            r = point - self.position
            angular_impulse = np.cross(r, impulse)
            self.angular_velocity += self.inv_inertia_tensor_world @ angular_impulse

    def integrate(self, dt: float):
        """
        Integriert die Bewegungsgleichungen über einen Zeitschritt

        :param dt: Zeitschritt
        :return:
        """
        if self._is_static:
            return

        # Linear: v += F/m * Δt, x += v * Δt
        acceleration = self.force * self.inv_mass   # a = F * m^(-1)
        self.velocity += acceleration * dt          # v(t+Δt) = v(t) + a*Δt
        self.position += self.velocity * dt         # x(t+Δt) = x(t) + v*Δt

        # Angular: ω += I^(-1) * τ * Δt, R += [ω]R * Δt
        angular_acceleration = self.inv_inertia_tensor_world @ self.torque
        self.angular_velocity += angular_acceleration * dt

        # Aktualisiert Rotation (vereinfachte Euler-Integration)
        omega_cross = self._skew_symmetric(self.angular_velocity)
        self.rotation += omega_cross @ self.rotation * dt

        # Orthonormalisieren Rotationsmatrix (verhindert nummerische Drift)
        self.rotation = self._orthonormalize(self.rotation)

        # setze akkumulierte Kräfte zurück
        self.force = np.zeros(3)
        self.torque = np.zeros(3)

    def get_velocity_at_point(self, point: np.ndarray) -> np.ndarray:
        """
        Berechnet die Geschwindigkeit an einem Punkt auf dem Körper
        >> Ein Punkt auf einem rotierenden Körper hat zusätzlich zur Translation (Bewegung) des Schwerpunkts eine Geschwindigkeit durch die Rotation.

        :param point: Punkt in Weltkoordinate
        :return: Geschwindigkeit an diesem Punkt
        """
        r = point - self.position
        return self.velocity + np.cross(self.angular_velocity, r)

    # Kinetische Energie
    def kinetic_energy(self) -> float:
        """
        Berechnet die gesamte kinetische Energie

        E_(trans) = (m·v²) / 2
        E_(rot) =  (I*v^2) / 2
        """
        trans_energy = 0.5 * self.mass * np.dot(self.velocity, self.velocity)
        rot_energy = 0.5 * np.dot(self.angular_velocity, self.inertia_tensor_world @ self.angular_velocity)

        return trans_energy + rot_energy


    @staticmethod
    def _skew_symmetric(v: np.ndarray) -> np.ndarray:
        """Erstellt die schiefsymmetrische Matrix für Kreuzprodukt"""
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

    @staticmethod
    def _orthonormalize(R: np.ndarray) -> np.ndarray:
        """Orthonormalisiert eine Rotationsmatrix (Gram-Schmidtsches Orthogonaliserungsverfahren)"""
        u, s, vh = np.linalg.svd(R)
        return u @ vh

















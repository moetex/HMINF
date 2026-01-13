
import numpy as np
from abc import ABC, abstractmethod

class Geometry(ABC):
    """Abstrakte Basisklasse für Geometrien"""

    @abstractmethod
    def compute_mass(self, density: float) -> float:
        """Berechnet die Masse basierend auf der Dichte"""
        pass

    @abstractmethod
    def compute_inertia_tensor(self, mass: float) -> float:
        """Berechnet den Trägheitstensor"""
        pass



    @abstractmethod
    def get_bounding_radius(self) -> float:
        """Gibt den Begrenzungsradius zurück"""
        pass

    @abstractmethod
    def get_support_point(self, direction: np.ndarray, position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
        """Gibt den am weitesten entfernten Punkt in einer Richtung zurück"""


class SphereGeometry(Geometry):
    """Kugelgeometrie."""

    def __init__(self, radius: float):
        if radius <= 0:
            raise ValueError("Radius muss positiv sein")
        self.radius = radius

    def compute_mass(self, density: float) -> float:
        """Masse = Dichte × (4/3)πr³"""
        volume = (4.0 / 3.0) * np.pi * self.radius ** 3
        return density * volume

    def compute_inertia_tensor(self, mass: float) -> np.ndarray:
        """Trägheitstensor für Kugel: I = (2/5)mr²"""
        I = (2.0 / 5.0) * mass * self.radius ** 2
        return np.eye(3) * I

    def get_bounding_radius(self) -> float:
        return self.radius

    def get_support_point(self, direction: np.ndarray, position: np.ndarray,
                          rotation: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(direction)
        if norm < 1e-10:
            return position
        return position + (direction / norm) * self.radius


class BoxGeometry(Geometry):
    """Quadergeometrie."""

    def __init__(self, width: float, height: float, depth: float):
        if width <= 0 or height <= 0 or depth <= 0:
            raise ValueError("Alle Dimensionen müssen positiv sein")
        self.width = width
        self.height = height
        self.depth = depth

    def compute_mass(self, density: float) -> float:
        """Masse = Dichte × Volumen"""
        volume = self.width * self.height * self.depth
        return density * volume

    def compute_inertia_tensor(self, mass: float) -> np.ndarray:
        """Trägheitstensor für Quader."""
        Ix = (1.0 / 12.0) * mass * (self.height ** 2 + self.depth ** 2)
        Iy = (1.0 / 12.0) * mass * (self.width ** 2 + self.depth ** 2)
        Iz = (1.0 / 12.0) * mass * (self.width ** 2 + self.height ** 2)
        return np.diag([Ix, Iy, Iz])

    def get_bounding_radius(self) -> float:
        """Diagonale des Quaders / 2"""
        return 0.5 * np.sqrt(self.width ** 2 + self.height ** 2 + self.depth ** 2)

    def get_support_point(self, direction: np.ndarray, position: np.ndarray,
                          rotation: np.ndarray) -> np.ndarray:
        """Gibt die Ecke zurück, die am weitesten in Richtung liegt."""
        # Transformiere Richtung in lokale Koordinaten
        local_dir = rotation.T @ direction

        # Finde die am weitesten entfernte Ecke
        half_extents = np.array([self.width, self.height, self.depth]) / 2.0
        local_point = np.sign(local_dir) * half_extents

        # Transformiere zurück in Weltkoordinaten
        return position + rotation @ local_point


class CylinderGeometry(Geometry):
    """Zylindergeometrie."""

    def __init__(self, radius: float, height: float):
        if radius <= 0 or height <= 0:
            raise ValueError("Radius und Höhe müssen positiv sein")
        self.radius = radius
        self.height = height

    def compute_mass(self, density: float) -> float:
        """Masse = Dichte × πr²h"""
        volume = np.pi * self.radius ** 2 * self.height
        return density * volume

    def compute_inertia_tensor(self, mass: float) -> np.ndarray:
        """Trägheitstensor für Zylinder (Achse in y-Richtung)."""
        Ix = (1.0 / 12.0) * mass * (3 * self.radius ** 2 + self.height ** 2)
        Iy = 0.5 * mass * self.radius ** 2
        Iz = Ix
        return np.diag([Ix, Iy, Iz])

    def get_bounding_radius(self) -> float:
        return max(self.radius,
                   0.5 * np.sqrt(4 * self.radius ** 2 + self.height ** 2))

    def get_support_point(self, direction: np.ndarray, position: np.ndarray,
                          rotation: np.ndarray) -> np.ndarray:
        """Approximation für Support-Funktion."""
        local_dir = rotation.T @ direction

        # Radiale Komponente
        radial = np.array([local_dir[0], 0, local_dir[2]])
        radial_norm = np.linalg.norm(radial)
        if radial_norm > 1e-10:
            radial = radial / radial_norm * self.radius

        # Höhenkomponente
        axial = np.array([0, np.sign(local_dir[1]) * self.height / 2.0, 0])

        local_point = radial + axial
        return position + rotation @ local_point


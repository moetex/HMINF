"""
Materialklasse für physikalische Eigenschaften.
"""
import numpy as np
from dataclasses import dataclass
from typing import Dict

@dataclass
class Material:
    """
    Attributes:
        name: Name des Materials
        density: Dichte in kg/m³
        restitution: Restitutionskoeffizient (0-1)
        static_friction: Haftreibungskoeffizient
        kinetic_friction: Gleitreibungskoeffizient
        color: Farbe zur Visualisierung
    """
    name: str
    density: float
    restitution: float
    static_friction: float
    kinetic_friction: float
    color: tuple = (0.5, 0.5, 0.5)

    def __pos_init__(self):
        if self.density <= 0:
            raise ValueError("Materialdichte muss positiv sein")
        if not 0 <= self.restitution <= 1:
            raise ValueError("Restitution muss zwischen 0 und 1 sein")
        if self.static_friction < 0 or self.kinetic_friction < 0:
            raise ValueError("Reibungskoeffizienten dürfen nicht positiv sein")
        if self.static_friction <= self.kinetic_friction:
            raise ValueError("Haftreibung muss größer oder gleich Gleitreibung sein")


class MaterialLibrary:
    """verwaltet vordefinierte Materialien"""

    def __init__(self):
        self._materials: Dict[str, Material] = {}
        self._initialize_default_materials()

    def _initialize_default_materials(self):
        """Initialisation Standard-Materialien"""
        self.add_material(Material(
            name="Stahl",
            density=7850.0,
            restitution=0.6,
            static_friction=0.74,
            kinetic_friction=0.57,
            color=(0.7, 0.7, 0.8)
        ))

        self.add_material(Material(
            name="Holz",
            density=700.0,
            restitution=0.4,
            static_friction=0.5,
            kinetic_friction=0.3,
            color=(0.6, 0.4, 0.2)
        ))
        """
        self.add_material(Material(
            name="",
            density=,
            restitution=,
            static_friction=,
            kinetic_friction=,
            color=()
        ))

        self.add_material(Material(
            name="",
            density=,
            restitution=,
            static_friction=,
            kinetic_friction=,
            color=()
        ))
        """
        #self.add_material(Material(
        #    name="",
        #    density=,
        #    restitution=,
        #    static_friction=,
        #    kinetic_friction=,
        #    color=()
        #))



    def add_material(self, material: Material):
        """Fügt ein Material zur Bibliothek hinzu"""
        self._materials[material.name] = material

    def get_material(self, name:str) -> Material:
        if name not in self._materials:
            raise KeyError(f"Material '{name}' nicht gefunden")
        return self._materials[name]

    def list_materials(self):
        return list(self._materials.keys())
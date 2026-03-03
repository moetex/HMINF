from dataclasses import dataclass
from typing import Dict
from vpython import color

@dataclass(frozen=True)
class Material:
    """
    Repräsentiert ein Material mit seinen Eigenschaften

    Attribute:
        name: Name des Materials
        density: Materialdichte in kg/m^3
    """

    name: str
    density: float
    color: object

@dataclass(frozen=True)
class PairProps:
    mu_s: float     # Haftreibung
    mu_k: float     # Gleitreibung
    e: float        # Restitution


class MaterialLibrary:
    """verwaltet vordefinierte Materialien"""

    MATERIALS = {
        'Holz': Material("Holz", 425.0, color.orange),
        'Stahl': Material("Stahl", 7850.0, color.gray(0.7)),

    }

    @classmethod
    def get(cls, name: str) -> Material:
        """Gibt Material anhand des Namens zurück"""
        return cls.MATERIALS.get(name, cls.MATERIALS["Stahl"])



class PairLookup:
    _pars = {
        frozenset(("Stahl", "Stahl")): PairProps(mu_s=0.2, mu_k=0.12, e=0.6),
        frozenset(("Stahl", "Holz")): PairProps(mu_s=0.5, mu_k=0.4, e=0.5),
        frozenset(("Holz", "Holz")): PairProps(mu_s=0.5, mu_k=0.3, e=0.5),
    }


    @classmethod
    def get(cls, a: Material, b: Material) -> PairProps:
        name = frozenset((a.name, b.name))
        if name in cls._pars:
            return cls._pars[name]
        # Fallback (wenn Paar fehlt)
        return PairProps(mu_s=0.4, mu_k=0.25, e=0.5)

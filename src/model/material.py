from dataclasses import dataclass
from typing import Dict


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
    #restitution: float
    #static_friction: float
    #kinetic_friction: float
    #color: tuple = (0.5, 0.5, 0.5)

    """
    def __post_init__(self):
        #""Validierung der Materialeigenschaften.""
        if self.density <= 0:
            raise ValueError("Dichte muss positiv sein")
        if not 0 <= self.restitution <= 1:
            raise ValueError("Restitution muss zwischen 0 und 1 liegen")
        if self.static_friction < 0 or self.kinetic_friction < 0:
            raise ValueError("Reibungskoeffizienten müssen nicht-negativ sein")
        if self.static_friction < self.kinetic_friction:
            raise ValueError("Haftreibung muss größer oder gleich Gleitreibung sein")
    """

@dataclass(frozen=True)
class PairProps:
    mu_s: float     # Haftreibung
    mu_k: float     # Gleitreibung
    e: float        # Restitution


class MaterialLibrary:
    """verwaltet vordefinierte Materialien"""
    STAHL = Material("Stahl", 7850.0)
    HOLZ = Material("Holz", 425.0)



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
        # Fallback (wenn Paar fehlt): moderate Standardwerte
        return PairProps(mu_s=0.4, mu_k=0.25, e=0.5)


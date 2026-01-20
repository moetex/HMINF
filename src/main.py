from __future__ import annotations
import sys
import traceback

from pathlib import Path
from src.controller.programController import Simulation
from src.util.config import SimConfig
from src.util.log import setup_logging
from src.sim.core import SimulationCore
from src.view.vpython_ui import VpythonUI


# ----------------------------------------
# Main Entry Point
# ----------------------------------------

def main():
    """Haupteinstiegspunkt"""
    stl_path = "res/labubu_keychain_voxel_mc_16k.stl"

    try:
        sim = Simulation(stl_path)
        sim.run()
    except FileNotFoundError:
        print(f"STL-Datei nicht gefunden: {stl_path} ")
        sys.exit(1)
    except Exception as e:
        print(f"Ein unerwarteter Fehler ist aufgetreten: {e} ")
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)


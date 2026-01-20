from __future__ import annotations
import sys
import traceback

from src.controller.programController import Simulation


# ----------------------------------------
# Main Entry Point
# ----------------------------------------

def main():
    """Haupteinstiegspunkt"""
    stl_path = "../res/labubu_keychain_voxel_mc_16k.stl"

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


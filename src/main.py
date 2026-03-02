from __future__ import annotations
import sys
import traceback
from pathlib import Path
import os, sys

from src.controller.programController import Simulation


# ----------------------------------------
# Main Entry Point
# ----------------------------------------
def resource_path(filename: str) -> str:
    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent  # dist\main

        candidates = [
            exe_dir / "res" / filename,
            exe_dir / "_internal" / "res" / filename,
            exe_dir / "_internal" / filename,   # fallback falls target anders war
        ]

        # last resort: suchen
        hits = list(exe_dir.rglob(filename))
        if hits:
            return str(hits[0])

        raise FileNotFoundError(f"{filename} not found. Tried: {candidates}")

    # normaler Run aus src/
    project_root = Path(__file__).resolve().parents[1]
    return str(project_root / "res" / filename)

def main():
    """Haupteinstiegspunkt"""
    stl_path = resource_path("labubu.stl")
    with open(stl_path, "rb") as f:
        f.read(16)
    print("DEBUG STL open OK")

    try:
        print("DEBUG sys.executable:", sys.executable)
        print("DEBUG cwd:", os.getcwd())
        print("DEBUG stl_path repr:", repr(stl_path))

        p = Path(stl_path)
        print("DEBUG exists:", p.exists(), "is_file:", p.is_file())
        print("DEBUG abs:", str(p.resolve()))
        print("DEBUG parent list:", [x.name for x in p.parent.glob("*")])
        sim = Simulation(stl_path)
        sim.run()
    except FileNotFoundError as e:
        print("FileNotFoundError:", e)
        traceback.print_exc()
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
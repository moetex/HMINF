from __future__ import annotations

from src.controller.programController import Simulation

def main():
    sim = Simulation(r"res/Cube.stl")
    sim.run()


if __name__ == '__main__':
    main()



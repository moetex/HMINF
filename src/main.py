from __future__ import annotations

from src.controller.programController import Simulation

def main():
    sim = Simulation(r"res/labubu_keychain_voxel_mc_16k.stl")
    sim.run()


if __name__ == '__main__':
    main()



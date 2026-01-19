from __future__ import annotations

from pathlib import Path

from src.util.config import SimConfig
from src.util.log import setup_logging
from src.sim.core import SimulationCore
from src.view.vpython_ui import VpythonUI


def main():
    cfg = SimConfig()
    log = setup_logging(debug=True, log_to_file=True)
    sim = SimulationCore(r"res/labubu_keychain_voxel_mc_16k.stl", cfg, log)
    ui = VpythonUI(sim, r"res/labubu_keychain_voxel_mc_16k.stl", cfg)
    ui.run()


if __name__ == '__main__':
    main()


"""
from pathlib import Path

from src.util.config import SimConfig
from src.util.log import setup_logging
from src.sim.core import SimulationCore
from src.view.vpython_ui import VpythonUI

def main():
    cfg = SimConfig()
    log = setup_logging(debug=True, log_to_file=True)

    # STL relativ zum Projektroot
    project_root = Path(__file__).resolve().parents[2]
    stl_path = project_root / "res" / "labubu_keychain_voxel_mc_16k.stl"

    sim = SimulationCore(str(stl_path), cfg, log)
    ui = VpythonUI(sim, str(stl_path), cfg)
    ui.run()

if __name__ == "__main__":
    main()
    """




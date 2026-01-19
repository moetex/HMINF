import logging
from pathlib import Path

def setup_logging(debug: bool = True, log_to_file: bool = True) -> logging.Logger:
    logger = logging.getLogger("sim")
    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    logger.handlers.clear()

    fmt = logging.Formatter("[%(levelname)s] %(asctime)s %(name)s: %(message)s")

    ch = logging.StreamHandler()
    ch.setFormatter(fmt)
    ch.setLevel(logging.DEBUG if debug else logging.INFO)
    logger.addHandler(ch)

    if log_to_file:
        Path("logs").mkdir(exist_ok=True)
        fh = logging.FileHandler("logs/sim.log", encoding="utf-8")
        fh.setFormatter(fmt)
        fh.setLevel(logging.DEBUG)
        logger.addHandler(fh)

    return logger

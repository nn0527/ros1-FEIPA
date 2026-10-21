#!/usr/bin/env python3
"""Run the unified test console from a source checkout, without sourcing ROS."""
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).resolve().parents[2]/'src'))
from px4_wall_ceiling_control.ceiling_wall_console.server import main

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Consolida los resúmenes JSON de varias misiones en una tabla con media y desviación estándar.

Equivale a `ros2 run usar_metrics consolidate`. Uso:
    scripts/consolidate_results.py results/batch_20261119_120000
    scripts/consolidate_results.py results/seed*.json --out results/tabla
"""

import sys

from usar_metrics.consolidate import main

if __name__ == '__main__':
    sys.exit(main())

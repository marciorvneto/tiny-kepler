#!/usr/bin/env python3
"""Plot orbital mechanics output from tiny-kepler.
Reads TSV from stdin: t  x  y  r  energy
Usage: ./out/main | python3 plot.py
"""

import sys
import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt(sys.stdin)
t, x, y, r, energy = data.T

fig, axes = plt.subplots(1, 3, figsize=(15, 5))

# Orbit trajectory
ax = axes[0]
ax.plot(x, y, linewidth=0.5)
ax.plot(0, 0, "yo", markersize=8)
ax.plot(x[0], y[0], "go", markersize=6, label="start")
ax.plot(x[-1], y[-1], "rx", markersize=8, label="end")
ax.set_aspect("equal")
ax.set_title("Orbit")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.legend()
ax.grid(True, alpha=0.3)

# Radius over time
ax = axes[1]
ax.plot(t, r, linewidth=0.5)
ax.set_title("Radius vs Time")
ax.set_xlabel("t")
ax.set_ylabel("r")
ax.grid(True, alpha=0.3)

# Energy over time
ax = axes[2]
ax.plot(t, energy, linewidth=0.5)
ax.set_title("Energy vs Time")
ax.set_xlabel("t")
ax.set_ylabel("ε")
ax.grid(True, alpha=0.3)

fig.tight_layout()
plt.savefig("orbit.png", dpi=150)
plt.show()

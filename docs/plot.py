#!/usr/bin/env python3
# Plots for the BVH comparison study. Reads docs/benchmark.csv (produced by
# `--bench` / [F1]) and writes PNG figures into docs/.
#
#   python3 docs/plot.py
import csv, os, sys
from collections import defaultdict
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
CSV  = os.path.join(HERE, "benchmark.csv")
if not os.path.exists(CSV):
    sys.exit("benchmark.csv not found: run ./bvh_raytracer --bench first")

rows = []
with open(CSV) as f:
    for r in csv.DictReader(f):
        rows.append({k: (int(v) if k in ("tris","node_count") else float(v)) if k not in ("strategy","density") else v
                     for k, v in r.items()})

STRAT = ["SAH", "Median", "Morton"]
COLOR = {"SAH": "#2a9d8f", "Median": "#e9c46a", "Morton": "#e76f51"}

# ---- Figure 1: build vs render tradeoff (Medium density) --------------------
med = {r["strategy"]: r for r in rows if r["density"] == "Medium"}
fig, ax = plt.subplots(figsize=(6,4))
for s in STRAT:
    r = med[s]
    ax.scatter(r["build_ms"], r["render_ms"], s=160, color=COLOR[s], label=s, zorder=3)
    ax.annotate(s, (r["build_ms"], r["render_ms"]),
                textcoords="offset points", xytext=(8,6), fontweight="bold")
ax.set_xlabel("Tiempo de construcción (ms)  ->  mas rapido a la izquierda")
ax.set_ylabel("Tiempo de render (ms)  ->  mas rapido abajo")
ax.set_title("Tradeoff build vs render (escena Medium, ~20k tri)")
ax.grid(alpha=0.3); ax.legend()
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_tradeoff.png"), dpi=140); plt.close(fig)

# ---- Figure 2: tree quality = nodes visited per ray (grouped by density) ----
dens = ["Small", "Medium", "Large"]
import numpy as np
x = np.arange(len(dens)); w = 0.25
fig, ax = plt.subplots(figsize=(7,4))
for i, s in enumerate(STRAT):
    vals = [next(r for r in rows if r["strategy"]==s and r["density"]==d)["nodes_per_ray"] for d in dens]
    ax.bar(x + (i-1)*w, vals, w, label=s, color=COLOR[s])
ax.set_xticks(x); ax.set_xticklabels(dens)
ax.set_ylabel("Nodos BVH visitados por rayo (menos = mejor)")
ax.set_title("Calidad del arbol por estrategia (independiente del hardware)")
ax.grid(axis="y", alpha=0.3); ax.legend()
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_quality.png"), dpi=140); plt.close(fig)

# ---- Figure 3: scaling — render time vs triangle count ----------------------
fig, ax = plt.subplots(figsize=(7,4))
for s in STRAT:
    pts = sorted([(r["tris"], r["render_ms"]) for r in rows if r["strategy"]==s])
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, "-o", color=COLOR[s], label=s)
ax.set_xlabel("Numero de triangulos")
ax.set_ylabel("Tiempo de render (ms)")
ax.set_title("Escalado: tiempo de render vs tamanio de la escena")
ax.grid(alpha=0.3); ax.legend()
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_scaling.png"), dpi=140); plt.close(fig)

print("wrote: fig_tradeoff.png, fig_quality.png, fig_scaling.png in", HERE)

#!/usr/bin/env python3
# Plots for the BVH comparison study. Reads docs/benchmark.csv (produced by
# `--bench` / [F1]) and writes PNG figures into docs/.
#
#   python3 docs/plot.py
#
# The CSV is a full matrix: backend in {Custom, GPU, Embree} x strategy in
# {SAH, Median, Morton} x density in {Small, Medium, Large}. Not every column
# applies to every backend (see the header comment in sr_benchmark.cpp), so each
# figure filters to the backends for which its metric is meaningful.
import csv, os, sys
from collections import defaultdict
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CSV  = os.path.join(HERE, "benchmark.csv")
if not os.path.exists(CSV):
    sys.exit("benchmark.csv not found: run ./bvh_raytracer --bench first")

rows = []
with open(CSV) as f:
    for r in csv.DictReader(f):
        rows.append({k: (int(v) if k in ("tris", "node_count") else float(v))
                        if k not in ("backend", "strategy", "density") else v
                     for k, v in r.items()})

# Back-compat: an old CSV without a `backend` column is all custom-BVH rows.
for r in rows:
    r.setdefault("backend", "Custom")

STRAT = ["SAH", "Median", "Morton"]
DENS  = ["Small", "Medium", "Large"]
COLOR = {"SAH": "#2a9d8f", "Median": "#e9c46a", "Morton": "#e76f51"}
BACKENDS = sorted({r["backend"] for r in rows})
HAS_GPU     = "GPU" in BACKENDS
HAS_EMBREE  = "Embree" in BACKENDS
HAS_DYNAMIC = "Dynamic" in BACKENDS

def get(backend, strat, dens):
    for r in rows:
        if r["backend"] == backend and r["strategy"] == strat and r["density"] == dens:
            return r
    return None

# ---- Figure 1: build vs render tradeoff (custom BVH, Medium density) --------
fig, ax = plt.subplots(figsize=(6, 4))
for s in STRAT:
    r = get("Custom", s, "Medium")
    if not r: continue
    ax.scatter(r["build_ms"], r["render_ms"], s=160, color=COLOR[s], label=s, zorder=3)
    ax.annotate(s, (r["build_ms"], r["render_ms"]),
                textcoords="offset points", xytext=(8, 6), fontweight="bold")
ax.set_xlabel("Tiempo de construccion (ms)  ->  mas rapido a la izquierda")
ax.set_ylabel("Tiempo de render CPU (ms)  ->  mas rapido abajo")
ax.set_title("Tradeoff build vs render (BVH propio, escena Medium)")
ax.grid(alpha=0.3); ax.legend()
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_tradeoff.png"), dpi=140); plt.close(fig)

# ---- Figure 2: tree quality = nodes visited per ray (custom BVH) ------------
# Hardware-independent: it's a property of the tree, so the GPU rows share it.
x = np.arange(len(DENS)); w = 0.25
fig, ax = plt.subplots(figsize=(7, 4))
for i, s in enumerate(STRAT):
    vals = [get("Custom", s, d)["nodes_per_ray"] for d in DENS]
    ax.bar(x + (i - 1) * w, vals, w, label=s, color=COLOR[s])
ax.set_xticks(x); ax.set_xticklabels(DENS)
ax.set_ylabel("Nodos BVH visitados por rayo (menos = mejor)")
ax.set_title("Calidad del arbol por estrategia (independiente del hardware)")
ax.grid(axis="y", alpha=0.3); ax.legend()
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_quality.png"), dpi=140); plt.close(fig)

# ---- Figure 3: scaling — full-frame render vs triangle count, CPU vs GPU ----
# render_ms is the full-frame cost, comparable between the CPU thread-pool and
# the GPU kernel (both traverse the same tree). Solid = CPU, dashed = GPU.
fig, ax = plt.subplots(figsize=(7, 4))
for s in STRAT:
    cpu = [get("Custom", s, d) for d in DENS]
    xs  = [r["tris"] for r in cpu]
    ax.plot(xs, [r["render_ms"] for r in cpu], "-o", color=COLOR[s], label=f"{s} CPU")
    if HAS_GPU:
        gpu = [get("GPU", s, d) for d in DENS]
        ax.plot(xs, [r["render_ms"] for r in gpu], "--s", color=COLOR[s], label=f"{s} GPU")
ax.set_xlabel("Numero de triangulos")
ax.set_ylabel("Tiempo de render por frame (ms)")
ax.set_title("Escalado del render: CPU (multihilo) vs GPU (OpenCL)")
ax.grid(alpha=0.3); ax.legend(fontsize=8, ncol=2)
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_scaling.png"), dpi=140); plt.close(fig)

# ---- Figure 4: traversal cost, our BVH vs Intel Embree (per strategy) --------
# Pure single-threaded primary-ray intersection on the SAME geometry -> a fair
# apples-to-apples of the traversal kernels. Embree HIGH/MEDIUM/LOW is built to
# mirror our SAH/Median/Morton, so we group by strategy.
if HAS_EMBREE:
    x = np.arange(len(STRAT)); w = 0.38
    fig, ax = plt.subplots(figsize=(7, 4))
    ours = [get("Custom", s, "Medium")["traversal_ms"] for s in STRAT]
    emb  = [get("Embree", s, "Medium")["traversal_ms"] for s in STRAT]
    ax.bar(x - w/2, ours, w, label="BVH propio (CPU)", color="#264653")
    ax.bar(x + w/2, emb,  w, label="Embree",            color="#e76f51")
    for i, v in enumerate(ours): ax.text(i - w/2, v, f"{v:.0f}", ha="center", va="bottom", fontsize=8)
    for i, v in enumerate(emb):  ax.text(i + w/2, v, f"{v:.0f}", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels(STRAT)
    ax.set_ylabel("Traversal de rayos primarios (ms) — menos es mejor")
    ax.set_title("Costo de traversal: BVH propio vs Intel Embree (escena Medium)")
    ax.grid(axis="y", alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_embree.png"), dpi=140); plt.close(fig)

# ---- Figure 5: GPU speedup over CPU (full-frame render, per strategy) --------
# speedup = render_ms(CPU) / render_ms(GPU): how many times faster the same tree
# renders when the traversal runs as one work-item per pixel on the GPU.
if HAS_GPU:
    x = np.arange(len(DENS)); w = 0.25
    fig, ax = plt.subplots(figsize=(7, 4))
    for i, s in enumerate(STRAT):
        vals = [get("Custom", s, d)["render_ms"] / get("GPU", s, d)["render_ms"] for d in DENS]
        bars = ax.bar(x + (i - 1) * w, vals, w, label=s, color=COLOR[s])
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width()/2, v, f"{v:.0f}x", ha="center", va="bottom", fontsize=8)
    ax.axhline(1.0, color="gray", lw=1, ls=":")
    ax.set_xticks(x); ax.set_xticklabels(DENS)
    ax.set_ylabel("Aceleracion GPU vs CPU (x veces)")
    ax.set_title("Speedup de la GPU (OpenCL) sobre el render multihilo de CPU")
    ax.grid(axis="y", alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_speedup.png"), dpi=140); plt.close(fig)

# ---- Figure: three backends head-to-head — full-frame render (Large) ---------
# All three backends render a COMPLETE frame (CPU thread-pool, GPU kernel, and
# Embree = our shading + Embree intersection), so render/frame is comparable.
# Grouped by heuristic on the Large scene; log-y because the range is wide.
if HAS_GPU and HAS_EMBREE:
    x = np.arange(len(STRAT)); w = 0.26
    series = [("CPU propio", "Custom", "#264653"),
              ("GPU OpenCL", "GPU",    "#2a9d8f"),
              ("Embree",     "Embree", "#e76f51")]
    fig, ax = plt.subplots(figsize=(7.5, 4.3))
    for i, (lab, bk, col) in enumerate(series):
        vals = [get(bk, s, "Large")["render_ms"] for s in STRAT]
        bars = ax.bar(x + (i - 1) * w, vals, w, label=lab, color=col)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width()/2, v, f"{v:.1f}", ha="center", va="bottom", fontsize=7)
    ax.set_yscale("log")
    ax.set_xticks(x); ax.set_xticklabels(STRAT)
    ax.set_ylabel("Render por frame (ms, escala log) — menos es mejor")
    ax.set_title("Los tres backends, frame completo (escena Large, 204 812 tri)")
    ax.grid(axis="y", alpha=0.3, which="both"); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_backends.png"), dpi=140); plt.close(fig)

# ---- Figure 6: dynamic BVH rebuilt per frame — build + traversal per frame ---
# When the tree is rebuilt EVERY frame the honest cost is build + traversal. SAH
# builds slowly and can't amortize it, so a cheap builder (Morton) wins on total.
# Solid = build, hatched = traversal; the number on top is the per-frame total.
if HAS_DYNAMIC:
    x = np.arange(len(DENS)); w = 0.25
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    for i, s in enumerate(STRAT):
        builds = [get("Dynamic", s, d)["build_ms"]     for d in DENS]
        travs  = [get("Dynamic", s, d)["traversal_ms"] for d in DENS]
        pos = x + (i - 1) * w
        ax.bar(pos, builds, w, color=COLOR[s], label=f"{s}")
        ax.bar(pos, travs,  w, bottom=builds, color=COLOR[s], alpha=0.4, hatch="//")
        for p, b, t in zip(pos, builds, travs):
            ax.text(p, b + t, f"{b+t:.1f}", ha="center", va="bottom", fontsize=7, fontweight="bold")
    tiers = [f"{d}\n({get('Dynamic', 'SAH', d)['tris']//72} peds)" for d in DENS]
    ax.set_xticks(x); ax.set_xticklabels(tiers)
    ax.set_ylabel("Costo por frame (ms): build (solido) + traversal (rayado)")
    ax.set_title("BVH dinamico reconstruido cada frame — build NO se amortiza")
    ax.grid(axis="y", alpha=0.3); ax.legend(title="Estrategia")
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_dynamic.png"), dpi=140); plt.close(fig)

made = ["fig_tradeoff", "fig_quality", "fig_scaling"]
if HAS_EMBREE:            made.append("fig_embree")
if HAS_GPU:              made.append("fig_speedup")
if HAS_GPU and HAS_EMBREE: made.append("fig_backends")
if HAS_DYNAMIC:          made.append("fig_dynamic")
print("wrote:", ", ".join(m + ".png" for m in made), "in", HERE)

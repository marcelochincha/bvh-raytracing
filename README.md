# BVH Raytracer — Estudio comparativo de estructuras de aceleración

Ray tracer por software (CPU, multihilo) cuya estructura central es un **BVH
(Bounding Volume Hierarchy)** construido con la **heurística de área de
superficie (SAH)** mediante *binning*. El proyecto es un **estudio comparativo**
de estructuras/estrategias de construcción para el trazado de rayos.

> Proyecto final — CS3014 Estructura de Datos Avanzados (2026-1).

---

## ¿Qué es un BVH?

El ray tracer ingenuo prueba cada rayo contra **todos** los triángulos: `O(N)`
por rayo, `O(P·N)` por frame. Un BVH agrupa los triángulos en un **árbol de
cajas alineadas a los ejes (AABB)** anidadas: un rayo prueba primero las cajas
baratas y solo desciende por las que golpea, descartando subárboles enteros. El
costo por rayo baja de `O(N)` a `~O(log N)`.

La calidad del árbol depende de **cómo se divide** cada caja. Aquí comparamos
varias estrategias de construcción (ver *Roadmap*).

---

## Estructura del código

```
src/
  main.cpp                      # bucle principal + SDL
  sr_config.hpp                 # parseo de argumentos (--width --height --fps)
  math/                         # álgebra lineal (vec3/mat4, helpers)
  io/                           # stb_image (carga de texturas)
  renderer/
    bvh.hpp / bvh.cpp           # ★ BVH: build (SAH + binning) + traversal
    sr_renderer.*               # rasterizador (soft-renderer base)
    sr_geometry.*               # meshes + carga .ply
    sr_camera.*  sr_texture.*  sr_framebuffer.hpp  sr_text.*  sr_render_config.hpp
  game/
    sr_game.cpp                 # escena, trazado, sombras, reflejos, multihilo
res/textures/skybox3/           # cubemap del cielo
docs/                           # informe (LaTeX)
```

## Dependencias

- **SDL2** (única dependencia). Opcionalmente `SDL2_mixer` si se reactiva el
  sonido (desactivado en este repo).

Instalación:

| Plataforma | Comando |
|---|---|
| macOS (Homebrew) | `brew install sdl2` |
| Windows | descargar las *development libraries* de SDL2 (MSVC o MinGW) |
| Linux (Debian/Ubuntu) | `sudo apt install libsdl2-dev` |

## Cómo compilar y ejecutar

Usa **CMake** (multiplataforma: macOS Apple Silicon nativo, Windows, Linux):

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# Ejecutar (desde la raíz del repo, para que encuentre res/)
./build/bin/bvh_raytracer --width 814 --height 480 --fps 60
```

> En Windows el binario será `build\bin\bvh_raytracer.exe`. Asegúrate de que
> `SDL2.dll` esté accesible (junto al ejecutable o en el `PATH`).

## Controles

- `TAB` — alternar rasterizador / ray tracer
- `B` — alternar BVH (rápido) vs fuerza bruta `O(N)` (lento)
- `V` — superponer las cajas del BVH (debug)
- `WASD` + ratón — cámara

## Roadmap

- [ ] Heurísticas de construcción adicionales (median split, Morton/LBVH)
- [ ] Referencia externa con Intel Embree
- [ ] Instrumentación de métricas (build/render, nodos visitados/rayo, memoria)
- [ ] Mejor visualización (slider de profundidad, ruta de un rayo de debug)
- [ ] Escenas más ricas (modelos estándar + generador procedural para escalado)

## Autores

- *(tu nombre)*
- *(compañeros)*

## Referencias

1. J. Goldsmith, J. Salmon. *Automatic Creation of Object Hierarchies for Ray Tracing*. IEEE CG&A, 1987.
2. D. MacDonald, K. Booth. *Heuristics for Ray Tracing Using Space Subdivision*. 1990. — la SAH.
3. I. Wald. *On Fast Construction of SAH-based Bounding Volume Hierarchies*. 2007. — construcción por *binning*.
4. T. Möller, B. Trumbore. *Fast, Minimum Storage Ray/Triangle Intersection*. 1997.
5. A. Williams et al. *An Efficient and Robust Ray-Box Intersection Algorithm*. 2005.
6. M. Pharr, W. Jakob, G. Humphreys. *Physically Based Rendering* (4th ed.), 2023. Ch. 4.

# Non-Destructive Mechanical Energy Absorption

Cellular mechanical metamaterial that absorbs energy via rotation-based magnetic snap-through, not material deformation. Goal: universal absorption — any loading axis, any modality (tension / compression / shear).

![Concept](https://github.com/user-attachments/assets/62debbbb-af52-409f-9879-c469a876ca61)

## Motivation

Existing cellular metamaterials inherit the low density of foams while improving predictability and re-usability through ordered microstructures that stay elastic across the loading-unloading cycle. Two limits in the existing literature:

1. **Single-axis bias** — most architectures absorb effectively only along one loading direction.
2. **Single-modality bias** — designs target either tension, compression, or shear, never all three.

Both limits stem from using **translation-based** snapping elements.

## Approach

Use **rotation-based** snap-through transitions instead of translation. Outcomes:

- Energy absorption from loading along any axis and in any modality.
- Design space expands beyond strictly periodic arrangements to quasi-periodic, polycrystalline, and amorphous structures.

## Node design

A node is a magnet-loaded disc assembly: a bottom disc carries a circular array of axial magnets; a top disc carries the same array, rotated by half a pitch so each top magnet sits between two bottom magnets. Both faces present the same poles, so the magnets repel.

<img width="611" alt="Node geometry" src="https://github.com/user-attachments/assets/ccc76e49-e9f9-4d89-b85d-46594f3210d5">

Rotating the top disc against the bottom disc produces a bistable energy landscape: torque builds, the top disc snaps to the next well, energy is absorbed, no permanent deformation. The mechanism is analogous to a bistable spring — equally spaced energy wells, continuous absorption as the system walks well-to-well.

![Bistable energy landscape](https://github.com/user-attachments/assets/d2c4b185-7717-41c2-8fc1-fc2d5d637716)

## Lattice

Nodes wire together into a chiral lattice that converts axial load into per-node rotation. The lattice absorbs energy from multiple loading directions.

https://github.com/user-attachments/assets/e0c1d725-a7ef-44b0-91d4-17e20f60732f

![Lattice](https://github.com/user-attachments/assets/ef758c9e-9f66-4841-b1a8-a769daa91c86)

## Repo layout

| Path | Contents |
|---|---|
| [`CAD/`](CAD) | STL files for the bot, full assembly, soft-jaws, and mounts |
| [`Machining/`](Machining) | Two-WCS softjaw prototyping + tabbed high-volume runs (Tormach + Fusion 360 CAM) |
| [`Testing/`](Testing) | MTS tensile tester runs on a 4×4 chiral structure, raw DAQ data, and force / displacement plots |

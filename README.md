# vexelray-sim-rigid

Rigid-body simulation for VexelRay: bodies, contact and constraints. It uses the same approach as
[vexelray-sim-fluid](../vexelray-sim-fluid): dynamics written in SupirVast IR, and one source that runs on both the
GPU and the CPU.

As with the fluid, the work is a series of experiments: each technique is built far enough to be measured, and the
implementation is chosen from the results. [docs/architecture.md](docs/architecture.md) says where it sits;
[docs/TODO.md](docs/TODO.md) has what has been measured so far and what comes next.

| Module | What it holds |
| --- | --- |
| `vexelray-sim-rigid-core` | The kernels, in SupirVast IR, and the diagnostics that judge a state. No engine, no window. So far: spheres in a walled box, with position-based contact (XPBD, Jacobi or Gauss–Seidel), substepped, and a grid broad phase. |
| `vexelray-sim-rigid-gui` | The simulation on the stack: `SphereSimulation` steps it on resident buffers on the window's own GPU, and `SphereView` ray-traces the spheres straight from the simulation's buffer, blended between the last two steps. |
| `vexelray-sim-rigid-demo` | A framework application: the scenarios, the solver's settings beside the readings they move. Its timing is Kronometer's. The physics is a fixed 60 Hz rate inside a tempo the playback speed scales, and a `Handoff` gives its steps to the render thread, which owns the GPU. |

It knows nothing of water. A body floating or pushed by a wake is
[vexelray-sim-physics](../vexelray-sim-physics), which couples this to the fluid.

## Status

**Two solvers and a broad phase, measured, and a demo.** Spheres with XPBD contact are right where an answer
is known: an exact free fall, a sphere resting on the floor, momentum kept in a collision to 1e-6. A grid broad
phase, on a counting sort that now lives in SupirVast, finds the same contacts as testing every pair, 11× faster at
4096 spheres.

Jacobi, every sphere solved at once, cannot rest a stack: a column of twenty sinks a few percent of a radius into
itself and jitters. Gauss–Seidel over a contact list, solved in rounds of contacts that share no sphere, one
invocation per contact, rests the column with under a millionth of Jacobi's moving energy, and lets a pile settle.
On a pile of 4096 it costs 1.2 ms a step at 10 substeps, about twice what Jacobi costs for the same overlap. The
demo runs Jacobi on the grid. There is no rotation, so no friction, and no restitution.

Next is the physics timing in [docs/physics-timing.md](docs/physics-timing.md): every contact solved every step, a
second Vulkan queue, and time that dilates rather than frames that drop. [docs/TODO.md](docs/TODO.md) has the
numbers and the rest of the list.

## Running

The stack's siblings must be installed to the local Maven repository first: `supirvast`, `vexelray`, `kronometer`,
`tactroller`, `atchung`, `vexelray-gui`, `vexelray-framework`, `vexelray-sim-core`.

```bash
mvn install
```

Runs the tests that take seconds; add `-Dsupirvast.requireGpu=true` to fail rather than skip where there is no GPU.
Tests that run seconds of simulated time and judge what the bodies did run only with `-Pphysics`. The sweep that
measures the solvers is not an assertion, and runs only with `-Drigid.sweep=true` (on the GPU, or on the CPU with
`-Drigid.backend=CPU`).

```bash
mvn -pl vexelray-sim-rigid-demo exec:exec
```

Opens the demo. Space pauses, R resets, N moves to the next scenario, `=` and `-` change the playback speed, H puts
the camera back; drag in the picture to turn it and use the wheel to zoom. With `-Dautomation=0` it opens an automation
socket, and `ottermate` (in `vexelray-gui/vexelray-gui-automation-cli`) can drive it and photograph the window. The
readings are landmarks named `reading.about`, `reading.time`, `reading.overlap`, `reading.energy`, `reading.speed`,
`reading.cost` and `reading.dropped`.

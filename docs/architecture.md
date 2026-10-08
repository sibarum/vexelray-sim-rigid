# Architecture

## Where this sits

| Repo | What it is |
| --- | --- |
| `vexelray-sim-core` | What every simulation shares: the kernel body, a step as passes over named buffers, the clock, the work budget, the camera, the panel's rows. Infrastructure, never technique. |
| `vexelray-sim-fluid` | The fluid solver and its experiments. |
| `vexelray-sim-rigid` | This: the rigid-body solver and its experiments. |
| `vexelray-sim-physics` | The two coupled, and the front door: what an application that wants physics depends on. |

**Rigid and fluid do not know about each other.** A body takes forces and impulses from outside and reports its pose
and velocity; it does not care what is pushing it. Turning the fluid's pressure into buoyancy and drag on a body, and
a body into a moving boundary for the fluid, is `vexelray-sim-physics`'. A contract both sides must agree on belongs
in `vexelray-sim-core`, and moves there when the second simulation asks for it, not before.

## The same bet as the fluid

The fluid's [architecture](../../vexelray-sim-fluid/docs/architecture.md) lays out the thesis: an expressive API and
a high-performance simulation in the same library, with kernels written in SupirVast IR. It also describes the
practice: experiments first, each producing evidence, with its limits written down beside its results. Both hold here
unchanged.

A step is data (`dev.vexelray.sim.core.step`): passes over named buffers. That is what lets `vexelray-sim-physics`
interleave this step with the fluid's in one recorded dispatch, without either of them knowing.

# vexelray-sim-rigid

Rigid-body simulation for VexelRay: bodies, contact and constraints. It uses the same approach as
[vexelray-sim-fluid](../vexelray-sim-fluid): dynamics written in SupirVast IR, and one source that runs on both the
GPU and the CPU.

This is a scaffold so far. As with the fluid, the work will be a series of experiments: each technique is built far
enough to be measured, and the implementation is chosen from the results.
[docs/architecture.md](docs/architecture.md) says where it sits; [docs/TODO.md](docs/TODO.md) lists what comes first.

| Module | What it holds |
| --- | --- |
| `vexelray-sim-rigid-core` | The kernels, in SupirVast IR, and the diagnostics that judge a state. No engine, no window. |
| `vexelray-sim-rigid-gui` | The simulation on the stack: a runner on resident GPU buffers, and a view of what the state holds. |

It knows nothing of water. A body floating or pushed by a wake is
[vexelray-sim-physics](../vexelray-sim-physics), which couples this to the fluid.

## Running

The stack's siblings must be installed to the local Maven repository first: `supirvast`, `vexelray`,
`tactroller`, `vexelray-gui`, `vexelray-sim-core`.

```bash
mvn install
```

Runs the tests that take seconds. Tests that run seconds of simulated time and judge what the bodies did run only with
`-Pphysics`.

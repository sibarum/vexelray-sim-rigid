# vexelray-sim-rigid

Rigid-body simulation for VexelRay: bodies, contact and constraints. It uses the same approach as
[vexelray-sim-fluid](../vexelray-sim-fluid): dynamics written in SupirVast IR, and one source that runs on both the
GPU and the CPU.

**Try it:** the Rigid Sim demo installs on Windows with the other VexelRay applications, from [sibarum.github.io](https://sibarum.github.io/).

As with the fluid, the work is a series of experiments: each technique is built far enough to be measured, and the
implementation is chosen from the results. [docs/architecture.md](docs/architecture.md) says where it sits;
[docs/TODO.md](docs/TODO.md) has what has been measured so far and what comes next.

| Module | What it holds |
| --- | --- |
| `vexelray-sim-rigid-core` | The kernels, in SupirVast IR, and the diagnostics that judge a state. No engine, no window. So far: spheres in a walled box, with position-based contact (XPBD, Jacobi or Gauss–Seidel) and restitution, substepped, and a grid broad phase. |
| `vexelray-sim-rigid-gui` | The simulation on the stack: `SphereSimulation` steps it on resident buffers on the window's own GPU, every contact solved, and keeps each finished step in a `ShownRing`; `SphereView` ray-traces the spheres straight from the ring's newest slot, blended between that step and the one before. |
| `vexelray-sim-rigid-demo` | A framework application: the scenarios, the solver's settings beside the readings they move. The physics is a `@Component` on a lane of its own, on the compute queue the application lends it, and the frame never waits for it. The clock is Kronometer's: a fixed 60 Hz rate inside a tempo the playback speed scales, whose due steps the frame counts for the physics lane to run. |

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
demo offers both, on the grid, and Gauss–Seidel there runs until every contact is solved. Spheres carry a spin and an
orientation, and Gauss–Seidel's contacts have Coulomb friction: a sliding sphere settles into rolling at 5/7 of its
speed, and one on a slope rolls or slides as theory says. A pile with friction rests: each contact keeps what it
took from one substep to the next and starts from it, and friction waits until every contact is parted.
Contacts bounce as much as the restitution asked for, and resist rolling as much as asked: a sphere rolling on the
floor slows at 5/7 μr g and stops, and a block let go on a smooth floor settles into a heap, where with friction
alone it spreads into one layer. The demo sets friction and rolling resistance alike.

The physics timing in [docs/physics-timing.md](docs/physics-timing.md) is built: every contact solved every step,
physics on a second Vulkan queue and a thread of its own, finished steps handed to the frame through a ring, and a
world clock counted by the steps that finish. When steps get slow the world slows and says by how much, and the
frame rate does not move. Physics runs on this GPU's own queue, the integrated GPU or the CPU, and a profile measures
each against the frame. [docs/TODO.md](docs/TODO.md) has the numbers and the rest of the list.

## Running

The stack's siblings must be installed to the local Maven repository first: `supirvast`, `vexelray`, `kronometer`,
`tactroller`, `atchung`, `vexelray-gui`, `vexelray-framework`, `vexelray-sim-core`.

```bash
mvn install
```

Runs the tests that take seconds; add `-Dsupirvast.requireGpu=true` to fail rather than skip where there is no GPU.
Tests that run seconds of simulated time and judge what the bodies did run only with `-Pphysics`, as do those that
take seconds to repeat what a faster case already shows, and `PhysicsThreadTest`, which opens a window. The sweep that
measures the solvers is not an assertion, and runs only with `-Drigid.sweep=true` (on the GPU, or on the CPU with
`-Drigid.backend=CPU`). The profile of the backends against the frame, `BackendProfileTest`, runs only with
`-Drigid.profile=true`, and takes a couple of minutes.

```bash
mvn -pl vexelray-sim-rigid-demo exec:exec
```

Opens the demo. Space pauses, R resets, N moves to the next scenario, `=` and `-` change the playback speed, H puts
the camera back; drag in the picture to turn it and use the wheel to zoom. With `-Dautomation=0` it opens an automation
socket, and `ottermate` (in `vexelray-gui/vexelray-gui-automation-cli`) can drive it and photograph the window. The
readings are landmarks named `reading.about`, `reading.time`, `reading.overlap`, `reading.energy`, `reading.speed`,
`reading.cost` and `reading.dilation`.

### Native executable

With a GraalVM JDK (25, as `JAVA_HOME`), from a Visual Studio developer prompt (or after `vcvars64.bat`, so
`link.exe` is MSVC's and `rc.exe` is on `PATH`), the demo builds to a native binary two ways from the same code, as
the fluid demo's does. Both are profile-gated, so ordinary builds stay fast:

```bash
mvn -Pnative-release -pl vexelray-sim-rigid-demo -am package -DskipTests
```

```bash
mvn -Pnative -pl vexelray-sim-rigid-demo -am package -DskipTests
```

- **release** gives `vexelray-sim-rigid-demo/target/rigid-sim.exe`: what ships, and what `installer.json` points at.
  A Windows GUI subsystem program with no console window; a socket's port is in its discovery file
  (`~/.vexelray/automation/`), which `ottermate` reads.
- **debug** gives `vexelray-sim-rigid-demo/target/rigid-sim-debug.exe`: a console program, so `--automation=0` prints
  the port `ottermate --launch` reads.

Both carry the framework's automation sockets, remote control and diagnostics, off until the launch asks
(`--automation=0`, `VEXELRAY_AUTOMATION=0`) or the title bar's plug switches one on; `--automation.available=none`
leaves them out of a run. Both link the icon, `vexelray-sim-rigid-demo/src/main/rc/rigid-sim.ico`: `rigid-sim.svg` (the suite icon canvas's
Rigid experiments primary) rendered by `vex-suite-common`'s `tools/Ico.java`; the window, file and light variants are
beside it. The libraries carry their own reachability metadata; what is this demo's own (its main class, LWJGL,
SupirVast's bundled SPIR-V tools, JDK entries) is in `vexelray-sim-rigid-demo/src/main/resources/META-INF/native-image/`,
started from the fluid demo's, whose README says how to record it again with the tracing agent. A metadata gap does not
fail the build, only the binary when it reaches the missing call, so run it afterwards. Only the GPU path has been
tried.

### Installer

`installer.json` describes a per-user install of the release build through
[`vexelray-installer`](https://github.com/sibarum/vexelray-installer), as the fluid demo's does: `rigid-sim.exe` with
its icon, Start menu and desktop shortcuts and an Apps entry. The installer scripts and the binaries are published
from [`vexelray-dist`](https://github.com/sibarum/vexelray-dist) (`apps/rigid-sim/`, releases tagged
`rigid-sim-v<version>`), and its [DEPLOYING.md](https://github.com/sibarum/vexelray-dist/blob/main/DEPLOYING.md) says
how a release goes out. The install page, with the one-liner, is
<https://sibarum.github.io/vexelray-framework/install/>.

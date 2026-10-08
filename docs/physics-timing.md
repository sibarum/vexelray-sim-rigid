# Physics timing: a fixed step, a variable rate, and time that dilates

A plan, not yet built. It spans SupirVast, vexelray-gui, Kronometer and this repo. Each stage is to be built far
enough to be measured, as everything else here is, and the plan is to be revised by what the measurements say.

## What it is for

The solver should never trade correctness for time. Every step advances the same simulated time and solves every
contact; how long that takes on the wall clock is whatever it takes. When the world gets expensive, a pile
collapsing or a thousand bodies meeting at once, the game slows down. It does not stutter, and it does not solve
less.

- **A fixed step, a variable rate.** `dt` never changes. Steps per wall-second do.
- **Time dilation, felt by the whole game world.** Gameplay runs on simulated time: enemies, projectiles, and the
  player's own character, whose input is taken at the steps. A slowdown cannot be turned to advantage, because
  nothing in the world escapes it. Menus, the HUD's own animation and anything else that is not the world stay on
  the wall clock, at full speed.
- **The frame never waits for physics.** The GPU's frame rate must not fall when physics does. The picture blends
  the last two finished states, by how far the step in progress is predicted to have got. The prediction is
  corrected by each step's measured duration, so motion stays smooth at whatever rate physics is managing.
- **Run where it runs best.** The same step on the discrete GPU on a queue of its own, on the integrated GPU, or on
  the CPU through Truffle. Which is best is a measurement, and may differ by machine and by scene.

Nothing here is specific to rigid bodies. The fluid's step is the same shape, and the plan is written so that
`vexelray-sim-physics` can run both under one clock.

## Where things stand

Surveyed 2026-10-08.

- **One queue, shared.** GuiApp makes the app's device with `Request.presentAndCompute`: one queue, from the
  graphics-and-present family (`vexelray-gui-core/.../GuiApp.java:241-271`). `AppCompute.lend` hands compute that
  same device, so physics submits on the graphics queue, from the main thread (`GuiApp.java:578-591`).
- **The frame waits for physics on the host.** The demo drains its `Handoff` in `FrameStage.APP`, submits each
  step, then blocks in `sim.finish()` before drawing (`Session.java:107-114`, `SphereView.java:54-56`). A step that
  takes 30 ms makes a 30 ms frame.
- **One `shown` buffer.** Physics writes the before-and-after centres into it and the view reads it, ordered only
  by that wait.
- **SupirVast has one queue in use, and no signals between queues.**
  - `GpuContext` takes up to four queues from the first compute-capable family, which on most GPUs is the
    graphics family. All resident work goes to queue 0.
  - There are fences, but no binary or timeline semaphores in use. The device can enable timeline semaphores;
    nothing creates one.
  - There are no timestamp queries and no indirect dispatch.
  - `DispatchSequence.run()` does not block, but returns no handle; the only way to know a sequence finished is
    `finish()`, which waits for everything.
- **The device is chosen once per process.** `-Dsupirvast.gpu=discrete|integrated|<name>` picks the physical
  device for every context (`DeviceSelection.java`). Choosing the Intel GPU for physics while the window renders on
  the discrete one needs a per-context choice.
- **The framework already has the threads.** `docs/threading.md` in vexelray-framework allows component threads
  with their own mailbox (T1.1), keeps Vulkan's window and present on the main thread (T3.1), and returns results
  through a queue drained in `FrameStage.APP` (T5.2). A physics worker is a component.
- **Kronometer has half the clock.**
  - `Settlement.STRETCH` lets the debt grow and logical time run slow, which is dilation.
  - `Handoff.phase` blends between the last two steps that ran, but it measures the schedule, not the work.
  - `Rate.degrade` reacts to slip, not to measured step cost.
  - Nothing feeds a step's wall duration back into a clock. sim-core's `BudgetController` measures work per
    wall-second, and is the nearest piece.

## The plan

### 1. SupirVast: queues, signals and timing

- **A compute queue that is not the graphics queue.** Look for a compute-only family (most discrete GPUs have one
  for async compute) and take queues from it, with a lower priority than graphics. Fall back to a second queue of
  the graphics family, then to sharing, so a device without either still works.
- **The device chosen per context.** `GpuContext.open(DeviceSelection)` alongside the system property, so one
  process can render on one GPU and simulate on another.
- **Timeline semaphores.** Created and owned by the context; a submission may signal a value and wait for one.
  This is what lets one queue wait for another inside the GPU instead of on the host.
- **A completion handle for resident work.** `DispatchSequence.run()` returns something to poll or wait on, and
  carries the timeline value it signals. `finish()` stays, for the code that wants it.
- **GPU timestamps.** A query pool written at a sequence's start and end, read back late and without waiting: the
  step's real cost, which the wall clock on the host blurs.
- **Buffers two queues share.** Resident buffers made with concurrent sharing across the families that use them,
  or with an explicit ownership transfer. Which is cheaper is measured.

Tests for each, on both backends where both apply: CPU semantics are trivially sequential, and the tests say so.

### 2. vexelray-gui: the app's device gets a second queue

- GuiApp asks for a compute queue next to the graphics one when the device has one, and records which it got.
- `AppCompute.lend` lends physics the compute queue, not the graphics queue.
- An app that asks for nothing gets exactly today's device. The change is additive, because every VexelRay
  application goes through this code.

### 3. A physics worker on its own thread

- A framework component with its own platform thread (`vexel-component-physics`), owning the physics queue. Under
  T3.1 the main thread still owns the window and present; it no longer owns the physics queue.
- **Every step solves every contact.** After the contact list is made, rounds run in batches; between batches the
  worker reads back the count of unsolved contacts, a few bytes, and runs more until none are left. Blocking is
  fine here: this is not the render thread.
- **Time-sliced.** A step is submitted in slices of a bounded number of dispatches, so that on a GPU shared with
  rendering no single submission occupies it for long. The slice size is measured against frame time, not
  guessed.
- The worker runs steps back to back while the game is running, and stops when it is held. It never decides when a
  step is due; the clock does.

### 4. Handing states to the frame without waiting

- At the end of each step, `shown` is copied into the next slot of a small ring (three slots to start), and the
  step signals its timeline value.
- The frame draws from the two newest finished slots. Its graphics submission waits on the timeline value of the
  newer one inside the GPU, so no host-side fence wait is left anywhere in a frame.
- The ring is what lets physics write step `n + 1` while the frame reads steps `n - 1` and `n`.
- For a backend on another device (the integrated GPU, or Truffle), the finished `shown` comes back to the host and
  is uploaded into the ring on the render device. It is eight floats a sphere, so the copy is small; it is measured.

### 5. Kronometer: a dilated clock

- **Game time advances by `dt` per finished step.** It is not scheduled against the wall at all; it is counted.
  `STRETCH` is the nearest settlement, and the dilated clock may be a new kind of domain rather than a setting of an
  existing one.
- **The blend comes from a prediction.** When a step starts, its duration is predicted from the measured durations
  of those before it (to start with, an exponential average, as `BudgetController` does). The frame's phase is the
  wall time since the step started over the predicted duration, clamped at one: the picture holds on the newest
  state rather than run past it. Each finished step corrects the prediction by its measured duration.
- **Two clocks for one application.** Gameplay, input sampling and anything that is the world run on game time;
  menus, UI animation and Kronometer's existing wall-clock domains do not. The playback tempo the demo has today
  scales game time, as it does now.
- **What dilation looks like is reported.** The ratio of game time to wall time is a reading, as slip is, so a
  slowdown is visible and measurable rather than felt.

### 6. Backends, and measuring them

- **The integrated GPU**: a `GpuContext` of its own, chosen per context (stage 1), with the hand-back through host
  memory (stage 4).
- **Truffle on the CPU**: `PassRunner.cpu` already runs the same passes; it needs only the hand-back.
- **A profiling harness** runs one scene on each backend and records, per step and per frame: the step's GPU time
  from timestamps, its wall time, the frame time while it runs, and the resulting dilation. The questions it
  answers:
  - Does a second queue on the discrete GPU keep the frame rate while physics is heavy, or does the GPU's scheduling
    let physics crowd the frame anyway?
  - What slice size keeps frame time flat on a shared GPU?
  - Is the integrated GPU, with its copy across devices, faster or slower than the discrete GPU's second queue?
  - Where does Truffle stop being competitive, by sphere count?

## Risks, and what would change the plan

- **Async compute is the driver's to schedule.** A second queue does not guarantee that graphics keeps its share.
  If the measurements show physics crowding the frame anyway, slicing becomes the primary defence and queue
  priority a hint.
- **Laptops.** On a laptop with an integrated and a discrete GPU, the window may present on either, and the copy
  between them may be through system memory. The harness should be run on one.
- **A runaway step.** Solving every contact means a pathological scene has no bound on its step time. Dilation
  absorbs that by design, but the reading should make it obvious, and a hard ceiling with a reported overrun may
  still be wanted for debugging.
- **Determinism.** The contact solve is deterministic across substeps (its order comes from the sphere pairs), but
  the sort is not stable. Whether replays need bit-for-bit repeatability is a question for later; nothing here
  makes it harder.

## Related

- [TODO.md](TODO.md): the solver measurements this builds on, in particular the contact list's.
- [architecture.md](architecture.md): where this repo sits.
- vexelray-framework's `docs/threading.md`: the rules the worker follows.
- Kronometer's `Handoff`, `Settlement` and `Rate`: what the dilated clock extends.

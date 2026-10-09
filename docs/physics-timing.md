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

**Built** (SupirVast `2c3aed3`, `4f8501b`, `3459a7b`; its `TODO.md` has the details). `Completion` from every run,
`Accelerator.onDevice(selector)`, devices with queues of several families and buffers shared across them,
`GpuContext.on(instance, device, family)`, timeline semaphores waited for and signalled by a run, and
`Completion.gpuNanos()`. Measured on this laptop: both the RTX 5070 Ti and the Intel GPU have a compute-only family,
and `TwoQueuesTest` orders work across the two queues on each. Ownership transfers were not needed: buffers are
made with concurrent sharing, and whether that costs anything against exclusive is still to be measured.

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

**Built** (vexelray-gui and vexelray-sim-core). An application asks with `GuiApp.Compute.OWN_QUEUE`, through
`new GuiApp(config, windows, compute)` or `HarnessApp.start(gui, config, compute)`. On a device with a compute-only
family, it gets one queue of that family at priority 0.5 beside the queue that draws, and timeline semaphores.
`GuiApp.Gpu.computeFamily()` and `ownQueue()` say what it got, and `AppCompute.lend` makes its context on that family.
`Compute.SHARED`, which every existing constructor uses, makes the device it always made. `ComputeQueueTest` (the
harness) checks both devices and that one with two families still draws. `AppComputeTest` (sim-core-gui) runs a
kernel on the lent queue of a live application. On this laptop the RTX draws on family 0 and computes on family 2.

Two things are left for later stages:

- **The demo does not ask yet.** With physics on a queue of its own, the next step can write `shown` while the
  frame before is still reading it. The host wait in `Session` orders the step against the draw that follows, not
  against the one already on the GPU. Stage 4's ring and timeline wait are what make it safe. The framework has no
  way to ask either: `VexelApplication` makes its `GuiApp` with `SHARED`, and the request goes through it when the
  demo needs it.
- **No fallback to a second queue of the graphics family.** `GpuContext` submits resident work to the first queue
  of its family and spreads other work over all of them, so on the graphics family it would submit to the queue
  that draws. The fallback needs `GpuContext` to take one queue of a family, and neither GPU here lacks a
  compute-only family to test it on. Without one, `OWN_QUEUE` shares, and says so in the log.

- GuiApp asks for a compute queue next to the graphics one when the device has one, and records which it got.
- `AppCompute.lend` lends physics the compute queue, not the graphics queue.
- An app that asks for nothing gets exactly today's device. The change is additive, because every VexelRay
  application goes through this code.

### 3. A physics worker on its own thread

**Built, except the component itself** (SupirVast, vexelray-gui, vexelray-framework, vexelray-sim-core and this
repo).

- **The queue is lent, and T3.1 now says so.**
  - `GuiApp.lendComputeQueue()` hands out a `ComputeQueue` once. A wiring that says `computeQueue()` gets a device
    made with one, and the processor generates that answer exactly when a `@Component` takes a `ComputeQueue`.
    `Shell.computeQueue()` lends it.
  - Two components taking it, or any provider, is a compile error. `AppCompute.on(queue)` makes the context.
  - vexelray-framework's `threading.md` keeps the window, present and the queue that draws on the main thread,
    and gives this queue to the one component's lane.
- **Every contact, every step.** `SphereStepper` runs Gauss–Seidel over the list until nothing is open: see
  [TODO.md](TODO.md), *Every contact solved, every step*. It waits once a pass, about 80 µs on the RTX, so a step of
  the pile of 4096 costs 2.07 ms where 32 fixed rounds cost 1.23. The answer is the same to the bit.
- **Sliced.** `SphereSimulation.slice(n)` submits at most `n` dispatches at a time, and `stepAndWait()` reports a
  step's submissions, wall time and GPU time. `PhysicsThreadTest` steps a pile of 1000 back to back on a platform
  thread, on the queue a live application lent, while the application's loop goes on drawing. A step costs 2.5 ms
  on the wall and 0.8 ms on the GPU, about ten waits. Sliced at 12 dispatches, it is 29 submissions a step and the
  same state, to the bit.

Not yet:

- **The demo's `@Component`.** It waits for stage 4, because the frame reads `shown` and the worker writes it, so
  nothing is safe to run there until the ring and the timeline wait exist. It also moves the demo to generated
  wiring, which places components.
- **The slice size, measured against frame time.** That needs a frame worth measuring, which is the demo in stage 4.
- **"Never decides when a step is due."** The allowance a clock gives the worker is stage 5's. Until then a worker
  runs what it is told to.

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

**Built** (SupirVast, vexelray, and this repo; the demo runs on it).

- **`ShownRing`** hands finished steps from the physics lane to the frame. It has three slots and three roles:
  - **Front:** the slot the frame reads.
  - **Ready:** the newest finished step the frame has not taken.
  - **Back:** a free slot, where the next step is kept.

  The writer never writes the front or the ready slot, so neither side ever waits for the other. A step the frame
  never took is overwritten: the frame shows the newest finished step, not every step. `ShownRingTest` holds all of
  this with no device.
- **One slot is enough, not two.** `shown` already holds each sphere before and after its step, so a slot is a copy
  of it (`Spheres.keep`, `SphereSimulation.keep(slot, value)`). The copy signals the simulation's timeline at the
  step's number.
- **The frame's wait is inside the GPU, and already met.** `SampledColorTarget.renderInto` takes a timeline and a
  value, and the draw's fragment stage waits for it. Physics publishes a step only once the step is known done, so
  the wait never holds the frame. It is what makes one queue's writes visible to the other.
  - `renderInto` still waits on its own fence for its own draw. That is the picture's cost, not physics'.
  - It is also what lets the ring free a slot the moment the frame takes the next one.
- **Generations.** A new simulation brings its own ring buffers. The old simulation is closed only once the frame
  has taken from the new one, so a buffer is never freed under a frame.
- **The demo is the first application with a component**, and its wiring is now generated:
  - **`Physics`**, on lane `physics`, is lent the `ComputeQueue`. It builds, steps back to back up to what the clock
    has made due, keeps each step in the ring, and reports its readings through `PhysicsNews`.
  - **`Session`**, on the main thread, tells it what is due (`Allow`), what to build (`Build`) and how to relax
    (`Relax`), and draws the ring's newest step.
  - When physics is more than two steps behind, it drops the steps it owes rather than run them in a burst: the
    world slows. Stage 5's dilated clock replaces both this and the blend's measure (the last interval between
    steps).

**Measured**, in the demo on the laptop (RTX 5070 Ti, 144 Hz display):

| Scene | A step, wall / GPU | The picture | Frames a second |
| --- | --- | --- | --- |
| Column of 20 | 1.0 / 0.3 ms | about 4 ms (1.2 ms paused) | 140 |
| Pile of 1000 | 3 / 1.7 ms | about 3.5 ms | 140 |

- **The frame rate holds while physics runs,** at the display's rate, to within the second-by-second noise.
  - A frame's longest each second is about 20 ms. Ottermate's own polling was running and is a likely cause; it
    has not been measured apart.
  - Switching scenario costs the world a second: the build is on the physics lane, and the world waits for it.
  - The frame does not wait for the build, apart from one 200 ms frame seen at a switch, which is not explained yet.
- **The picture costs more while physics runs.** On the column it rose from 1.2 ms to about 4. Both queues share
  the GPU, and the draw's fence wait now includes waiting its turn. Slicing (stage 3) is the lever, and its size
  against frame time is still to be measured.
- **A finding on the way.** The framework closes parts in reverse order of construction, so a part built before
  the window, here the tree, is closed after the device. It must not hold anything made on the device. The view's
  pipeline did, and destroying it crashed the driver at exit. The session, built after the device, now closes it.

Not yet:

- **The backend on another device**, with the hand-back through the host: stage 6.
- **A frame that stops waiting for its own draw** would need a release signal before a slot could be reused.
  `ShownRing`'s Javadoc says so.

- At the end of each step, `shown` is copied into the next slot of a small ring (three slots to start), and the
  step signals its timeline value.
- The frame draws from the two newest finished slots. Its graphics submission waits on the timeline value of the
  newer one inside the GPU, so no host-side fence wait is left anywhere in a frame.
- The ring is what lets physics write step `n + 1` while the frame reads steps `n - 1` and `n`.
- For a backend on another device (the integrated GPU, or Truffle), the finished `shown` comes back to the host and
  is uploaded into the ring on the render device. It is eight floats a sphere, so the copy is small; it is measured.

### 5. Kronometer: a dilated clock

**Built** (Kronometer and this repo; the demo runs on it).

- **`Dilated` is a new kind of domain**, not a setting of an existing one: `rate.dilated(mostBehind)`.
  - **Due:** the fixed grid still says when a step is due, in its tempo. The world never runs ahead of the wall.
  - **Counted:** game time is the steps that finish times `dt`.
  - **Forgiven, not repaid:** steps owed past `mostBehind` (two, in the demo) are never run, and game time falls
    behind.
  - **Not a settlement:** stretching the timeline would have slowed the interface's animation with the world.
    Kronometer's own design rules that out, since slip belongs to its one timeline.
  - `DilatedTest`, on a driven clock, holds each of these.
- **The blend is a prediction, and differs from this plan in one way.** The phase is the time since the newest
  finished step over the predicted interval to the next. Each finished step corrects it, taking in a quarter of the
  new interval.
  - The plan measured from when a step *started*, over its predicted duration. When physics is faster than real
    time, a step takes 1 ms and then waits for the grid, so that phase would reach 1 after 1 ms and hold the
    picture still for the other 15.
  - The interval between finishes is the grid's at full speed and the step's own cost when the world is slowed,
    so the picture moves at the pace steps really land at either way.
  - A pause is not taken into the prediction, and nothing that came due before a release is run.
- **Two clocks in the demo.** The physics lane runs steps while the world's clock says one is due; the clock wakes
  it, on the timeline, as each comes due. The frame blends by the clock's phase. The panel, the camera and the
  interface stay on the wall's time. The playback speed scales the world's tempo, as before.
- **Dilation is a reading:** "World speed 80% of the wall's · 803 steps forgiven · a step every 10.3 ms". It is
  taken over 60 due steps, so it is steady rather than jumping frame to frame.

**Measured**, in the demo: the pile of 1000 at 2× speed and 40 × 10 per step, where a step costs about 9.5 ms
against the 8.3 ms the grid allows.

- **The world slows to 77–81%**, which is what the arithmetic says. The predicted interval follows the steps at
  about 10.3 ms.
- **The frame rate does not move:** 144 frames a second, each second, with the longest frame about 9 ms, while
  physics takes 9 ms of GPU time a step.
- One 97 ms frame at the moment of the rebuild, as at a scenario switch in stage 4: still unexplained.
- Pause and resume: game time stops while held, and the prediction comes back at the steps' own interval, not the
  pause's.

Not yet:

- **Input taken at the steps.** The demo has no character to steer. The clock is what a game's input would be
  sampled on: at `take()`, with the step's `dt`.
- **A hard ceiling on a runaway step,** reported as an overrun, which the risks below still ask for.

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

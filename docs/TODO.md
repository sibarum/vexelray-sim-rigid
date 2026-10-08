# TODO

## Spheres, Jacobi XPBD — built, measured, and not good enough to stack

`dev.vexelray.sim.rigid.sphere`: spheres in a walled box, kept apart by position-based contact, substepped. Every
sphere solves its own contacts against the positions as they stood (Jacobi), so a pass is one invocation per sphere
with no atomics. There is no rotation, so no friction, and no restitution. `Spheres`' Javadoc
has the details.

**What holds** (`SpheresTest` on every build, `PileTest` with `-Pphysics`, on the CPU and the GPU):

- A free fall is symplectic Euler's to f32 precision.
- A sphere dropped on the floor rests on it.
- An inelastic collision keeps momentum to 1e-6 and leaves the spheres moving together.
- A dropped pile stays finite and inside the box, nothing passes through anything, and the energy only falls.

**A finding on the way: the velocity must not be read back from positions.** XPBD's `v = (x − x₀)/h` lost 0.26% of a
free fall's speed in half a second from 8 m, because it subtracts two nearly equal positions. `v += (x − x̃)/h`, from
the predicted position, fixed the fall but still lost 1e-4 of a collision's momentum, because by then the move has
been rounded into a position. What works is adding up each constraint's move as it was computed, and adding that.
Any later solver that derives velocity from position should start from that.

**What does not hold: resting.** From `SphereSweepTest` (`-Drigid.sweep=true`) on the GPU, 60 Hz frames, after 3 s:

| Solver | Substeps × iterations | Column of 20: overlap / KE | Pile of 343: overlap | Cost, column / pile |
| --- | --- | --- | --- | --- |
| constant ω = 1 | 10 × 1 | 2.6% / 0.45 J | 2.6% | 0.10 / 0.34 ms |
| constant ω = 0.5 | 10 × 1 | 4.9% / 4.0e-3 J | 4.6% | 0.10 / 0.34 ms |
| averaged ω = 1 | 5 × 1 | 11% / 0.25 J | 42% | 0.06 / 0.19 ms |
| averaged ω = 1 | 10 × 1 | 3.6% / 4.1e-2 J | 10% | 0.10 / 0.34 ms |
| averaged ω = 1 | 20 × 1 | 1.0% / 5.7e-4 J | 2.5% | 0.16 / 0.67 ms |
| averaged ω = 1 | 5 × 2 | 9.0% / 6.0e-2 J | 17% | 0.08 / 0.33 ms |
| averaged ω = 1 | 1 × 10 | 39% / 0.58 J | 85% | 0.07 / 0.33 ms |

Overlap is the deepest any two spheres sink into each other, as a share of the radius. KE is the column's kinetic
energy, which should be zero; the column's mass is 20 kg. Read the table this way:

- **Jacobi hears a stack's weight late.** A pass passes a correction one sphere along, so a column of twenty needs
  twenty passes to feel its own top. Until then each sphere sinks by gravity's `g h²` per substep. That is the
  overlap.
- **Constant ω = 1 overshoots.** A sphere in a column has two contacts, each moving it the whole of its share. The
  column oscillates and never stops: 0.34 m/s at the top after 3 s. Averaging, or halving ω, damps it.
- **Substeps beat iterations, by far.** For the same number of passes, 20 × 1 is two orders of magnitude better than
  1 × 10, which is XPBD's own finding.
- **Only 20 substeps nearly rest**, at 1% overlap. The pile costs 0.67 ms a step for 343 spheres, and that is with
  every pair tested.

## A grid broad phase — built and measured

`SphereGrid`, `Spheres.bin` and `Spheres.solve(SphereGrid)`. Each substep, after predict, every sphere is binned into
a cell as wide as the widest sphere, and SupirVast's `CountingSort` (moved down from the fluid's `Sort`, and made
general: the caller computes the key, and the sort can list items by key rather than move them) lists the spheres by
cell. The solve then tests the 27 cells around a sphere's own. The spheres stay where they are, so a sphere's index
means the same thing to the tests, the picture and the demo.

**What holds**: `GridTest` steps 600 crowded spheres of mixed sizes, some past the walls, both ways from one state;
they agree to 1e-5 m, the order of summation being the only difference. With half the neighbour cells left out,
the test fails by 0.39 m. `PileTest` runs with and without the grid.

**What it costs**, on the GPU, averaged ω = 1, ms a 60 Hz step:

| Scene | Substeps | Every pair | Grid |
| --- | --- | --- | --- |
| Column of 20 | 10 / 20 | 0.10 / 0.16 | 0.16 / 0.29 |
| Pile of 343 | 10 / 20 | 0.36 / 0.67 | 0.27 / 0.46 |
| Pile of 4096 | 10 / 20 | 3.4 / 6.8 | 0.33 / 0.60 |

- **The sort's cost is its dispatches.** Five passes a substep, about 1 µs each in the recorded sequence, whatever
  their size: what makes the column dearer with the grid.
- **Every pair is cheaper on the GPU than n² suggests.** 4096 spheres cost 10× what 343 do, not 140×: the GPU was
  not full at 343. The grid is still 11× faster at 4096, and the gap only widens.
- **The broad phase does not settle anything.** The pile of 4096 still has 10% overlap and moving energy after
  4 s at 20 substeps, either way. That is the solver's, and Gauss–Seidel's to answer.

## Gauss–Seidel by cell — built, measured: it settles, and is too slow for a pile

`Spheres.solveContacts`, chosen by `SphereStep.Solve.GAUSS_SEIDEL_BY_CELL`. The grid's cells are coloured by each index
modulo three, 27 colours. A cell owns the contacts among its own spheres and with the 13 neighbouring cells after
it, and solves them one after another, moving both spheres of each by their shares. Two cells of a colour are three
apart, so nothing one reads or writes is the other's: no atomics. A pass is the 27 colours, then a walls pass.

**What holds**: everything `SpheresTest` and `PileTest` hold for Jacobi, momentum in the collision to 1e-6
included, on both backends.

**A finding on the way: move both spheres of a contact, not one sphere a turn.** The first version coloured cells
by parity, 8 colours, and moved only the sphere whose turn it was, by its share of the overlap it saw. The second of
a pair then sees an overlap the first has already shrunk, and the collision of 1 kg into 3 kg gained 26% of its
momentum. Letting the second take all that was left fixed the pair (1.7e-5, f32 rounding) and broke the piles: a
sphere with several contacts took the whole of each, and the pile of 4096 gained energy, to 385 J. Both are the
same fault, a contact's two moves made from two states; moving both at once removes it.

**Measured**, GPU, 60 Hz, ω = 1, against Jacobi averaged ω = 1 on the grid:

| Scene | Substeps | Jacobi: overlap / KE / ms | Gauss–Seidel: overlap / KE / ms |
| --- | --- | --- | --- |
| Column of 20 | 10 | 3.6% / 4.1e-2 J / 0.17 | 2.1% / 8.0e-9 J / 0.20 |
| Column of 20 | 20 | 1.0% / 5.7e-4 J / 0.29 | 0.52% / 2.2e-8 J / 0.36 |
| Pile of 343 | 10 | 12% / 3.0e-3 J / 0.25 | 1.5% / 5.3e-5 J / 7.8 |
| Pile of 343 | 20 | 1.9% / 1.4e-3 J / 0.46 | 1.0% / 4.3e-5 J / 15 |
| Pile of 4096 | 10 | 42% / 0.18 J / 0.33 | 7.0% / 0.13 J / 6.6 |
| Pile of 4096 | 20 | 12% / 0.14 J / 0.62 | 1.8% / 2.9e-2 J / 13 |

- **A column rests.** At 10 substeps its moving energy is 8e-9 J, five million times less than Jacobi's, for the
  same cost: the column has cells in only 3 of the 27 colours. Iterations work again too: 1 × 10 leaves 1.5e-5 J,
  where Jacobi's left 0.58 J.
- **A pile settles, and costs 20–33× as much.** Overlap falls six-fold at 4096 spheres, but a pass is 27
  dispatches of one invocation per cell, each working through a few dozen contacts in turn: about 25 µs a
  dispatch, with most of the GPU idle. That is the whole cost, and the next thing to take on.
- **One substep is not enough to sort by.** 1 × 10 on the pile of 4096 leaves spheres coincident and 2.5e3 J of
  moving energy, and Jacobi's 1 × 10 left the pile of 343 at 85%: a dropped sphere crosses a cell in a substep, so
  the cells it was sorted into are stale before the pass ends. Substeps are not only accuracy here; they keep the
  broad phase honest.

## Gauss–Seidel over a contact list — built, measured: right, and a fifth of the cost by cell

`SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT`: `Spheres.clearContacts`, `listContacts` and `contactRound`. After the
sort, each substep lists every pair within 2% of touching. Then each pass is rounds, one dispatch each, with one
invocation per contact. Every unsolved contact claims both its spheres by an atomic max of its priority, and a
contact holding both solves in the next dispatch, moving both spheres as the by-cell solve does. Three claim arrays
take turns (claimed into, checked, cleared), so a priority needs no room for its round. A pass is 32 rounds, one more
dispatch to check the last, and the walls.

**What holds**: everything the other solves hold, momentum to 1e-6 included, on both backends. Through one step of
600 crowded spheres of mixed masses, momentum stays at zero to 1e-6, so no two contacts that share a sphere were ever
solved in the same round.

**Three findings on the way:**

- **The order must be the same every substep.** Priorities first came from a contact's place in the list. The
  atomic append makes that different every substep, and a settled pile of 343 stopped at 3.5e-2 J of moving energy,
  where it should keep falling. Each substep solved the same contacts in another order and found a slightly
  different answer, and the velocity felt the difference. Priorities from the pair `a · n + o` settle the same pile
  to 5e-9 J, over 1000× below the by-cell solve. The by-cell solve still lists a cell's spheres in the sort's
  unstable order, and its pile stops near 1e-4 J; its readings vary from run to run where the list's do not.
- **Priorities should still change from round to round.** From the pair alone they never change, and a chain of
  contacts each outranking the next is solved one a round. The pile of 4096 then needed 48 rounds. With the
  dispatch mixed in, still the same in every substep, 32 rounds leave nothing unsolved. At 24, 16 contacts a step
  were left, and 48 gives the same answer as 32, more slowly.
- **List what is nearly touching, not only what touches.** A column at rest touches exactly, so its contacts were
  not listed. When a correction pushed one sphere into the next, the overlap waited a substep and was then taken in
  one move: 0.30 J where the by-cell solve has 8e-9. A margin of 2% of the radii's sum gives 4.8e-9 J. 10% and 30%
  only lengthen the list, until the rounds run out.

**Measured**, GPU, 60 Hz, 10 substeps, ω = 1 (Jacobi averaged); overlap / moving energy / ms a step:

| Scene | Jacobi, grid | Gauss–Seidel by cell | Gauss–Seidel, list |
| --- | --- | --- | --- |
| Column of 20 | 3.6% / 4.1e-2 J / 0.16 | 2.1% / 8.0e-9 J / 0.19 | 2.1% / 4.8e-9 J / 0.63 |
| Pile of 343 | 7.9% / 6.3e-3 J / 0.26 | 2.1% / 1.7e-3 J / 7.7 | 2.4% / 9.1e-6 J / 0.8–2.0 |
| Pile of 4096 | 37% / 0.33 J / 0.33 | 8.7% / 0.13 J / 6.6 | 11% / 7.7e-2 J / 1.2 |
| Pile of 4096, 20 substeps | 10% / 0.21 J / 0.60 | 2.8% / 3.4e-2 J / 12.8 | 3.3% / 0.11 J / 2.3 |

- **It is the Gauss–Seidel to keep.** It is as right as by cell, rests better, and costs a fifth of it on the pile of
  4096. The by-cell solve is left as a measured record, to delete.
- **Jacobi is still cheaper for a pile's overlap.** At 4096, Jacobi at 20 substeps (10%, 0.60 ms) matches the list
  at 10 (11%, 1.2 ms). What the list buys is rest: a stack or a settled pile goes still, and Jacobi's never does.
- **The cost is dispatches.** 32 rounds and 4 more passes a substep, about 3 µs each. Most contacts are solved in
  the first few rounds, and the rest of the dispatches find little to do. Timing varies up to twice from run to run
  here, the list on the pile of 343 most.
- **One substep is still not enough**, for the reason found by cell: 1 × 10 leaves the pile of 4096 coincident.

## Every contact solved, every step — built, measured

`SphereStepper`, and `SphereStep.opening`, `more` and `closing`. Gauss–Seidel over the list no longer runs a fixed
number of rounds. A pass opens with a batch of rounds and a report. Every round marks a word while any contact is
still open after its check, and the report copies that word and the list's length into a readout, which the host
reads where it is (SupirVast's readouts). While anything is open, eight more rounds go, and another report. The
rounds cycle through 49 recorded positions (0, then 1 to 48 over and over), so a pass can run any number of them.
The opening batch is sized from what the passes before needed: one wait a pass, nearly always.

**What holds**: `UntilDoneTest`, on both backends, steps a crowded pile of mixed sizes and masses both ways. Run
until done, it leaves the same state as 47 fixed rounds, to the bit, because its rounds are the same dispatches and
those after the last contact find nothing to do. `SphereStepperTest` checks, with no backend, the segments, the
cycle past 48, the batch's growth and slow shrinking, and that a pass that never ends is an error and not a hang.
The sweep finds the same states as 32 fixed rounds on every scene, and nothing missed, where 24 rounds left 15.8
contacts a step unsolved on the pile of 4096.

**Measured**, GPU, ms a step, 10 substeps unless said:

| Scene | 32 fixed rounds | Until done | Waits a step | Most rounds a pass |
| --- | --- | --- | --- | --- |
| Column of 20 | 0.65 | 1.33 | 10.1 | 16 |
| Pile of 343 | 0.92 | 1.63 | 10.1 | 24 |
| Pile of 4096 | 1.23 | 2.07 | 10.1 | 32 |
| Pile of 4096, 20 substeps | 2.32 | 3.77 | 20.2 | 32 |

- **The cost is the waits**, about 80 µs each on the RTX, one a pass. The empty rounds a generous batch runs are a few
  µs each and do not show. This is the price of the rule, and it is paid on the physics thread, not the frame's.
- **A cheaper shape, if it is wanted:** run a whole step with the batch the last needed, wait once, and run the step
  again with more rounds only when a pass was left open. That needs the state kept from before the step. Not built.
- **Contacts the list has no room for are still unsolved**, and reported (`unlisted`) rather than fixed: a capacity
  of six places a sphere has always been enough here. Growing it means new buffers, which a running simulation does
  not yet do.

## Next

- [ ] **Physics timing: a fixed step, a variable rate, and time that dilates.** Every step solves every contact,
      however long that takes. The game slows rather than solve less, and the frame never waits. That needs a
      second Vulkan queue, a physics worker thread, a ring of finished states, and a clock that counts steps. The
      plan, in six stages across SupirVast, vexelray-gui, Kronometer and this repo, is
      [physics-timing.md](physics-timing.md). It replaces capping Gauss–Seidel's rounds: rounds run until the list
      is done.
- [ ] **The demo offers the solve**, Jacobi or Gauss–Seidel over the list, so a column can be watched coming to
      rest; and the by-cell solve is deleted.
- [ ] **Restitution**, as a velocity pass after the position solve, measured on a bounce's height.
- [ ] **Rotation and friction**: an orientation per body, and friction at contacts, so a pile holds a slope and a
      sphere rolls.
- [ ] **Boxes**: orientation, a contact manifold, and stacking measured the way the column is.
- [ ] **Fixed point against f32**, as the fluid's scatter was measured. The velocity finding above is a reason to
      look: f32 positions already cost something here.
- [ ] **The budgeted clock** (`dev.vexelray.sim.core.time.Clock`) driving the step, as it drives the shallow-water
      patch.
- [ ] **Contact against static SDF geometry**, the same field the fluid meets.
- [ ] **Rest costs nothing**: bodies that have settled sleep.
- [ ] **Let the demo's pattern settle, then hand it to the fluid.** The demo is built: its timing is Kronometer's
      (a fixed rate in a playback tempo, its steps given to the render thread by a `Handoff`, the picture blended by
      `Handoff.phase`). What it needed that was not a rigid body's went to its natural level:
      - Kronometer: `Handoff`.
      - SupirVast: `PassRunner`, `Body.and` and `Body.or`, with `Body` and the pass types moved down from sim-core.
      - sim-core-gui: `AppCompute`, `OrbitControl` and `DemoLook`.

      Still the demo's own, and to be weighed once the fluid's turn shows which parts are shared: `Controls`,
      `Panel`, `Readings` and `Session`. vexelray-sim-fluid's `docs/TODO.md` has the refactor.
- [ ] **A grid in the picture as well as the solver.** The view tests every sphere for every pixel: 1.4 ms for 343
      spheres at 768², and 4.1 ms for the demo's pile of 1000, now that the solve searches a grid and the picture
      is the dearer half. The view can read the same `starts` and `order`.

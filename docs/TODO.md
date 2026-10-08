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

## Gauss–Seidel by contact — built, measured: it settles, and is too slow for a pile

`Spheres.solveContacts`, chosen by `SphereStep.Solve.GAUSS_SEIDEL`. The grid's cells are coloured by each index
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

## Next

- [ ] **Gauss–Seidel at the GPU's width.** The solve is right and the parallelism is wrong: one serial invocation
      per cell, 27 times a pass. A contact list built after the sort, one entry per touching pair, and coloured
      per contact so that no two of a colour share a sphere, would solve a colour with one invocation per contact:
      as many colours as a sphere has contacts (12 at most for equal spheres), each wide. Measured against the
      table above, on cost first.
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

# Boxes: a contact manifold, and stacking measured

Planned on 2026-10-19. Stage 1 is built; the rest is not yet. The plan follows the sphere work: each stage is built far enough to
be measured against a known answer, and records what it found under its own heading. Where a measurement changes the
plan, the stage says how.

## What it is for

Spheres are the one shape where a contact is always one point on the line between two centres. A box needs what a
general rigid body needs:

- **An inertia that depends on direction.** A box's inertia differs about each of its axes, so a free box tumbles,
  and a contact's effective mass depends on how the box is turned.
- **A contact manifold.** Two faces resting on each other touch over an area. A single point would let a box rock
  about it, so a manifold of up to four points has to hold a resting box flat.
- **A narrow phase.** For boxes it is separating axes and clipping, not one distance.

Spheres stay as they are. Boxes are built beside them, sharing the pieces that are not about shape: substeps, the
Gauss–Seidel rounds, the stepper that runs until done, the grid, the ring and the clock. A box against a sphere comes
last, once both are measured on their own.

## Decisions, and why

**A package of its own, `dev.vexelray.sim.rigid.box`, rather than a general body now.** `Spheres` is 1800 lines that
lean on a sphere's simplicity: isotropic inertia with no buffer, a contact found by one distance, one point a pair.
Generalising it first would change every measured sphere result, for a shape that has not been measured yet. A shared
body type is worth weighing once boxes show which parts really are the same.

**Contacts found once a step, kept through its substeps.** Spheres find their contacts every substep, which is cheap
for spheres. A box–box test is a few hundred operations, and clipping makes the points. The approach from Müller et
al., *Detailed Rigid Body Simulation with Extended Position Based Dynamics* (2020), is to find each manifold once a
step, with the bounds widened by how far a body can move in the step. Each point is kept as an anchor in each body's
own frame, plus a normal. Every substep then measures the penetration again from where the anchors have moved. This
also settles the warm start that friction needed for spheres: a manifold's points are the same points for the whole
step, so what each one took carries over from substep to substep with no matching. Carrying impulses across steps is
left out until a stack shows it is needed; that would need clip features to match points between steps.

**One invocation solves one pair's whole manifold.** The rounds of contacts sharing no body stay as they are, with a
pair as the unit, not a point. Its up to four points are solved in turn inside the invocation. A round per point
would put a resting box's four points into four separate rounds, for no gain, since they share both bodies anyway.

**Positional contact and Coulomb friction as the spheres have them.** At each point the normal correction uses the
generalised inverse mass `w = 1/m + (r × n)ᵀ I⁻¹ (r × n)`, with `I⁻¹` turned into the world frame,
`R I_body⁻¹ Rᵀ`. Friction is the same clamp as the spheres' at `μ λₙ`, and restitution the same velocity pass. A box
needs no rolling resistance: its faces stop it.

**The angular momentum is the state.** With nothing touching a box, its angular momentum in the world's frame stays
the same, while its angular velocity follows the body round. Keeping `L` and working out `ω` from it turns a free box
correctly, including the flip about the intermediate axis, and keeps `L` exactly. The plan first had an explicit
gyroscopic term here; stage 1 measured it and the alternatives.

## The plan

### 1. A box on its own: state, inertia, free motion — built, measured (2026-10-19)

`dev.vexelray.sim.rigid.box`: `Boxes`, `BoxStep` and `BoxDiagnostics`. A box carries the spheres' centre, velocity
and orientation, half-extents `hx, hy, hz`, and **its angular momentum `lx, ly, lz` in the world's frame in place of
an angular velocity**. Its principal moments, `m (b² + c²) / 3` from the half-extents, need no buffer. A substep is
only `predict` so far.

**What changed from the plan: the angular momentum is the state, and there is no gyroscopic term.** In free motion
`L` is what stays constant, so `predict` never writes it. It works out `ω = R I⁻¹ Rᵀ L`, turns the box half a
substep, works out `ω` again, and turns the box the whole substep by that. The turn is second order, and `L` is kept
to the bit. Measured against the alternatives on a 0.1 × 0.2 × 0.4 m box tumbling at `(1, 2, 3)` rad/s for 10 s at
10 substeps:

| Predict | Drift in `L` | Drift in energy |
| --- | --- | --- |
| Explicit, `ω += h I⁻¹ (−ω × I ω)`, as planned (host, doubles) | `|L|` +3.8% | grows with it |
| `ω` as the state, `L` worked out from it and back every substep | 5e-7 / 1.2e-6 | +1.4e-4 / −3.5e-4 |
| The same, rotations exact for a quaternion not quite of unit length | — | +7.5e-6 / +1.1e-4 |
| `L` as the state | 0 / 0 | +1.3e-7 / +3.5e-8 |
| `L` as the state, host, doubles | — | −3.7e-7 |

Pairs are CPU / GPU.

**A finding on the way: in f32, a round trip through `R I Rᵀ` and back leans one way.** In the middle rows the energy
drifted in proportion to the number of substeps, at a rate that changed sign between backends. In doubles the same
sums lost 3.7e-7, falling fourfold as the substeps doubled. Two biases added up:

- A quaternion normalised in f32 is not quite of unit length, and turning by it gives `(1 − s²) p + s² R p`.
  Dividing by `|q|²` in the turn fixes that, and is kept.
- The GPU's division is not correctly rounded, so `I` and `I⁻¹` worked out by division are not quite inverses.
  Keeping `L` as the state takes the round trip away.

When contacts come, they add to `L`.

`BoxFreeTest`, both backends:

| Case | Known answer | Holds to |
| --- | --- | --- |
| Free fall from 8 m, 0.5 s | symplectic Euler's sum | 1e-5 m |
| 5 rad/s about the largest or smallest axis, 10 s | spin kept, turned by 50 rad about it | 1e-4 rad/s, 1e-3 rad |
| The same, nudged by 0.1 rad/s about each other axis | stays near the axis | under 0.056 rad off |
| Tumble at `(1, 2, 3)` rad/s, 10 s | `L` and energy kept | `L` to the bit, energy to 1e-6 |
| 5 rad/s about the intermediate axis, nudged by 0.1 about the smallest | turns over every half period of the closed form (Landau and Lifshitz §37), 3.403 s | 0.5%; measured 0.2% |

The flip is a sharp test. The nudge leaves the spin 8e-5 of `M²` from the separatrix, and the period depends on the
logarithm of that distance. With the predict that drifted by 1e-4 in energy, the flips were 1–2% off.

### 2. A box against the walls

The eight corners are each tested against each wall, giving up to four points a wall. They are solved like the
spheres' walls, in the substep's wall pass.

Known answers, with gravity tilted for a slope as `FrictionTest` does:
- A box dropped flat rests flat: no tilt, no drift, its moving energy falling below 1e-6 J.
- A box dropped on a corner or an edge settles onto a face.
- On a slope a box stays put while `μ ≥ tan θ`, and slides at `g (sin θ − μ cos θ)` when `μ` is less.
- A tall box on a high-friction slope tips over when `tan θ > width / height`, before it slides.

### 3. The picture, and the demo

A box is drawn by a slab test in its own frame, with the orientation blended between steps as the spheres' is. The
`shown` layout is the spheres' with half-extents in place of the radius. The demo gets a body-shape setting, and
scenarios for a tumbling box and a box dropped on a corner. Squashing is for spheres only.

### 4. Box against box: the narrow phase

A separating-axis test over the 15 axes: three face normals from each box and the nine edge cross products. The axis
of least overlap is kept, with a bias toward face axes so that a face resting on a face does not flicker to an edge.

- For a face axis, the incident face is clipped against the reference face's side planes (Sutherland–Hodgman), and the
  result is reduced to at most four points, keeping the deepest point and the widest spread.
- For an edge axis, one point at the closest points of the two edges.

The narrow phase is tested on its own, on the CPU and the GPU, against cases worked out on the host: face on face,
offset and turned; edge across edge; a corner into a face; and boxes that touch on one axis and are parted on
another.

### 5. Box against box: the solve, and stacking measured

Pairs go into the Gauss–Seidel list, and the stepper runs until every pair is solved, as it does for spheres.

The measures, each judged the way the column of spheres is: overlap as a share of the half-height, moving energy after
3 s, and ms a step.

| Scene | Known answer |
| --- | --- |
| Two boxes, head on | momentum kept to 1e-6, as the spheres' collision is |
| Column of 10, then 20 | rests upright: the top's sideways drift under 1 mm, and its moving energy under 1e-6 J |
| A turned box on a box | rests turned: no creep in angle |
| **Harmonic overhang** | blocks of length `L`, each stuck out `L / 2k` past the one below. It stands at 0.95 of that and falls at 1.05. This tests torque and the manifold's spread exactly: a manifold that lost points would tip early |
| Pyramid of 10 at the base | stands; overlap and energy as for the column |
| A wall of bricks, half-offset | stands; struck by a box, it is pushed rather than exploding |

### 6. Many boxes: the grid, and the cost

The grid sorts boxes by their bounding spheres, which wastes cells on long thin boxes. That is fine for cubes, and
for anything else it is measured before it is replaced. The sweep gains box scenes: a pile of 343 boxes and one of
1000, each with friction, giving overlap, energy and ms a step on the GPU and the CPU.

### 7. Spheres and boxes together

A sphere against a box is the closest point on the box to the sphere's centre, as one point. This stage decides
whether the two shapes share one body list and one contact list, or step as two that meet. It is chosen from what
stages 1–6 found about how much the solves share.

## Risks, and what would change the plan

- **Points fixed for a step may not be enough at speed.** If a box can rotate far within a step, its anchors drift
  from the real closest points. If a fast-tumbling box sinks or snags, find contacts every substep for pairs that
  turned more than a threshold.
- **Four points may be too few for a turned box on a turned box.** Clipping can give eight. If the reduction to four
  shows as rocking, the harmonic overhang will catch it, and the limit rises.
- **The GPU kernel that lost a store** (TODO, friction) has not been found. A box pass is bigger than any sphere pass,
  so every pass holds its body in locals, loaded once and stored once, from the start.
- **Edge contacts flicker.** A single edge–edge point that comes and goes between steps makes a resting stack jitter.
  The face bias in stage 4 is the first answer, and keeping a pair's axis from the last step is the second.

## Related

- [TODO.md](TODO.md): the sphere solvers and what they found, which this plan builds on.
- [physics-timing.md](physics-timing.md): the stepping, the ring and the clock, which boxes use unchanged.

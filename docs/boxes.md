# Boxes: a contact manifold, and stacking measured

Planned on 2026-10-19. Stages 1 to 3 are built; the rest is not yet. The plan follows the sphere work: each stage is
built far enough to be measured against a known answer, and records what it found under its own heading. Where a
measurement changes the plan, the stage says how.

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

### 2. A box against the walls — built, measured (2026-10-19)

`Boxes.walls` and `Boxes.velocity`, after `predict` in each substep. One invocation per box. Each of its eight
corners is a contact with each of the six walls, found every substep: a corner is already an anchor in the box's
own frame, so the walls need nothing found once a step.

- **Normals.** A corner outside a wall is corrected by its depth over the generalised inverse mass,
  `w = im + (r × n)ᵀ I⁻¹ (r × n)`, so a push off-centre turns the box as well as moving it. The corners are swept
  `passes` times. Each keeps the normal impulse it has taken this substep (`lambda`), and the whole is kept from going
  below zero, as sequential impulses do.
- **Friction, once every normal is done.** A corner's slip since the substep began is removed by a tangential impulse.
  Each corner keeps the whole tangential impulse it has taken this substep (`rub`), held to `μs λₙ` or cut to
  `μk λₙ` past it, and only the change is applied, over the same number of sweeps.
- **Velocity.** Every move and every angular impulse `r × p` is added up as it is computed. `velocity` adds them as
  `v += d / h` and `L += j / h`.

`BoxWallTest`, both backends, 10 substeps, 4 sweeps:

| Case | Known answer | Measured |
| --- | --- | --- |
| A 0.2 × 0.1 × 0.2 m box dropped 1 cm flat, μ = 0.5 | rests flat where it fell | tilt 1.2e-7 rad, drift 6e-8 m, 1e-11 J |
| A 0.1 × 0.2 × 0.4 m box dropped from 0.3 m on a corner | settles on a face | on its widest, tilt 3e-7 rad, 4e-11 J |
| Flat box, 20° slope, μ = 0.5 > tan θ | stays put | settles 6e-6 m down, then 1e-6 m in 2 s on the CPU, none on the GPU |
| Flat box, 30° slope, μ = 0.3 < tan θ | slides at `g (sin θ − μ cos θ)` = 2.3563 m/s² | 2.3563 m/s² |
| A 0.1 × 0.4 × 0.1 m box, μ = 1 | tips at `tan θ = 0.25`, 14.04° | stands at 12° and 13.5°, falls at 16° |

**Sweeps.** At 1 sweep a box resting flat sits 6 µm low and tilts 1e-4 rad, and one on the 20° slope creeps 1.3 mm in
2 s. At 2 sweeps the creep is 0.14 mm; at 4 it is gone, and at 8 the tilt falls to 5e-9 rad. Four is the default.

Two faults found on the way, each like one the spheres had:

1. **One friction sweep, each corner clamping its own share, let a box creep down a slope** at 6e-4 m/s. Each
   corner's push turned the box, and left slip at the corners already rubbed. The fix was the spheres' first fix:
   keep the whole tangential impulse a corner has taken, clamp the whole, and sweep more than once.
2. **The slip was the difference of two positions.** A corner's travel as `(x + r) − (x₀ + r₀)` keeps only what f32
   holds of a position a metre from the origin. The box on the slope crept at 1.5e-5 m/s, a fifth of an ulp a
   substep, on both backends. The centre's travel is now `h v` plus the moves, added up as computed, and only the turn's
   part, `r − r₀`, is a difference, of two short vectors. That is the spheres' velocity finding again, for friction.

A box sliding down the 30° slope leans 8.5e-5 rad while it slides. Every substep it sinks `h² g` into the floor
before the corners are corrected, and the corners are not corrected evenly, because the friction under the box tips
it forward. The lean is that unevenness, and it goes when the box stops.

### 3. The picture, and the demo — built (2026-10-19)

**Core.** `Boxes.show`, once at the end of a step, keeps each box's centre and orientation before the step and after
it, with its half-extents, `Boxes.SHOWN_STRIDE` (20) floats a box. Its layout is its own, not the spheres': a box has
no pressing, and needs a turn and three extents where a sphere has a radius. `BoxShowTest`, both backends, checks that
the "after" slots hold the state the step ended on and the "before" slots the step before's.

**The picture.** `BoxShader` meets each box with the slab test in its own frame:

- The centre is blended between the two steps as a sphere's is, and the orientation by normalised lerp, the short way
  round.
- The ray is turned into the box's frame by a rotation that divides by `|q|²`, so the blended quaternion needs no
  normalising.
- A sphere around each box is tested first, so a pixel runs the slab test only on boxes it may meet nearer than
  anything yet.
- The face the ray entered by is the normal.
- A band along each edge, 6% of the smallest half-extent, is drawn darker, so that a box's turn reads even where two
  faces are lit alike.

The camera's ray, the sky, the floor and the light moved out of `SphereShader` into `Picture`, which both shaders use.

**The stack, generalised over the shape.** The spheres' path through the GUI had sphere-specific names and types
throughout:

- `RigidSimulation` is what a picture and the stepping thread need of either simulation. `SphereSimulation` and the
  new `BoxSimulation` implement it.
- A ring's `Generation` says its `Shape` and body count. `ShownSlots` copies however many words a simulation's
  `shown` has; the keep kernel was always a plain copy.
- `RigidRunner` (was `SphereRunner`) installs either kind.
- `RigidView` (was `SphereView`) makes one pipeline a shape, as it first draws that shape.

**The demo.** The physics lane builds a sphere or a box simulation as the scenario holds, and the readings show what
fits: a box's moving energy, the part of it in turning, and its fastest centre. Two scenarios:

- *Spinning box*: no gravity, 5 rad/s about the middle axis, nudged about the shortest. It turns over every 3.4 s,
  stage 1's flip, there to watch.
- *Dropped boxes*: six boxes of different shapes, turned and dropped onto their corners. Each settles on a face. They
  are spaced so that they never meet, because boxes do not meet each other yet.

Boxes take only the substeps and friction controls; a box scenario rebuilt for new substeps starts over rather than
carrying its state. Squashing is for spheres only. Seen with `ottermate` on this GPU: the spinning box turns over
between 1 s and 3.5 s, and the dropped boxes rest flat, each edge drawn.

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

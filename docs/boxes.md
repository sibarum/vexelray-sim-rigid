# Boxes: a contact manifold, and stacking measured

Planned on 2026-10-19. All seven stages are built. The plan follows the sphere work: each stage is
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
needs no rolling resistance: its faces stop it. *Changed in stage 5: positional contact would not stack, and boxes now
use Box2D v3's soft step, at the level of velocities, walls and pairs alike.*

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

*This stage was built as a positional solve, recorded as it was below. Stage 5 replaced it with the soft step, which
passes the same tests better; the findings here about friction stand.*

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

### 4. Box against box: the narrow phase — built, measured (2026-10-19)

`BoxContact.collide`: one invocation per pair, from a list of pairs, writing the pair's manifold. The manifold is
`BoxContact.MANIFOLD_WORDS` (40) floats: the normal from the first box to the second, the count, the axis that won,
and up to four points. Each point is an anchor on each box in that box's own frame, and a depth. A margin
(`Boxes.params`' `margin`) also finds pairs about to touch, at negative depth, for stage 5.

- **Separating axes.** The 15 axes, each giving how far it parts the pair. The axis that parts them most wins, with
  Box2D's bias. The second box's face beats the first's only if its separation is more than 0.98 of it plus 1% of the
  smallest half-extent, and an edge axis beats the best face only by the same.
- **A face: the vertices of the overlap, not Sutherland–Hodgman.** Clipping a polygon needs arrays of changing
  length, which IR has no locals for. Instead, the clipped polygon's vertices are found directly. They are of three
  kinds:
  - the incident face's corners inside the reference face;
  - the reference face's corners inside the incident face, seen along the normal;
  - the 16 crossings of their edges.

  Those no shallower than minus the margin go into a scratch buffer, `candidates`, at most 24 a pair.
- **Reducing to four.** The deepest point, the farthest from it, and the largest triangle on each side, as Bullet and
  Box2D reduce. Fixed directions would need no scratch, but a symmetric overlap ties them and can drop a corner.
- **An edge.** The closest points of the two edges each box puts furthest toward the other.

Two thresholds were needed. Points within 1e-5 of a face's edge, as a share, count as inside it: two faces the same
size, one on the other, meet exactly at their edges, and without the slack a column of cubes would have found no
points. And a third point must stand off the line through the first two by 1e-4 of its length (see below).

`BoxContactTest`, both backends, each case worked out by hand, every point checked on both boxes in the world's frame:

| Case | Known answer |
| --- | --- |
| Cubes 5 cm apart | no contact |
| A cube 1 cm into a slab | its four bottom corners, 1 cm deep, normal up, the slab's face |
| A cube 1 cm into a cube the same size | the same four corners, though every corner is on the other face's edge |
| A cube turned 45° on the slab | its diamond of corners |
| A cube over the slab's edge | clipped there: two corners and two crossings |
| A turned cube centred over the edge | a triangle: three points |
| The cube and slab as (cube, slab) | the cube's face the reference, normal down, the same points seen on the slab |
| Two cubes, an edge along z under one along x | one point, where they cross, edge axis 12 |
| A cube corner-down into the slab | one point, the corner |
| Shadows overlapping on the first's axes, parted on the second's | no contact |
| A cube 5 mm above the slab | nothing with no margin; with 1 cm, four corners at −5 mm |

And 500 random pairs (468 touching, 1,196 points) checked property by property: every anchor on its own box's
surface to 1e-5 m, every depth the anchors' gap along the normal to 1e-5 m, no point twice, and a point wherever the
spheres inside the two boxes overlap.

**A finding on the way: a corner found twice.** A corner of the overlap can be found as more than one kind of vertex,
as an incident corner and as a crossing at its end, and the copies differ by rounding. With an absolute least area
for the third and fourth points, the GPU kept a copy of one of a triangle's corners as a fourth point, 1e-7 m from
the first. The least area is now relative, 1e-4 of the first two points' distance squared.

What stage 5 will need that is not here: the pairs come from a list the test makes, not a broad phase; and the
normal is the world's at detection, so a pair that turns within a step keeps it. The risks section has that.

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

**Built, measured (2026-10-19), as Box2D v3's soft step, after XPBD would not stack.** What is built:

- `BoxStepper` runs a step:
  - it collides every pair once (`keep`, `collide`, `carry`);
  - it reads the manifolds back and sorts the touching pairs on the host into rounds of pairs sharing no box: a greedy
    colouring, two rounds for a column and six for the pyramid, at one wait a step;
  - it runs the substeps with that many rounds.

  Every pair is in the list; the broad phase is stage 6.
- Each substep, as Box2D v3 does it:
  - gravity into the velocities;
  - a warm start, applying every contact's impulse from before;
  - the solve;
  - positions integrated (the turn from `L`, as stage 1 made it);
  - a relax: the solve again with no bias where surfaces overlap, which takes out the push-apart velocity.

  Each sweep runs every round of pairs (`BoxPairs.solve`, a pair's whole manifold in turn), then the walls
  (`Boxes.walls`, every corner against every wall). Pairs and walls share one contact routine, `Boxes.contact`.
- A contact's normal impulse drives its closing speed toward a bias:
  - apart by `s > 0`: `s / h`, so a gap closes within the substep and no further (speculative);
  - overlapping: the soft constraint's `max(β s, −3 m/s)`, with its mass and impulse scales. The 3 m/s cap is the
    depenetration speed limit, Box2D's default.
- Friction is a velocity impulse against sliding, accumulated as a vector, clamped to `μs λₙ` (or cut to `μk λₙ`).
- Impulses carry from substep to substep and step to step: a wall's corners are fixed, and a pair's points are
  matched to the last step's by anchor. A warm start at the level of velocities carries the load alone.

**Two things changed from Box2D's defaults, both measured.**

- **The stiffness: 150 Hz, held to a quarter of the substep rate, not 30.** A soft contact is a spring, and a column
  on springs is an inverted pendulum: it stands while the springs resist rocking more than the column's weight
  pushes it over. That resistance falls with the square of the bodies' width, and Box2D's 30 Hz is tuned for bodies
  of a metre. For 10 cm cubes it comes to about 90 N·m/rad; ten cubes push over with about 40 and twenty with
  about 180. At 30 Hz, ten sank 5 mm at the foot and fell; at 60 Hz, 1.2 mm; at 150 Hz, 0.2 mm, and they rest.
  The overlap goes as the inverse square of the stiffness, as a spring's does.
- **Four sweeps of each a substep, not one.** At 150 Hz one sweep left ten cubes moving at 1e-4 J; four bring them
  to rest.

`BoxStackTest`, both backends, 10 substeps, friction 0.5:

| Scene | Known answer | Measured |
| --- | --- | --- |
| Two cubes head on at 1 m/s each | momentum kept; inelastic | momentum 0 exactly; touching, 4e-5 m/s |
| Column of 10 cubes | rests: top within 1 mm, under 1e-6 J | 0.2 mm, 2e-10 J, overlap 0.4% of a half |
| Column of 20 cubes | rests | stands, but sways: 9–12 mm, 1e-6 J at 3 s (see below) |
| Harmonic overhang, 6 blocks, 0.95 | stands | drops 1.5 mm (the top tilts onto its edge), slides 0.4 mm, 4e-11 J |
| Harmonic overhang, 1.05 | falls | a block drops 17.5 cm |
| Pyramid of 55, ten at the base | stands | no cube moves 0.3 mm; 3e-11 J; 6 rounds, 540 points |

The walls on the soft step pass stage 2's tests better than the positional solve did. A box dropped flat is still to
1e-17 J and sits 4 µm in. On the 20° slope it settles 1e-7 m and does not creep. The tall box is still at 12° and
falls at 16°, and the slide is `g (sin θ − μ cos θ)` to four digits. The pyramid on the CPU takes 470 ms a step, so
it runs only with `-Pphysics`.

**What does not hold yet: twenty cubes do not quite rest.** They stand, but sway on their soft contacts, the top
going a couple of millimetres back and forth over seconds, at 1e-5 J. Placing them exactly touching, rather than
dropped 1 mm, and symmetric sweeps change little; more sweeps and substeps trim it. It is the column's rocking mode,
damped slowly by contacts stiff enough to stand but soft enough to give.

**What came first: the positional solve, and why it was left.** XPBD with the manifolds kept through the step was
built first, and measured:

- **A single contact was right.** A box on a fixed box rested exactly as on the floor, tilt 3e-7 rad.
- **Stacks were a convergence problem.** Two free boxes tilted 3.1e-4 rad at 10 × 1, falling as `h²` with
  substeps (6.4e-6 at 80 × 1) and with sweeps (1.5e-7 at 10 × 16). Gauss–Seidel stopped short is biased: the point
  corrected first carries more, and with no friction a tilted face is a slope. The error grew with height: twenty
  cubes reached only 1e-5 J at any setting tried, and frictionless columns of ten or more fell at every setting.
- **A positional warm start rested two boxes and unsettled tall stacks.** The impulse carried from substep to
  substep includes last substep's correction of error as well as the load, and a column rings with it, as the
  spheres' frictionless column did. Warm friction carried on acts as a spring.
- **A friction fault, found on the way and kept fixed.** A point's tangential impulse was clamped only while it was
  pressed and slipping, so a point that let go kept its friction, and a warm start applied it again every substep.
  Now the whole is always clamped, and a point that lets go gives its friction back.
- **A blow-up once boxes fell.** A toppling box swings corners in that the step's manifold does not hold; the next
  step found them centimetres deep and corrected them in one substep, at tens of metres a second. On the soft step
  the push is capped at 3 m/s.

**Cost:** about 8 to 11 ms a step on the GPU for ten to fifty-five boxes, as first measured. *Stage 6 found most of
that was the measurement: the tests timed the kernels' compiling with the steps.*

### 6. Many boxes: the grid, and the cost — built, measured (2026-10-19)

**The cost first: the "fixed 8 ms" was mostly the measurement.** The stacking tests timed `scene.start` along with
the steps, and start lowers and compiles every kernel, which takes seconds; spread over 180 steps it added about 5 ms
to each. Timed apart, part by part, on the GPU at 10 substeps, a step of ten or fifty-five boxes was 2.8 to 4.6 ms.
About 1.5 to 2 ms of that was the round trip to the host: writing the rounds, one upload per round buffer, 0.7 to 1.6
ms; reading every manifold back, 0.45 ms; the collide's wait, 0.16 ms. Two changes:

- **One upload a step.** The rounds are one buffer, `rounds`. Each round's pass binds a one-word buffer, `which0` to
  `which31`, written once, that says where its round begins.
- **One small read.** A pass after the collide, `BoxBroad.counts`, writes `contacts`, the only buffer the host reads:
  each slot's partner where it touches, its number of points, and each box's number of partners found.

That brings the host's share to about 0.8 ms (a 0.15 ms wait, a 0.35 ms read, a 0.3 ms write), and a step of ten or
fifty-five boxes at 10 substeps to 2.3 to 2.5 ms; the rest is the GPU's own work, about 300 dispatches.

**The grid.** `BoxBroad`:

- Boxes are binned by their centres and sorted by SupirVast's counting sort, as the spheres are. A cell is the widest
  box's bounding diameter plus the margin, so two boxes that can touch are in the same or neighbouring cells.
- Each box tests the boxes of higher index in its 27 cells, by their bounding boxes in the world's frame, widened by
  the margin. That is tighter than the bounding spheres the plan had, and as cheap.
- The pairs it finds go into the box's own `SLOTS` = 32 places in the pair list, in ascending order of partner, the
  rest −1. A pair is then always slot `first · 32 + m`, and `carry` finds last step's slot by looking through the first
  box's slots for the same partner. That keeps impulses carried across steps without any atomics or stable sorting.
- A box with more than 32 partners keeps the first it finds, and the rest are counted as `unlisted`.

`BoxBroadTest`, both backends: 150 boxes, half-extents 2 to 8 cm, turned at random, packed into a metre cube. The grid
finds the same 218 touching pairs that collide over all 11,175 pairs does, each with the same number of points, and
no box runs out of slots. Every stacking test gives the same numbers on the grid as on the every-pair list, to the
digit: the same contacts, solved in the same order, since slots run in ascending order of partner as the list did.

**The piles.** Cubes of 8 cm, half a kilogram, a loose block jittered and turned by up to 0.6 rad, dropped into a
walled box, friction 0.5; after 4 s, on the GPU:

| Pile | Energy | Deepest point | Touching pairs / points | Rounds | Time a step |
| --- | --- | --- | --- | --- | --- |
| 343 | 1.6e-8 J | 0.47 mm (1.2% of a half) | 695 / 1898 | 11 | 6.8 ms |
| 1000 | 1.0e-6 J | 0.90 mm | 2378 / 5832 | 14 | 7.8 ms |

No pair was left unsolved and no box short of slots. While a pile is falling, the first second, a step costs 24 to 27
ms: impacts drive points to 8–13 mm deep, the push-apart limit at work, and each new number of rounds records a new
list of passes. On the CPU through Truffle a pile of 343 takes about a minute a simulated second, and runs only with
`-Pphysics`; the pile of 1000 runs on the GPU only. The demo has a *Pile of 343 boxes*.

Not done here: the rounds are still made on the host, at one wait a step. On the GPU, as the spheres' contact list
makes them, they would save that wait, and that is the next cost to take on. The grid's cells are sized by the widest
box, which would waste cells on a few long thin boxes among small ones; measured only with cubes.

### 7. Spheres and boxes together — built, measured (2026-10-19)

**Decided: one body list, on the soft step.** By the end of stage 6 the two shapes did not share a solver. The
spheres' is positional: Gauss–Seidel moves the bodies, and the velocities follow from the moves. The boxes' is at
the level of velocities: impulses change the velocities, and the positions follow. A contact between a sphere and a
box has to be solved by one of them, at one point in the substep. Two lists stepping apart and meeting would hand
impulses across that mismatch every substep, with nowhere clean to do it. So a sphere is now a shape in the boxes'
solver: a body whose `shape` is `SPHERE_SHAPE`, its radius in all three half-extents. The spheres' own solver stays
as it is: the measured experiment the sphere scenarios run on, and the one with the grid and the rounds made on the
GPU.

What a sphere needed:

- **Inertia:** `⅖ m r²` about every axis, from its shape, where a box's comes from its extents.
- **Walls:** one contact a wall, its point nearest the wall, where a box has eight corners.
- **Narrow phase:** `BoxContact` keeps the boxes' test and, where a sphere is in the pair, writes one point over it:
  - two spheres: along the line between the centres;
  - a sphere and a box: the box's point nearest the sphere's centre, the normal from it to the centre; or, where the
    centre is inside the box, the face it is least far in from.
- **Its point in the solve:** the radius along the normal, never an anchor turned with the sphere. A rolling ball's
  anchor would leave the surface within a step. For the same reason `carry` hands a one-point contact's impulses to
  the one point of the next step, whatever its anchors did.
- **Broad phase:** a sphere's bounds are its radius on every axis.
- **Rolling resistance**, the spheres' answer again (`Boxes.roll`). It is an angular impulse against the two bodies
  turning against each other, rolling and twisting, warm started, held to `μr R λₙ`. It applies only where a sphere is
  in the contact; a box's faces stop it.
- **The picture:** the shape in a spare word of `shown`; the box shader meets a sphere by its quadratic and paints it
  in octants of its own frame.

The rolling resistance was not in the stage's plan; a measurement put it there. A mixed pile of 64 balls and 64 cubes,
friction 0.5, came to rest only in part: the cubes at 9e-6 m/s, the balls still rolling at 3 cm/s after 4 s. Friction
holds a contact point still, and a rolling ball's is still. The spheres found this first.

`BoxContactTest`, both backends, each hand-worked:

| Case | Known answer |
| --- | --- |
| A ball 1 cm into the slab | one point under its centre, normal up |
| The same pair the other way round | normal down |
| A ball over the slab's edge, 3 cm out and up | the point on the edge, normal at 45°, depth `r − 0.03 √2` |
| A ball with its centre inside the slab | pushed out through the nearest face, 6 cm deep |
| Two balls, 5 and 4 cm, 8 cm apart | 1 cm deep, along the line between them |
| A ball 5 cm above the slab | no contact |

`BoxSphereTest`, both backends but the pile:

| Case | Known answer | Measured |
| --- | --- | --- |
| A ball on a free cube | rests on its top, centred | 0.02 mm in, 1e-6 m off, 1e-11 J |
| A ball sliding at 2 m/s on a fixed slab, μ = 0.3 | rolls at `5/7 v₀`, `ω = v / r` | 1.4286 m/s, −28.571 rad/s, both exact to the digits printed |
| A ball rolling at 1 m/s, μr = 0.1, on the floor and on a slab | slows at `5/7 μr g`: 0.6496 m/s after 0.5 s | 0.6496 m/s, still rolling, both |
| A ball of 1 kg at 2 m/s into a free box of 1 kg | momentum kept; inelastic | 2.000000; both at 1.0000 m/s |
| A box on four fixed balls | rests level | tilt 3e-8 rad, 2e-17 J |
| 64 balls and 64 cubes, μ 0.5, μr 0.3 | rests | 4e-12 J, deepest point 0.21 mm, 10 rounds |

The broad phase and every box test give the same numbers as before, since a body that is a box takes the same path.
The demo has *Balls and boxes*, with the rolling control now passed to box scenes.

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

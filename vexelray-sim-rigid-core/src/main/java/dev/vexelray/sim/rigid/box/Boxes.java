package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.Buffer;
import dev.supirvast.vastir.core.Expr;
import dev.supirvast.vastir.core.Function;
import dev.supirvast.vastir.core.LocalVar;
import dev.supirvast.vastir.pass.CountingSort;
import dev.supirvast.vastir.type.Type;

import java.util.ArrayList;
import java.util.List;

import static dev.supirvast.vastir.build.Body.F32;
import static dev.supirvast.vastir.build.Body.add;
import static dev.supirvast.vastir.build.Body.and;
import static dev.supirvast.vastir.build.Body.div;
import static dev.supirvast.vastir.build.Body.f;
import static dev.supirvast.vastir.build.Body.gt;
import static dev.supirvast.vastir.build.Body.i;
import static dev.supirvast.vastir.build.Body.load;
import static dev.supirvast.vastir.build.Body.lt;
import static dev.supirvast.vastir.build.Body.max;
import static dev.supirvast.vastir.build.Body.mod;
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.neg;
import static dev.supirvast.vastir.build.Body.not;
import static dev.supirvast.vastir.build.Body.or;
import static dev.supirvast.vastir.build.Body.sqrt;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.toFloat;
import static dev.supirvast.vastir.build.Body.v;

/**
 * Boxes: solid cuboids, against the walls and each other. A box's state and its motion with nothing touching it
 * (stage 1 of {@code docs/boxes.md}), its corners against the walls ({@link #walls}), and the contact solve the walls
 * and the pairs of boxes share ({@link #contact}): Box2D v3's soft step, at the level of velocities.
 *
 * <p>A box carries its centre {@code x, y, z}, velocity {@code u, v, w} and orientation {@code qx, qy, qz, qw}, as a
 * sphere does; in place of a sphere's angular velocity, its angular momentum about its centre in the world's frame,
 * {@code lx, ly, lz}; and in place of a radius, its half-extents {@code hx, hy, hz} along its own axes. A solid box of
 * mass {@code m} has the principal moments {@code m (b² + c²) / 3}, {@code b} and {@code c} the other two
 * half-extents, so its inertia, like a sphere's, needs no buffer: it is worked out from {@code im} and the half-extents
 * where it is needed. A box with inverse mass zero is fixed.
 *
 * <h2>Turning: the angular momentum is the state, not the angular velocity</h2>
 *
 * A sphere's inertia is the same about every axis, so with nothing touching it its angular velocity never changes. A
 * box's is not, and what stays the same in free motion is the angular momentum in the world's frame,
 * {@code L = R I Rᵀ ω}. The angular velocity follows the body round: {@code I ω̇ + ω × I ω = 0} in the body's frame,
 * the gyroscopic term.
 *
 * <p>{@link #positions} keeps {@code L} rather than integrating that term. It works out the angular velocity from
 * {@code L}, {@code ω = R I⁻¹ Rᵀ L}, turns the box half a substep by it, and works out the angular velocity there; then
 * it turns the box the whole substep by that midpoint angular velocity. The turn is second order, and {@code L} is
 * kept exactly, since nothing else writes it while the box is free. A contact's angular impulse {@code r × P} is
 * added to {@code L}.
 *
 * <p>On a box tumbling at {@code (1, 2, 3)} rad/s for 10 s at 10 substeps ({@code BoxFreeTest}), this keeps the
 * energy to 1.3e-7 on the CPU and 3.5e-8 on the GPU, where the same sums in doubles leave 3.7e-7: the integration's
 * own error. The explicit form, {@code ω += h I⁻¹ (−ω × I ω)}, grew {@code |L|} by 3.8%; the angular velocity as the
 * state, worked out to {@code L} and back every substep, drifted 1e-4 in energy, because in f32 every round trip
 * rounded the same way. {@code docs/boxes.md} has the measurements.
 *
 * <h2>Contact: the soft step</h2>
 *
 * Each substep, as Box2D v3 does it ({@code BoxStep} orders the passes):
 *
 * <ol>
 *   <li>{@link #velocities}: gravity into the velocity.</li>
 *   <li><b>Warm start</b>: every contact's impulse from before applied again, to the velocities.</li>
 *   <li><b>Solve</b>: each contact's normal impulse, soft, then its friction.</li>
 *   <li>{@link #positions}: the box moved and turned by its velocities.</li>
 *   <li><b>Relax</b>: the solve again with no bias, which takes out the velocity the soft push apart left.</li>
 * </ol>
 *
 * <p>A contact's normal impulse drives its closing speed toward a bias: where the two surfaces are apart by
 * {@code s > 0}, {@code s / h}, so that they may close the gap within the substep and no further, speculatively; where
 * they overlap, a spring and damper's, {@code max(β s, −push)}, applied with the soft constraint's mass and impulse
 * scales ({@link #softness}), so that an overlap is pushed apart over a few substeps, and never faster than
 * {@link #PUSH} metres a second. That limit is what keeps a box found deep in another from leaving it at tens of
 * metres a second. The impulses are accumulated, the normal kept at least zero, and carried from substep to substep
 * and step to step: a warm start, which at the level of velocities carries the load alone, where the positional one
 * carried the last substep's correction of error too, and set a column ringing.
 *
 * <p>Friction is an impulse against the contact's sliding velocity, accumulated as a vector in the contact plane, the
 * whole held to {@code μs λₙ} or cut to {@code μk λₙ} past it.
 *
 * <p>Lengths are in metres, times in seconds, {@code y} up; the walled box the bodies live in runs from the origin to
 * {@code (sx, sy, sz)}, as the spheres' does.
 */
public final class Boxes {

    /** The workgroup every pass must be registered with, as for spheres. */
    public static final int WORKGROUP = CountingSort.BLOCK;

    /**
     * {@code [h, gx, gy, gz, sx, sy, sz, muStatic, muKinetic, margin, bias, massScale, impulseScale, staticBias,
     * staticMassScale, staticImpulseScale, push, muRolling]}, see {@link #params}.
     */
    public static final int PARAM_COUNT = 18;

    static final int H = 0;
    static final int GX = 1;
    static final int GY = 2;
    static final int GZ = 3;
    static final int SX = 4;
    static final int SY = 5;
    static final int SZ = 6;
    static final int MU_STATIC = 7;
    static final int MU_KINETIC = 8;
    static final int MARGIN = 9;
    /** A contact between two moving boxes: its softness ({@link #softness}). */
    static final int BIAS = 10;
    static final int MASS_SCALE = 11;
    static final int IMPULSE_SCALE = 12;
    /** A contact with a wall or a fixed box: twice as stiff, as Box2D makes it. */
    static final int STATIC_BIAS = 13;
    static final int STATIC_MASS_SCALE = 14;
    static final int STATIC_IMPULSE_SCALE = 15;
    static final int PUSH_SPEED = 16;
    static final int MU_ROLLING = 17;

    /**
     * A contact's stiffness, in hertz, held to a quarter of the substep rate as Box2D holds it. Box2D's default is 30,
     * for bodies of a metre or so; a soft contact is a spring, and a column on springs buckles once its weight's
     * moment outgrows the springs' resistance to rocking, which falls with the square of the bodies' width. At 30 Hz a
     * column of ten 10 cm cubes sank 5 mm at its foot and fell; at 150 Hz it rests, 0.2 mm in.
     */
    public static final double CONTACT_HERTZ = 150;
    /** And its damping ratio: Box2D's default, heavily overdamped, so a contact does not bounce. */
    public static final double DAMPING_RATIO = 10;
    /** The fastest a contact pushes two overlapping surfaces apart, in metres a second: Box2D's default. */
    public static final double PUSH = 3;

    /** The sweeps a substep makes over its contacts, solving and relaxing each, unless told otherwise. */
    public static final int WALL_PASSES = 4;
    /** The corners of a box, by the bits of their index: bit 0 the sign along x, bit 1 along y, bit 2 along z. */
    public static final int CORNERS = 8;
    /** The walls, low then high along x, then y, then z. */
    public static final int WALLS = 6;
    /** Words a box keeps in {@code lambda}: each corner's normal impulse against each wall. */
    public static final int LAMBDA_WORDS = WALLS * CORNERS;
    /** Words a box keeps in {@code rub}: each corner's tangential impulse against each wall. */
    public static final int RUB_WORDS = 3 * LAMBDA_WORDS;
    /** Words a body keeps in {@code roll}: a sphere's rolling impulse against each wall, where its tangential would be. */
    public static final int ROLL_WORDS = RUB_WORDS;

    /** Every per-box field, in the order {@link BoxStep} holds them. */
    static final String[] BOX = {"x", "y", "z", "u", "v", "w", "lx", "ly", "lz", "qx", "qy", "qz", "qw", "im", "hx",
            "hy", "hz", "shape"};

    /** A body's {@code shape}: a box, its half-extents {@code hx, hy, hz}; or a sphere, its radius in all three. */
    public static final float BOX_SHAPE = 0;
    public static final float SPHERE_SHAPE = 1;
    /** A solid sphere's moment of inertia, as a share of {@code m r²}. */
    public static final double SPHERE_INERTIA = 0.4;

    /** What a sweep over the contacts does. */
    public enum Sweep {
        /** Each contact's impulses from before applied again, before anything is solved. */
        WARM,
        /** The normals, soft, toward the bias that closes a gap or pushes an overlap apart; then the friction. */
        SOLVE,
        /** The same, with no bias where they overlap: the velocity the push apart left is taken out. */
        RELAX
    }

    private Boxes() {
    }

    /**
     * The parameters as {@link #PARAM_COUNT} words.
     *
     * @param h  the substep, in seconds
     * @param gx gravity along x, in metres per second squared; likewise {@code gy} (negative is down) and {@code gz}
     * @param sx the walled box's extent along x, in metres; likewise {@code sy} and {@code sz}
     */
    public static int[] params(double h, double gx, double gy, double gz, double sx, double sy, double sz) {
        return params(h, gx, gy, gz, sx, sy, sz, 0, 0);
    }

    /**
     * As above, with friction.
     *
     * @param muStatic  a contact holds while the tangential impulse it needs is at most this times its normal one
     * @param muKinetic a contact that slips is resisted by this times its normal impulse
     */
    public static int[] params(double h, double gx, double gy, double gz, double sx, double sy, double sz,
                               double muStatic, double muKinetic) {
        return params(h, gx, gy, gz, sx, sy, sz, muStatic, muKinetic, 0);
    }

    /**
     * As above, with the contact margin.
     *
     * @param margin how far apart two surfaces may be and still have their contact found and solved, speculatively,
     *               in metres; 0, the default, for only those that touch or overlap
     */
    public static int[] params(double h, double gx, double gy, double gz, double sx, double sy, double sz,
                               double muStatic, double muKinetic, double margin) {
        return params(h, gx, gy, gz, sx, sy, sz, muStatic, muKinetic, margin, 0);
    }

    /**
     * As above, with rolling resistance, which a contact with a sphere in it applies ({@link #roll}).
     *
     * @param muRolling how much a sphere's contact resists the two bodies turning against each other, rolling or
     *                  twisting: an angular impulse of at most this times the normal impulse and the radius; 0, the
     *                  default, for none
     */
    public static int[] params(double h, double gx, double gy, double gz, double sx, double sy, double sz,
                               double muStatic, double muKinetic, double margin, double muRolling) {
        if (!(muRolling >= 0) || Double.isInfinite(muRolling)) {
            throw new IllegalArgumentException("rolling resistance is finite and at least 0, got " + muRolling);
        }
        if (!(margin >= 0) || Double.isInfinite(margin)) {
            throw new IllegalArgumentException("a margin is finite and at least 0, got " + margin);
        }
        if (!(h > 0) || !(sx > 0) || !(sy > 0) || !(sz > 0)) {
            throw new IllegalArgumentException("h > 0 and a box of positive size, got h " + h + ", box " + sx + " × "
                    + sy + " × " + sz);
        }
        if (!(muStatic >= 0) || !(muKinetic >= 0) || Double.isInfinite(muStatic) || Double.isInfinite(muKinetic)) {
            throw new IllegalArgumentException("friction finite and at least 0, got " + muStatic + " and "
                    + muKinetic);
        }
        double hertz = Math.min(CONTACT_HERTZ, 0.25 / h);
        double[] soft = softness(hertz, DAMPING_RATIO, h);
        double[] stiff = softness(2 * hertz, DAMPING_RATIO, h);
        return new int[] {bits(h), bits(gx), bits(gy), bits(gz), bits(sx), bits(sy), bits(sz), bits(muStatic),
                bits(muKinetic), bits(margin), bits(soft[0]), bits(soft[1]), bits(soft[2]), bits(stiff[0]),
                bits(stiff[1]), bits(stiff[2]), bits(PUSH), bits(muRolling)};
    }

    /**
     * A soft constraint's {@code [bias rate, mass scale, impulse scale]}, Box2D's {@code b2MakeSoft}: a spring of
     * {@code hertz} and damping ratio {@code zeta}, stepped implicitly by {@code h}. With {@code ω = 2π hertz},
     * {@code a₁ = 2ζ + hω}, {@code a₂ = hω a₁} and {@code a₃ = 1 / (1 + a₂)}: the bias rate {@code ω / a₁}, the mass
     * scale {@code a₂ a₃}, the impulse scale {@code a₃}.
     */
    public static double[] softness(double hertz, double zeta, double h) {
        if (hertz == 0) {
            return new double[] {0, 1, 0};
        }
        double omega = 2 * Math.PI * hertz;
        double a1 = 2 * zeta + h * omega;
        double a2 = h * omega * a1;
        double a3 = 1 / (1 + a2);
        return new double[] {omega / a1, a2 * a3, a3};
    }

    /**
     * A solid box's principal moments of inertia, about its own x, y and z, for mass {@code m} and half-extents
     * {@code hx, hy, hz}: {@code m (hy² + hz²) / 3} and the like.
     */
    public static double[] inertia(double m, double hx, double hy, double hz) {
        return new double[] {m * (hy * hy + hz * hz) / 3, m * (hx * hx + hz * hz) / 3, m * (hx * hx + hy * hy) / 3};
    }

    /** A body's principal moments, by its shape: a box's as above, a sphere's {@code ⅖ m r²}, its radius {@code hx}. */
    public static double[] inertia(float shape, double m, double hx, double hy, double hz) {
        if (shape > 0.5f) {
            double moment = SPHERE_INERTIA * m * hx * hx;
            return new double[] {moment, moment, moment};
        }
        return inertia(m, hx, hy, hz);
    }

    // --- moving the boxes ----------------------------------------------------------------------------------

    static final String[] VELOCITIES_NAMES = {"u", "v", "w", "im", "params"};
    static final List<Buffer> VELOCITIES_BUFFERS = bind(VELOCITIES_NAMES);

    /** One invocation per box: gravity into its velocity, as a substep begins. */
    public static Function velocities() {
        List<Buffer> bs = VELOCITIES_BUFFERS;
        Buffer params = bs.get(4);
        int[] gravity = {GX, GY, GZ};
        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> t.when(gt(load(bs.get(3), v(s)), f(0)), free -> {
            LocalVar h = free.let("h", load(params, i(H)));
            for (int a = 0; a < 3; a++) {
                free.store(bs.get(a), v(s), add(load(bs.get(a), v(s)), mul(v(h), load(params, i(gravity[a])))));
            }
        }));
        return function("boxesVelocities", b);
    }

    static final String[] POSITIONS_NAMES = {"x", "y", "z", "u", "v", "w", "im", "params", "lx", "ly", "lz", "qx",
            "qy", "qz", "qw", "hx", "hy", "hz", "shape"};
    static final List<Buffer> POSITIONS_BUFFERS = bind(POSITIONS_NAMES);

    /**
     * One invocation per box: moved along its velocity, by symplectic Euler as a sphere is, and turned, its angular
     * momentum kept ({@link Boxes above}). The box is loaded once and stored once.
     */
    public static Function positions() {
        List<Buffer> bs = POSITIONS_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] velocity = {bs.get(3), bs.get(4), bs.get(5)};
        Buffer im = bs.get(6);
        Buffer params = bs.get(7);
        Buffer[] spin = {bs.get(8), bs.get(9), bs.get(10)};  // the angular momentum, lx, ly, lz
        Buffer[] turn = {bs.get(11), bs.get(12), bs.get(13), bs.get(14)};
        Buffer[] half = {bs.get(15), bs.get(16), bs.get(17)};

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar inverseMass = t.let("im", load(im, v(s)));
            t.when(gt(v(inverseMass), f(0)), free -> {
                LocalVar h = free.let("h", load(params, i(H)));
                for (int a = 0; a < 3; a++) {
                    free.store(at[a], v(s), add(load(at[a], v(s)), mul(v(h), load(velocity[a], v(s)))));
                }
                LocalVar[] q = new LocalVar[4];
                for (int k = 0; k < 4; k++) {
                    q[k] = free.let("q", load(turn[k], v(s)));
                }
                LocalVar[] momentum = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    momentum[a] = free.let("momentum", load(spin[a], v(s)));
                }
                LocalVar[] inverse = inverseInertia(free, half, bs.get(18), s, inverseMass);

                // The angular velocity now, half a substep's turn by it, the angular velocity there, and the whole
                // turn by that.
                LocalVar[] omega = rotate(free, q, scale(free, inverse, rotate(free, conjugate(free, q), momentum)));
                LocalVar halfH = free.let("halfH", mul(f(0.5f), v(h)));
                LocalVar[] mid = turned(free, q, omega, halfH);
                LocalVar[] omegaMid = rotate(free, mid, scale(free, inverse, rotate(free, conjugate(free, mid),
                        momentum)));
                LocalVar[] end = turned(free, q, omegaMid, h);
                for (int k = 0; k < 4; k++) {
                    free.store(turn[k], v(s), v(end[k]));
                }
            });
        });
        return function("boxesPositions", b);
    }

    // --- the walls -----------------------------------------------------------------------------------------

    static final String[] WALLS_NAMES = {"x", "y", "z", "qx", "qy", "qz", "qw", "im", "hx", "hy", "hz", "params", "u",
            "v", "w", "lx", "ly", "lz", "lambda", "rub", "shape", "roll"};
    static final List<Buffer> WALLS_BUFFERS = bind(WALLS_NAMES);

    /**
     * One invocation per box: one {@code sweep} over its eight corners against the six walls, each corner a contact
     * with each wall ({@link #contact}), the wall fixed and its normal pointing in. A corner is a contact while it is
     * within the margin of the wall, or holds an impulse from before. Each corner keeps its normal impulse against
     * each wall in {@code lambda} and its tangential impulse in {@code rub}, from substep to substep and step to
     * step. With {@code reverse}, the walls and the corners are taken last to first.
     */
    public static Function walls(Sweep sweep, boolean reverse) {
        List<Buffer> bs = WALLS_BUFFERS;
        Buffer[] atBuffer = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] turnBuffer = {bs.get(3), bs.get(4), bs.get(5), bs.get(6)};
        Buffer im = bs.get(7);
        Buffer[] half = {bs.get(8), bs.get(9), bs.get(10)};
        Buffer params = bs.get(11);
        Buffer[] velocity = {bs.get(12), bs.get(13), bs.get(14)};
        Buffer[] spin = {bs.get(15), bs.get(16), bs.get(17)};
        Buffer lambda = bs.get(18);
        Buffer rub = bs.get(19);
        Buffer shape = bs.get(20);
        Buffer rolls = bs.get(21);
        int[] extentParam = {SX, SY, SZ};

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar inverseMass = t.let("im", load(im, v(s)));
            t.when(gt(v(inverseMass), f(0)), free -> {
                Moving box = new Moving(free, s, inverseMass, atBuffer, turnBuffer, half, velocity, spin, shape);
                Soft soft = new Soft(free, params, true);
                LocalVar base = free.let("base", mul(v(s), i(LAMBDA_WORDS)));
                LocalVar[] extent = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    extent[a] = free.let("extent", load(params, i(extentParam[a])));
                }
                for (int order = 0; order < WALLS; order++) {
                    int wall = reverse ? WALLS - 1 - order : order;
                    int axis = wall / 2;
                    boolean high = wall % 2 == 1;
                    int firstSlot = wall * CORNERS;
                    LocalVar[] n = box.normal(free, axis, high);
                    // A sphere: one contact, its point nearest the wall, its offset the radius against the normal.
                    free.when(box.isSphere(), round -> {
                        LocalVar[] r = new LocalVar[3];
                        for (int a = 0; a < 3; a++) {
                            r[a] = round.let("r", neg(mul(v(box.half[0]), v(n[a]))));
                        }
                        LocalVar at = round.let("at", add(v(base), i(firstSlot)));
                        LocalVar tangentAt = round.let("tangentAt", mul(v(at), i(3)));
                        LocalVar reach = round.let("reach", add(v(box.at[axis]), v(r[axis])));
                        LocalVar gap = round.let("gap", high ? sub(v(extent[axis]), v(reach)) : v(reach));
                        contact(round, sweep, gap, n, null, null, box, r, lambda, at, rub, tangentAt, soft);
                        roll(round, sweep, null, box, box.half[0], lambda, at, rolls, tangentAt, params);
                    });
                    free.when(not(box.isSphere()), boxed -> box.eachCorner(boxed, reverse, (corner, c, offset, r) -> {
                        LocalVar at = corner.let("at", add(v(base), add(i(firstSlot), v(c))));
                        LocalVar tangentAt = corner.let("tangentAt", mul(v(at), i(3)));
                        LocalVar reach = corner.let("reach", add(v(box.at[axis]), v(r[axis])));
                        LocalVar gap = corner.let("gap", high ? sub(v(extent[axis]), v(reach)) : v(reach));
                        contact(corner, sweep, gap, n, null, null, box, r, lambda, at, rub, tangentAt, soft);
                    }));
                }
                box.store(free, velocity, spin);
            });
        });
        String name = sweep == Sweep.WARM ? "boxesWallsWarm" : sweep == Sweep.SOLVE ? "boxesWalls" : "boxesWallsRelax";
        return function(reverse ? name + "Back" : name, b);
    }

    // --- one contact ---------------------------------------------------------------------------------------

    /** A contact's softness and limits, from the parameters: between moving boxes, or against something fixed. */
    static final class Soft {
        final LocalVar h;
        final LocalVar margin;
        final LocalVar bias;
        final LocalVar massScale;
        final LocalVar impulseScale;
        final LocalVar push;
        final LocalVar muStatic;
        final LocalVar muKinetic;

        Soft(Body b, Buffer params, boolean fixed) {
            h = b.let("h", load(params, i(H)));
            margin = b.let("margin", load(params, i(MARGIN)));
            bias = b.let("bias", load(params, i(fixed ? STATIC_BIAS : BIAS)));
            massScale = b.let("massScale", load(params, i(fixed ? STATIC_MASS_SCALE : MASS_SCALE)));
            impulseScale = b.let("impulseScale", load(params, i(fixed ? STATIC_IMPULSE_SCALE : IMPULSE_SCALE)));
            push = b.let("push", load(params, i(PUSH_SPEED)));
            muStatic = b.let("muS", load(params, i(MU_STATIC)));
            muKinetic = b.let("muK", load(params, i(MU_KINETIC)));
        }

        /** The softness of a fixed contact in place of a moving one's, where {@code fixed}. */
        void stiffen(Body b, Expr fixed, Buffer params) {
            b.when(fixed, stiff -> {
                stiff.set(bias, load(params, i(STATIC_BIAS)));
                stiff.set(massScale, load(params, i(STATIC_MASS_SCALE)));
                stiff.set(impulseScale, load(params, i(STATIC_IMPULSE_SCALE)));
            });
        }
    }

    /**
     * One contact point, one {@code sweep}: between {@code first} at {@code rA} from its centre, or a wall where it is
     * null, and {@code second} at {@code rB}, the normal {@code n} from the first to the second, the surfaces apart by
     * {@code gap} along it, negative where they overlap. Its normal impulse is at {@code normals[normalAt]}, and its
     * tangential impulse, the second's, at {@code tangents[tangentAt]} on.
     *
     * <p>Warm: both applied again. Solve: the normal impulse toward the bias ({@link Boxes above}), the whole kept at
     * least zero, then the friction, the whole held to the cone; each applied as the change. Relax: the same with no
     * bias where the surfaces overlap. A contact further apart than the margin with no impulse does nothing.
     */
    static void contact(Body b, Sweep sweep, LocalVar gap, LocalVar[] n, Moving first, LocalVar[] rA, Moving second,
                        LocalVar[] rB, Buffer normals, LocalVar normalAt, Buffer tangents, LocalVar tangentAt,
                        Soft soft) {
        LocalVar normal = b.let("normal", load(normals, v(normalAt)));
        LocalVar[] tangent = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            tangent[a] = b.let("tangent", load(tangents, add(v(tangentAt), i(a))));
        }
        Expr holding = or(gt(v(normal), f(0)), gt(dot(tangent, tangent), f(0)));
        if (sweep == Sweep.WARM) {
            b.when(holding, warm -> {
                LocalVar[] p = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    p[a] = warm.let("p", add(mul(v(normal), v(n[a])), v(tangent[a])));
                }
                apply(warm, first, rA, second, rB, p);
            });
            return;
        }
        b.when(or(lt(v(gap), v(soft.margin)), holding), active -> {
            // The normal: toward the bias, soft where the surfaces overlap.
            LocalVar k = active.let("k", inverseMassAlong(active, first, rA, second, rB, n));
            LocalVar closing = active.let("closing", dot(relative(active, first, rA, second, rB), n));
            LocalVar bias = active.let("biasSpeed", f(0));
            LocalVar massScale = active.let("mScale", f(1));
            LocalVar impulseScale = active.let("iScale", f(0));
            active.when(gt(v(gap), f(0)), apart -> apart.set(bias, div(v(gap), v(soft.h))));
            if (sweep == Sweep.SOLVE) {
                active.when(not(gt(v(gap), f(0))), overlap -> {
                    overlap.set(bias, max(mul(v(soft.bias), v(gap)), neg(v(soft.push))));
                    overlap.set(massScale, v(soft.massScale));
                    overlap.set(impulseScale, v(soft.impulseScale));
                });
            }
            LocalVar change = active.let("change", sub(neg(mul(div(v(massScale), v(k)), add(v(closing), v(bias)))),
                    mul(v(impulseScale), v(normal))));
            LocalVar total = active.let("total", max(f(0), add(v(normal), v(change))));
            active.store(normals, v(normalAt), v(total));
            LocalVar[] p = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                p[a] = active.let("p", mul(sub(v(total), v(normal)), v(n[a])));
            }
            apply(active, first, rA, second, rB, p);

            // The friction: against the sliding velocity, the whole held to the cone of the normal impulse now.
            LocalVar[] sliding = relative(active, first, rA, second, rB);
            LocalVar along = active.let("along", dot(sliding, n));
            for (int a = 0; a < 3; a++) {
                active.set(sliding[a], sub(v(sliding[a]), mul(v(along), v(n[a]))));
            }
            LocalVar[] whole = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                whole[a] = active.let("whole", v(tangent[a]));
            }
            LocalVar speed = active.let("speed", sqrt(dot(sliding, sliding)));
            active.when(gt(v(speed), f(1e-12f)), slipping -> {
                LocalVar[] t = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    t[a] = slipping.let("t", div(v(sliding[a]), v(speed)));
                }
                LocalVar kt = slipping.let("kt", inverseMassAlong(slipping, first, rA, second, rB, t));
                LocalVar stop = slipping.let("stop", div(v(speed), v(kt)));
                for (int a = 0; a < 3; a++) {
                    slipping.set(whole[a], sub(v(whole[a]), mul(v(stop), v(t[a]))));
                }
            });
            LocalVar size = active.let("size", sqrt(dot(whole, whole)));
            active.when(gt(v(size), mul(v(soft.muStatic), v(total))), kinetic -> {
                LocalVar cut = kinetic.let("cut", div(mul(v(soft.muKinetic), v(total)), v(size)));
                for (int a = 0; a < 3; a++) {
                    kinetic.set(whole[a], mul(v(whole[a]), v(cut)));
                }
            });
            LocalVar[] rubbed = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                active.store(tangents, add(v(tangentAt), i(a)), v(whole[a]));
                rubbed[a] = active.let("rubbed", sub(v(whole[a]), v(tangent[a])));
            }
            apply(active, first, rA, second, rB, rubbed);
        });
    }

    /**
     * Rolling resistance at one contact with a sphere in it, after {@link #contact} has solved the contact this
     * sweep: an angular impulse against the two bodies turning against each other, {@code ωB − ωA} (a wall turns not
     * at all), rolling and twisting alike, as the spheres' own {@code Spheres.roll} is. Accumulated in
     * {@code rolls[rollAt]} on, warm started, and the whole held to {@code μr R λₙ}, {@code R} the sphere's radius,
     * or {@code rA rB / (rA + rB)} for two, and {@code λₙ} the contact's normal impulse now. A torque of at most
     * {@code μr R N} with no spring: a ball stops and stays, rather than rocking back. Without it a sphere rolls on
     * forever, as the spheres found: friction holds the contact point still, and a rolling ball's is still.
     */
    static void roll(Body b, Sweep sweep, Moving first, Moving second, LocalVar radius, Buffer normals,
                     LocalVar normalAt, Buffer rolls, LocalVar rollAt, Buffer params) {
        LocalVar[] old = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            old[a] = b.let("rollOld", load(rolls, add(v(rollAt), i(a))));
        }
        if (sweep == Sweep.WARM) {
            applyAngular(b, first, second, old);
            return;
        }
        LocalVar muRolling = b.let("muR", load(params, i(MU_ROLLING)));
        LocalVar[] whole = new LocalVar[3];
        LocalVar[] turning = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            whole[a] = b.let("rollWhole", v(old[a]));
            Expr against = first == null ? f(0) : v(first.omega[a]);
            turning[a] = b.let("turning", sub(v(second.omega[a]), against));
        }
        LocalVar speed = b.let("turnSpeed", sqrt(dot(turning, turning)));
        b.when(and(gt(v(speed), f(1e-12f)), gt(v(muRolling), f(0))), turns -> {
            LocalVar[] t = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                t[a] = turns.let("t", div(v(turning[a]), v(speed)));
            }
            Expr k = dot(t, second.inverseInertiaTimes(turns, t));
            if (first != null) {
                k = add(k, dot(t, first.inverseInertiaTimes(turns, t)));
            }
            LocalVar stop = turns.let("stopTurn", div(v(speed), k));
            for (int a = 0; a < 3; a++) {
                turns.set(whole[a], sub(v(whole[a]), mul(v(stop), v(t[a]))));
            }
        });
        LocalVar most = b.let("mostRoll", mul(mul(v(muRolling), v(radius)), max(f(0), load(normals, v(normalAt)))));
        LocalVar size = b.let("rollSize", sqrt(dot(whole, whole)));
        b.when(gt(v(size), v(most)), cut -> {
            LocalVar scale = cut.let("rollCut", div(v(most), v(size)));
            for (int a = 0; a < 3; a++) {
                cut.set(whole[a], mul(v(whole[a]), v(scale)));
            }
        });
        LocalVar[] change = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            b.store(rolls, add(v(rollAt), i(a)), v(whole[a]));
            change[a] = b.let("rollChange", sub(v(whole[a]), v(old[a])));
        }
        applyAngular(b, first, second, change);
    }

    /** The angular impulse {@code j} on the second, and against the first. */
    private static void applyAngular(Body b, Moving first, Moving second, LocalVar[] j) {
        second.turn(b, j);
        if (first != null) {
            LocalVar[] back = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                back[a] = b.let("back", neg(v(j[a])));
            }
            first.turn(b, back);
        }
    }

    /** The second's point's velocity less the first's, or less nothing at a wall. */
    private static LocalVar[] relative(Body b, Moving first, LocalVar[] rA, Moving second, LocalVar[] rB) {
        LocalVar[] vb = second.pointVelocity(b, rB);
        if (first == null) {
            return vb;
        }
        LocalVar[] va = first.pointVelocity(b, rA);
        LocalVar[] out = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            out[a] = b.let("relative", sub(v(vb[a]), v(va[a])));
        }
        return out;
    }

    private static Expr inverseMassAlong(Body b, Moving first, LocalVar[] rA, Moving second, LocalVar[] rB,
                                         LocalVar[] n) {
        Expr k = second.inverseMassAlong(b, rB, n);
        return first == null ? k : add(k, first.inverseMassAlong(b, rA, n));
    }

    /** The impulse {@code p} on the second at {@code rB}, and against the first at {@code rA}. */
    private static void apply(Body b, Moving first, LocalVar[] rA, Moving second, LocalVar[] rB, LocalVar[] p) {
        second.apply(b, rB, p);
        if (first != null) {
            LocalVar[] back = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                back[a] = b.let("back", neg(v(p[a])));
            }
            first.apply(b, rA, back);
        }
    }

    // --- one box, as a solve holds it ----------------------------------------------------------------------

    /** What a corner's work is given: the body, the corner's index, its offset in the box's frame, and in the world's. */
    @FunctionalInterface
    interface CornerWork {
        void at(Body b, LocalVar corner, LocalVar[] offset, LocalVar[] r);
    }

    /**
     * A box as a solve holds it: its centre and turn, which a solve reads and does not change, and its velocity,
     * angular momentum and angular velocity, which it does; loaded once and stored once (the spheres' GPU once lost a
     * store that was loaded again). The inertia is turned into the world's frame by the turn the box has, which a
     * velocity solve does not change.
     */
    static final class Moving {
        final LocalVar s;
        /** The body's shape, {@link Boxes#BOX_SHAPE} or {@link Boxes#SPHERE_SHAPE}, its radius then {@code half[0]}. */
        final LocalVar shape;
        final LocalVar inverseMass;
        final LocalVar[] at = new LocalVar[3];
        final LocalVar[] q = new LocalVar[4];
        final LocalVar[] half = new LocalVar[3];
        final LocalVar[] inverse;
        final LocalVar[] velocity = new LocalVar[3];
        final LocalVar[] momentum = new LocalVar[3];
        final LocalVar[] omega;

        Moving(Body b, LocalVar s, LocalVar inverseMass, Buffer[] atBuffer, Buffer[] turnBuffer, Buffer[] halfBuffer,
               Buffer[] velocityBuffer, Buffer[] spinBuffer, Buffer shapeBuffer) {
            this.s = s;
            this.inverseMass = inverseMass;
            this.shape = b.let("shape", load(shapeBuffer, v(s)));
            for (int a = 0; a < 3; a++) {
                at[a] = b.let("at", load(atBuffer[a], v(s)));
                half[a] = b.let("half", load(halfBuffer[a], v(s)));
                velocity[a] = b.let("vel", load(velocityBuffer[a], v(s)));
                momentum[a] = b.let("mom", load(spinBuffer[a], v(s)));
            }
            for (int k = 0; k < 4; k++) {
                q[k] = b.let("q", load(turnBuffer[k], v(s)));
            }
            inverse = inverseInertia(b, halfBuffer, shapeBuffer, s, inverseMass);
            omega = inverseInertiaTimes(b, momentum);
        }

        /** Whether the body is a sphere. */
        Expr isSphere() {
            return gt(v(shape), f(0.5f));
        }

        /** The unit normal of a wall, pointing inside: {@code +e_axis} for the low wall, {@code −e_axis} for the high. */
        LocalVar[] normal(Body b, int axis, boolean high) {
            LocalVar[] n = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                n[a] = b.let("n", f(a == axis ? (high ? -1 : 1) : 0));
            }
            return n;
        }

        /**
         * {@code work} for each of the eight corners, in a loop, with the corner where the box is turned now: in order,
         * or with {@code reverse} from the last.
         */
        void eachCorner(Body b, boolean reverse, CornerWork work) {
            LocalVar k = b.let("k", i(0));
            b.loop(lt(v(k), i(CORNERS)), corner -> {
                LocalVar c = corner.let("corner", reverse ? sub(i(CORNERS - 1), v(k)) : v(k));
                LocalVar[] offset = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    Expr bit = mod(div(v(c), i(1 << a)), i(2));
                    offset[a] = corner.let("offset", mul(v(half[a]), sub(mul(f(2), toFloat(bit)), f(1))));
                }
                work.at(corner, c, offset, rotate(corner, q, offset));
                corner.set(k, add(v(k), i(1)));
            });
        }

        /** {@code I⁻¹ x} with the inertia turned into the world's frame: {@code R I_body⁻¹ Rᵀ x}. */
        LocalVar[] inverseInertiaTimes(Body b, LocalVar[] x) {
            return rotate(b, q, scale(b, inverse, rotate(b, conjugate(b, q), x)));
        }

        /** The generalised inverse mass at {@code r} along the unit {@code n}: {@code im + (r × n)ᵀ I⁻¹ (r × n)}. */
        Expr inverseMassAlong(Body b, LocalVar[] r, LocalVar[] n) {
            LocalVar[] rn = cross(b, r, n);
            return add(v(inverseMass), dot(rn, inverseInertiaTimes(b, rn)));
        }

        /** The velocity of the point at {@code r} from the centre: {@code v + ω × r}. */
        LocalVar[] pointVelocity(Body b, LocalVar[] r) {
            LocalVar[] turning = cross(b, omega, r);
            LocalVar[] out = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                out[a] = b.let("pv", add(v(velocity[a]), v(turning[a])));
            }
            return out;
        }

        /** The impulse {@code p} at {@code r}: {@code v += im p}, {@code L += r × p}, {@code ω += I⁻¹ (r × p)}. */
        void apply(Body b, LocalVar[] r, LocalVar[] p) {
            for (int a = 0; a < 3; a++) {
                b.set(velocity[a], add(v(velocity[a]), mul(v(inverseMass), v(p[a]))));
            }
            LocalVar[] angular = cross(b, r, p);
            LocalVar[] turn = inverseInertiaTimes(b, angular);
            for (int a = 0; a < 3; a++) {
                b.set(momentum[a], add(v(momentum[a]), v(angular[a])));
                b.set(omega[a], add(v(omega[a]), v(turn[a])));
            }
        }

        /** The angular impulse {@code j}: {@code L += j}, {@code ω += I⁻¹ j}. */
        void turn(Body b, LocalVar[] j) {
            LocalVar[] spin = inverseInertiaTimes(b, j);
            for (int a = 0; a < 3; a++) {
                b.set(momentum[a], add(v(momentum[a]), v(j[a])));
                b.set(omega[a], add(v(omega[a]), v(spin[a])));
            }
        }

        /** The velocity and angular momentum stored back, once. */
        void store(Body b, Buffer[] velocityBuffer, Buffer[] spinBuffer) {
            for (int a = 0; a < 3; a++) {
                b.store(velocityBuffer[a], v(s), v(velocity[a]));
                b.store(spinBuffer[a], v(s), v(momentum[a]));
            }
        }
    }

    // --- for a picture -------------------------------------------------------------------------------------

    /**
     * Floats per box in {@code shown}: the centre before the step and after it, each with a spare word; the
     * half-extents and a spare; then the orientation before the step and after it.
     */
    public static final int SHOWN_STRIDE = 20;
    /** Where in a box's {@code shown} its centre before the step starts, and after it. */
    public static final int CENTRE_BEFORE = 0;
    public static final int CENTRE_AFTER = 4;
    /** Where its shape is, {@link #BOX_SHAPE} or {@link #SPHERE_SHAPE}: the centre before's spare word. */
    public static final int SHAPE = 3;
    /** Where its half-extents are. */
    public static final int HALF = 8;
    /** Where its orientation before the step starts, and after it. */
    public static final int TURNED_BEFORE = 12;
    public static final int TURNED_AFTER = 16;

    static final String[] SHOW_NAMES = {"x", "y", "z", "qx", "qy", "qz", "qw", "hx", "hy", "hz", "shown", "shape"};
    static final List<Buffer> SHOW_BUFFERS = bind(SHOW_NAMES);

    /**
     * One invocation per box, at the end of a step: the centre and orientation the last step ended on moved to the
     * "before" slots of {@code shown}, and this step's put in the "after" slots, with the half-extents. A picture
     * blends the two by how far the frame is between the steps, as it does a sphere's.
     */
    public static Function show() {
        List<Buffer> bs = SHOW_BUFFERS;
        Buffer shown = bs.get(10);
        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar base = t.let("base", mul(v(s), i(SHOWN_STRIDE)));
            for (int a = 0; a < 3; a++) {
                t.store(shown, add(v(base), i(CENTRE_BEFORE + a)), load(shown, add(v(base), i(CENTRE_AFTER + a))));
                t.store(shown, add(v(base), i(CENTRE_AFTER + a)), load(bs.get(a), v(s)));
                t.store(shown, add(v(base), i(HALF + a)), load(bs.get(7 + a), v(s)));
            }
            for (int k = 0; k < 4; k++) {
                t.store(shown, add(v(base), i(TURNED_BEFORE + k)), load(shown, add(v(base), i(TURNED_AFTER + k))));
                t.store(shown, add(v(base), i(TURNED_AFTER + k)), load(bs.get(3 + k), v(s)));
            }
            t.store(shown, add(v(base), i(SHAPE)), load(bs.get(11), v(s)));
        });
        return function("boxesShow", b);
    }

    /**
     * {@code shown} as a picture would first be given it, from the starting state: before and after both the start,
     * so the first frame does not blend from nowhere. {@code q} is {@code qx, qy, qz, qw}, {@code half}
     * {@code hx, hy, hz}.
     */
    public static float[] shownAtStart(float[] x, float[] y, float[] z, float[][] q, float[][] half, float[] shape) {
        float[] shown = new float[SHOWN_STRIDE * x.length];
        float[][] at = {x, y, z};
        for (int s = 0; s < x.length; s++) {
            int base = SHOWN_STRIDE * s;
            for (int a = 0; a < 3; a++) {
                shown[base + CENTRE_BEFORE + a] = shown[base + CENTRE_AFTER + a] = at[a][s];
                shown[base + HALF + a] = half[a][s];
            }
            shown[base + SHAPE] = shape[s];
            for (int k = 0; k < 4; k++) {
                shown[base + TURNED_BEFORE + k] = shown[base + TURNED_AFTER + k] = q[k][s];
            }
        }
        return shown;
    }

    // --- building blocks -----------------------------------------------------------------------------------

    private static Expr dot(LocalVar[] a, LocalVar[] b) {
        return add(add(mul(v(a[0]), v(b[0])), mul(v(a[1]), v(b[1]))), mul(v(a[2]), v(b[2])));
    }

    /**
     * Body {@code s}'s inverse principal moments: a box's {@code 3 im / (hy² + hz²)} and the like; a sphere's, its
     * radius in {@code hx}, {@code im / (⅖ r²)} about every axis.
     */
    static LocalVar[] inverseInertia(Body b, Buffer[] half, Buffer shape, LocalVar s, LocalVar inverseMass) {
        LocalVar[] h2 = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            LocalVar e = b.let("e", load(half[a], v(s)));
            h2[a] = b.let("h2", mul(v(e), v(e)));
        }
        LocalVar[] inverse = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            Expr across = add(v(h2[(a + 1) % 3]), v(h2[(a + 2) % 3]));
            inverse[a] = b.let("inverse", div(mul(f(3), v(inverseMass)), across));
        }
        b.when(gt(load(shape, v(s)), f(0.5f)), round -> {
            for (int a = 0; a < 3; a++) {
                round.set(inverse[a], div(v(inverseMass), mul(f((float) SPHERE_INERTIA), v(h2[0]))));
            }
        });
        return inverse;
    }

    /**
     * {@code q} turned by the angular velocity {@code omega} for {@code dt}: {@code q + ½ (dt ω, 0) ⊗ q}, normalised,
     * first order in the turn as the spheres' is.
     */
    private static LocalVar[] turned(Body b, LocalVar[] q, LocalVar[] omega, LocalVar dt) {
        LocalVar[] a = new LocalVar[3];
        for (int k = 0; k < 3; k++) {
            a[k] = b.let("half", mul(mul(f(0.5f), v(dt)), v(omega[k])));
        }
        // (a, 0) ⊗ (v, w) = (w a + a × v, −a · v)
        LocalVar[] now = new LocalVar[4];
        for (int k = 0; k < 3; k++) {
            int k1 = (k + 1) % 3;
            int k2 = (k + 2) % 3;
            now[k] = b.let("turned", add(v(q[k]), add(mul(v(a[k]), v(q[3])),
                    sub(mul(v(a[k1]), v(q[k2])), mul(v(a[k2]), v(q[k1]))))));
        }
        now[3] = b.let("turned", sub(v(q[3]), add(mul(v(a[0]), v(q[0])),
                add(mul(v(a[1]), v(q[1])), mul(v(a[2]), v(q[2]))))));
        LocalVar len2 = b.let("len2", add(add(mul(v(now[0]), v(now[0])), mul(v(now[1]), v(now[1]))),
                add(mul(v(now[2]), v(now[2])), mul(v(now[3]), v(now[3])))));
        LocalVar inverse = b.let("inverse", div(f(1), sqrt(max(v(len2), f(1e-30f)))));
        for (int k = 0; k < 4; k++) {
            b.set(now[k], mul(v(now[k]), v(inverse)));
        }
        return now;
    }

    /**
     * {@code p} turned by the quaternion {@code q}: {@code p + w t + q × t}, with {@code t = 2 q × p / |q|²}. Dividing
     * by {@code |q|²} makes it a rotation whatever {@code q}'s length. {@code q} is normalised, but only to f32's
     * rounding, and a turn by a {@code q} of length {@code s} is {@code (1 − s²) p + s² R p}: where the rounding leans
     * one way, every turn adds the same error ({@link Boxes above}).
     */
    static LocalVar[] rotate(Body b, LocalVar[] q, LocalVar[] p) {
        LocalVar[] axis = {q[0], q[1], q[2]};
        LocalVar twice = b.let("twice", div(f(2), add(add(mul(v(q[0]), v(q[0])), mul(v(q[1]), v(q[1]))),
                add(mul(v(q[2]), v(q[2])), mul(v(q[3]), v(q[3]))))));
        LocalVar[] t = cross(b, axis, p);
        for (int a = 0; a < 3; a++) {
            b.set(t[a], mul(v(twice), v(t[a])));
        }
        LocalVar[] qt = cross(b, axis, t);
        LocalVar[] out = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            out[a] = b.let("rotated", add(v(p[a]), add(mul(v(q[3]), v(t[a])), v(qt[a]))));
        }
        return out;
    }

    private static LocalVar[] conjugate(Body b, LocalVar[] q) {
        return new LocalVar[] {b.let("cq", neg(v(q[0]))), b.let("cq", neg(v(q[1]))), b.let("cq", neg(v(q[2]))),
                q[3]};
    }

    private static LocalVar[] scale(Body b, LocalVar[] by, LocalVar[] p) {
        LocalVar[] out = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            out[a] = b.let("scaled", mul(v(by[a]), v(p[a])));
        }
        return out;
    }

    private static LocalVar[] cross(Body b, LocalVar[] x, LocalVar[] y) {
        return new LocalVar[] {
                b.let("cx", sub(mul(v(x[1]), v(y[2])), mul(v(x[2]), v(y[1])))),
                b.let("cy", sub(mul(v(x[2]), v(y[0])), mul(v(x[0]), v(y[2])))),
                b.let("cz", sub(mul(v(x[0]), v(y[1])), mul(v(x[1]), v(y[0]))))};
    }

    private static List<Buffer> bind(String... names) {
        List<Buffer> buffers = new ArrayList<>();
        for (int k = 0; k < names.length; k++) {
            buffers.add(new Buffer(names[k], k, F32));
        }
        return List.copyOf(buffers);
    }

    private static int bits(double value) {
        return Float.floatToRawIntBits((float) value);
    }

    private static Function function(String name, Body b) {
        return new Function(name, new Type.FunctionType(Type.VOID, List.of()), b.finish());
    }
}

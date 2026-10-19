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
import static dev.supirvast.vastir.build.Body.div;
import static dev.supirvast.vastir.build.Body.f;
import static dev.supirvast.vastir.build.Body.gt;
import static dev.supirvast.vastir.build.Body.i;
import static dev.supirvast.vastir.build.Body.load;
import static dev.supirvast.vastir.build.Body.lt;
import static dev.supirvast.vastir.build.Body.max;
import static dev.supirvast.vastir.build.Body.min;
import static dev.supirvast.vastir.build.Body.mod;
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.neg;
import static dev.supirvast.vastir.build.Body.or;
import static dev.supirvast.vastir.build.Body.sqrt;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.toFloat;
import static dev.supirvast.vastir.build.Body.toInt;
import static dev.supirvast.vastir.build.Body.v;

/**
 * Boxes: solid cuboids, against the walls so far. Stages 1 and 2 of {@code docs/boxes.md}: a box's state, its inertia,
 * its motion with nothing touching it, and its corners against the walls ({@link #walls}), with friction.
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
 * <p>{@link #predict} keeps {@code L} rather than integrating that term. It works out the angular velocity from
 * {@code L}, {@code ω = R I⁻¹ Rᵀ L}, turns the box half a substep by it, and works out the angular velocity there; then
 * it turns the box the whole substep by that midpoint angular velocity. The turn is second order, and {@code L} is
 * kept exactly, since nothing writes it.
 *
 * <p>On a box tumbling at {@code (1, 2, 3)} rad/s for 10 s at 10 substeps ({@code BoxFreeTest}), this keeps the
 * energy to 1.3e-7 on the CPU and 3.5e-8 on the GPU, where the same sums in doubles leave 3.7e-7: what is left is
 * the integration's own error, a quarter as much at twice the substeps. Two other ways were measured on the same
 * tumble:
 *
 * <ul>
 *   <li><b>The explicit form</b>, {@code ω += h I⁻¹ (−ω × I ω)}, as Müller et al. write it: first order, and every
 *       substep adds a step at right angles to {@code L}, so {@code |L|} only grows, by 3.8%.</li>
 *   <li><b>The angular velocity as the state</b>, with {@code L} worked out from it and back every substep: the same
 *       turn as here. In f32 every round trip rounded the same way, and the energy drifted steadily, 1.4e-4 on the
 *       CPU and −3.5e-4 on the GPU, twice as fast at twice the substeps. Making {@link #rotate} exact for a
 *       quaternion not quite of unit length cut the CPU's to 7.5e-6 and left the GPU's at 1.1e-4, whose division is
 *       not correctly rounded, so that {@code I} and {@code I⁻¹} are not quite inverses. Keeping {@code L} removes
 *       the round trip, and with it both.</li>
 * </ul>
 *
 * <p>A constraint that turns a box will add to {@code L}, as a sphere's add to its angular velocity.
 *
 * <p>Lengths are in metres, times in seconds, {@code y} up; the walled box the bodies live in runs from the origin to
 * {@code (sx, sy, sz)}, as the spheres' does.
 */
public final class Boxes {

    /** The workgroup every pass must be registered with, as for spheres. */
    public static final int WORKGROUP = CountingSort.BLOCK;

    /** {@code [h, gx, gy, gz, sx, sy, sz, muStatic, muKinetic, passes]}, see {@link #params}. */
    public static final int PARAM_COUNT = 10;

    static final int H = 0;
    static final int GX = 1;
    static final int GY = 2;
    static final int GZ = 3;
    static final int SX = 4;
    static final int SY = 5;
    static final int SZ = 6;
    static final int MU_STATIC = 7;
    static final int MU_KINETIC = 8;
    static final int PASSES = 9;

    /** The sweeps over its corners a box's {@link #walls} pass makes, unless told otherwise. */
    public static final int WALL_PASSES = 4;
    /** The corners of a box, by the bits of their index: bit 0 the sign along x, bit 1 along y, bit 2 along z. */
    public static final int CORNERS = 8;
    /** The walls, low then high along x, then y, then z. */
    public static final int WALLS = 6;
    /** Words a box keeps in {@code lambda}: each corner's normal impulse against each wall, this substep. */
    public static final int LAMBDA_WORDS = WALLS * CORNERS;
    /** Words a box keeps in {@code rub}: each corner's whole tangential impulse against each wall, this substep. */
    public static final int RUB_WORDS = 3 * LAMBDA_WORDS;

    /** Every per-box field, in the order {@link BoxStep} holds them. */
    static final String[] BOX = {"x", "y", "z", "u", "v", "w", "lx", "ly", "lz", "qx", "qy", "qz", "qw", "im", "hx",
            "hy", "hz", "oqx", "oqy", "oqz", "oqw", "dx", "dy", "dz", "jx", "jy", "jz"};

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
        return params(h, gx, gy, gz, sx, sy, sz, 0, 0, WALL_PASSES);
    }

    /**
     * As above, with friction and the walls' sweeps.
     *
     * @param muStatic  a contact holds while the tangential impulse it needs is at most this times its normal one
     * @param muKinetic a contact that slips is resisted by this times its normal impulse
     * @param passes    the sweeps over its corners each box's {@link #walls} makes a substep
     */
    public static int[] params(double h, double gx, double gy, double gz, double sx, double sy, double sz,
                               double muStatic, double muKinetic, int passes) {
        if (!(h > 0) || !(sx > 0) || !(sy > 0) || !(sz > 0)) {
            throw new IllegalArgumentException("h > 0 and a box of positive size, got h " + h + ", box " + sx + " × "
                    + sy + " × " + sz);
        }
        if (!(muStatic >= 0) || !(muKinetic >= 0) || Double.isInfinite(muStatic) || Double.isInfinite(muKinetic)
                || passes < 1) {
            throw new IllegalArgumentException("friction finite and at least 0, and a pass at least, got " + muStatic
                    + ", " + muKinetic + " and " + passes);
        }
        return new int[] {bits(h), bits(gx), bits(gy), bits(gz), bits(sx), bits(sy), bits(sz), bits(muStatic),
                bits(muKinetic), bits(passes)};
    }

    /**
     * A solid box's principal moments of inertia, about its own x, y and z, for mass {@code m} and half-extents
     * {@code hx, hy, hz}: {@code m (hy² + hz²) / 3} and the like.
     */
    public static double[] inertia(double m, double hx, double hy, double hz) {
        return new double[] {m * (hy * hy + hz * hz) / 3, m * (hx * hx + hz * hz) / 3, m * (hx * hx + hy * hy) / 3};
    }

    // --- the passes ----------------------------------------------------------------------------------------

    static final String[] PREDICT_NAMES = {"x", "y", "z", "u", "v", "w", "im", "params", "lx", "ly", "lz", "qx", "qy",
            "qz", "qw", "hx", "hy", "hz", "oqx", "oqy", "oqz", "oqw"};
    static final List<Buffer> PREDICT_BUFFERS = bind(PREDICT_NAMES);

    /**
     * One invocation per box: gravity into the velocity, and the box moved along it, by symplectic Euler as a sphere
     * is; and the box turned, its angular momentum kept ({@link Boxes above}). How the box was turned is kept
     * in {@code oqx, oqy, oqz, oqw}, for {@link #walls}' friction to measure how far a corner has slipped this
     * substep. The box is loaded once and stored once.
     */
    public static Function predict() {
        List<Buffer> bs = PREDICT_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] velocity = {bs.get(3), bs.get(4), bs.get(5)};
        Buffer im = bs.get(6);
        Buffer params = bs.get(7);
        Buffer[] spin = {bs.get(8), bs.get(9), bs.get(10)};  // the angular momentum, lx, ly, lz
        Buffer[] turn = {bs.get(11), bs.get(12), bs.get(13), bs.get(14)};
        Buffer[] half = {bs.get(15), bs.get(16), bs.get(17)};
        Buffer[] wasTurned = {bs.get(18), bs.get(19), bs.get(20), bs.get(21)};
        int[] gravity = {GX, GY, GZ};

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar inverseMass = t.let("im", load(im, v(s)));
            t.when(gt(v(inverseMass), f(0)), free -> {
                LocalVar h = free.let("h", load(params, i(H)));
                for (int a = 0; a < 3; a++) {
                    LocalVar from = free.let("from", load(at[a], v(s)));
                    LocalVar vel = free.let("vel", add(load(velocity[a], v(s)),
                            mul(v(h), load(params, i(gravity[a])))));
                    free.store(velocity[a], v(s), v(vel));
                    free.store(at[a], v(s), add(v(from), mul(v(h), v(vel))));
                }

                LocalVar[] q = new LocalVar[4];
                for (int k = 0; k < 4; k++) {
                    q[k] = free.let("q", load(turn[k], v(s)));
                    free.store(wasTurned[k], v(s), v(q[k]));
                }
                LocalVar[] momentum = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    momentum[a] = free.let("momentum", load(spin[a], v(s)));
                }
                LocalVar[] inverse = inverseInertia(free, half, s, inverseMass);

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
        return function("boxesPredict", b);
    }

    static final String[] WALLS_NAMES = {"x", "y", "z", "qx", "qy", "qz", "qw", "im", "hx", "hy", "hz", "params", "u",
            "v", "w", "oqx", "oqy", "oqz", "oqw", "dx", "dy", "dz", "jx", "jy", "jz", "lambda", "rub"};
    static final List<Buffer> WALLS_BUFFERS = bind(WALLS_NAMES);

    /**
     * One invocation per box: its eight corners kept inside the six walls, and friction where they touch.
     *
     * <p>A corner is a contact with a wall while it is outside it, or while it has pushed against it this substep.
     * Each is corrected as XPBD corrects a contact with no compliance: by its depth over the generalised inverse mass
     * {@code w = im + (r × n)ᵀ I⁻¹ (r × n)}, {@code r} from the centre to the corner and {@code I⁻¹} turned into the
     * world's frame, so a push off-centre turns the box as well as moving it. The corners are swept in turn, each
     * against the box as the ones before it left it, {@code passes} times. Each corner keeps the normal impulse it
     * has taken this substep in {@code lambda}, and the whole of it is kept from falling below zero, as sequential
     * impulses do: a later sweep can draw a corner back by no more than it pushed.
     *
     * <p>Then friction, once every normal is corrected, which is what the spheres found it must wait for. A corner's
     * slip is how far it has moved along the wall since the substep began ({@link Box#slip}); removing it takes a
     * tangential impulse of {@code |slip| / w}, {@code w} along the slip. Each corner keeps the whole tangential
     * impulse it has taken this substep in {@code rub}, and the whole is held to {@code μs λₙ}, or cut to
     * {@code μk λₙ} past it, and only the change applied; the corners are swept {@code passes} times. One sweep, each
     * corner clamping only its own share, left a box on a slope creeping down it: each corner's push turned the box,
     * and left slip at the corners already rubbed.
     *
     * <p>Every move and every angular impulse {@code r × p} is added up as it is computed, into {@code dx, dy, dz} and
     * {@code jx, jy, jz}, for {@link #velocity}. The box is loaded once and stored once.
     */
    public static Function walls() {
        List<Buffer> bs = WALLS_BUFFERS;
        Buffer[] atBuffer = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] turnBuffer = {bs.get(3), bs.get(4), bs.get(5), bs.get(6)};
        Buffer im = bs.get(7);
        Buffer[] half = {bs.get(8), bs.get(9), bs.get(10)};
        Buffer params = bs.get(11);
        Buffer[] velocityBuffer = {bs.get(12), bs.get(13), bs.get(14)};
        Buffer[] wasTurnedBuffer = {bs.get(15), bs.get(16), bs.get(17), bs.get(18)};
        Buffer[] movedBuffer = {bs.get(19), bs.get(20), bs.get(21)};
        Buffer[] impulseBuffer = {bs.get(22), bs.get(23), bs.get(24)};
        Buffer lambda = bs.get(25);
        Buffer rub = bs.get(26);
        int[] extentParam = {SX, SY, SZ};

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar inverseMass = t.let("im", load(im, v(s)));
            t.when(gt(v(inverseMass), f(0)), free -> {
                Box box = new Box(free, s, inverseMass, atBuffer, turnBuffer, half, velocityBuffer,
                        free.let("h", load(params, i(H))), wasTurnedBuffer);
                LocalVar[] extent = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    extent[a] = free.let("extent", load(params, i(extentParam[a])));
                }
                LocalVar base = free.let("base", mul(v(s), i(LAMBDA_WORDS)));
                LocalVar slot = free.let("slot", i(0));
                free.loop(lt(v(slot), i(LAMBDA_WORDS)), clear -> {
                    clear.store(lambda, add(v(base), v(slot)), f(0));
                    clear.set(slot, add(v(slot), i(1)));
                });

                LocalVar passes = free.let("passes", toInt(load(params, i(PASSES))));
                LocalVar pass = free.let("pass", i(0));
                free.loop(lt(v(pass), v(passes)), sweep -> {
                    for (int wall = 0; wall < WALLS; wall++) {
                        int axis = wall / 2;
                        boolean high = wall % 2 == 1;
                        int first = wall * CORNERS;
                        LocalVar[] n = box.normal(sweep, axis, high);
                        box.eachCorner(sweep, (corner, c, offset, r) -> {
                            LocalVar reach = corner.let("reach", add(v(box.at[axis]), v(r[axis])));
                            LocalVar dist = corner.let("dist", high ? sub(v(extent[axis]), v(reach)) : v(reach));
                            LocalVar at = corner.let("at", add(v(base), add(i(first), v(c))));
                            LocalVar old = corner.let("old", load(lambda, v(at)));
                            corner.when(or(lt(v(dist), f(0)), gt(v(old), f(0))), touch -> {
                                LocalVar w = touch.let("w", box.inverseMassAlong(touch, r, n));
                                LocalVar total = touch.let("total", max(f(0), sub(v(old), div(v(dist), v(w)))));
                                touch.store(lambda, v(at), v(total));
                                LocalVar by = touch.let("by", sub(v(total), v(old)));
                                LocalVar[] p = new LocalVar[3];
                                for (int a = 0; a < 3; a++) {
                                    p[a] = touch.let("p", mul(v(by), v(n[a])));
                                }
                                box.push(touch, r, p);
                            });
                        });
                    }
                    sweep.set(pass, add(v(pass), i(1)));
                });

                LocalVar muStatic = free.let("muS", load(params, i(MU_STATIC)));
                LocalVar muKinetic = free.let("muK", load(params, i(MU_KINETIC)));
                free.when(gt(add(v(muStatic), v(muKinetic)), f(0)), rubbing -> {
                    LocalVar rubBase = rubbing.let("rubBase", mul(v(s), i(RUB_WORDS)));
                    LocalVar word = rubbing.let("word", i(0));
                    rubbing.loop(lt(v(word), i(RUB_WORDS)), clear -> {
                        clear.store(rub, add(v(rubBase), v(word)), f(0));
                        clear.set(word, add(v(word), i(1)));
                    });
                    LocalVar rubPass = rubbing.let("rubPass", i(0));
                    rubbing.loop(lt(v(rubPass), v(passes)), sweep -> {
                        for (int wall = 0; wall < WALLS; wall++) {
                            int axis = wall / 2;
                            LocalVar[] n = box.normal(sweep, axis, wall % 2 == 1);
                            int first = wall * CORNERS;
                            box.eachCorner(sweep, (corner, c, offset, r) -> {
                                LocalVar normal = corner.let("normal",
                                        load(lambda, add(v(base), add(i(first), v(c)))));
                                corner.when(gt(v(normal), f(0)), touching -> rubCorner(touching, box, offset, r, n,
                                        normal, rub, add(v(rubBase), mul(add(i(first), v(c)), i(3))), muStatic,
                                        muKinetic));
                            });
                        }
                        sweep.set(rubPass, add(v(rubPass), i(1)));
                    });
                });

                box.store(free, atBuffer, turnBuffer);
                for (int a = 0; a < 3; a++) {
                    free.store(movedBuffer[a], v(s), add(load(movedBuffer[a], v(s)), v(box.moved[a])));
                    free.store(impulseBuffer[a], v(s), add(load(impulseBuffer[a], v(s)), v(box.impulse[a])));
                }
            });
        });
        return function("boxesWalls", b);
    }

    /**
     * One corner's friction against one wall, in one sweep: the impulse that would remove its slip, added to the whole
     * tangential impulse it has taken this substep, which is kept at {@code rub[at]} on; the whole held to
     * {@code μs λₙ}, or cut to {@code μk λₙ} past it; and only the change applied.
     */
    private static void rubCorner(Body b, Box box, LocalVar[] offset, LocalVar[] r, LocalVar[] n, LocalVar normal,
                                  Buffer rub, Expr at, LocalVar muStatic, LocalVar muKinetic) {
        LocalVar where = b.let("where", at);
        LocalVar[] slip = box.slip(b, offset, r, n);
        LocalVar length = b.let("length", sqrt(dot(slip, slip)));
        b.when(gt(v(length), f(1e-12f)), slipping -> {
            LocalVar[] along = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                along[a] = slipping.let("along", div(v(slip[a]), v(length)));
            }
            LocalVar take = slipping.let("take", div(v(length), box.inverseMassAlong(slipping, r, along)));
            LocalVar[] old = new LocalVar[3];
            LocalVar[] whole = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                old[a] = slipping.let("old", load(rub, add(v(where), i(a))));
                whole[a] = slipping.let("whole", sub(v(old[a]), mul(v(take), v(along[a]))));
            }
            LocalVar size = slipping.let("size", sqrt(dot(whole, whole)));
            slipping.when(gt(v(size), mul(v(muStatic), v(normal))), kinetic -> {
                LocalVar cut = kinetic.let("cut", div(mul(v(muKinetic), v(normal)), v(size)));
                for (int a = 0; a < 3; a++) {
                    kinetic.set(whole[a], mul(v(whole[a]), v(cut)));
                }
            });
            LocalVar[] p = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                slipping.store(rub, add(v(where), i(a)), v(whole[a]));
                p[a] = slipping.let("p", sub(v(whole[a]), v(old[a])));
            }
            box.push(slipping, r, p);
        });
    }

    static final String[] VELOCITY_NAMES = {"u", "v", "w", "lx", "ly", "lz", "dx", "dy", "dz", "jx", "jy", "jz",
            "params"};
    static final List<Buffer> VELOCITY_BUFFERS = bind(VELOCITY_NAMES);

    /**
     * One invocation per box: what the constraints moved it by this substep, as a velocity, {@code v += d / h}; and
     * the angular impulse they gave it, as angular momentum, {@code L += j / h}; both cleared. Added up as they were
     * computed, for the reason the spheres' moves are, and never read back from two positions.
     */
    public static Function velocity() {
        List<Buffer> bs = VELOCITY_BUFFERS;
        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar perH = t.let("perH", div(f(1), load(bs.get(12), i(H))));
            for (int a = 0; a < 6; a++) {
                Buffer state = bs.get(a);
                Buffer added = bs.get(6 + a);
                t.store(state, v(s), add(load(state, v(s)), mul(load(added, v(s)), v(perH))));
                t.store(added, v(s), f(0));
            }
        });
        return function("boxesVelocity", b);
    }

    // --- one box, held in locals ---------------------------------------------------------------------------

    /** What a corner's work is given: the body, the corner's index, its offset in the box's frame, and in the world's. */
    @FunctionalInterface
    private interface CornerWork {
        void at(Body b, LocalVar corner, LocalVar[] offset, LocalVar[] r);
    }

    /**
     * A box as a pass holds it: its centre and orientation in locals, loaded once and stored once (the spheres' GPU
     * once lost a store that was loaded again), with what it has been moved and turned by.
     */
    private static final class Box {
        final LocalVar s;
        final LocalVar inverseMass;
        final LocalVar[] at = new LocalVar[3];
        final LocalVar[] q = new LocalVar[4];
        final LocalVar[] half = new LocalVar[3];
        final LocalVar[] inverse;
        /** How far the predict moved the centre this substep, {@code h v}. */
        final LocalVar[] stepped = new LocalVar[3];
        final LocalVar[] wasTurned = new LocalVar[4];
        final LocalVar[] moved = new LocalVar[3];
        final LocalVar[] impulse = new LocalVar[3];
        final LocalVar one;

        Box(Body b, LocalVar s, LocalVar inverseMass, Buffer[] atBuffer, Buffer[] turnBuffer, Buffer[] halfBuffer,
            Buffer[] velocityBuffer, LocalVar h, Buffer[] wasTurnedBuffer) {
            this.s = s;
            this.inverseMass = inverseMass;
            for (int a = 0; a < 3; a++) {
                at[a] = b.let("at", load(atBuffer[a], v(s)));
                half[a] = b.let("half", load(halfBuffer[a], v(s)));
                stepped[a] = b.let("stepped", mul(v(h), load(velocityBuffer[a], v(s))));
                moved[a] = b.let("moved", f(0));
                impulse[a] = b.let("impulse", f(0));
            }
            for (int k = 0; k < 4; k++) {
                q[k] = b.let("q", load(turnBuffer[k], v(s)));
                wasTurned[k] = b.let("wasTurned", load(wasTurnedBuffer[k], v(s)));
            }
            inverse = inverseInertia(b, halfBuffer, s, inverseMass);
            one = b.let("one", f(1));
        }

        /** The unit normal of a wall, pointing inside: {@code +e_axis} for the low wall, {@code −e_axis} for the high. */
        LocalVar[] normal(Body b, int axis, boolean high) {
            LocalVar[] n = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                n[a] = b.let("n", f(a == axis ? (high ? -1 : 1) : 0));
            }
            return n;
        }

        /** {@code work} for each of the eight corners, in a loop, with the corner where the box is turned now. */
        void eachCorner(Body b, CornerWork work) {
            LocalVar c = b.let("corner", i(0));
            b.loop(lt(v(c), i(CORNERS)), corner -> {
                LocalVar[] offset = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    Expr bit = mod(div(v(c), i(1 << a)), i(2));
                    offset[a] = corner.let("offset", mul(v(half[a]), sub(mul(f(2), toFloat(bit)), f(1))));
                }
                work.at(corner, c, offset, rotate(corner, q, offset));
                corner.set(c, add(v(c), i(1)));
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

        /** The positional impulse {@code p} at {@code r}: the box moved by {@code im p} and turned by {@code I⁻¹ (r × p)}. */
        void push(Body b, LocalVar[] r, LocalVar[] p) {
            for (int a = 0; a < 3; a++) {
                LocalVar move = b.let("move", mul(v(inverseMass), v(p[a])));
                b.set(at[a], add(v(at[a]), v(move)));
                b.set(moved[a], add(v(moved[a]), v(move)));
            }
            LocalVar[] angular = cross(b, r, p);
            for (int a = 0; a < 3; a++) {
                b.set(impulse[a], add(v(impulse[a]), v(angular[a])));
            }
            LocalVar[] now = turned(b, q, inverseInertiaTimes(b, angular), one);
            for (int k = 0; k < 4; k++) {
                b.set(q[k], v(now[k]));
            }
        }

        /**
         * How far the corner at {@code offset}, now at {@code r} from the centre, has moved along the wall of normal
         * {@code n} since the substep began: the centre's travel, {@code h v} and what the constraints moved it by, as
         * they were computed, and the turn's, {@code r} less where the corner was from the centre then.
         *
         * <p>Not the difference of two positions. That keeps only what f32 holds of a position a metre from the
         * origin, and a box resting on a 20° slope crept down it at 1.5e-5 m/s, a fifth of an ulp a substep.
         */
        LocalVar[] slip(Body b, LocalVar[] offset, LocalVar[] r, LocalVar[] n) {
            LocalVar[] then = rotate(b, wasTurned, offset);
            LocalVar[] moved = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                moved[a] = b.let("travel", add(add(v(stepped[a]), v(this.moved[a])), sub(v(r[a]), v(then[a]))));
            }
            LocalVar across = b.let("across", dot(moved, n));
            LocalVar[] slip = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                slip[a] = b.let("slip", sub(v(moved[a]), mul(v(across), v(n[a]))));
            }
            return slip;
        }

        /** The centre and orientation stored back, once. */
        void store(Body b, Buffer[] atBuffer, Buffer[] turnBuffer) {
            for (int a = 0; a < 3; a++) {
                b.store(atBuffer[a], v(s), v(at[a]));
            }
            for (int k = 0; k < 4; k++) {
                b.store(turnBuffer[k], v(s), v(q[k]));
            }
        }
    }

    // --- building blocks -----------------------------------------------------------------------------------

    private static Expr dot(LocalVar[] a, LocalVar[] b) {
        return add(add(mul(v(a[0]), v(b[0])), mul(v(a[1]), v(b[1]))), mul(v(a[2]), v(b[2])));
    }

    /** Box {@code s}'s inverse principal moments, {@code 3 im / (hy² + hz²)} and the like. */
    private static LocalVar[] inverseInertia(Body b, Buffer[] half, LocalVar s, LocalVar inverseMass) {
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
    private static LocalVar[] rotate(Body b, LocalVar[] q, LocalVar[] p) {
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

package dev.vexelray.sim.rigid.sphere;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.Buffer;
import dev.supirvast.vastir.core.Expr;
import dev.supirvast.vastir.core.Function;
import dev.supirvast.vastir.core.LocalVar;
import dev.supirvast.vastir.type.Type;

import java.util.ArrayList;
import java.util.List;

import static dev.supirvast.vastir.build.Body.F32;
import static dev.supirvast.vastir.build.Body.add;
import static dev.supirvast.vastir.build.Body.div;
import static dev.supirvast.vastir.build.Body.eq;
import static dev.supirvast.vastir.build.Body.f;
import static dev.supirvast.vastir.build.Body.gt;
import static dev.supirvast.vastir.build.Body.i;
import static dev.supirvast.vastir.build.Body.load;
import static dev.supirvast.vastir.build.Body.lt;
import static dev.supirvast.vastir.build.Body.max;
import static dev.supirvast.vastir.build.Body.min;
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.not;
import static dev.supirvast.vastir.build.Body.sqrt;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.v;

/**
 * Spheres in a walled box, under gravity, kept apart by position-based contact: the first rigid experiment.
 *
 * <p>A step is cut into substeps, and each substep is four kinds of pass ({@link SphereStep} orders them):
 *
 * <ol>
 *   <li>{@link #predict}: gravity into the velocity, and the sphere moved along it — symplectic Euler,
 *       unconstrained.</li>
 *   <li>{@link #solve}: each sphere finds every sphere it overlaps and sums the move that would part them, its share
 *       of each overlap by inverse mass. Jacobi: every sphere reads the positions as they were before the pass, so
 *       the spheres are independent, one invocation each, and no atomics.</li>
 *   <li>{@link #apply}: the moves added, and the walls enforced by projecting back inside; and each move, as it was
 *       computed, added up in {@code dx, dy, dz}.</li>
 *   <li>{@link #velocity}: what the constraints moved the sphere by, as a velocity: {@code v += d / h}. That is what
 *       makes a contact inelastic — the move that parted two spheres cancels the velocity that brought them
 *       together — and what makes a resting sphere rest.</li>
 * </ol>
 *
 * <p>XPBD usually writes the last as {@code v = (x − x₀) / h}, from the position at the start of the substep. It is
 * the same in exact arithmetic and not in f32, where it was measured:
 *
 * <ul>
 *   <li>The difference of two positions metres from the origin, a substep apart, keeps only a few digits of the
 *       velocity. A free fall from 8 m came out 0.26% slow after half a second.</li>
 *   <li>Reading the constraints' move back as {@code x − x̃}, from the predicted position, fixed the fall but not a
 *       collision: the move has been rounded into a position by then, and two spheres meeting at 1.5 m lost a
 *       ten-thousandth of their momentum.</li>
 * </ul>
 *
 * Adding up the moves as they were computed avoids both: an untouched sphere's velocity is exactly symplectic
 * Euler's, and two spheres' moves are equal and opposite by mass to the rounding of one product.
 *
 * <p>Solve and apply may repeat within a substep, but small substeps with one pass each are what XPBD's own
 * measurements favour, and {@link SphereStep} defaults to that.
 *
 * <h2>Two ways to relax, and why it is a parameter</h2>
 *
 * A Jacobi pass over-corrects where a sphere has many contacts: each one moves it the whole of its share, and the
 * shares add. Two remedies, chosen by the {@link #AVERAGED} parameter:
 *
 * <ul>
 *   <li><b>Constant</b>: every move scaled by {@code ω}. Two spheres in contact take equal and opposite shares, so the
 *       total momentum does not change by a bit — but a sphere in a pile with twelve contacts is still pushed twelve
 *       times, and needs {@code ω} small, or many passes, to settle.</li>
 *   <li><b>Averaged</b>: a sphere's sum divided by its own contact count, times {@code ω}. A crowded sphere moves a
 *       sensible amount at once; but two spheres with different counts no longer take equal shares, so momentum is
 *       only conserved where counts agree.</li>
 * </ul>
 *
 * Which is better for piles and stacks is the experiment's question, and its answer is measured, not assumed.
 *
 * <h2>What it is not, yet</h2>
 *
 * <ul>
 *   <li><b>Every pair is tested.</b> A sphere loops over all the others: {@code n²} tests per pass. There is no broad
 *       phase. It is the simplest thing that is right, so that the solver can be judged before the search is.</li>
 *   <li><b>No rotation, so no friction.</b> A sphere is a point with a radius. Frictionless spheres stack in a column
 *       held by walls, and a pile of them slumps flat; both are the technique, not bugs.</li>
 *   <li><b>No restitution.</b> Every contact is perfectly inelastic.</li>
 *   <li><b>Only spheres</b>, which is what makes contact a distance. Boxes need orientation and a contact manifold.</li>
 * </ul>
 *
 * <p>Lengths are in metres, times in seconds; the box runs from the origin to {@code (sx, sy, sz)} with {@code y} up.
 * A sphere with inverse mass zero is fixed.
 */
public final class Spheres {

    /** The workgroup every pass must be registered with. Nothing here needs a particular one, nor a subgroup. */
    public static final int WORKGROUP = 64;

    /** {@code [h, gx, gy, gz, sx, sy, sz, omega, averaged]}, see {@link #params}. */
    public static final int PARAM_COUNT = 9;

    static final int H = 0;
    static final int GX = 1;
    static final int GY = 2;
    static final int GZ = 3;
    static final int SX = 4;
    static final int SY = 5;
    static final int SZ = 6;
    static final int OMEGA = 7;
    static final int AVERAGED = 8;

    /** Below this squared distance two centres are taken as one point, which has no direction to part them along. */
    private static final float COINCIDENT = 1e-12f;

    private Spheres() {
    }

    /**
     * The parameters as {@link #PARAM_COUNT} words.
     *
     * @param h        the substep, in seconds
     * @param gx       gravity along x, in metres per second squared; likewise {@code gy} (negative is down) and {@code gz}
     * @param sx       the box's extent along x, in metres; likewise {@code sy} and {@code sz}
     * @param omega    the relaxation: the share of each solve's move that is taken, in (0, 1]
     * @param averaged whether a sphere's move is divided by its contact count ({@link Spheres above})
     */
    public static int[] params(double h, double gx, double gy, double gz, double sx, double sy, double sz,
                               double omega, boolean averaged) {
        if (!(h > 0) || !(sx > 0) || !(sy > 0) || !(sz > 0) || !(omega > 0) || !(omega <= 1)) {
            throw new IllegalArgumentException("h > 0, a box of positive size and 0 < omega <= 1, got h " + h
                    + ", box " + sx + " × " + sy + " × " + sz + ", omega " + omega);
        }
        return new int[] {bits(h), bits(gx), bits(gy), bits(gz), bits(sx), bits(sy), bits(sz), bits(omega),
                bits(averaged ? 1 : 0)};
    }

    // --- the buffers each pass binds, in order -------------------------------------------------------------

    /** Every per-sphere field, in the order {@link SphereStep} holds them. */
    static final String[] SPHERE = {"x", "y", "z", "u", "v", "w", "r", "im", "cx", "cy", "cz", "dx", "dy", "dz"};

    static final String[] PREDICT_NAMES = {"x", "y", "z", "u", "v", "w", "im", "params"};
    static final List<Buffer> PREDICT_BUFFERS = bind(PREDICT_NAMES);

    static final String[] SOLVE_NAMES = {"x", "y", "z", "r", "im", "cx", "cy", "cz", "params"};
    static final List<Buffer> SOLVE_BUFFERS = bind(SOLVE_NAMES);

    static final String[] APPLY_NAMES = {"x", "y", "z", "r", "im", "cx", "cy", "cz", "dx", "dy", "dz", "params"};
    static final List<Buffer> APPLY_BUFFERS = bind(APPLY_NAMES);

    static final String[] VELOCITY_NAMES = {"u", "v", "w", "dx", "dy", "dz", "params"};
    static final List<Buffer> VELOCITY_BUFFERS = bind(VELOCITY_NAMES);

    // --- the passes ----------------------------------------------------------------------------------------

    /** One invocation per sphere: gravity into the velocity, and the sphere moved along it. */
    public static Function predict() {
        List<Buffer> bs = PREDICT_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] velocity = {bs.get(3), bs.get(4), bs.get(5)};
        Buffer im = bs.get(6);
        Buffer params = bs.get(7);
        int[] gravity = {GX, GY, GZ};

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> t.when(gt(load(im, v(s)), f(0)), free -> {
            LocalVar h = free.let("h", load(params, i(H)));
            for (int a = 0; a < 3; a++) {
                LocalVar vel = free.let("vel", add(load(velocity[a], v(s)), mul(v(h), load(params, i(gravity[a])))));
                free.store(velocity[a], v(s), v(vel));
                free.store(at[a], v(s), add(load(at[a], v(s)), mul(v(h), v(vel))));
            }
        }));
        return function("spheresPredict", b);
    }

    /**
     * One invocation per sphere: the move that would part it from every sphere it overlaps, its share of each by
     * inverse mass, relaxed as the parameters say, into {@code cx, cy, cz}. Reads positions only, so every sphere
     * sees the same state.
     */
    public static Function solve() {
        List<Buffer> bs = SOLVE_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer radius = bs.get(3);
        Buffer im = bs.get(4);
        Buffer[] correction = {bs.get(5), bs.get(6), bs.get(7)};
        Buffer params = bs.get(8);

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        LocalVar count = b.let("count", new Expr.InvocationCount());
        b.when(lt(v(s), v(count)), t -> {
            LocalVar[] mine = {t.let("xs", load(at[0], v(s))), t.let("ys", load(at[1], v(s))),
                    t.let("zs", load(at[2], v(s)))};
            LocalVar rs = t.let("rs", load(radius, v(s)));
            LocalVar ws = t.let("ws", load(im, v(s)));
            LocalVar[] sum = {t.let("sx", f(0)), t.let("sy", f(0)), t.let("sz", f(0))};
            LocalVar contacts = t.let("contacts", f(0));
            LocalVar o = t.let("o", i(0));
            t.loop(lt(v(o), v(count)), pass -> {
                pass.when(not(eq(v(o), v(s))), other -> {
                    LocalVar[] d = new LocalVar[3];
                    for (int a = 0; a < 3; a++) {
                        d[a] = other.let("d", sub(v(mine[a]), load(at[a], v(o))));
                    }
                    LocalVar d2 = other.let("d2", add(mul(v(d[0]), v(d[0])),
                            add(mul(v(d[1]), v(d[1])), mul(v(d[2]), v(d[2])))));
                    LocalVar reach = other.let("reach", add(v(rs), load(radius, v(o))));
                    LocalVar wsum = other.let("wsum", add(v(ws), load(im, v(o))));
                    other.when(lt(v(d2), mul(v(reach), v(reach))), near ->
                            near.when(gt(v(d2), f(COINCIDENT)), apart ->
                                    apart.when(gt(v(wsum), f(0)), touching -> {
                                        LocalVar dist = touching.let("dist", sqrt(v(d2)));
                                        // The overlap, this sphere's share of it, per unit of the separation vector.
                                        LocalVar share = touching.let("share", div(mul(sub(v(reach), v(dist)),
                                                div(v(ws), v(wsum))), v(dist)));
                                        for (int a = 0; a < 3; a++) {
                                            touching.set(sum[a], add(v(sum[a]), mul(v(share), v(d[a]))));
                                        }
                                        touching.set(contacts, add(v(contacts), f(1)));
                                    })));
                });
                pass.set(o, add(v(o), i(1)));
            });
            LocalVar omega = t.let("omega", load(params, i(OMEGA)));
            LocalVar averaged = t.let("averaged", load(params, i(AVERAGED)));
            LocalVar scale = t.let("scale", v(omega));
            t.when(gt(v(averaged), f(0)), avg -> avg.set(scale, div(v(omega), max(v(contacts), f(1)))));
            for (int a = 0; a < 3; a++) {
                t.store(correction[a], v(s), mul(v(scale), v(sum[a])));
            }
        });
        return function("spheresSolve", b);
    }

    /** One invocation per sphere: the solve's move added, then the walls, by projecting the sphere back inside. */
    public static Function apply() {
        List<Buffer> bs = APPLY_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer radius = bs.get(3);
        Buffer im = bs.get(4);
        Buffer[] correction = {bs.get(5), bs.get(6), bs.get(7)};
        Buffer[] displaced = {bs.get(8), bs.get(9), bs.get(10)};
        Buffer params = bs.get(11);
        int[] extent = {SX, SY, SZ};

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> t.when(gt(load(im, v(s)), f(0)), free -> {
            LocalVar r = free.let("r", load(radius, v(s)));
            for (int a = 0; a < 3; a++) {
                LocalVar c = free.let("c", load(correction[a], v(s)));
                LocalVar moved = free.let("moved", add(load(at[a], v(s)), v(c)));
                LocalVar high = free.let("high", sub(load(params, i(extent[a])), v(r)));
                LocalVar inside = free.let("inside", max(v(r), min(v(high), v(moved))));
                free.store(at[a], v(s), v(inside));
                // The move as computed, not as the rounded position has it; the wall's push only where it pushed.
                free.store(displaced[a], v(s), add(load(displaced[a], v(s)), add(v(c), sub(v(inside), v(moved)))));
            }
        }));
        return function("spheresApply", b);
    }

    /** One invocation per sphere: what the constraints moved it by this substep, added as a velocity, and cleared. */
    public static Function velocity() {
        List<Buffer> bs = VELOCITY_BUFFERS;
        Buffer[] velocity = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] displaced = {bs.get(3), bs.get(4), bs.get(5)};
        Buffer params = bs.get(6);

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar perH = t.let("perH", div(f(1), load(params, i(H))));
            for (int a = 0; a < 3; a++) {
                t.store(velocity[a], v(s), add(load(velocity[a], v(s)), mul(load(displaced[a], v(s)), v(perH))));
                t.store(displaced[a], v(s), f(0));
            }
        });
        return function("spheresVelocity", b);
    }

    // --- for a picture -------------------------------------------------------------------------------------

    /** Floats per sphere in {@code shown}: the centre before the step, the radius, the centre after, and one spare. */
    public static final int SHOWN_STRIDE = 8;

    static final String[] SHOW_NAMES = {"x", "y", "z", "r", "shown"};
    static final List<Buffer> SHOW_BUFFERS = bind(SHOW_NAMES);

    /**
     * One invocation per sphere, at the end of a step: the centre the last step ended on moved to the "before" slot
     * of {@code shown}, and the centre this one ended on put in the "after" slot, with the radius. A picture blends
     * the two by how far the frame is between the steps, and reads one buffer to do it.
     */
    public static Function show() {
        List<Buffer> bs = SHOW_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer radius = bs.get(3);
        Buffer shown = bs.get(4);

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar base = t.let("base", mul(v(s), i(SHOWN_STRIDE)));
            for (int a = 0; a < 3; a++) {
                t.store(shown, add(v(base), i(a)), load(shown, add(v(base), i(4 + a))));
                t.store(shown, add(v(base), i(4 + a)), load(at[a], v(s)));
            }
            t.store(shown, add(v(base), i(3)), load(radius, v(s)));
        });
        return function("spheresShow", b);
    }

    // --- building blocks -----------------------------------------------------------------------------------

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

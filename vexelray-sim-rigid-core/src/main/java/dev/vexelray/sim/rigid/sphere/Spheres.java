package dev.vexelray.sim.rigid.sphere;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.AtomicOp;
import dev.supirvast.vastir.core.BinaryOp;
import dev.supirvast.vastir.core.Buffer;
import dev.supirvast.vastir.core.Expr;
import dev.supirvast.vastir.core.Function;
import dev.supirvast.vastir.core.LocalVar;
import dev.supirvast.vastir.pass.CountingSort;
import dev.supirvast.vastir.type.Type;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import static dev.supirvast.vastir.build.Body.F32;
import static dev.supirvast.vastir.build.Body.I32;
import static dev.supirvast.vastir.build.Body.add;
import static dev.supirvast.vastir.build.Body.and;
import static dev.supirvast.vastir.build.Body.div;
import static dev.supirvast.vastir.build.Body.eq;
import static dev.supirvast.vastir.build.Body.f;
import static dev.supirvast.vastir.build.Body.floor;
import static dev.supirvast.vastir.build.Body.gt;
import static dev.supirvast.vastir.build.Body.i;
import static dev.supirvast.vastir.build.Body.load;
import static dev.supirvast.vastir.build.Body.lt;
import static dev.supirvast.vastir.build.Body.max;
import static dev.supirvast.vastir.build.Body.min;
import static dev.supirvast.vastir.build.Body.mod;
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.neg;
import static dev.supirvast.vastir.build.Body.not;
import static dev.supirvast.vastir.build.Body.sqrt;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.toInt;
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
 * <h2>Which spheres a solve tests</h2>
 *
 * {@link #solve()} tests every other sphere, {@code n²} tests a pass: the simplest thing that is right, and the
 * reference. {@link #solve(SphereGrid)} tests only the spheres in the 27 cells around a sphere's own, after
 * {@link #bin} and SupirVast's counting sort have listed the spheres by cell. The two find the same contacts, and
 * differ only in the order each sphere sums them, so they agree to rounding.
 *
 * <h2>Gauss–Seidel</h2>
 *
 * Contact after contact, each against the positions the ones before it left, moving both spheres of a contact at
 * once; then {@link #walls}. A stack hears its own weight within one pass, which Jacobi cannot.
 * {@link #contactRound} works over a list of contacts made each substep ({@link #listContacts}), in rounds that each
 * solve a set of contacts sharing no sphere, one invocation a contact, until none is left open. A first version, in
 * 27 colours of the grid's cells with one serial invocation a cell, was as right and five times the cost, and is
 * gone; {@code docs/TODO.md} keeps its measurements.
 *
 * <h2>What it is not, yet</h2>
 *
 * <ul>
 *   <li><b>No friction, so nothing turns a sphere.</b> A sphere carries an angular velocity {@code ax, ay, az} and an
 *       orientation {@code qx, qy, qz, qw}, which {@link #predict} turns and to which {@link #velocity} adds the
 *       constraints' turns {@code tx, ty, tz}; but no constraint turns one yet, so a sphere keeps the spin it was
 *       given. Frictionless spheres stack in a column held by walls, and a pile of them slumps flat; both are the
 *       technique, not bugs.</li>
 *   <li><b>Only spheres</b>, which is what makes contact a distance. Boxes need orientation and a contact manifold.</li>
 * </ul>
 *
 * <p>Lengths are in metres, times in seconds; the box runs from the origin to {@code (sx, sy, sz)} with {@code y} up.
 * A sphere with inverse mass zero is fixed.
 */
public final class Spheres {

    /** The workgroup every pass must be registered with. Nothing here needs a particular one, nor a subgroup. */
    public static final int WORKGROUP = CountingSort.BLOCK;

    /** {@code [h, gx, gy, gz, sx, sy, sz, omega, averaged, restitution]}, see {@link #params}. */
    public static final int PARAM_COUNT = 10;

    static final int H = 0;
    static final int GX = 1;
    static final int GY = 2;
    static final int GZ = 3;
    static final int SX = 4;
    static final int SY = 5;
    static final int SZ = 6;
    static final int OMEGA = 7;
    static final int AVERAGED = 8;
    static final int RESTITUTION = 9;

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
        return params(h, gx, gy, gz, sx, sy, sz, omega, averaged, 0);
    }

    /**
     * As above, with restitution.
     *
     * @param restitution how much of a contact's approach speed it parts at, from 0, the default, which leaves spheres
     *                    touching, to 1, which keeps the speed ({@link #bounce})
     */
    public static int[] params(double h, double gx, double gy, double gz, double sx, double sy, double sz,
                               double omega, boolean averaged, double restitution) {
        if (!(restitution >= 0) || !(restitution <= 1)) {
            throw new IllegalArgumentException("restitution is in [0, 1], got " + restitution);
        }
        if (!(h > 0) || !(sx > 0) || !(sy > 0) || !(sz > 0) || !(omega > 0) || !(omega <= 1)) {
            throw new IllegalArgumentException("h > 0, a box of positive size and 0 < omega <= 1, got h " + h
                    + ", box " + sx + " × " + sy + " × " + sz + ", omega " + omega);
        }
        return new int[] {bits(h), bits(gx), bits(gy), bits(gz), bits(sx), bits(sy), bits(sz), bits(omega),
                bits(averaged ? 1 : 0), bits(restitution)};
    }

    // --- the buffers each pass binds, in order -------------------------------------------------------------

    /** Every per-sphere field, in the order {@link SphereStep} holds them. */
    static final String[] SPHERE = {"x", "y", "z", "u", "v", "w", "r", "im", "cx", "cy", "cz", "dx", "dy", "dz",
            "ax", "ay", "az", "tx", "ty", "tz", "qx", "qy", "qz", "qw"};

    /**
     * A solid sphere's moment of inertia, as a share of {@code m r²}: {@code I = ⅖ m r²}, the same about every axis, so
     * its inverse is {@code im / (⅖ r²)} and needs no buffer of its own.
     */
    public static final double INERTIA = 0.4;

    static final String[] PREDICT_NAMES = {"x", "y", "z", "u", "v", "w", "im", "params", "ax", "ay", "az", "qx", "qy",
            "qz", "qw"};
    static final List<Buffer> PREDICT_BUFFERS = bind(PREDICT_NAMES);

    static final String[] SOLVE_NAMES = {"x", "y", "z", "r", "im", "cx", "cy", "cz", "params"};
    static final List<Buffer> SOLVE_BUFFERS = bind(SOLVE_NAMES);

    static final String[] APPLY_NAMES = {"x", "y", "z", "r", "im", "cx", "cy", "cz", "dx", "dy", "dz", "params"};
    static final List<Buffer> APPLY_BUFFERS = bind(APPLY_NAMES);

    static final String[] VELOCITY_NAMES = {"u", "v", "w", "dx", "dy", "dz", "params", "cx", "cy", "cz", "ax", "ay",
            "az", "tx", "ty", "tz", "qx", "qy", "qz", "qw"};
    static final List<Buffer> VELOCITY_BUFFERS = bind(VELOCITY_NAMES);

    // --- the passes ----------------------------------------------------------------------------------------

    /**
     * One invocation per sphere: gravity into the velocity, and the sphere moved along it; and the sphere turned by
     * its angular velocity. Nothing applies a torque, so the angular velocity is left as it is.
     */
    public static Function predict() {
        List<Buffer> bs = PREDICT_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] velocity = {bs.get(3), bs.get(4), bs.get(5)};
        Buffer im = bs.get(6);
        Buffer params = bs.get(7);
        Buffer[] spin = {bs.get(8), bs.get(9), bs.get(10)};
        Buffer[] q = {bs.get(11), bs.get(12), bs.get(13), bs.get(14)};
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
            Expr[] angle = new Expr[3];
            for (int a = 0; a < 3; a++) {
                angle[a] = mul(v(h), load(spin[a], v(s)));
            }
            turn(free, s, q, angle);
        }));
        return function("spheresPredict", b);
    }

    /**
     * One invocation per sphere: the move that would part it from every sphere it overlaps, its share of each by
     * inverse mass, relaxed as the parameters say, into {@code cx, cy, cz}. Reads positions only, so every sphere
     * sees the same state. Every other sphere is tested: {@code n²} tests a pass.
     */
    public static Function solve() {
        List<Buffer> bs = SOLVE_BUFFERS;
        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        LocalVar count = b.let("count", new Expr.InvocationCount());
        b.when(lt(v(s), v(count)), t -> {
            Solving solving = Solving.begin(t, s, bs);
            LocalVar o = t.let("o", i(0));
            t.loop(lt(v(o), v(count)), pass -> {
                solving.against(pass, o);
                pass.set(o, add(v(o), i(1)));
            });
            store(t, s, solving.move(t), bs.get(5), bs.get(6), bs.get(7));
        });
        return function("spheresSolve", b);
    }

    static final String[] GRID_SOLVE_NAMES = {"x", "y", "z", "r", "im", "cx", "cy", "cz", "params", "keys", "starts",
            "order"};
    static final List<Buffer> GRID_SOLVE_BUFFERS = bind(GRID_SOLVE_NAMES, 9);

    /**
     * {@link #solve()}, with the spheres sorted into {@code grid}'s cells: a sphere tests only those in its own
     * cell and the 26 around it, which is every sphere it can touch while a cell is at least the widest sphere's
     * diameter. {@code keys} is the cell each sphere was sorted into, and {@code starts} and {@code order} list
     * each cell's spheres, as {@link #bin} and the sort left them.
     */
    public static Function solve(SphereGrid grid) {
        List<Buffer> bs = GRID_SOLVE_BUFFERS;
        Buffer keys = bs.get(9);
        int nx = grid.nx();
        int ny = grid.ny();

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            Solving solving = Solving.begin(t, s, bs);
            LocalVar key = t.let("key", load(keys, v(s)));
            LocalVar ix = t.let("ix", mod(v(key), i(nx)));
            LocalVar iy = t.let("iy", mod(div(v(key), i(nx)), i(ny)));
            LocalVar iz = t.let("iz", div(v(key), i(nx * ny)));
            around(t, grid, ix, iy, iz, bs.get(10), bs.get(11), solving);
            store(t, s, solving.move(t), bs.get(5), bs.get(6), bs.get(7));
        });
        return function("spheresGridSolve", b);
    }

    /**
     * Spheres {@code a} and {@code o} parted, if they overlap: each moved along the line between them by its share
     * of {@code ω} times the overlap, by inverse mass, and each move added to what the constraints have moved it.
     *
     * <p>Both at once, from one reading of their positions: so the two moves are equal and opposite by mass. Moving one
     * sphere a turn, each by its share of the overlap it saw, was measured and is wrong: the second of a pair sees an
     * overlap the first has shrunk, and a collision of 1 kg into 3 kg gained a quarter of its momentum.
     */
    private static void contact(Body b, LocalVar a, LocalVar o, Buffer[] at, Buffer radius, Buffer im,
                                Buffer[] displaced, LocalVar omega) {
        LocalVar[] d = new LocalVar[3];
        for (int axis = 0; axis < 3; axis++) {
            d[axis] = b.let("d", sub(load(at[axis], v(a)), load(at[axis], v(o))));
        }
        LocalVar d2 = b.let("d2", add(mul(v(d[0]), v(d[0])), add(mul(v(d[1]), v(d[1])), mul(v(d[2]), v(d[2])))));
        LocalVar reach = b.let("reach", add(load(radius, v(a)), load(radius, v(o))));
        LocalVar wa = b.let("wa", load(im, v(a)));
        LocalVar wo = b.let("wo", load(im, v(o)));
        LocalVar wsum = b.let("wsum", add(v(wa), v(wo)));
        b.when(lt(v(d2), mul(v(reach), v(reach))), near ->
                near.when(gt(v(d2), f(COINCIDENT)), apart ->
                        apart.when(gt(v(wsum), f(0)), touching -> {
                            LocalVar dist = touching.let("dist", sqrt(v(d2)));
                            // The relaxed overlap per unit of the separation vector and of inverse mass.
                            LocalVar k = touching.let("k", div(mul(v(omega), sub(v(reach), v(dist))),
                                    mul(v(dist), v(wsum))));
                            for (int axis = 0; axis < 3; axis++) {
                                LocalVar step = touching.let("step", mul(v(k), v(d[axis])));
                                LocalVar ma = touching.let("ma", mul(v(step), v(wa)));
                                LocalVar mo = touching.let("mo", neg(mul(v(step), v(wo))));
                                touching.store(at[axis], v(a), add(load(at[axis], v(a)), v(ma)));
                                touching.store(at[axis], v(o), add(load(at[axis], v(o)), v(mo)));
                                touching.store(displaced[axis], v(a), add(load(displaced[axis], v(a)), v(ma)));
                                touching.store(displaced[axis], v(o), add(load(displaced[axis], v(o)), v(mo)));
                            }
                        })));
    }

    // --- Gauss–Seidel over a contact list ------------------------------------------------------------------

    /**
     * The most spheres a contact list can order: a contact's priority is its pair {@code a · n + o} scrambled, and
     * that must fit in 31 bits.
     */
    public static final int MOST_LISTED_SPHERES = 46_340;

    /**
     * How near two spheres are listed as a contact, past touching, as a share of the sum of their radii. A pair
     * that only comes to touch as the substep's passes move the spheres is otherwise missed until the next
     * substep, and the overlap it has gathered by then is taken in one move, which the velocity feels. 2% was
     * measured to be enough for a column at rest; more only lengthens the list.
     */
    public static final float LIST_MARGIN = 0.02f;

    /** Words of {@code contactCount}: the list's length this substep, and two readings for the sweep. */
    public static final int CONTACT_COUNT_WORDS = 3;
    /** The contacts listed this substep; past the capacity, the ones that did not fit. */
    public static final int LISTED = 0;
    /** Contacts left unsolved when a pass's rounds ran out, added up since the start. */
    public static final int MISSED = 1;
    /** The longest list any substep has made, to set the capacity by. */
    public static final int LONGEST = 2;

    /**
     * Words of {@code open}: two counts of the contacts a round left open, taken in turn ({@link #contactRound}'s
     * {@code round} says which), so one is counted into while the other is cleared for the next.
     */
    public static final int OPEN_WORDS = 2;

    /** Words of {@code readout}, the few the host reads after every batch of rounds: {@link #report}. */
    public static final int READOUT_WORDS = 2;
    /** The contacts the batch's last round left open: zero, and the pass is done. */
    public static final int READ_OPEN = 0;
    /** The list's length this substep; past the capacity, contacts that did not fit and are not solved. */
    public static final int READ_LISTED = 1;

    /** The claim arrays a contact list's rounds take in turn: one claimed into, one checked, one cleared. */
    public static final String[] CLAIMS = {"claims0", "claims1", "claims2"};

    static final String[] CLEAR_NAMES = {CLAIMS[0], CLAIMS[1], CLAIMS[2], "contactCount"};
    static final List<Buffer> CLEAR_BUFFERS = bind(CLEAR_NAMES, 0);

    /**
     * One invocation per sphere, before the list is made: every claim cleared, and the list emptied, its length
     * kept in {@link #LONGEST} if it is the longest yet.
     */
    public static Function clearContacts() {
        List<Buffer> bs = CLEAR_BUFFERS;
        Buffer count = bs.get(3);
        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            for (int k = 0; k < CLAIMS.length; k++) {
                t.store(bs.get(k), v(s), i(0));
            }
            t.when(eq(v(s), i(0)), first -> {
                first.atomic(AtomicOp.MAX, count, i(LONGEST), load(count, i(LISTED)));
                first.store(count, i(LISTED), i(0));
            });
        });
        return function("spheresClearContacts", b);
    }

    static final String[] LIST_NAMES = {"x", "y", "z", "r", "keys", "starts", "order", "contactA", "contactB",
            "contactDone", "contactCount"};
    static final List<Buffer> LIST_BUFFERS = bind(LIST_NAMES, 4);

    /**
     * One invocation per sphere, after the sort: every sphere within {@link #LIST_MARGIN} of touching it, with a
     * higher index, so each pair once, appended to the list by an atomic count. A pair that would not fit in
     * {@code capacity} is counted and left out. Which contacts a substep solves is fixed here, from the positions as
     * predicted.
     */
    public static Function listContacts(SphereGrid grid, int capacity) {
        List<Buffer> bs = LIST_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer radius = bs.get(3);
        Buffer keys = bs.get(4);
        Buffer count = bs.get(10);
        int nx = grid.nx();
        int ny = grid.ny();

        Body b = new Body();
        LocalVar a = b.let("a", new Expr.InvocationId());
        b.when(lt(v(a), new Expr.InvocationCount()), t -> {
            LocalVar key = t.let("key", load(keys, v(a)));
            LocalVar ix = t.let("ix", mod(v(key), i(nx)));
            LocalVar iy = t.let("iy", mod(div(v(key), i(nx)), i(ny)));
            LocalVar iz = t.let("iz", div(v(key), i(nx * ny)));
            LocalVar[] mine = {t.let("xa", load(at[0], v(a))), t.let("ya", load(at[1], v(a))),
                    t.let("za", load(at[2], v(a)))};
            LocalVar ra = t.let("ra", load(radius, v(a)));
            around(t, grid, ix, iy, iz, bs.get(5), bs.get(6), (run, o) -> run.when(gt(v(o), v(a)), later -> {
                LocalVar[] d = new LocalVar[3];
                for (int axis = 0; axis < 3; axis++) {
                    d[axis] = later.let("d", sub(v(mine[axis]), load(at[axis], v(o))));
                }
                LocalVar d2 = later.let("d2", add(mul(v(d[0]), v(d[0])),
                        add(mul(v(d[1]), v(d[1])), mul(v(d[2]), v(d[2])))));
                LocalVar near = later.let("near", mul(add(v(ra), load(radius, v(o))), f(1 + LIST_MARGIN)));
                later.when(lt(v(d2), mul(v(near), v(near))), listed -> {
                    LocalVar slot = listed.fetchAtomic("slot", AtomicOp.ADD, count, i(LISTED), i(1));
                    listed.when(lt(v(slot), i(capacity)), fits -> {
                        fits.store(bs.get(7), v(slot), v(a));
                        fits.store(bs.get(8), v(slot), v(o));
                        fits.store(bs.get(9), v(slot), i(0));
                    });
                });
            }));
        });
        return function("spheresListContacts", b);
    }

    static final String[] ROUND_NAMES = {"x", "y", "z", "r", "im", "dx", "dy", "dz", "params", "contactA",
            "contactB", "contactDone", "contactCount", "checked", "claimed", "cleared", "round", "open"};
    static final List<Buffer> ROUND_BUFFERS = bind(ROUND_NAMES, 9);

    /**
     * Words of a {@code round}: whether it checks, whether it claims, the pass, the dispatch its claims are made
     * with, the {@code open} word it marks, and the dispatch the claims it checks were made with.
     */
    public static final int ROUND_WORDS = 6;

    /**
     * One invocation per place in the list, {@code capacity} of them: one round of solving the contacts in sets no
     * two of which share a sphere, which is what lets a set be solved in parallel, each contact moving both its
     * spheres at once ({@link #contact}).
     *
     * <p>A round has two halves, and a dispatch is the second half of one round and the first of the next:
     *
     * <ol>
     *   <li><b>Check</b>: a contact that holds both its spheres' claims for the last round solves, and is done for
     *       the pass.</li>
     *   <li><b>Claim</b>: every contact not done claims both its spheres, by an atomic max of its priority; the
     *       highest claim holds a sphere, and a contact holding both solves at the next check.</li>
     * </ol>
     *
     * Three claim arrays take turns: a dispatch checks the one the last round claimed into, claims into the next,
     * and clears the third for the round after — each contact clearing its own two spheres, which are the only ones
     * it will claim. So no round's claims meet another's, and a priority needs no room for which round it is.
     *
     * <p>A contact's priority comes from its pair, {@code a · n + o}, and the dispatch ({@link #priority}): one to
     * one within a dispatch, so no two contacts tie; shuffled between dispatches, so a chain of contacts each
     * outranking the next is not solved one a round; and the same in every substep. That last was measured, and
     * matters more than anything else here. Priorities from the contact's place in the list, which the atomic
     * append makes different every substep, left a settled pile of 343 at 3.5e-2 J of moving energy, against 5e-9
     * J from the pair: each substep solved the same contacts in another order, found a slightly different answer,
     * and the velocity felt the difference. Priorities from the pair alone, never shuffled, settle as well but
     * need twice the rounds.
     *
     * <p>A claim left over from an earlier dispatch, in an array no contact has cleared since, can at worst hold a
     * sphere no one then wins this round; it cannot make two contacts hold one sphere, which needs two equal
     * priorities in one dispatch.
     *
     * <p>{@code round} says {@code [checks, claims, pass, dispatch, mark, checks dispatch]}. A contact still open
     * after the check sets {@code open[mark]} to one, and the first invocation clears the other word for the next
     * round, so a pass is done when a round leaves its word at zero ({@link #report}). The dispatch the check
     * compares against is given rather than taken as one less, so that a pass of any length can cycle through a
     * fixed set of rounds. A dispatch that does not claim ends the pass: a contact still not done then is counted
     * in {@link #MISSED}, and waits for the next pass or substep.
     */
    public static Function contactRound(int spheres, int capacity) {
        List<Buffer> bs = ROUND_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] displaced = {bs.get(5), bs.get(6), bs.get(7)};
        Buffer first = bs.get(9);
        Buffer second = bs.get(10);
        Buffer done = bs.get(11);
        Buffer count = bs.get(12);
        Buffer checked = bs.get(13);
        Buffer claimed = bs.get(14);
        Buffer cleared = bs.get(15);
        Buffer round = bs.get(16);
        Buffer marks = bs.get(17);

        Body b = new Body();
        LocalVar c = b.let("c", new Expr.InvocationId());
        LocalVar mark = b.let("mark", load(round, i(4)));
        b.when(eq(v(c), i(0)), clearing -> clearing.store(marks, sub(i(1), v(mark)), i(0)));
        LocalVar listed = b.let("listed", load(count, i(LISTED)));
        b.when(gt(v(listed), i(capacity)), over -> over.set(listed, i(capacity)));
        b.when(lt(v(c), v(listed)), t -> {
            LocalVar pass = t.let("pass", load(round, i(2)));
            t.when(not(eq(load(done, v(c)), v(pass))), open -> {
                LocalVar a = open.let("a", load(first, v(c)));
                LocalVar o = open.let("o", load(second, v(c)));
                LocalVar pair = open.let("pair", add(mul(v(a), i(spheres)), v(o)));
                LocalVar dispatch = open.let("dispatch", load(round, i(3)));
                LocalVar won = open.let("won", i(0));
                open.when(gt(load(round, i(0)), i(0)), checking -> {
                    // What this contact claimed with in the last dispatch.
                    LocalVar last = checking.let("last", priority(v(pair), load(round, i(5))));
                    checking.when(and(eq(load(checked, v(a)), v(last)), eq(load(checked, v(o)), v(last))),
                            held -> held.set(won, i(1)));
                });
                open.when(gt(v(won), i(0)), solving -> {
                    contact(solving, a, o, at, bs.get(3), bs.get(4), displaced,
                            solving.let("omega", load(bs.get(8), i(OMEGA))));
                    solving.store(done, v(c), v(pass));
                });
                open.when(eq(v(won), i(0)), waiting -> {
                    waiting.store(cleared, v(a), i(0));
                    waiting.store(cleared, v(o), i(0));
                    waiting.when(gt(load(round, i(1)), i(0)), claiming -> {
                        claiming.store(marks, v(mark), i(1));
                        LocalVar mine = claiming.let("mine", priority(v(pair), v(dispatch)));
                        claiming.atomic(AtomicOp.MAX, claimed, v(a), v(mine));
                        claiming.atomic(AtomicOp.MAX, claimed, v(o), v(mine));
                    });
                    waiting.when(eq(load(round, i(1)), i(0)),
                            out -> out.atomic(AtomicOp.ADD, count, i(MISSED), i(1)));
                });
            });
        });
        return function("spheresContactRound", b);
    }

    static final String[] REPORT_NAMES = {"open", "contactCount", "round", "readout"};
    static final List<Buffer> REPORT_BUFFERS = bind(REPORT_NAMES, 0);

    /**
     * One invocation, after a batch of {@link #contactRound}s: whether the batch's last round left a contact open,
     * which {@code round} (that round's) says where to read, and the list's length, copied to {@code readout} for
     * the host to read where it is ({@link #READOUT_WORDS}).
     *
     * <p>A pass ends only when this reads zero, and then both words of {@code open} are zero: the last round left its
     * own so, and cleared the other. So the next pass starts from nothing with no clearing of its own.
     */
    public static Function report() {
        List<Buffer> bs = REPORT_BUFFERS;
        Buffer marks = bs.get(0);
        Buffer count = bs.get(1);
        Buffer round = bs.get(2);
        Buffer readout = bs.get(3);
        Body b = new Body();
        LocalVar c = b.let("c", new Expr.InvocationId());
        b.when(eq(v(c), i(0)), first -> {
            first.store(readout, i(READ_OPEN), load(marks, load(round, i(4))));
            first.store(readout, i(READ_LISTED), load(count, i(LISTED)));
        });
        return function("spheresReport", b);
    }

    /**
     * A contact's priority in dispatch {@code g} of a substep: its pair times an odd number, offset by the dispatch,
     * kept to 31 bits. One to one for any one dispatch, so no two contacts tie; the same in every substep; and
     * shuffled from one dispatch to the next.
     */
    private static Expr priority(Expr pair, Expr g) {
        return bitAnd(add(mul(pair, i(0x9E3779B1)), mul(g, i(0x632BE5AB))), i(Integer.MAX_VALUE));
    }

    private static Expr bitAnd(Expr a, Expr b) {
        return new Expr.Binary(BinaryOp.BIT_AND, a, b);
    }

    static final String[] WALLS_NAMES = {"x", "y", "z", "r", "im", "dx", "dy", "dz", "params"};
    static final List<Buffer> WALLS_BUFFERS = bind(WALLS_NAMES);

    /**
     * One invocation per sphere, after a pass of {@link #contactRound}s: the sphere projected back inside the walls,
     * as {@link #apply} does after the Jacobi move, and the push added to what the constraints have moved it.
     */
    public static Function walls() {
        List<Buffer> bs = WALLS_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] displaced = {bs.get(5), bs.get(6), bs.get(7)};
        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> t.when(gt(load(bs.get(4), v(s)), f(0)), free ->
                shift(free, s, new Expr[] {f(0), f(0), f(0)}, at, bs.get(3), displaced, bs.get(8))));
        return function("spheresWalls", b);
    }

    /** Offers {@code solving} every sphere in cell {@code (ix, iy, iz)} and the 26 around it. */
    private static void around(Body t, SphereGrid grid, LocalVar ix, LocalVar iy, LocalVar iz, Buffer starts,
                               Buffer order, Solving solving) {
        around(t, grid, ix, iy, iz, starts, order, solving::against);
    }

    /** Offers {@code visit} every sphere in cell {@code (ix, iy, iz)} and the 26 around it. */
    private static void around(Body t, SphereGrid grid, LocalVar ix, LocalVar iy, LocalVar iz, Buffer starts,
                               Buffer order, BiConsumer<Body, LocalVar> visit) {
        int nx = grid.nx();
        int ny = grid.ny();
        int nz = grid.nz();
        LocalVar n = t.let("n", i(0));
        t.loop(lt(v(n), i(27)), next -> {
            LocalVar jx = next.let("jx", add(v(ix), sub(mod(v(n), i(3)), i(1))));
            LocalVar jy = next.let("jy", add(v(iy), sub(mod(div(v(n), i(3)), i(3)), i(1))));
            LocalVar jz = next.let("jz", add(v(iz), sub(div(v(n), i(9)), i(1))));
            next.when(and(within(jx, nx), and(within(jy, ny), within(jz, nz))), inside -> {
                LocalVar cell = inside.let("cell", add(v(jx), mul(i(nx), add(v(jy), mul(i(ny), v(jz))))));
                LocalVar k = inside.let("k", load(starts, v(cell)));
                LocalVar end = inside.let("end", load(starts, add(v(cell), i(1))));
                inside.loop(lt(v(k), v(end)), run -> {
                    LocalVar o = run.let("o", load(order, v(k)));
                    visit.accept(run, o);
                    run.set(k, add(v(k), i(1)));
                });
            });
            next.set(n, add(v(n), i(1)));
        });
    }

    private static Expr within(LocalVar index, int bound) {
        return and(not(lt(v(index), i(0))), lt(v(index), i(bound)));
    }

    private static void store(Body t, LocalVar s, LocalVar[] move, Buffer... into) {
        for (int a = 0; a < 3; a++) {
            t.store(into[a], v(s), v(move[a]));
        }
    }

    /**
     * One sphere's solve as it goes: what it knows of itself, and the move summed so far. Both Jacobi solves use
     * it, so they differ only in which spheres they offer it.
     */
    private record Solving(LocalVar s, LocalVar[] mine, LocalVar rs, LocalVar ws, LocalVar[] sum, LocalVar contacts,
                           Buffer[] at, Buffer radius, Buffer im, Buffer params) {

        /** {@code x, y, z, r, im} are both solves' first five bindings, and {@code params} its ninth. */
        static Solving begin(Body t, LocalVar s, List<Buffer> bs) {
            Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
            LocalVar[] mine = {t.let("xs", load(at[0], v(s))), t.let("ys", load(at[1], v(s))),
                    t.let("zs", load(at[2], v(s)))};
            LocalVar rs = t.let("rs", load(bs.get(3), v(s)));
            LocalVar ws = t.let("ws", load(bs.get(4), v(s)));
            LocalVar[] sum = {t.let("sx", f(0)), t.let("sy", f(0)), t.let("sz", f(0))};
            LocalVar contacts = t.let("contacts", f(0));
            return new Solving(s, mine, rs, ws, sum, contacts, at, bs.get(3), bs.get(4), bs.get(8));
        }

        /** Sphere {@code o}'s overlap with this one, if any, and this one's share of the move that parts them. */
        void against(Body b, LocalVar o) {
            b.when(not(eq(v(o), v(s))), other -> {
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
        }

        /** The summed move, relaxed as the parameters say. */
        LocalVar[] move(Body t) {
            LocalVar omega = t.let("omega", load(params, i(OMEGA)));
            LocalVar averaged = t.let("averaged", load(params, i(AVERAGED)));
            LocalVar scale = t.let("scale", v(omega));
            t.when(gt(v(averaged), f(0)), avg -> avg.set(scale, div(v(omega), max(v(contacts), f(1)))));
            LocalVar[] move = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                move[a] = t.let("move", mul(v(scale), v(sum[a])));
            }
            return move;
        }
    }

    static final String[] BIN_NAMES = {"x", "y", "z", "counts", "keys", "ranks"};
    static final List<Buffer> BIN_BUFFERS = bind(BIN_NAMES, 3);

    /**
     * One invocation per sphere: the cell of {@code grid} its centre is in, counted for the sort
     * ({@link CountingSort#count}). A centre outside the grid is counted in the nearest cell, which is still right:
     * the cells only decide which spheres are tested, never which touch.
     */
    public static Function bin(SphereGrid grid) {
        List<Buffer> bs = BIN_BUFFERS;
        int[] cells = {grid.nx(), grid.ny(), grid.nz()};
        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar[] c = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                c[a] = t.let("c", toInt(max(f(0), min(f(cells[a] - 1),
                        floor(div(load(bs.get(a), v(s)), f(grid.cell())))))));
            }
            Expr key = add(v(c[0]), mul(i(cells[0]), add(v(c[1]), mul(i(cells[1]), v(c[2])))));
            CountingSort.count(t, v(s), key, bs.get(3), bs.get(4), bs.get(5));
        });
        return function("spheresBin", b);
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

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> t.when(gt(load(im, v(s)), f(0)), free -> {
            Expr[] move = {load(correction[0], v(s)), load(correction[1], v(s)), load(correction[2], v(s))};
            shift(free, s, move, at, radius, displaced, params);
        }));
        return function("spheresApply", b);
    }

    /**
     * Sphere {@code s} moved by {@code move}, then projected back inside the walls; and the move, as computed, added
     * to what the constraints have moved it this substep, with the wall's push only where it pushed.
     */
    private static void shift(Body free, LocalVar s, Expr[] move, Buffer[] at, Buffer radius, Buffer[] displaced,
                              Buffer params) {
        int[] extent = {SX, SY, SZ};
        LocalVar r = free.let("r", load(radius, v(s)));
        for (int a = 0; a < 3; a++) {
            LocalVar c = free.let("c", move[a]);
            LocalVar moved = free.let("moved", add(load(at[a], v(s)), v(c)));
            LocalVar high = free.let("high", sub(load(params, i(extent[a])), v(r)));
            LocalVar inside = free.let("inside", max(v(r), min(v(high), v(moved))));
            free.store(at[a], v(s), v(inside));
            // The move as computed, not as the rounded position has it; the wall's push only where it pushed.
            free.store(displaced[a], v(s), add(load(displaced[a], v(s)), add(v(c), sub(v(inside), v(moved)))));
        }
    }

    /**
     * One invocation per sphere: what the constraints moved it by this substep, added as a velocity, and cleared. The
     * velocity as it was before, the one the substep predicted with, is kept in {@code cx, cy, cz}, free by now, for
     * {@link #bounce} to read how fast contacts were closing.
     *
     * <p>Likewise for turning: what the constraints turned the sphere by, {@code tx, ty, tz} as a rotation vector,
     * added to the angular velocity as {@code t / h}, the orientation turned by it, and cleared. The turns are added
     * up as they are computed, for the reason the moves are ({@link Spheres above}), and never read back from two
     * orientations.
     */
    public static Function velocity() {
        List<Buffer> bs = VELOCITY_BUFFERS;
        Buffer[] velocity = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] displaced = {bs.get(3), bs.get(4), bs.get(5)};
        Buffer params = bs.get(6);
        Buffer[] before = {bs.get(7), bs.get(8), bs.get(9)};
        Buffer[] spin = {bs.get(10), bs.get(11), bs.get(12)};
        Buffer[] turned = {bs.get(13), bs.get(14), bs.get(15)};
        Buffer[] q = {bs.get(16), bs.get(17), bs.get(18), bs.get(19)};

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar perH = t.let("perH", div(f(1), load(params, i(H))));
            for (int a = 0; a < 3; a++) {
                LocalVar was = t.let("was", load(velocity[a], v(s)));
                t.store(before[a], v(s), v(was));
                t.store(velocity[a], v(s), add(v(was), mul(load(displaced[a], v(s)), v(perH))));
                t.store(displaced[a], v(s), f(0));
            }
            Expr[] angle = new Expr[3];
            for (int a = 0; a < 3; a++) {
                LocalVar by = t.let("by", load(turned[a], v(s)));
                t.store(spin[a], v(s), add(load(spin[a], v(s)), mul(v(by), v(perH))));
                t.store(turned[a], v(s), f(0));
                angle[a] = v(by);
            }
            turn(t, s, q, angle);
        });
        return function("spheresVelocity", b);
    }

    /**
     * Sphere {@code s}'s orientation {@code q}, a unit quaternion {@code (x, y, z, w)}, turned by the small rotation
     * {@code angle}, a rotation vector in radians: {@code q + ½ (angle, 0) ⊗ q}, normalised. First order, as XPBD
     * integrates it; a turn of θ comes out as {@code 2 atan(θ / 2)}, short by about {@code θ³ / 12}. A quaternion of
     * zero, which an orientation never written would be, stays zero rather than becoming a number that is not one.
     */
    private static void turn(Body t, LocalVar s, Buffer[] q, Expr[] angle) {
        LocalVar[] a = new LocalVar[3];
        for (int k = 0; k < 3; k++) {
            a[k] = t.let("half", mul(f(0.5f), angle[k]));
        }
        LocalVar[] was = new LocalVar[4];
        for (int k = 0; k < 4; k++) {
            was[k] = t.let("q", load(q[k], v(s)));
        }
        // (a, 0) ⊗ (v, w) = (w a + a × v, −a · v)
        LocalVar[] now = new LocalVar[4];
        for (int k = 0; k < 3; k++) {
            int k1 = (k + 1) % 3;
            int k2 = (k + 2) % 3;
            now[k] = t.let("turned", add(v(was[k]), add(mul(v(a[k]), v(was[3])),
                    sub(mul(v(a[k1]), v(was[k2])), mul(v(a[k2]), v(was[k1]))))));
        }
        now[3] = t.let("turned", sub(v(was[3]), add(mul(v(a[0]), v(was[0])),
                add(mul(v(a[1]), v(was[1])), mul(v(a[2]), v(was[2]))))));
        LocalVar len2 = t.let("len2", add(add(mul(v(now[0]), v(now[0])), mul(v(now[1]), v(now[1]))),
                add(mul(v(now[2]), v(now[2])), mul(v(now[3]), v(now[3])))));
        LocalVar inverse = t.let("inverse", div(f(1), sqrt(max(v(len2), f(1e-30f)))));
        for (int k = 0; k < 4; k++) {
            t.store(q[k], v(s), mul(v(now[k]), v(inverse)));
        }
    }

    static final String[] BOUNCE_NAMES = {"x", "y", "z", "r", "im", "u", "v", "w", "cx", "cy", "cz", "dx", "dy", "dz",
            "params"};
    static final List<Buffer> BOUNCE_BUFFERS = bind(BOUNCE_NAMES);
    static final String[] GRID_BOUNCE_NAMES = {"x", "y", "z", "r", "im", "u", "v", "w", "cx", "cy", "cz", "dx", "dy",
            "dz", "params", "keys", "starts", "order"};
    static final List<Buffer> GRID_BOUNCE_BUFFERS = bind(GRID_BOUNCE_NAMES, 15);

    /** How near a sphere is taken to touch another or a wall, for {@link #bounce}: the contact list's margin. */
    private static final float TOUCH_MARGIN = LIST_MARGIN;

    /**
     * One invocation per sphere, after {@link #velocity}: restitution, the change of velocity that makes each contact
     * part at {@code e} times the speed it was closing at, {@code e} being {@link #params}' {@code restitution}. Into
     * {@code dx, dy, dz}, which {@link #velocity} has cleared, for {@link #bounced} to add; every sphere reads the
     * velocities as {@link #velocity} left them, so each pair's two changes are equal and opposite by mass, and
     * momentum is kept.
     *
     * <p>XPBD's own velocity pass (Müller et al., 2020). For a contact touching after the substep's solve, with normal
     * {@code n} from the other sphere to this one: {@code v̄ₙ} is how fast the two were closing before the solve, from
     * the velocities {@link #velocity} kept, and {@code vₙ} how fast they part after it. Where they were closing faster
     * than {@code 2 |g| h}, the parting speed is set to {@code −e v̄ₙ}, each sphere taking its share of the change by
     * inverse mass. Slower than that is a resting contact, and is left alone: a stack would otherwise jitter on the
     * speed gravity gives it in one substep. A wall is a contact with a sphere of no inverse mass.
     *
     * <p>With {@code e} zero this does nothing at all, so a solve without restitution is the solve it always was. A
     * sphere touching several others takes the sum of every contact's change, which, like Jacobi's moves, can
     * overshoot where many contacts close at once.
     */
    public static Function bounce() {
        return bounce(null);
    }

    /** {@link #bounce()}, testing only the spheres in {@code grid}'s cells around this one, as {@link #solve(SphereGrid)} does. */
    public static Function bounce(SphereGrid grid) {
        List<Buffer> bs = grid == null ? BOUNCE_BUFFERS : GRID_BOUNCE_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer radius = bs.get(3);
        Buffer im = bs.get(4);
        Buffer[] velocity = {bs.get(5), bs.get(6), bs.get(7)};
        Buffer[] before = {bs.get(8), bs.get(9), bs.get(10)};
        Buffer[] change = {bs.get(11), bs.get(12), bs.get(13)};
        Buffer params = bs.get(14);
        int[] extent = {SX, SY, SZ};

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        LocalVar count = b.let("count", new Expr.InvocationCount());
        b.when(lt(v(s), v(count)), t -> {
            LocalVar e = t.let("e", load(params, i(RESTITUTION)));
            LocalVar wa = t.let("wa", load(im, v(s)));
            t.when(and(gt(v(e), f(0)), gt(v(wa), f(0))), on -> {
                LocalVar h = on.let("h", load(params, i(H)));
                LocalVar gx = on.let("gx", load(params, i(GX)));
                LocalVar gy = on.let("gy", load(params, i(GY)));
                LocalVar gz = on.let("gz", load(params, i(GZ)));
                LocalVar resting = on.let("resting", mul(mul(f(2), v(h)),
                        sqrt(add(mul(v(gx), v(gx)), add(mul(v(gy), v(gy)), mul(v(gz), v(gz)))))));
                LocalVar ra = on.let("ra", load(radius, v(s)));
                LocalVar[] mine = new LocalVar[3];
                LocalVar[] vel = new LocalVar[3];
                LocalVar[] was = new LocalVar[3];
                LocalVar[] sum = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    mine[a] = on.let("p", load(at[a], v(s)));
                    vel[a] = on.let("vel", load(velocity[a], v(s)));
                    was[a] = on.let("was", load(before[a], v(s)));
                    sum[a] = on.let("sum", f(0));
                }
                java.util.function.BiConsumer<Body, LocalVar> against = (run, o) -> run.when(
                        not(eq(v(o), v(s))), other -> {
                            LocalVar[] d = new LocalVar[3];
                            for (int a = 0; a < 3; a++) {
                                d[a] = other.let("d", sub(v(mine[a]), load(at[a], v(o))));
                            }
                            LocalVar d2 = other.let("d2", add(mul(v(d[0]), v(d[0])),
                                    add(mul(v(d[1]), v(d[1])), mul(v(d[2]), v(d[2])))));
                            LocalVar near = other.let("near", mul(add(v(ra), load(radius, v(o))),
                                    f(1 + TOUCH_MARGIN)));
                            LocalVar wsum = other.let("wsum", add(v(wa), load(im, v(o))));
                            other.when(and(lt(v(d2), mul(v(near), v(near))), gt(v(d2), f(COINCIDENT))),
                                    touching -> {
                                        LocalVar dist = touching.let("dist", sqrt(v(d2)));
                                        LocalVar closing = touching.let("closing", f(0));
                                        LocalVar parting = touching.let("parting", f(0));
                                        for (int a = 0; a < 3; a++) {
                                            LocalVar n = touching.let("n", div(v(d[a]), v(dist)));
                                            touching.set(closing, add(v(closing), mul(v(n),
                                                    sub(v(was[a]), load(before[a], v(o))))));
                                            touching.set(parting, add(v(parting), mul(v(n),
                                                    sub(v(vel[a]), load(velocity[a], v(o))))));
                                        }
                                        touching.when(lt(v(closing), neg(v(resting))), bouncing -> {
                                            LocalVar k = bouncing.let("k", div(mul(sub(neg(mul(v(e),
                                                    v(closing))), v(parting)), v(wa)), mul(v(wsum), v(dist))));
                                            for (int a = 0; a < 3; a++) {
                                                bouncing.set(sum[a], add(v(sum[a]), mul(v(k), v(d[a]))));
                                            }
                                        });
                                    });
                        });
                if (grid == null) {
                    LocalVar o = on.let("o", i(0));
                    on.loop(lt(v(o), v(count)), pass -> {
                        against.accept(pass, o);
                        pass.set(o, add(v(o), i(1)));
                    });
                } else {
                    int nx = grid.nx();
                    int ny = grid.ny();
                    LocalVar key = on.let("key", load(bs.get(15), v(s)));
                    LocalVar ix = on.let("ix", mod(v(key), i(nx)));
                    LocalVar iy = on.let("iy", mod(div(v(key), i(nx)), i(ny)));
                    LocalVar iz = on.let("iz", div(v(key), i(nx * ny)));
                    around(on, grid, ix, iy, iz, bs.get(16), bs.get(17), against);
                }
                // The walls: each a contact with no inverse mass, so the sphere takes the whole change.
                for (int a = 0; a < 3; a++) {
                    int axis = a;
                    LocalVar reach = on.let("reach", mul(v(ra), f(1 + TOUCH_MARGIN)));
                    on.when(lt(v(mine[a]), v(reach)), low -> low.when(lt(v(was[axis]), neg(v(resting))),
                            closing -> closing.set(sum[axis], add(v(sum[axis]),
                                    sub(neg(mul(v(e), v(was[axis]))), v(vel[axis]))))));
                    on.when(gt(v(mine[a]), sub(load(params, i(extent[a])), v(reach))), high -> high.when(
                            gt(v(was[axis]), v(resting)), closing -> closing.set(sum[axis], add(v(sum[axis]),
                                    sub(neg(mul(v(e), v(was[axis]))), v(vel[axis]))))));
                }
                for (int a = 0; a < 3; a++) {
                    on.store(change[a], v(s), v(sum[a]));
                }
            });
        });
        return function(grid == null ? "spheresBounce" : "spheresGridBounce", b);
    }

    static final String[] BOUNCED_NAMES = {"u", "v", "w", "dx", "dy", "dz"};
    static final List<Buffer> BOUNCED_BUFFERS = bind(BOUNCED_NAMES);

    /** One invocation per sphere: the change {@link #bounce} made, added to the velocity, and cleared. */
    public static Function bounced() {
        List<Buffer> bs = BOUNCED_BUFFERS;
        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            for (int a = 0; a < 3; a++) {
                t.store(bs.get(a), v(s), add(load(bs.get(a), v(s)), load(bs.get(3 + a), v(s))));
                t.store(bs.get(3 + a), v(s), f(0));
            }
        });
        return function("spheresBounced", b);
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

    /** {@code shown}, and the slot of a ring a finished step is kept in, for {@link #keep}. */
    public static final String[] KEEP_NAMES = {"shown", "slot"};
    public static final List<Buffer> KEEP_BUFFERS = bind(KEEP_NAMES);

    /**
     * One invocation per word of {@code shown}: the whole of it copied into {@code slot}, where a picture can read a
     * finished step while the next one writes {@code shown} again.
     */
    public static Function keep() {
        List<Buffer> bs = KEEP_BUFFERS;
        Body b = new Body();
        LocalVar k = b.let("k", new Expr.InvocationId());
        b.when(lt(v(k), new Expr.InvocationCount()), t -> t.store(bs.get(1), v(k), load(bs.get(0), v(k))));
        return function("spheresKeep", b);
    }

    // --- building blocks -----------------------------------------------------------------------------------

    private static List<Buffer> bind(String... names) {
        return bind(names, names.length);
    }

    /** The first {@code floats} names bound as f32, the rest as i32. */
    private static List<Buffer> bind(String[] names, int floats) {
        List<Buffer> buffers = new ArrayList<>();
        for (int k = 0; k < names.length; k++) {
            buffers.add(new Buffer(names[k], k, k < floats ? F32 : I32));
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

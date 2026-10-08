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
 * Two passes that solve contact after contact, each against the positions the ones before it left, moving both
 * spheres of a contact at once; then {@link #walls}. A stack hears its own weight within one pass, which Jacobi
 * cannot.
 *
 * <ul>
 *   <li>{@link #contactRound}: over a list of contacts made each substep ({@link #listContacts}), in rounds that
 *       each solve a set of contacts sharing no sphere, one invocation a contact. The one to use.</li>
 *   <li>{@link #solveContacts}: in 27 colours of the grid's cells, one serial invocation a cell. As right, and
 *       five times the cost; kept as a measured record.</li>
 * </ul>
 *
 * Both are measured in {@code docs/TODO.md}, with what they cost.
 *
 * <h2>What it is not, yet</h2>
 *
 * <ul>
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
    public static final int WORKGROUP = CountingSort.BLOCK;

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

    static final String[] CONTACT_NAMES = {"x", "y", "z", "r", "im", "dx", "dy", "dz", "params", "starts", "order"};
    static final List<Buffer> CONTACT_BUFFERS = bind(CONTACT_NAMES, 9);

    /** How many colours {@link #solveContacts} takes: each of a cell's indices, modulo three. */
    public static final int COLOURS = 27;

    /**
     * Gauss–Seidel by contact: one pass of the 27 solves the contacts owned by the cells of one colour, a colour
     * being each of a cell's indices modulo three, and moves both spheres of each contact at once, by their shares
     * of its overlap by inverse mass.
     *
     * <p>A cell owns the contacts between its own spheres, and those between its spheres and the spheres of the 13
     * cells around it that come after it — so every contact has one owner. A cell writes only its own spheres and
     * those of its neighbours, and two cells of a colour are three cells apart, so what one reads and writes the
     * other never touches: each colour is parallel and needs no atomics. A correction made by one colour is what the
     * next colour sees, within the same pass; which is what Jacobi lacks, and why a stack hears its own weight late
     * under it.
     *
     * <p>One invocation per cell of the colour, {@link #cellsOf} of them, which works through its contacts one after
     * another, each against the positions as the ones before it left them. Both spheres of a contact move together,
     * so the contact's moves are equal and opposite by mass, and are added up as computed as the Jacobi moves are.
     * Moving one sphere a turn instead, each taking its share of the overlap it saw, was measured and is wrong: the
     * second of a pair sees an overlap the first has shrunk, and a collision of 1 kg into 3 kg gained a quarter of
     * its momentum.
     *
     * <p>Every contact takes {@code ω} of its correction; the {@link #AVERAGED} parameter means nothing here, since
     * no contact is solved twice from one state. The walls are not here: {@link #walls} follows the colours.
     */
    public static Function solveContacts(SphereGrid grid, int colour) {
        List<Buffer> bs = CONTACT_BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer radius = bs.get(3);
        Buffer im = bs.get(4);
        Buffer[] displaced = {bs.get(5), bs.get(6), bs.get(7)};
        Buffer params = bs.get(8);
        Buffer starts = bs.get(9);
        Buffer order = bs.get(10);
        int nx = grid.nx();
        int ny = grid.ny();
        int nz = grid.nz();
        int[] rest = {colour % 3, colour / 3 % 3, colour / 9};
        int hx = third(nx, rest[0]);
        int hy = third(ny, rest[1]);

        Body b = new Body();
        LocalVar id = b.let("id", new Expr.InvocationId());
        b.when(lt(v(id), new Expr.InvocationCount()), t -> {
            LocalVar ix = t.let("ix", add(mul(mod(v(id), i(hx)), i(3)), i(rest[0])));
            LocalVar iy = t.let("iy", add(mul(mod(div(v(id), i(hx)), i(hy)), i(3)), i(rest[1])));
            LocalVar iz = t.let("iz", add(mul(div(v(id), i(hx * hy)), i(3)), i(rest[2])));
            LocalVar own = t.let("own", add(v(ix), mul(i(nx), add(v(iy), mul(i(ny), v(iz))))));
            LocalVar omega = t.let("omega", load(params, i(OMEGA)));
            LocalVar ka = t.let("ka", load(starts, v(own)));
            LocalVar endA = t.let("endA", load(starts, add(v(own), i(1))));
            t.loop(lt(v(ka), v(endA)), first -> {
                LocalVar a = first.let("a", load(order, v(ka)));
                // The own cell is the middle of the 27, and the 13 after it are the cells it owns contacts with.
                LocalVar n = first.let("n", i(13));
                first.loop(lt(v(n), i(27)), next -> {
                    LocalVar jx = next.let("jx", add(v(ix), sub(mod(v(n), i(3)), i(1))));
                    LocalVar jy = next.let("jy", add(v(iy), sub(mod(div(v(n), i(3)), i(3)), i(1))));
                    LocalVar jz = next.let("jz", add(v(iz), sub(div(v(n), i(9)), i(1))));
                    next.when(and(within(jx, nx), and(within(jy, ny), within(jz, nz))), inside -> {
                        LocalVar cell = inside.let("cell", add(v(jx), mul(i(nx), add(v(jy), mul(i(ny), v(jz))))));
                        LocalVar kb = inside.let("kb", load(starts, v(cell)));
                        // Within the own cell, each pair once: only the spheres after this one.
                        inside.when(eq(v(n), i(13)), same -> same.set(kb, add(v(ka), i(1))));
                        LocalVar endB = inside.let("endB", load(starts, add(v(cell), i(1))));
                        inside.loop(lt(v(kb), v(endB)), second -> {
                            LocalVar o = second.let("o", load(order, v(kb)));
                            contact(second, a, o, at, radius, im, displaced, omega);
                            second.set(kb, add(v(kb), i(1)));
                        });
                    });
                    next.set(n, add(v(n), i(1)));
                });
                first.set(ka, add(v(ka), i(1)));
            });
        });
        return function("spheresSolveContacts" + colour, b);
    }

    /**
     * Spheres {@code a} and {@code o} parted, if they overlap: each moved along the line between them by its share
     * of {@code ω} times the overlap, by inverse mass, and each move added to what the constraints have moved it.
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

    /** The cells of {@code grid} that {@link #solveContacts} visits for {@code colour}: zero where it has none. */
    public static int cellsOf(SphereGrid grid, int colour) {
        return third(grid.nx(), colour % 3) * third(grid.ny(), colour / 3 % 3) * third(grid.nz(), colour / 9);
    }

    /** How many of the indices {@code [0, n)} are {@code rest} modulo three. */
    private static int third(int n, int rest) {
        return Math.max(0, (n - rest + 2) / 3);
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
            "contactB", "contactDone", "contactCount", "checked", "claimed", "cleared", "round"};
    static final List<Buffer> ROUND_BUFFERS = bind(ROUND_NAMES, 9);

    /** Words of a {@code round}: whether it checks, whether it claims, the pass, and the dispatch in the substep. */
    public static final int ROUND_WORDS = 4;

    /**
     * One invocation per place in the list, {@code capacity} of them: one round of solving the contacts in sets no
     * two of which share a sphere, which is what lets a set be solved in parallel, each contact moving both its
     * spheres as {@link #solveContacts} does.
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
     * <p>{@code round} says {@code [checks, claims, pass, dispatch]}. The last dispatch of a pass checks and does
     * not claim; a contact still not done then is counted in {@link #MISSED}, and waits for the next pass or
     * substep.
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

        Body b = new Body();
        LocalVar c = b.let("c", new Expr.InvocationId());
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
                    LocalVar last = checking.let("last", priority(v(pair), sub(v(dispatch), i(1))));
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
     * One invocation per sphere, after {@link #solveContacts}' colours: the sphere projected back inside the walls,
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

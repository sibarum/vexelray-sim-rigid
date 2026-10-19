package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.Buffer;
import dev.supirvast.vastir.core.Expr;
import dev.supirvast.vastir.core.Function;
import dev.supirvast.vastir.core.LocalVar;
import dev.supirvast.vastir.pass.CountingSort;
import dev.supirvast.vastir.type.Type;
import dev.vexelray.sim.rigid.sphere.SphereGrid;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import static dev.supirvast.vastir.build.Body.F32;
import static dev.supirvast.vastir.build.Body.I32;
import static dev.supirvast.vastir.build.Body.abs;
import static dev.supirvast.vastir.build.Body.add;
import static dev.supirvast.vastir.build.Body.and;
import static dev.supirvast.vastir.build.Body.div;
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
import static dev.supirvast.vastir.build.Body.not;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.toInt;
import static dev.supirvast.vastir.build.Body.v;

/**
 * Boxes' broad phase, stage 6 of {@code docs/boxes.md}: which pairs of boxes are near enough to need the narrow phase.
 *
 * <p>The boxes are sorted into a grid by their centres, as the spheres are, by SupirVast's counting sort
 * ({@link #bin}). A cell is as wide as the widest box's bounding sphere plus the margin, so two boxes that can touch
 * have their centres in the same cell or neighbouring ones. Each box then tests the boxes of its own cell and the 26
 * around it whose index is higher than its own ({@link #list}): a pair is listed if the two boxes' bounding boxes in
 * the world's frame come within the margin of each other on every axis.
 *
 * <h2>Slots</h2>
 *
 * Each box has {@link #SLOTS} slots in the pair list, {@code pairs}, and fills them with its partners in ascending
 * order, the rest −1. A pair's slot is then {@code first · SLOTS + m}, and from one step to the next the same pair is
 * found by looking through its first box's slots for the same partner ({@link BoxPairs#carry}), which is what lets
 * its points keep their impulses. A box with more partners than slots keeps the first it finds and leaves the rest
 * for the step; how many each found is in {@code contacts}, for the host to see.
 *
 * <p>{@code contacts}, the one buffer the host reads a step ({@link #counts}): for each slot, its partner where the
 * pair touches and −1 where it does not; then each slot's number of points; then each box's number of partners found.
 */
public final class BoxBroad {

    /** The places in the pair list each box has for partners of higher index. */
    public static final int SLOTS = 32;

    private BoxBroad() {
    }

    /** The grid over the walled box, its cells as wide as the widest box's bounding sphere plus the margin. */
    public static SphereGrid grid(double sx, double sy, double sz, double widestHalfDiagonal, double margin) {
        return SphereGrid.covering(sx, sy, sz, 2 * widestHalfDiagonal + margin);
    }

    static final String[] BIN_NAMES = {"x", "y", "z", "counts", "keys", "ranks"};
    static final List<Buffer> BIN_BUFFERS = bind(BIN_NAMES, "x", "y", "z");

    /** One invocation per box: counted into the cell its centre is in, for the sort. */
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
        return function("boxesBin", b);
    }

    static final String[] LIST_NAMES = {"x", "y", "z", "qx", "qy", "qz", "qw", "hx", "hy", "hz", "params", "keys",
            "starts", "order", "pairs", "contacts", "shape"};
    static final List<Buffer> LIST_BUFFERS = bind(LIST_NAMES, "x", "y", "z", "qx", "qy", "qz", "qw", "hx", "hy", "hz",
            "params", "shape");

    /**
     * One invocation per box: its slots in {@code pairs} filled with the boxes of higher index near it, in ascending
     * order, as {@link BoxBroad above}; and how many it found written to {@code contacts}, after the {@code 2 · slots}
     * words {@link #counts} writes.
     *
     * @param slots every box's slots together, boxes times {@link #SLOTS}
     */
    public static Function list(SphereGrid grid, int slots) {
        List<Buffer> bs = LIST_BUFFERS;
        Buffer params = bs.get(10);
        Buffer keys = bs.get(11);
        Buffer starts = bs.get(12);
        Buffer order = bs.get(13);
        Buffer pairs = bs.get(14);
        Buffer contacts = bs.get(15);

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar base = t.let("base", mul(v(s), i(SLOTS)));
            LocalVar m = t.let("m", i(0));
            t.loop(lt(v(m), i(SLOTS)), clear -> {
                LocalVar at = clear.let("at", mul(add(v(base), v(m)), i(2)));
                clear.store(pairs, v(at), v(s));
                clear.store(pairs, add(v(at), i(1)), i(-1));
                clear.set(m, add(v(m), i(1)));
            });
            LocalVar margin = t.let("margin", load(params, i(Boxes.MARGIN)));
            LocalVar[] centre = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                centre[a] = t.let("c", load(bs.get(a), v(s)));
            }
            LocalVar[] reach = reach(t, bs, s);
            LocalVar found = t.let("found", i(0));
            LocalVar key = t.let("key", load(keys, v(s)));
            LocalVar ix = t.let("ix", mod(v(key), i(grid.nx())));
            LocalVar iy = t.let("iy", mod(div(v(key), i(grid.nx())), i(grid.ny())));
            LocalVar iz = t.let("iz", div(v(key), i(grid.nx() * grid.ny())));
            around(t, grid, ix, iy, iz, starts, order, (near, o) -> near.when(gt(v(o), v(s)), higher -> {
                LocalVar[] other = reach(higher, bs, o);
                Expr close = null;
                for (int a = 0; a < 3; a++) {
                    Expr axis = not(gt(abs(sub(load(bs.get(a), v(o)), v(centre[a]))),
                            add(add(v(reach[a]), v(other[a])), v(margin))));
                    close = close == null ? axis : and(close, axis);
                }
                higher.when(close, listed -> {
                    listed.when(lt(v(found), i(SLOTS)), room -> {
                        // Insertion into the box's slots, kept in ascending order of partner.
                        LocalVar place = room.let("place", v(found));
                        LocalVar moving = room.let("moving", i(1));
                        room.loop(and(gt(v(place), i(0)), gt(v(moving), i(0))), shift -> {
                            LocalVar before = shift.let("before", load(pairs, add(mul(add(v(base),
                                    sub(v(place), i(1))), i(2)), i(1))));
                            shift.when(gt(v(before), v(o)), up -> {
                                up.store(pairs, add(mul(add(v(base), v(place)), i(2)), i(1)), v(before));
                                up.set(place, sub(v(place), i(1)));
                            });
                            shift.when(not(gt(v(before), v(o))), stop -> stop.set(moving, i(0)));
                        });
                        room.store(pairs, add(mul(add(v(base), v(place)), i(2)), i(1)), v(o));
                    });
                    listed.set(found, add(v(found), i(1)));
                });
            }));
            t.store(contacts, add(i(2 * slots), v(s)), v(found));
        });
        return function("boxesList", b);
    }

    static final String[] COUNTS_NAMES = {"pairs", "manifold", "contacts"};
    static final List<Buffer> COUNTS_BUFFERS = bind(COUNTS_NAMES, "manifold");

    /**
     * One invocation per slot, after the collide: in {@code contacts}, the slot's partner where its pair touches and
     * −1 where it does not, and after {@code slots} words, its number of points; what the host reads to make the
     * rounds.
     */
    public static Function counts(int slots) {
        List<Buffer> bs = COUNTS_BUFFERS;
        Body b = new Body();
        LocalVar p = b.let("p", new Expr.InvocationId());
        b.when(lt(v(p), new Expr.InvocationCount()), t -> {
            LocalVar partner = t.let("partner", load(bs.get(0), add(mul(v(p), i(2)), i(1))));
            LocalVar count = t.let("count", toInt(load(bs.get(1), add(mul(v(p), i(BoxContact.MANIFOLD_WORDS)),
                    i(BoxContact.COUNT)))));
            LocalVar touching = t.let("touching", i(-1));
            t.when(gt(v(count), i(0)), some -> some.set(touching, v(partner)));
            t.store(bs.get(2), v(p), v(touching));
            t.store(bs.get(2), add(i(slots), v(p)), v(count));
        });
        return function("boxesCounts", b);
    }

    // --- building blocks -----------------------------------------------------------------------------------

    /**
     * Body {@code s}'s half-extents in the world's frame: the half-size of the box, axis-aligned, that bounds it,
     * {@code Σ_k h_k |R_ak|}; a sphere's radius.
     */
    private static LocalVar[] reach(Body b, List<Buffer> bs, LocalVar s) {
        LocalVar x = b.let("qx", load(bs.get(3), v(s)));
        LocalVar y = b.let("qy", load(bs.get(4), v(s)));
        LocalVar z = b.let("qz", load(bs.get(5), v(s)));
        LocalVar w = b.let("qw", load(bs.get(6), v(s)));
        Expr[][] r = {
                {sub(f(1), mul(f(2), add(mul(v(y), v(y)), mul(v(z), v(z))))),
                        mul(f(2), sub(mul(v(x), v(y)), mul(v(z), v(w)))),
                        mul(f(2), add(mul(v(x), v(z)), mul(v(y), v(w))))},
                {mul(f(2), add(mul(v(x), v(y)), mul(v(z), v(w)))),
                        sub(f(1), mul(f(2), add(mul(v(x), v(x)), mul(v(z), v(z))))),
                        mul(f(2), sub(mul(v(y), v(z)), mul(v(x), v(w))))},
                {mul(f(2), sub(mul(v(x), v(z)), mul(v(y), v(w)))),
                        mul(f(2), add(mul(v(y), v(z)), mul(v(x), v(w)))),
                        sub(f(1), mul(f(2), add(mul(v(x), v(x)), mul(v(y), v(y)))))}};
        LocalVar[] half = new LocalVar[3];
        for (int k = 0; k < 3; k++) {
            half[k] = b.let("h", load(bs.get(7 + k), v(s)));
        }
        LocalVar[] out = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            out[a] = b.let("reach", add(add(mul(v(half[0]), abs(r[a][0])), mul(v(half[1]), abs(r[a][1]))),
                    mul(v(half[2]), abs(r[a][2]))));
        }
        // A sphere's bounds are its radius on every axis, however it is turned.
        b.when(gt(load(bs.get(16), v(s)), f(0.5f)), round -> {
            for (int a = 0; a < 3; a++) {
                round.set(out[a], v(half[0]));
            }
        });
        return out;
    }

    /** Offers {@code visit} every box in cell {@code (ix, iy, iz)} and the 26 around it, as the spheres' grid does. */
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

    /** The named buffers in order, as i32 but for {@code floats}. */
    private static List<Buffer> bind(String[] names, String... floats) {
        List<String> real = List.of(floats);
        List<Buffer> buffers = new ArrayList<>();
        for (int k = 0; k < names.length; k++) {
            buffers.add(new Buffer(names[k], k, real.contains(names[k]) ? F32 : I32));
        }
        return List.copyOf(buffers);
    }

    private static Function function(String name, Body b) {
        return new Function(name, new Type.FunctionType(Type.VOID, List.of()), b.finish());
    }
}

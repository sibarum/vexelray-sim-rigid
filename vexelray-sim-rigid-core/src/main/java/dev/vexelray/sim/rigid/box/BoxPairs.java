package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.Buffer;
import dev.supirvast.vastir.core.Expr;
import dev.supirvast.vastir.core.Function;
import dev.supirvast.vastir.core.LocalVar;
import dev.supirvast.vastir.type.Type;

import java.util.ArrayList;
import java.util.List;

import static dev.supirvast.vastir.build.Body.F32;
import static dev.supirvast.vastir.build.Body.I32;
import static dev.supirvast.vastir.build.Body.add;
import static dev.supirvast.vastir.build.Body.eq;
import static dev.supirvast.vastir.build.Body.f;
import static dev.supirvast.vastir.build.Body.gt;
import static dev.supirvast.vastir.build.Body.i;
import static dev.supirvast.vastir.build.Body.load;
import static dev.supirvast.vastir.build.Body.lt;
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.not;
import static dev.supirvast.vastir.build.Body.or;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.toInt;
import static dev.supirvast.vastir.build.Body.v;

/**
 * Box against box: the solve, stage 5 of {@code docs/boxes.md}. The pairs that touch, as {@link BoxContact#collide}
 * found them once a step, are solved in rounds of pairs that share no box, one invocation a pair, each its whole
 * manifold in turn, each point by {@link Boxes#contact}: the soft step, at the level of velocities, as the walls are.
 *
 * <p>A point is two anchors, one in each box's frame, and the normal is the world's, as the step began. Each sweep
 * measures the point's gap again from where its anchors are now, {@code (pB − pA) · n}, so a point found at the
 * margin, apart, closes only as far as the gap, and a point the boxes have moved apart lets go. A pair with a fixed
 * box is solved with the walls' stiffer softness.
 *
 * <p>Each point keeps its normal impulse and its tangential impulse, the second box's, in {@code impulses}, from
 * substep to substep; and across steps, where {@link #carry} finds it again in the new manifold.
 */
public final class BoxPairs {

    /** Floats per pair in {@code impulses}: each of its points' normal impulse and tangential impulse. */
    public static final int IMPULSE_WORDS = BoxContact.MOST_POINTS * 4;

    public static final String[] NAMES = {"x", "y", "z", "qx", "qy", "qz", "qw", "im", "hx", "hy", "hz", "u", "v", "w",
            "lx", "ly", "lz", "params", "pairs", "manifold", "impulses", "round"};
    public static final List<Buffer> BUFFERS = bindAll(NAMES);

    private BoxPairs() {
    }

    /**
     * One invocation per place in a round: the pair at that place in {@code round}, or none where it holds −1, and
     * one {@code sweep} over its manifold's points; with {@code reverse}, last to first.
     */
    public static Function solve(Boxes.Sweep sweep, boolean reverse) {
        List<Buffer> bs = BUFFERS;
        Buffer[] at = {bs.get(0), bs.get(1), bs.get(2)};
        Buffer[] turn = {bs.get(3), bs.get(4), bs.get(5), bs.get(6)};
        Buffer im = bs.get(7);
        Buffer[] half = {bs.get(8), bs.get(9), bs.get(10)};
        Buffer[] velocity = {bs.get(11), bs.get(12), bs.get(13)};
        Buffer[] spin = {bs.get(14), bs.get(15), bs.get(16)};
        Buffer params = bs.get(17);
        Buffer pairs = bs.get(18);
        Buffer manifold = bs.get(19);
        Buffer impulses = bs.get(20);
        Buffer round = bs.get(21);

        Body b = new Body();
        LocalVar k = b.let("k", new Expr.InvocationId());
        b.when(lt(v(k), new Expr.InvocationCount()), t -> {
            LocalVar pair = t.let("pair", load(round, v(k)));
            t.when(not(lt(v(pair), i(0))), solving -> {
                LocalVar out = solving.let("out", mul(v(pair), i(BoxContact.MANIFOLD_WORDS)));
                LocalVar count = solving.let("count", toInt(load(manifold, add(v(out), i(BoxContact.COUNT)))));
                solving.when(gt(v(count), i(0)), touching -> {
                    LocalVar ia = touching.let("ia", load(pairs, mul(v(pair), i(2))));
                    LocalVar ib = touching.let("ib", load(pairs, add(mul(v(pair), i(2)), i(1))));
                    LocalVar imA = touching.let("imA", load(im, v(ia)));
                    LocalVar imB = touching.let("imB", load(im, v(ib)));
                    Boxes.Moving first = new Boxes.Moving(touching, ia, imA, at, turn, half, velocity, spin);
                    Boxes.Moving second = new Boxes.Moving(touching, ib, imB, at, turn, half, velocity, spin);
                    Boxes.Soft soft = new Boxes.Soft(touching, params, false);
                    soft.stiffen(touching, or(eq(v(imA), f(0)), eq(v(imB), f(0))), params);
                    LocalVar[] n = new LocalVar[3];
                    for (int a = 0; a < 3; a++) {
                        n[a] = touching.let("n", load(manifold, add(v(out), i(BoxContact.NORMAL + a))));
                    }
                    LocalVar held = touching.let("held", mul(v(pair), i(IMPULSE_WORDS)));
                    for (int order = 0; order < BoxContact.MOST_POINTS; order++) {
                        int slot = reverse ? BoxContact.MOST_POINTS - 1 - order : order;
                        touching.when(lt(i(slot), v(count)), present -> {
                            LocalVar where = present.let("where", add(v(out), i(BoxContact.POINTS
                                    + slot * BoxContact.POINT_WORDS)));
                            LocalVar[] rA = Boxes.rotate(present, first.q,
                                    load3(present, manifold, where, BoxContact.ANCHOR_FIRST));
                            LocalVar[] rB = Boxes.rotate(present, second.q,
                                    load3(present, manifold, where, BoxContact.ANCHOR_SECOND));
                            Expr gap = add(add(
                                    mul(sub(add(v(second.at[0]), v(rB[0])), add(v(first.at[0]), v(rA[0]))), v(n[0])),
                                    mul(sub(add(v(second.at[1]), v(rB[1])), add(v(first.at[1]), v(rA[1]))), v(n[1]))),
                                    mul(sub(add(v(second.at[2]), v(rB[2])), add(v(first.at[2]), v(rA[2]))), v(n[2])));
                            LocalVar normalAt = present.let("normalAt", add(v(held), i(4 * slot)));
                            LocalVar tangentAt = present.let("tangentAt", add(v(normalAt), i(1)));
                            Boxes.contact(present, sweep, present.let("gap", gap), n, first, rA, second, rB,
                                    impulses, normalAt, impulses, tangentAt, soft);
                        });
                    }
                    first.store(touching, velocity, spin);
                    second.store(touching, velocity, spin);
                });
            });
        });
        String name = "boxPairs" + sweep.name().charAt(0) + sweep.name().substring(1).toLowerCase();
        return new Function(reverse ? name + "Back" : name, new Type.FunctionType(Type.VOID, List.of()), b.finish());
    }

    public static final String[] CARRY_NAMES = {"pairs", "hx", "hy", "hz", "manifold", "impulses", "manifoldWas",
            "impulsesWas"};
    public static final List<Buffer> CARRY_BUFFERS = bindAll(CARRY_NAMES);

    /** How near a point must be to one of the step before, as a share of the first box's smallest half-extent. */
    public static final float SAME_POINT = 0.05f;

    /**
     * One invocation per pair, before the step's collide: its manifold and impulses copied aside, for
     * {@link #carry} to find them after the collide has written new ones.
     */
    public static Function keep() {
        List<Buffer> bs = CARRY_BUFFERS;
        Body b = new Body();
        LocalVar p = b.let("p", new Expr.InvocationId());
        b.when(lt(v(p), new Expr.InvocationCount()), t -> {
            LocalVar out = t.let("out", mul(v(p), i(BoxContact.MANIFOLD_WORDS)));
            LocalVar held = t.let("held", mul(v(p), i(IMPULSE_WORDS)));
            for (int w = 0; w < BoxContact.MANIFOLD_WORDS; w++) {
                t.store(bs.get(6), add(v(out), i(w)), load(bs.get(4), add(v(out), i(w))));
            }
            for (int w = 0; w < IMPULSE_WORDS; w++) {
                t.store(bs.get(7), add(v(held), i(w)), load(bs.get(5), add(v(held), i(w))));
            }
        });
        return new Function("boxPairsKeep", new Type.FunctionType(Type.VOID, List.of()), b.finish());
    }

    /**
     * One invocation per pair, after the step's collide: each new point takes the impulses of the point of the step
     * before whose anchor on the first box is nearest it, if one is within {@link #SAME_POINT} of that box's smallest
     * half-extent; else it starts from nothing. A box at rest finds the same points step after step, so its load is
     * carried on, rather than lost every step and taken up again.
     */
    public static Function carry() {
        List<Buffer> bs = CARRY_BUFFERS;
        Buffer manifold = bs.get(4);
        Buffer impulses = bs.get(5);
        Buffer was = bs.get(6);
        Buffer impulsesWas = bs.get(7);
        Body b = new Body();
        LocalVar p = b.let("p", new Expr.InvocationId());
        b.when(lt(v(p), new Expr.InvocationCount()), t -> {
            LocalVar out = t.let("out", mul(v(p), i(BoxContact.MANIFOLD_WORDS)));
            LocalVar held = t.let("held", mul(v(p), i(IMPULSE_WORDS)));
            LocalVar count = t.let("count", toInt(load(manifold, add(v(out), i(BoxContact.COUNT)))));
            LocalVar before = t.let("before", toInt(load(was, add(v(out), i(BoxContact.COUNT)))));
            LocalVar ia = t.let("ia", load(bs.get(0), mul(v(p), i(2))));
            LocalVar smallest = t.let("smallest", Body.min(load(bs.get(1), v(ia)), Body.min(load(bs.get(2), v(ia)),
                    load(bs.get(3), v(ia)))));
            LocalVar reach = t.let("reach", mul(mul(f(SAME_POINT), v(smallest)), mul(f(SAME_POINT), v(smallest))));
            for (int point = 0; point < BoxContact.MOST_POINTS; point++) {
                int slot = point;
                LocalVar[] taken = new LocalVar[4];
                for (int w = 0; w < 4; w++) {
                    taken[w] = t.let("taken", f(0));
                }
                t.when(lt(i(slot), v(count)), present -> {
                    LocalVar[] anchor = load3(present, manifold, present.let("where", add(v(out),
                            i(BoxContact.POINTS + slot * BoxContact.POINT_WORDS))), BoxContact.ANCHOR_FIRST);
                    LocalVar nearest = present.let("nearest", v(reach));
                    for (int old = 0; old < BoxContact.MOST_POINTS; old++) {
                        int oldSlot = old;
                        present.when(lt(i(oldSlot), v(before)), earlier -> {
                            LocalVar[] then = load3(earlier, was, earlier.let("whereWas", add(v(out),
                                    i(BoxContact.POINTS + oldSlot * BoxContact.POINT_WORDS))), BoxContact.ANCHOR_FIRST);
                            LocalVar[] gap = new LocalVar[3];
                            for (int a = 0; a < 3; a++) {
                                gap[a] = earlier.let("gap", sub(v(anchor[a]), v(then[a])));
                            }
                            LocalVar d2 = earlier.let("d2", dot(gap, gap));
                            earlier.when(lt(v(d2), v(nearest)), nearer -> {
                                nearer.set(nearest, v(d2));
                                for (int w = 0; w < 4; w++) {
                                    nearer.set(taken[w], load(impulsesWas, add(v(held), i(4 * oldSlot + w))));
                                }
                            });
                        });
                    }
                });
                for (int w = 0; w < 4; w++) {
                    t.store(impulses, add(v(held), i(4 * slot + w)), v(taken[w]));
                }
            }
        });
        return new Function("boxPairsCarry", new Type.FunctionType(Type.VOID, List.of()), b.finish());
    }

    private static LocalVar[] load3(Body b, Buffer buffer, LocalVar base, int offset) {
        return new LocalVar[] {b.let("p", load(buffer, add(v(base), i(offset)))),
                b.let("p", load(buffer, add(v(base), i(offset + 1)))),
                b.let("p", load(buffer, add(v(base), i(offset + 2))))};
    }

    private static Expr dot(LocalVar[] p, LocalVar[] q) {
        return add(add(mul(v(p[0]), v(q[0])), mul(v(p[1]), v(q[1]))), mul(v(p[2]), v(q[2])));
    }

    private static List<Buffer> bindAll(String[] names) {
        List<Buffer> buffers = new ArrayList<>();
        for (int k = 0; k < names.length; k++) {
            boolean ints = names[k].equals("pairs") || names[k].equals("round");
            buffers.add(new Buffer(names[k], k, ints ? I32 : F32));
        }
        return List.copyOf(buffers);
    }
}

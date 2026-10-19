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
import static dev.supirvast.vastir.build.Body.abs;
import static dev.supirvast.vastir.build.Body.add;
import static dev.supirvast.vastir.build.Body.and;
import static dev.supirvast.vastir.build.Body.div;
import static dev.supirvast.vastir.build.Body.f;
import static dev.supirvast.vastir.build.Body.gt;
import static dev.supirvast.vastir.build.Body.i;
import static dev.supirvast.vastir.build.Body.load;
import static dev.supirvast.vastir.build.Body.lt;
import static dev.supirvast.vastir.build.Body.max;
import static dev.supirvast.vastir.build.Body.min;
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.neg;
import static dev.supirvast.vastir.build.Body.not;
import static dev.supirvast.vastir.build.Body.sqrt;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.toFloat;
import static dev.supirvast.vastir.build.Body.v;

/**
 * Box against box: the narrow phase, stage 4 of {@code docs/boxes.md}. One invocation per pair of boxes finds whether
 * they touch, and if so their contact manifold: a normal, and up to four points, each an anchor in each box's own
 * frame and a depth.
 *
 * <h2>Separating axes</h2>
 *
 * Two boxes are apart if some axis parts their shadows. Fifteen are enough: the three face normals of each, and the
 * nine crosses of an edge of one with an edge of the other. Along each the two shadows overlap by some amount, or are
 * parted by some; the pair touches if no axis parts them by more than the margin ({@link Boxes#params}'s
 * {@code margin}), and the contact is along the axis that parts them most, or overlaps them least.
 *
 * <p>Not quite the least. A box resting on a box has a face axis of each that overlaps by the same amount, and edge
 * axes nearly as little, and rounding would pick among them step by step, so a resting pair would flicker between
 * contacts. So a face of the second box wins over the first's only if its separation is more than
 * {@link #RELATIVE} times the first's plus {@link #ABSOLUTE} of the smallest half-extent, and an edge axis over the
 * best face only by the same: Box2D's rule.
 *
 * <h2>A face: the manifold by clipping</h2>
 *
 * The face whose axis won is the reference face; on the other box, the incident face is the one turned most against
 * it. The contact region is where the two faces overlap, seen along the normal, and the vertices of that overlap
 * are of three kinds: the incident face's corners inside the reference face, the reference face's corners inside the
 * incident face, and the crossings of their edges. All are found, each with its depth below the reference face, and
 * those deeper than minus the margin kept, at most {@link #CANDIDATES}, in {@code candidates}, the pair's scratch.
 *
 * <p>Then four are chosen, as Bullet and Box2D reduce a manifold: the deepest; the one farthest from it; and the
 * ones that make the largest triangle with those two, one on each side. Fixed directions would be cheaper and would
 * not need the scratch, but ties among them, which a symmetric overlap makes, could drop a corner of the region.
 *
 * <p>A corner of the region can be found more than once, as an incident corner and as a crossing at its end, say, and
 * the copies differ by rounding. So a third point must stand off the line through the first two by
 * {@link #OFF_LINE} of that line's length: with an absolute least area instead, the GPU kept one copy of a corner of
 * a triangle as a fourth point.
 *
 * <h2>An edge: one point</h2>
 *
 * Where an edge axis won, the contact is the closest points of the two edges, one on each box, the edges being the
 * ones each box puts furthest toward the other along the axis.
 *
 * <h2>What is kept</h2>
 *
 * Per pair, {@link #MANIFOLD_WORDS} floats in {@code manifold}: the normal in the world's frame, from the pair's first
 * box toward its second; the number of points; the axis that won (0–2 a face of the first, 3–5 of the second, 6–14
 * the edge crosses, {@code 6 + 3i + j}); and each point as the anchor on the first box in its own frame, the anchor on
 * the second in its own, and the depth, positive where they overlap.
 */
public final class BoxContact {

    /** Floats per pair in {@code manifold}. */
    public static final int MANIFOLD_WORDS = 40;
    public static final int NORMAL = 0;
    public static final int COUNT = 3;
    public static final int AXIS = 4;
    public static final int POINTS = 8;
    public static final int POINT_WORDS = 8;
    /** Within a point: the anchor on the first box, on the second, and the depth. */
    public static final int ANCHOR_FIRST = 0;
    public static final int ANCHOR_SECOND = 3;
    public static final int DEPTH = 6;
    /** The most points a manifold keeps. */
    public static final int MOST_POINTS = 4;

    /** The most candidate points a face contact considers: 4 corners each way and 16 crossings. */
    public static final int CANDIDATES = 24;
    /** Floats per candidate in {@code candidates}: the point on the incident face, and its depth. */
    public static final int CANDIDATE_WORDS = 4;
    /** Floats per pair in {@code candidates}. */
    public static final int SCRATCH_WORDS = CANDIDATES * CANDIDATE_WORDS;

    /** The face bias: an axis must part the pair by this share more than the one it displaces... */
    public static final float RELATIVE = 0.98f;
    /** ...plus this share of the smallest half-extent of the two boxes. */
    public static final float ABSOLUTE = 0.01f;

    /** Below this squared length an edge cross is taken as two parallel edges, which have no axis between them. */
    private static final float PARALLEL = 1e-10f;
    /** Two candidates nearer than this, or a triangle smaller than this, are taken as one point, or none. */
    private static final float SAME = 1e-12f;
    /**
     * How far past an edge, as a share, a point still counts as inside: two faces the same size, one on the other, meet
     * exactly at their edges, and without it rounding would leave the pair no points at all.
     */
    private static final float SLACK = 1e-5f;
    /**
     * How far off the line through the first two points a third must be, as a share of that line's length, to be a
     * point of its own: a copy of one of the two, which rounding can make, stands off it by about 1e-7 of it.
     */
    private static final float OFF_LINE = 1e-4f;

    public static final String[] NAMES = {"x", "y", "z", "qx", "qy", "qz", "qw", "hx", "hy", "hz", "params", "pairs",
            "manifold", "candidates"};
    public static final List<Buffer> BUFFERS = bind();

    private BoxContact() {
    }

    /**
     * One invocation per pair: the pair's two boxes, from {@code pairs} as {@code (first, second)}, tested along the
     * fifteen axes, and their manifold written to {@code manifold}, as {@link BoxContact above}.
     */
    public static Function collide() {
        List<Buffer> bs = BUFFERS;
        Buffer params = bs.get(10);
        Buffer pairs = bs.get(11);
        Buffer manifold = bs.get(12);
        Buffer candidates = bs.get(13);

        Body b = new Body();
        LocalVar p = b.let("p", new Expr.InvocationId());
        b.when(lt(v(p), new Expr.InvocationCount()), slot -> {
            // An empty slot, its second box −1, has no contact.
            LocalVar partner = slot.let("partner", load(pairs, add(mul(v(p), i(2)), i(1))));
            slot.store(manifold, add(mul(v(p), i(MANIFOLD_WORDS)), i(COUNT)), f(0));
            slot.when(not(lt(v(partner), i(0))), t -> {
            Solid first = new Solid(t, bs, load(pairs, mul(v(p), i(2))));
            Solid second = new Solid(t, bs, v(partner));
            LocalVar margin = t.let("margin", load(params, i(Boxes.MARGIN)));
            LocalVar[] d = sub3(t, second.centre, first.centre);
            LocalVar out = t.let("out", mul(v(p), i(MANIFOLD_WORDS)));
            LocalVar scratch = t.let("scratch", mul(v(p), i(SCRATCH_WORDS)));

            // The faces: the most each box's axes part the pair, and which.
            LocalVar parted = t.let("parted", f(-1e30f));
            LocalVar[] bestFace = {t.let("bestA", f(-1e30f)), t.let("bestB", f(-1e30f))};
            LocalVar[] bestFaceAxis = {t.let("axisA", f(0)), t.let("axisB", f(0))};
            Solid[] boxes = {first, second};
            for (int side = 0; side < 2; side++) {
                Solid own = boxes[side];
                Solid other = boxes[1 - side];
                for (int k = 0; k < 3; k++) {
                    Expr reach = v(own.half[k]);
                    for (int j = 0; j < 3; j++) {
                        reach = add(reach, mul(v(other.half[j]), abs(dot(own.axes[k], other.axes[j]))));
                    }
                    LocalVar sep = t.let("sep", sub(abs(dot(d, own.axes[k])), reach));
                    t.set(parted, max(v(parted), v(sep)));
                    int axis = k;
                    int which = side;
                    t.when(gt(v(sep), v(bestFace[side])), better -> {
                        better.set(bestFace[which], v(sep));
                        better.set(bestFaceAxis[which], f(axis));
                    });
                }
            }

            // The edges: the most an edge cross parts them, and the two edges it was found between.
            LocalVar bestEdge = t.let("bestEdge", f(-1e30f));
            Edge edge = new Edge(t);
            for (int k = 0; k < 3; k++) {
                for (int j = 0; j < 3; j++) {
                    LocalVar[] across = cross(t, first.axes[k], second.axes[j]);
                    LocalVar length2 = t.let("length2", dot(across, across));
                    int ka = k;
                    int jb = j;
                    t.when(gt(v(length2), f(PARALLEL)), crossing -> {
                        LocalVar length = crossing.let("length", sqrt(v(length2)));
                        LocalVar[] axis = scale3(crossing, across, div(f(1), v(length)));
                        Expr reach = f(0);
                        for (int m = 0; m < 3; m++) {
                            reach = add(reach, add(mul(v(first.half[m]), abs(dot(first.axes[m], axis))),
                                    mul(v(second.half[m]), abs(dot(second.axes[m], axis)))));
                        }
                        LocalVar sep = crossing.let("sep", sub(abs(dot(d, axis)), reach));
                        crossing.set(parted, max(v(parted), v(sep)));
                        crossing.when(gt(v(sep), v(bestEdge)), better -> {
                            better.set(bestEdge, v(sep));
                            edge.record(better, first, second, ka, jb, axis, d);
                        });
                    });
                }
            }

            // The face bias: the second box's face over the first's, and an edge over the faces, only by a margin.
            Expr smallest = v(first.half[0]);
            for (Solid box : boxes) {
                for (int k = 0; k < 3; k++) {
                    smallest = min(smallest, v(box.half[k]));
                }
            }
            LocalVar slack = t.let("slack", mul(f(ABSOLUTE), smallest));
            LocalVar fromSecond = t.let("fromSecond", f(0));
            t.when(gt(v(bestFace[1]), add(mul(f(RELATIVE), v(bestFace[0])), v(slack))), second2 ->
                    second2.set(fromSecond, f(1)));
            LocalVar faceSep = t.let("faceSep", v(bestFace[0]));
            t.when(gt(v(fromSecond), f(0.5f)), second2 -> second2.set(faceSep, v(bestFace[1])));
            LocalVar byEdge = t.let("byEdge", f(0));
            t.when(gt(v(bestEdge), add(mul(f(RELATIVE), v(faceSep)), v(slack))), edge2 -> edge2.set(byEdge, f(1)));

            LocalVar count = t.let("count", i(0));
            LocalVar touching = t.let("touching", f(1));
            t.when(gt(v(parted), v(margin)), apart -> apart.set(touching, f(0)));
            t.when(gt(v(touching), f(0.5f)), touch -> {
                touch.when(gt(v(byEdge), f(0.5f)), edges -> {
                    edge.contact(edges, first, second, bestEdge, manifold, out);
                    edges.set(count, i(1));
                });
                touch.when(not(gt(v(byEdge), f(0.5f))), faces -> new Face(faces, first, second, fromSecond,
                        bestFaceAxis, d, margin).contact(faces, candidates, scratch, count, manifold, out));
            });
            t.store(manifold, add(v(out), i(COUNT)), toFloat(v(count)));
            });
        });
        return new Function("boxesCollide", new Type.FunctionType(Type.VOID, List.of()), b.finish());
    }

    // --- a box, as the narrow phase holds it ---------------------------------------------------------------

    /** A box loaded once: its centre, its axes (the columns of its turn) and its half-extents. */
    private static final class Solid {
        final LocalVar[] centre = new LocalVar[3];
        final LocalVar[][] axes = new LocalVar[3][];
        final LocalVar[] half = new LocalVar[3];

        Solid(Body b, List<Buffer> bs, Expr index) {
            LocalVar s = b.let("box", index);
            for (int a = 0; a < 3; a++) {
                centre[a] = b.let("c", load(bs.get(a), v(s)));
                half[a] = b.let("h", load(bs.get(7 + a), v(s)));
            }
            LocalVar x = b.let("qx", load(bs.get(3), v(s)));
            LocalVar y = b.let("qy", load(bs.get(4), v(s)));
            LocalVar z = b.let("qz", load(bs.get(5), v(s)));
            LocalVar w = b.let("qw", load(bs.get(6), v(s)));
            // The columns of the turn's matrix: where its own x, y and z axes point.
            axes[0] = let3(b, sub(f(1), mul(f(2), add(mul(v(y), v(y)), mul(v(z), v(z))))),
                    mul(f(2), add(mul(v(x), v(y)), mul(v(z), v(w)))),
                    mul(f(2), sub(mul(v(x), v(z)), mul(v(y), v(w)))));
            axes[1] = let3(b, mul(f(2), sub(mul(v(x), v(y)), mul(v(z), v(w)))),
                    sub(f(1), mul(f(2), add(mul(v(x), v(x)), mul(v(z), v(z))))),
                    mul(f(2), add(mul(v(y), v(z)), mul(v(x), v(w)))));
            axes[2] = let3(b, mul(f(2), add(mul(v(x), v(z)), mul(v(y), v(w)))),
                    mul(f(2), sub(mul(v(y), v(z)), mul(v(x), v(w)))),
                    sub(f(1), mul(f(2), add(mul(v(x), v(x)), mul(v(y), v(y))))));
        }

        /** {@code point} in this box's own frame: its offset from the centre along each axis. */
        LocalVar[] own(Body b, LocalVar[] point) {
            LocalVar[] offset = sub3(b, point, centre);
            return let3(b, dot(offset, axes[0]), dot(offset, axes[1]), dot(offset, axes[2]));
        }
    }

    // --- an edge contact -----------------------------------------------------------------------------------

    /** The two edges of the best edge axis, recorded as it is found, and the one point between them. */
    private static final class Edge {
        final LocalVar[] normal;
        final LocalVar[] onFirst;
        final LocalVar[] alongFirst;
        final LocalVar halfFirst;
        final LocalVar[] onSecond;
        final LocalVar[] alongSecond;
        final LocalVar halfSecond;
        final LocalVar axis;

        Edge(Body b) {
            normal = zero3(b);
            onFirst = zero3(b);
            alongFirst = zero3(b);
            halfFirst = b.let("edgeHalfA", f(0));
            onSecond = zero3(b);
            alongSecond = zero3(b);
            halfSecond = b.let("edgeHalfB", f(0));
            axis = b.let("edgeAxis", f(0));
        }

        /**
         * The edges along {@code first}'s axis {@code k} and {@code second}'s axis {@code j} that each box puts
         * furthest toward the other along {@code axis}, which is turned to point from the first to the second.
         */
        void record(Body b, Solid first, Solid second, int k, int j, LocalVar[] axis, LocalVar[] d) {
            LocalVar toward = b.let("toward", f(1));
            b.when(lt(dot(d, axis), f(0)), back -> back.set(toward, f(-1)));
            for (int a = 0; a < 3; a++) {
                b.set(normal[a], mul(v(axis[a]), v(toward)));
                b.set(onFirst[a], v(first.centre[a]));
                b.set(onSecond[a], v(second.centre[a]));
                b.set(alongFirst[a], v(first.axes[k][a]));
                b.set(alongSecond[a], v(second.axes[j][a]));
            }
            for (int m = 0; m < 3; m++) {
                if (m != k) {
                    LocalVar sign = sign(b, dot(first.axes[m], normal));
                    for (int a = 0; a < 3; a++) {
                        b.set(onFirst[a], add(v(onFirst[a]), mul(mul(v(sign), v(first.half[m])),
                                v(first.axes[m][a]))));
                    }
                }
                if (m != j) {
                    LocalVar sign = sign(b, dot(second.axes[m], normal));
                    for (int a = 0; a < 3; a++) {
                        b.set(onSecond[a], sub(v(onSecond[a]), mul(mul(v(sign), v(second.half[m])),
                                v(second.axes[m][a]))));
                    }
                }
            }
            b.set(halfFirst, v(first.half[k]));
            b.set(halfSecond, v(second.half[j]));
            b.set(this.axis, f(6 + 3 * k + j));
        }

        /** The closest points of the two edges, each held to its edge, as the one point of the manifold. */
        void contact(Body b, Solid first, Solid second, LocalVar bestEdge, Buffer manifold, LocalVar out) {
            LocalVar[] w = sub3(b, onFirst, onSecond);
            LocalVar cos = b.let("cos", dot(alongFirst, alongSecond));
            LocalVar dA = b.let("dA", dot(alongFirst, w));
            LocalVar dB = b.let("dB", dot(alongSecond, w));
            LocalVar den = b.let("den", sub(f(1), mul(v(cos), v(cos))));
            LocalVar s = b.let("s", div(sub(mul(v(cos), v(dB)), v(dA)), v(den)));
            LocalVar u = b.let("u", div(sub(v(dB), mul(v(cos), v(dA))), v(den)));
            b.set(s, max(neg(v(halfFirst)), min(v(halfFirst), v(s))));
            b.set(u, max(neg(v(halfSecond)), min(v(halfSecond), v(u))));
            LocalVar[] pointFirst = axpy(b, onFirst, v(s), alongFirst);
            LocalVar[] pointSecond = axpy(b, onSecond, v(u), alongSecond);
            write(b, manifold, out, normal, axis);
            writePoint(b, manifold, add(v(out), i(POINTS)), first.own(b, pointFirst), second.own(b, pointSecond),
                    neg(v(bestEdge)));
        }
    }

    // --- a face contact ------------------------------------------------------------------------------------

    /** The reference face and the incident face, and the work of clipping one against the other. */
    private static final class Face {
        final Solid first;
        final Solid second;
        final LocalVar flip;
        final LocalVar margin;
        // The reference box and its face: centre, normal toward the incident box, in-plane axes, half-extents.
        final LocalVar[] refCentre;
        final LocalVar[] n = new LocalVar[3];
        final LocalVar[] u = new LocalVar[3];
        final LocalVar[] w = new LocalVar[3];
        final LocalVar hn;
        final LocalVar hu;
        final LocalVar hw;
        final LocalVar axis;
        // The incident face: its centre, in-plane axes and half-extents.
        final LocalVar[] face = new LocalVar[3];
        final LocalVar[] a = new LocalVar[3];
        final LocalVar[] c = new LocalVar[3];
        final LocalVar ha;
        final LocalVar hc;

        Face(Body b, Solid first, Solid second, LocalVar fromSecond, LocalVar[] bestFaceAxis, LocalVar[] d,
             LocalVar margin) {
            this.first = first;
            this.second = second;
            this.margin = margin;
            // The reference box is the first unless the second's face won; flip is then −1.
            flip = b.let("flip", f(1));
            LocalVar k = b.let("k", v(bestFaceAxis[0]));
            refCentre = copy3(b, first.centre);
            LocalVar[][] refAxes = {copy3(b, first.axes[0]), copy3(b, first.axes[1]), copy3(b, first.axes[2])};
            LocalVar[] refHalf = copy3(b, first.half);
            LocalVar[] incCentre = copy3(b, second.centre);
            LocalVar[][] incAxes = {copy3(b, second.axes[0]), copy3(b, second.axes[1]), copy3(b, second.axes[2])};
            LocalVar[] incHalf = copy3(b, second.half);
            b.when(gt(v(fromSecond), f(0.5f)), swap -> {
                swap.set(flip, f(-1));
                swap.set(k, v(bestFaceAxis[1]));
                for (int e = 0; e < 3; e++) {
                    swap.set(refCentre[e], v(second.centre[e]));
                    swap.set(incCentre[e], v(first.centre[e]));
                    swap.set(refHalf[e], v(second.half[e]));
                    swap.set(incHalf[e], v(first.half[e]));
                    for (int m = 0; m < 3; m++) {
                        swap.set(refAxes[m][e], v(second.axes[m][e]));
                        swap.set(incAxes[m][e], v(first.axes[m][e]));
                    }
                }
            });
            axis = b.let("faceAxis", add(v(k), mul(f(1.5f), sub(f(1), v(flip)))));

            // The reference face: axis k, and the two after it in turn.
            for (int e = 0; e < 3; e++) {
                n[e] = b.let("n", f(0));
                u[e] = b.let("u", f(0));
                w[e] = b.let("w", f(0));
            }
            hn = b.let("hn", f(0));
            hu = b.let("hu", f(0));
            hw = b.let("hw", f(0));
            for (int kLoop = 0; kLoop < 3; kLoop++) {
                int kk = kLoop;
                int ku = (kk + 1) % 3;
                int kw = (kk + 2) % 3;
                b.when(and(gt(v(k), f(kk - 0.5f)), lt(v(k), f(kk + 0.5f))), chosen -> {
                    for (int e = 0; e < 3; e++) {
                        chosen.set(n[e], v(refAxes[kk][e]));
                        chosen.set(u[e], v(refAxes[ku][e]));
                        chosen.set(w[e], v(refAxes[kw][e]));
                    }
                    chosen.set(hn, v(refHalf[kk]));
                    chosen.set(hu, v(refHalf[ku]));
                    chosen.set(hw, v(refHalf[kw]));
                });
            }
            LocalVar[] apart = sub3(b, incCentre, refCentre);
            b.when(lt(dot(apart, n), f(0)), back -> {
                for (int e = 0; e < 3; e++) {
                    back.set(n[e], neg(v(n[e])));
                }
            });

            // The incident face: the other box's face turned most against the normal.
            LocalVar most = b.let("most", f(-1));
            LocalVar[] faceAxis = zero3(b);
            LocalVar hf = b.let("hf", f(0));
            for (int e = 0; e < 3; e++) {
                a[e] = b.let("a", f(0));
                c[e] = b.let("cc", f(0));
            }
            ha = b.let("ha", f(0));
            hc = b.let("hc", f(0));
            for (int mLoop = 0; mLoop < 3; mLoop++) {
                int m = mLoop;
                int ma = (m + 1) % 3;
                int mc = (m + 2) % 3;
                LocalVar turned = b.let("turned", abs(dot(incAxes[m], n)));
                b.when(gt(v(turned), v(most)), better -> {
                    better.set(most, v(turned));
                    for (int e = 0; e < 3; e++) {
                        better.set(faceAxis[e], v(incAxes[m][e]));
                        better.set(a[e], v(incAxes[ma][e]));
                        better.set(c[e], v(incAxes[mc][e]));
                    }
                    better.set(hf, v(incHalf[m]));
                    better.set(ha, v(incHalf[ma]));
                    better.set(hc, v(incHalf[mc]));
                });
            }
            LocalVar facing = sign(b, dot(faceAxis, n));
            for (int e = 0; e < 3; e++) {
                face[e] = b.let("face", sub(v(incCentre[e]), mul(mul(v(facing), v(hf)), v(faceAxis[e]))));
            }
        }

        /** The candidates found and kept, four chosen, and the manifold written. */
        void contact(Body b, Buffer candidates, LocalVar scratch, LocalVar count, Buffer manifold, LocalVar out) {
            // The incident face's corners, in order round it, and each in the reference face's terms.
            float[][] signs = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
            LocalVar[][] corner = new LocalVar[4][];
            LocalVar[] cu = new LocalVar[4];
            LocalVar[] cw = new LocalVar[4];
            LocalVar[] depth = new LocalVar[4];
            for (int e = 0; e < 4; e++) {
                Expr[] at = new Expr[3];
                for (int x = 0; x < 3; x++) {
                    at[x] = add(v(face[x]), add(mul(mul(f(signs[e][0]), v(ha)), v(a[x])),
                            mul(mul(f(signs[e][1]), v(hc)), v(c[x]))));
                }
                corner[e] = let3(b, at[0], at[1], at[2]);
                LocalVar[] rel = sub3(b, corner[e], refCentre);
                cu[e] = b.let("cu", dot(rel, u));
                cw[e] = b.let("cw", dot(rel, w));
                depth[e] = b.let("depth", sub(v(hn), dot(rel, n)));
                Expr inside = and(lt(abs(v(cu[e])), mul(v(hu), f(1 + SLACK))), lt(abs(v(cw[e])), mul(v(hw), f(1 + SLACK))));
                keep(b, candidates, scratch, count, inside, corner[e], v(depth[e]));
            }

            // Where the incident face's edges cross the reference face's.
            for (int e = 0; e < 4; e++) {
                int next = (e + 1) % 4;
                for (int line = 0; line < 4; line++) {
                    boolean acrossU = line < 2;
                    float side = line % 2 == 0 ? 1 : -1;
                    LocalVar from = acrossU ? cu[e] : cw[e];
                    LocalVar to = acrossU ? cu[next] : cw[next];
                    LocalVar alongFrom = acrossU ? cw[e] : cu[e];
                    LocalVar alongTo = acrossU ? cw[next] : cu[next];
                    LocalVar limit = acrossU ? hu : hw;
                    LocalVar bound = acrossU ? hw : hu;
                    int e0 = e;
                    LocalVar span = b.let("span", sub(v(to), v(from)));
                    b.when(gt(abs(v(span)), f(SAME)), crosses -> {
                        LocalVar s = crosses.let("s", div(sub(mul(f(side), v(limit)), v(from)), v(span)));
                        LocalVar along = crosses.let("along", add(v(alongFrom), mul(v(s), sub(v(alongTo),
                                v(alongFrom)))));
                        Expr within = and(and(gt(v(s), f(-SLACK)), lt(v(s), f(1 + SLACK))),
                                lt(abs(v(along)), mul(v(bound), f(1 + SLACK))));
                        Expr[] at = new Expr[3];
                        for (int x = 0; x < 3; x++) {
                            at[x] = add(v(corner[e0][x]), mul(v(s), sub(v(corner[next][x]), v(corner[e0][x]))));
                        }
                        LocalVar[] point = let3(crosses, at[0], at[1], at[2]);
                        Expr deep = add(v(depth[e0]), mul(v(s), sub(v(depth[next]), v(depth[e0]))));
                        keep(crosses, candidates, scratch, count, within, point, deep);
                    });
                }
            }

            // The reference face's corners inside the incident face, seen along the normal.
            LocalVar[] off = sub3(b, face, refCentre);
            LocalVar fu = b.let("fu", dot(off, u));
            LocalVar fw = b.let("fw", dot(off, w));
            LocalVar au = b.let("au", mul(v(ha), dot(a, u)));
            LocalVar aw = b.let("aw", mul(v(ha), dot(a, w)));
            LocalVar bu = b.let("bu", mul(v(hc), dot(c, u)));
            LocalVar bw = b.let("bw", mul(v(hc), dot(c, w)));
            LocalVar det = b.let("det", sub(mul(v(au), v(bw)), mul(v(aw), v(bu))));
            b.when(gt(abs(v(det)), f(SAME)), seen -> {
                for (float[] sign : signs) {
                    LocalVar ru = seen.let("ru", sub(mul(f(sign[0]), v(hu)), v(fu)));
                    LocalVar rw = seen.let("rw", sub(mul(f(sign[1]), v(hw)), v(fw)));
                    LocalVar alpha = seen.let("alpha", div(sub(mul(v(ru), v(bw)), mul(v(rw), v(bu))), v(det)));
                    LocalVar beta = seen.let("beta", div(sub(mul(v(au), v(rw)), mul(v(aw), v(ru))), v(det)));
                    Expr inside = and(lt(abs(v(alpha)), f(1 + SLACK)), lt(abs(v(beta)), f(1 + SLACK)));
                    Expr[] at = new Expr[3];
                    for (int x = 0; x < 3; x++) {
                        at[x] = add(v(face[x]), add(mul(mul(v(alpha), v(ha)), v(a[x])),
                                mul(mul(v(beta), v(hc)), v(c[x]))));
                    }
                    LocalVar[] point = let3(seen, at[0], at[1], at[2]);
                    LocalVar[] rel = sub3(seen, point, refCentre);
                    keep(seen, candidates, scratch, count, inside, point, sub(v(hn), dot(rel, n)));
                }
            });

            LocalVar[] normal = scale3(b, n, v(flip));
            write(b, manifold, out, normal, axis);
            scratchOf = scratch;
            LocalVar found = b.let("found", v(count));
            b.set(count, i(0));
            b.when(gt(v(found), i(0)), some -> reduce(some, candidates, scratch, found, count, manifold, out));
        }

        /** Candidate {@code point}, at {@code depth} below the reference face, kept if {@code when} and deep enough. */
        private void keep(Body b, Buffer candidates, LocalVar scratch, LocalVar count, Expr when, LocalVar[] point,
                          Expr depth) {
            LocalVar deep = b.let("deep", depth);
            Expr room = lt(v(count), i(CANDIDATES));
            b.when(and(and(when, gt(v(deep), neg(v(margin)))), room), kept -> {
                LocalVar at = kept.let("at", add(v(scratch), mul(v(count), i(CANDIDATE_WORDS))));
                for (int x = 0; x < 3; x++) {
                    kept.store(candidates, add(v(at), i(x)), v(point[x]));
                }
                kept.store(candidates, add(v(at), i(3)), v(deep));
                kept.set(count, add(v(count), i(1)));
            });
        }

        /**
         * Four of the {@code found} candidates: the deepest, the farthest from it, and the two that make the largest
         * triangle with those, one on each side; each written as a point, and {@code count} left as how many.
         */
        private void reduce(Body b, Buffer candidates, LocalVar scratch, LocalVar found, LocalVar count,
                            Buffer manifold, LocalVar out) {
            LocalVar deepest = b.let("deepest", i(0));
            LocalVar most = b.let("mostDeep", f(-1e30f));
            each(b, found, (k, at) -> {
                LocalVar deep = k.let("d", load(candidates, add(v(at), i(3))));
                k.when(gt(v(deep), v(most)), better -> {
                    better.set(most, v(deep));
                    better.set(deepest, v(at));
                });
            });
            LocalVar[] pa = loadPoint(b, candidates, deepest);
            LocalVar farthest = b.let("farthest", i(-1));
            LocalVar far = b.let("far", f(SAME));
            each(b, found, (k, at) -> {
                LocalVar[] q = loadPoint(k, candidates, at);
                LocalVar[] gap = sub3(k, q, pa);
                LocalVar d2 = k.let("d2", dot(gap, gap));
                k.when(gt(v(d2), v(far)), better -> {
                    better.set(far, v(d2));
                    better.set(farthest, v(at));
                });
            });
            emit(b, candidates, deepest, count, manifold, out);
            b.when(gt(v(farthest), i(-1)), second -> {
                LocalVar[] pb = loadPoint(second, candidates, farthest);
                LocalVar[] ab = sub3(second, pb, pa);
                LocalVar left = second.let("left", i(-1));
                LocalVar right = second.let("right", i(-1));
                // A third point must stand off the line through the first two by some share of its length, or it is
                // one of them again, moved by rounding.
                LocalVar mostLeft = second.let("mostLeft", mul(f(OFF_LINE), v(far)));
                LocalVar mostRight = second.let("mostRight", neg(v(mostLeft)));
                each(second, found, (k, at) -> {
                    LocalVar[] q = loadPoint(k, candidates, at);
                    LocalVar[] aq = sub3(k, q, pa);
                    LocalVar area = k.let("area", dot(cross(k, ab, aq), n));
                    k.when(gt(v(area), v(mostLeft)), better -> {
                        better.set(mostLeft, v(area));
                        better.set(left, v(at));
                    });
                    k.when(lt(v(area), v(mostRight)), better -> {
                        better.set(mostRight, v(area));
                        better.set(right, v(at));
                    });
                });
                emit(second, candidates, farthest, count, manifold, out);
                second.when(gt(v(left), i(-1)), third -> emit(third, candidates, left, count, manifold, out));
                second.when(gt(v(right), i(-1)), fourth -> emit(fourth, candidates, right, count, manifold, out));
            });
        }

        /** The candidate at {@code at} written as the manifold's next point, its anchors on each box. */
        private void emit(Body b, Buffer candidates, LocalVar at, LocalVar count, Buffer manifold, LocalVar out) {
            LocalVar[] incident = loadPoint(b, candidates, at);
            LocalVar deep = b.let("deep", load(candidates, add(v(at), i(3))));
            // The point on the reference face is the incident point raised by its depth along the normal.
            LocalVar[] reference = axpy(b, incident, v(deep), n);
            LocalVar onFirstIsRef = b.let("firstRef", mul(f(0.5f), add(v(flip), f(1))));
            Expr[] onFirst = new Expr[3];
            Expr[] onSecond = new Expr[3];
            for (int x = 0; x < 3; x++) {
                Expr lift = mul(v(deep), v(n[x]));
                onFirst[x] = add(v(incident[x]), mul(v(onFirstIsRef), lift));
                onSecond[x] = add(v(incident[x]), mul(sub(f(1), v(onFirstIsRef)), lift));
            }
            LocalVar[] pointFirst = let3(b, onFirst[0], onFirst[1], onFirst[2]);
            LocalVar[] pointSecond = let3(b, onSecond[0], onSecond[1], onSecond[2]);
            writePoint(b, manifold, add(v(out), add(i(POINTS), mul(v(count), i(POINT_WORDS)))),
                    first.own(b, pointFirst), second.own(b, pointSecond), v(deep));
            b.set(count, add(v(count), i(1)));
        }

        private static LocalVar[] loadPoint(Body b, Buffer candidates, LocalVar at) {
            return let3(b, load(candidates, v(at)), load(candidates, add(v(at), i(1))),
                    load(candidates, add(v(at), i(2))));
        }

        /** {@code work} for each of the {@code found} candidates, with the index of its first word. */
        private void each(Body b, LocalVar found, java.util.function.BiConsumer<Body, LocalVar> work) {
            LocalVar k = b.let("k", i(0));
            b.loop(lt(v(k), v(found)), next -> {
                LocalVar at = next.let("at", add(v(scratchOf), mul(v(k), i(CANDIDATE_WORDS))));
                work.accept(next, at);
                next.set(k, add(v(k), i(1)));
            });
        }

        private LocalVar scratchOf;
    }

    // --- writing -------------------------------------------------------------------------------------------

    private static void write(Body b, Buffer manifold, LocalVar out, LocalVar[] normal, LocalVar axis) {
        for (int x = 0; x < 3; x++) {
            b.store(manifold, add(v(out), i(NORMAL + x)), v(normal[x]));
        }
        b.store(manifold, add(v(out), i(AXIS)), v(axis));
    }

    private static void writePoint(Body b, Buffer manifold, Expr at, LocalVar[] onFirst, LocalVar[] onSecond,
                                   Expr depth) {
        LocalVar where = b.let("where", at);
        for (int x = 0; x < 3; x++) {
            b.store(manifold, add(v(where), i(ANCHOR_FIRST + x)), v(onFirst[x]));
            b.store(manifold, add(v(where), i(ANCHOR_SECOND + x)), v(onSecond[x]));
        }
        b.store(manifold, add(v(where), i(DEPTH)), depth);
    }

    // --- vectors of locals ---------------------------------------------------------------------------------

    private static LocalVar[] let3(Body b, Expr x, Expr y, Expr z) {
        return new LocalVar[] {b.let("vx", x), b.let("vy", y), b.let("vz", z)};
    }

    private static LocalVar[] zero3(Body b) {
        return let3(b, f(0), f(0), f(0));
    }

    private static LocalVar[] copy3(Body b, LocalVar[] p) {
        return let3(b, v(p[0]), v(p[1]), v(p[2]));
    }

    private static LocalVar[] sub3(Body b, LocalVar[] p, LocalVar[] q) {
        return let3(b, sub(v(p[0]), v(q[0])), sub(v(p[1]), v(q[1])), sub(v(p[2]), v(q[2])));
    }

    private static LocalVar[] scale3(Body b, LocalVar[] p, Expr s) {
        LocalVar k = b.let("k", s);
        return let3(b, mul(v(p[0]), v(k)), mul(v(p[1]), v(k)), mul(v(p[2]), v(k)));
    }

    /** {@code p + s · dir}. */
    private static LocalVar[] axpy(Body b, LocalVar[] p, Expr s, LocalVar[] dir) {
        LocalVar k = b.let("k", s);
        return let3(b, add(v(p[0]), mul(v(k), v(dir[0]))), add(v(p[1]), mul(v(k), v(dir[1]))),
                add(v(p[2]), mul(v(k), v(dir[2]))));
    }

    private static Expr dot(LocalVar[] p, LocalVar[] q) {
        return add(add(mul(v(p[0]), v(q[0])), mul(v(p[1]), v(q[1]))), mul(v(p[2]), v(q[2])));
    }

    private static LocalVar[] cross(Body b, LocalVar[] p, LocalVar[] q) {
        return let3(b, sub(mul(v(p[1]), v(q[2])), mul(v(p[2]), v(q[1]))),
                sub(mul(v(p[2]), v(q[0])), mul(v(p[0]), v(q[2]))),
                sub(mul(v(p[0]), v(q[1])), mul(v(p[1]), v(q[0]))));
    }

    /** 1 where {@code x} is at least zero, −1 where it is less. */
    private static LocalVar sign(Body b, Expr x) {
        LocalVar s = b.let("sign", f(1));
        b.when(lt(x, f(0)), negative -> negative.set(s, f(-1)));
        return s;
    }

    private static List<Buffer> bind() {
        List<Buffer> buffers = new ArrayList<>();
        for (int k = 0; k < NAMES.length; k++) {
            buffers.add(new Buffer(NAMES[k], k, NAMES[k].equals("pairs") ? I32 : F32));
        }
        return List.copyOf(buffers);
    }
}

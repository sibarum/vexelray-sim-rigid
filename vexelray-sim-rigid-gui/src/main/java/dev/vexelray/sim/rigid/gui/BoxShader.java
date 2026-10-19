package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.Buffer;
import dev.supirvast.vastir.core.Expr;
import dev.supirvast.vastir.core.LocalVar;
import dev.vexelray.sim.rigid.box.Boxes;

import static dev.supirvast.vastir.build.Body.F32;
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
import static dev.supirvast.vastir.build.Body.mix;
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.neg;
import static dev.supirvast.vastir.build.Body.not;
import static dev.supirvast.vastir.build.Body.sqrt;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.toFloat;
import static dev.supirvast.vastir.build.Body.v;

/**
 * The boxes' picture: one pixel, one ray, tested against every box exactly, in SupirVast IR, on the same sky, floor and
 * light as the spheres' ({@link Picture}).
 *
 * <p>A box is met by the slab test in its own frame. Its centre is blended between the last two steps by the push
 * constant {@code alpha}, as a sphere's is, and its orientation by normalised lerp, the short way round. The ray is
 * taken into the box's frame by the inverse turn, where the box is {@code |p_a| ≤ h_a} along each axis: each pair of
 * faces is a slab the ray is inside between two distances, and the box is where all three overlap. The face the ray
 * entered by is the normal, turned back into the world's frame. A sphere around each box is tested first, so a pixel
 * spends the slab test only on boxes it may meet nearer than anything yet.
 *
 * <p>A body whose shape is a sphere ({@code Boxes.SPHERE_SHAPE}) is met by its quadratic instead, in its own frame, and
 * painted in octants of that frame, paler and darker, as the spheres' own picture paints them, so that it shows
 * rolling.
 *
 * <p>A box's edges are drawn darker, a band 6% of its smallest half-extent wide, so that how it is turned reads even
 * where two faces are lit alike.
 */
final class BoxShader {

    /** Binding of the shown buffer at set 0; the host's binding must agree. */
    static final int SHOWN_BINDING = 0;

    private static final Buffer SHOWN = new Buffer("shown", SHOWN_BINDING, F32);

    /** The edge band's width, as a share of the box's smallest half-extent, and how dark it is. */
    private static final double EDGE = 0.06;
    private static final double EDGE_LIGHT = 0.55;

    /** Below this a ray is taken as running along a slab, never crossing it. */
    private static final double PARALLEL = 1e-9;

    /** How far toward white a sphere's paler octants are, as in the spheres' picture. */
    private static final double PALE = 0.45;

    private BoxShader() {
    }

    /** The fragment stage as SPIR-V. Pair it with {@code Fullscreen.triangleVertexWithUvSpirv()}. */
    static byte[] fragmentSpirv() {
        Body b = new Body();
        Picture.Frame frame = new Picture.Frame(b);
        LocalVar[] eye = frame.eye;
        LocalVar[] d = frame.d;
        LocalVar alpha = frame.alpha;

        LocalVar best = b.let("best", f(1e30));
        LocalVar hit = b.let("hit", f(-1));
        LocalVar edged = b.let("edged", f(0));
        LocalVar pale = b.let("pale", f(0));
        LocalVar[] normal = {b.let("nx", f(0)), b.let("ny", f(0)), b.let("nz", f(0))};
        LocalVar k = b.let("k", i(0));
        b.loop(lt(v(k), v(frame.count)), pass -> {
            LocalVar base = pass.let("base", mul(v(k), i(Boxes.SHOWN_STRIDE)));
            LocalVar[] o = new LocalVar[3];
            LocalVar[] half = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                Expr c = mix(load(SHOWN, add(v(base), i(Boxes.CENTRE_BEFORE + a))),
                        load(SHOWN, add(v(base), i(Boxes.CENTRE_AFTER + a))), v(alpha));
                o[a] = pass.let("o", sub(v(eye[a]), c));
                half[a] = pass.let("half", load(SHOWN, add(v(base), i(Boxes.HALF + a))));
            }
            LocalVar reach2 = pass.let("reach2", Picture.dot(half, half));
            LocalVar along = pass.let("along", Picture.dot(o, d));
            LocalVar disc = pass.let("disc", sub(mul(v(along), v(along)), sub(Picture.dot(o, o), v(reach2))));
            pass.when(gt(v(disc), f(0)), meets -> {
                LocalVar root = meets.let("root", sqrt(v(disc)));
                Expr ahead = gt(sub(v(root), v(along)), f(0));
                Expr sooner = lt(sub(neg(v(along)), v(root)), v(best));
                meets.when(and(ahead, sooner), near -> slabs(near, k, base, o, half, d, alpha, best, hit, edged,
                        pale, normal));
            });
            pass.set(k, add(v(k), i(1)));
        });

        LocalVar[] rgb = Picture.background(b, frame, best);
        b.when(gt(v(hit), f(-0.5)), box -> {
            Picture.shade(box, rgb, frame, hit, normal, mul(f(PALE), v(pale)));
            for (int a = 0; a < 3; a++) {
                box.set(rgb[a], mul(v(rgb[a]), mix(f(1), f(EDGE_LIGHT), v(edged))));
            }
        });
        return Picture.finish(b, rgb);
    }

    /**
     * Box {@code k}, met by the ray if it is: kept as the nearest, with its normal and whether the ray met it on an
     * edge's band, if nearer than {@code best}. {@code o} is the eye less the box's blended centre.
     */
    private static void slabs(Body b, LocalVar k, LocalVar base, LocalVar[] o, LocalVar[] half, LocalVar[] d,
                              LocalVar alpha, LocalVar best, LocalVar hit, LocalVar edged, LocalVar pale,
                              LocalVar[] normal) {
        // The orientation, blended the short way round, and left unnormalised: rotate divides by its length.
        LocalVar[] before = new LocalVar[4];
        LocalVar[] after = new LocalVar[4];
        for (int j = 0; j < 4; j++) {
            before[j] = b.let("q0", load(SHOWN, add(v(base), i(Boxes.TURNED_BEFORE + j))));
            after[j] = b.let("q1", load(SHOWN, add(v(base), i(Boxes.TURNED_AFTER + j))));
        }
        LocalVar way = b.let("way", f(1));
        Expr aligned = add(Picture.dot(before, after), mul(v(before[3]), v(after[3])));
        b.when(lt(aligned, f(0)), back -> back.set(way, f(-1)));
        LocalVar[] q = new LocalVar[4];
        for (int j = 0; j < 4; j++) {
            q[j] = b.let("q", mix(mul(v(before[j]), v(way)), v(after[j]), v(alpha)));
        }
        LocalVar[] inverse = {b.let("iq", neg(v(q[0]))), b.let("iq", neg(v(q[1]))), b.let("iq", neg(v(q[2]))), q[3]};
        LocalVar[] os = rotate(b, inverse, o);
        LocalVar[] ds = rotate(b, inverse, d);

        // A sphere, its radius the first half-extent: met where a quadratic says, in its own frame, and painted in
        // octants of that frame, as the spheres' picture paints them, so that its turning shows.
        LocalVar round = b.let("round", load(SHOWN, add(v(base), i(Boxes.SHAPE))));
        b.when(gt(v(round), f(0.5f)), ball -> {
            LocalVar hh = ball.let("hh", Picture.dot(os, ds));
            LocalVar cc = ball.let("cc", sub(Picture.dot(os, os), mul(v(half[0]), v(half[0]))));
            LocalVar disc = ball.let("ballDisc", sub(mul(v(hh), v(hh)), mul(Picture.dot(ds, ds), v(cc))));
            ball.when(gt(v(disc), f(0)), crosses -> {
                LocalVar t = crosses.let("tBall", div(sub(neg(v(hh)), sqrt(v(disc))), Picture.dot(ds, ds)));
                crosses.when(and(gt(v(t), f(0)), lt(v(t), v(best))), nearer -> {
                    nearer.set(best, v(t));
                    nearer.set(hit, toFloat(v(k)));
                    LocalVar[] own = new LocalVar[3];
                    for (int a = 0; a < 3; a++) {
                        own[a] = nearer.let("own", add(v(os[a]), mul(v(t), v(ds[a]))));
                    }
                    LocalVar[] world = rotate(nearer, q, own);
                    for (int a = 0; a < 3; a++) {
                        nearer.set(normal[a], v(world[a]));
                    }
                    nearer.set(edged, f(0));
                    nearer.set(pale, f(0));
                    nearer.when(gt(mul(mul(v(own[0]), v(own[1])), v(own[2])), f(0)), patch -> patch.set(pale, f(1)));
                });
            });
        });

        // Each slab's entry and exit; the box is where the ray is inside all three.
        LocalVar enter = b.let("enter", f(-1e30));
        LocalVar leave = b.let("leave", f(1e30));
        LocalVar face = b.let("face", f(0));
        for (int a = 0; a < 3; a++) {
            int axis = a;
            LocalVar step = b.let("step", v(ds[a]));
            b.when(lt(abs(v(step)), f(PARALLEL)), flat -> flat.set(step, f(PARALLEL)));
            LocalVar t1 = b.let("t1", div(sub(neg(v(half[a])), v(os[a])), v(step)));
            LocalVar t2 = b.let("t2", div(sub(v(half[a]), v(os[a])), v(step)));
            LocalVar near = b.let("near", min(v(t1), v(t2)));
            b.set(leave, min(v(leave), max(v(t1), v(t2))));
            b.when(gt(v(near), v(enter)), later -> {
                later.set(enter, v(near));
                later.set(face, f(axis));
            });
        }
        Expr meets = and(and(and(lt(v(enter), v(leave)), gt(v(enter), f(0))), lt(v(enter), v(best))),
                not(gt(v(round), f(0.5f))));
        b.when(meets, nearer -> {
            nearer.set(best, v(enter));
            nearer.set(hit, toFloat(v(k)));
            // The face entered by, outward: against the ray along its axis.
            LocalVar[] own = new LocalVar[3];
            LocalVar[] gap = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                LocalVar on = nearer.let("on", f(0));
                nearer.when(and(gt(v(face), f(a - 0.5)), lt(v(face), f(a + 0.5))), entered ->
                        entered.set(on, f(1)));
                own[a] = nearer.let("own", neg(mul(v(on), v(ds[a]))));
                LocalVar p = nearer.let("p", add(v(os[a]), mul(v(enter), v(ds[a]))));
                gap[a] = nearer.let("gap", sub(v(half[a]), abs(v(p))));
            }
            LocalVar[] world = rotate(nearer, q, own);
            for (int a = 0; a < 3; a++) {
                nearer.set(normal[a], v(world[a]));
            }
            // The point's distance to the nearest edge is the middle of its three gaps: the smallest is the face's.
            Expr sum = add(add(v(gap[0]), v(gap[1])), v(gap[2]));
            Expr least = min(v(gap[0]), min(v(gap[1]), v(gap[2])));
            Expr most = max(v(gap[0]), max(v(gap[1]), v(gap[2])));
            LocalVar toEdge = nearer.let("toEdge", sub(sub(sum, least), most));
            LocalVar band = nearer.let("band", mul(f(EDGE), min(v(half[0]), min(v(half[1]), v(half[2])))));
            nearer.set(edged, f(0));
            nearer.set(pale, f(0));
            nearer.when(lt(v(toEdge), v(band)), edge -> edge.set(edged, f(1)));
        });
    }

    /**
     * {@code p} turned by the quaternion {@code q}, of any length: {@code p + w t + q × t}, with
     * {@code t = 2 q × p / |q|²}.
     */
    private static LocalVar[] rotate(Body b, LocalVar[] q, LocalVar[] p) {
        LocalVar twice = b.let("twice", div(f(2), add(add(mul(v(q[0]), v(q[0])), mul(v(q[1]), v(q[1]))),
                add(mul(v(q[2]), v(q[2])), mul(v(q[3]), v(q[3]))))));
        LocalVar[] axis = {q[0], q[1], q[2]};
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

    private static LocalVar[] cross(Body b, LocalVar[] x, LocalVar[] y) {
        return new LocalVar[] {
                b.let("cx", sub(mul(v(x[1]), v(y[2])), mul(v(x[2]), v(y[1])))),
                b.let("cy", sub(mul(v(x[2]), v(y[0])), mul(v(x[0]), v(y[2])))),
                b.let("cz", sub(mul(v(x[0]), v(y[1])), mul(v(x[1]), v(y[0]))))};
    }
}

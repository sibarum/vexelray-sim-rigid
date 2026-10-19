package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.Buffer;
import dev.supirvast.vastir.core.CoreModule;
import dev.supirvast.vastir.core.EntryPoint;
import dev.supirvast.vastir.core.Expr;
import dev.supirvast.vastir.core.Function;
import dev.supirvast.vastir.core.InterfaceVar;
import dev.supirvast.vastir.core.LocalVar;
import dev.supirvast.vastir.core.PushConstants;
import dev.supirvast.vastir.core.ShaderStage;
import dev.supirvast.vastir.lower.CoreToSpirv;
import dev.supirvast.vastir.type.Type;
import dev.vexelray.sim.rigid.sphere.Spheres;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

import static dev.supirvast.vastir.build.Body.F32;
import static dev.supirvast.vastir.build.Body.VEC2;
import static dev.supirvast.vastir.build.Body.VEC4;
import static dev.supirvast.vastir.build.Body.add;
import static dev.supirvast.vastir.build.Body.and;
import static dev.supirvast.vastir.build.Body.clamp;
import static dev.supirvast.vastir.build.Body.component;
import static dev.supirvast.vastir.build.Body.div;
import static dev.supirvast.vastir.build.Body.f;
import static dev.supirvast.vastir.build.Body.floor;
import static dev.supirvast.vastir.build.Body.gt;
import static dev.supirvast.vastir.build.Body.i;
import static dev.supirvast.vastir.build.Body.input;
import static dev.supirvast.vastir.build.Body.load;
import static dev.supirvast.vastir.build.Body.lt;
import static dev.supirvast.vastir.build.Body.max;
import static dev.supirvast.vastir.build.Body.min;
import static dev.supirvast.vastir.build.Body.mix;
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.neg;
import static dev.supirvast.vastir.build.Body.pushed;
import static dev.supirvast.vastir.build.Body.sqrt;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.toFloat;
import static dev.supirvast.vastir.build.Body.toInt;
import static dev.supirvast.vastir.build.Body.v;
import static dev.supirvast.vastir.build.Body.vec3;
import static dev.supirvast.vastir.build.Body.vec4;

/**
 * The spheres' picture: one pixel, one ray, tested against every sphere exactly, written by hand in SupirVast IR.
 *
 * <p>A sphere is not marched as a distance field. A ray meets a sphere where a quadratic says it does, so each pixel
 * solves one quadratic per sphere and keeps the nearest root: a few hundred spheres cost a few hundred small sums a
 * pixel, where a march of their union would cost that many per <em>step</em>. Every sphere is tested; a grid would
 * cut that down, and is the same broad phase the solver wants.
 *
 * <p>Each sphere's centre is blended between the last two steps by the push constant {@code alpha}, which the host
 * takes from Kronometer's {@code Handoff.phase}: the picture shows the state one step behind the frame, smoothly, at
 * any refresh rate. The floor inside the box is a checker, so motion across it reads; outside, the picture is sky.
 *
 * <p>A sphere is drawn squashed by what presses it, as {@code Spheres.show} measured it: an ellipsoid of the sphere's
 * volume, flattened along the contacts by the push constant {@code squash} times the overlaps, as shares of the
 * radius. At 1.5 the flattening, less the bulge that keeps the volume, closes an overlap, so spheres that overlap
 * look pressed together instead; past it a sphere is moved to keep touching what it presses on. Every sphere is still
 * tested by one quadratic, of a sphere that bounds the squashed one, and only those met nearer than anything yet are
 * tested as squashed.
 *
 * <p>A sphere is painted like a beach ball, in eight octants of its own frame, alternately its colour and paler, so
 * that its spin shows: the orientation is blended between the steps as the centre is.
 *
 * <p>Coordinates are the simulation's: metres, {@code y} up, the box from the origin to {@code (sx, sy, sz)}. The
 * camera arrives as an eye and three axes, which the host works out from its orbit.
 */
final class SphereShader {

    /** Binding of the shown buffer at set 0; the host's binding must agree. */
    static final int SHOWN_BINDING = 0;

    /**
     * The most a sphere is squashed by: the gain times its flattening's trace is held to this, which keeps
     * {@code I − g F} well inside positive, and lets a sphere of radius {@code r (1 + 0.53 g tr F)} bound the squashed
     * one, since {@code det(I − g F) ≥ 1 − g tr F} and {@code (1 − x)^(−1/3) ≤ 1 + 0.53 x} up to {@code x = ½}.
     */
    private static final double MOST_SQUASH = 0.5;

    private static final Buffer SHOWN = new Buffer("shown", SHOWN_BINDING, F32);

    /** How far toward white a sphere's paler octants are. */
    private static final double PALE = 0.45;

    private SphereShader() {
    }

    /** The fragment stage as SPIR-V. Pair it with {@code Fullscreen.triangleVertexWithUvSpirv()}. */
    static byte[] fragmentSpirv() {
        Body b = new Body();
        Picture.Frame frame = new Picture.Frame(b);
        LocalVar[] eye = frame.eye;
        LocalVar[] d = frame.d;
        LocalVar alpha = frame.alpha;

        // The nearest sphere the ray meets, squashed: first the sphere that bounds it, then, if that is met nearer
        // than anything yet, the squashed sphere itself.
        LocalVar squash = b.let("squash", frame.squash());
        LocalVar best = b.let("best", f(1e30));
        LocalVar hit = b.let("hit", f(-1));
        LocalVar[] normal = {b.let("nx", f(0)), b.let("ny", f(0)), b.let("nz", f(0))};
        LocalVar[] local = {b.let("lx", f(0)), b.let("ly", f(0)), b.let("lz", f(0))};
        LocalVar k = b.let("k", i(0));
        b.loop(lt(v(k), v(frame.count)), pass -> {
            LocalVar base = pass.let("base", mul(v(k), i(Spheres.SHOWN_STRIDE)));
            LocalVar[] c = new LocalVar[3];
            LocalVar[] o = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                c[a] = pass.let("c", mix(load(SHOWN, add(v(base), i(a))), load(SHOWN, add(v(base), i(4 + a))), v(alpha)));
                o[a] = pass.let("o", sub(v(eye[a]), v(c[a])));
            }
            LocalVar r = pass.let("r", load(SHOWN, add(v(base), i(3))));
            LocalVar trace = pass.let("trace", pressing(base, Spheres.TRACE, alpha));
            LocalVar leanLength = pass.let("leanLength", pressing(base, Spheres.LEAN_LENGTH, alpha));
            LocalVar gain = pass.let("gain", min(v(squash), div(f(MOST_SQUASH), max(v(trace), f(1e-9)))));
            LocalVar lift = pass.let("lift", max(f(0), sub(mul(v(gain), f(2.0 / 3)), f(1))));
            LocalVar bound = pass.let("bound", add(mul(v(r), add(f(1), mul(f(0.53), mul(v(gain), v(trace))))),
                    mul(v(lift), mul(v(r), v(leanLength)))));
            LocalVar half = pass.let("half", dot(o, d));
            LocalVar disc = pass.let("disc", sub(mul(v(half), v(half)), sub(dot(o, o), mul(v(bound), v(bound)))));
            pass.when(gt(v(disc), f(0)), meets -> {
                LocalVar root = meets.let("root", sqrt(v(disc)));
                Expr ahead = gt(sub(v(root), v(half)), f(0));
                Expr sooner = lt(sub(neg(v(half)), v(root)), v(best));
                meets.when(and(ahead, sooner), near -> squashed(near, k, base, c, r, gain, lift, eye, d, alpha,
                        best, hit, normal, local));
            });
            pass.set(k, add(v(k), i(1)));
        });

        LocalVar[] rgb = Picture.background(b, frame, best);
        b.when(gt(v(hit), f(-0.5)), ball -> {
            LocalVar patch = ball.let("patch", f(0));
            ball.when(gt(turned(ball, hit, alpha, local), f(0)), pale -> pale.set(patch, f(PALE)));
            Picture.shade(ball, rgb, frame, hit, normal, v(patch));
        });
        return Picture.finish(b, rgb);
    }

    /**
     * Sphere {@code k}, squashed, met by the ray if it is: kept as the nearest, with its normal, if nearer than
     * {@code best}.
     *
     * <p>The squashed sphere is {@code c' + M u} for {@code |u| ≤ r}, with {@code M = s (I − g F)}: {@code F} the
     * flattening, {@code g} the gain, and {@code s = det(I − g F)^(−1/3)}, so the volume is the sphere's. The ray is
     * taken into {@code u}'s space by {@code M⁻¹}, where the shape is the sphere again and a quadratic finds it, and
     * the normal there, {@code u}, comes back as {@code M⁻¹ u}, since {@code M} is symmetric. {@code M⁻¹} is
     * {@code adj(I − g F) / det · s⁻¹}, and {@code s⁻¹}, the cube root of the determinant, two Newton steps from
     * {@code 1 − g tr F / 3}, which is right to first order already.
     *
     * <p>{@code c'} is the centre moved {@code lift · r} along the lean: squashed about its centre by more than the
     * overlap, a sphere would draw back from what it presses on, and is moved to meet it.
     */
    private static void squashed(Body b, LocalVar k, LocalVar base, LocalVar[] c, LocalVar r, LocalVar gain,
                                 LocalVar lift, LocalVar[] eye, LocalVar[] d, LocalVar alpha, LocalVar best,
                                 LocalVar hit, LocalVar[] normal, LocalVar[] local) {
        // I − g F, as xx, yy, zz, xy, xz, yz.
        LocalVar[] m = new LocalVar[6];
        for (int j = 0; j < 6; j++) {
            Expr pressed = mul(v(gain), pressing(base, Spheres.FLATTENING + j, alpha));
            m[j] = b.let("m", j < 3 ? sub(f(1), pressed) : neg(pressed));
        }
        LocalVar[] adj = {
                b.let("adj", sub(mul(v(m[1]), v(m[2])), mul(v(m[5]), v(m[5])))),
                b.let("adj", sub(mul(v(m[0]), v(m[2])), mul(v(m[4]), v(m[4])))),
                b.let("adj", sub(mul(v(m[0]), v(m[1])), mul(v(m[3]), v(m[3])))),
                b.let("adj", sub(mul(v(m[4]), v(m[5])), mul(v(m[3]), v(m[2])))),
                b.let("adj", sub(mul(v(m[3]), v(m[5])), mul(v(m[4]), v(m[1])))),
                b.let("adj", sub(mul(v(m[3]), v(m[4])), mul(v(m[0]), v(m[5]))))};
        LocalVar det = b.let("det", add(mul(v(m[0]), v(adj[0])), add(mul(v(m[3]), v(adj[3])),
                mul(v(m[4]), v(adj[4])))));
        LocalVar cube = b.let("cube", sub(f(1), div(sub(f(3), add(v(m[0]), add(v(m[1]), v(m[2])))), f(3))));
        for (int step = 0; step < 2; step++) {
            b.set(cube, div(add(mul(f(2), v(cube)), div(v(det), mul(v(cube), v(cube)))), f(3)));
        }
        LocalVar scale = b.let("scale", div(v(cube), v(det)));
        LocalVar[] inverse = new LocalVar[6];
        for (int j = 0; j < 6; j++) {
            inverse[j] = b.let("inv", mul(v(adj[j]), v(scale)));
        }

        LocalVar[] o = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            Expr moved = add(v(c[a]), mul(mul(v(lift), v(r)), pressing(base, Spheres.LEAN + a, alpha)));
            o[a] = b.let("o", sub(v(eye[a]), moved));
        }
        LocalVar[] os = times(b, inverse, o);
        LocalVar[] ds = times(b, inverse, d);
        LocalVar aa = b.let("aa", dot(ds, ds));
        LocalVar hh = b.let("hh", dot(os, ds));
        LocalVar disc = b.let("disc", sub(mul(v(hh), v(hh)), mul(v(aa), sub(dot(os, os), mul(v(r), v(r))))));
        b.when(gt(v(disc), f(0)), meets -> {
            LocalVar t = meets.let("t", div(sub(neg(v(hh)), sqrt(v(disc))), v(aa)));
            meets.when(and(gt(v(t), f(0)), lt(v(t), v(best))), nearer -> {
                nearer.set(best, v(t));
                nearer.set(hit, toFloat(v(k)));
                LocalVar[] u = new LocalVar[3];
                for (int a = 0; a < 3; a++) {
                    u[a] = nearer.let("u", add(v(os[a]), mul(v(t), v(ds[a]))));
                    nearer.set(local[a], v(u[a]));
                }
                LocalVar[] n = times(nearer, inverse, u);
                for (int a = 0; a < 3; a++) {
                    nearer.set(normal[a], v(n[a]));
                }
            });
        });
    }

    /**
     * Which of its eight octants the point {@code local} on sphere {@code hit} is in, in the sphere's own frame: the
     * product of its coordinates there, positive in four octants and negative in the other four.
     *
     * <p>The orientation is blended between the steps as the centre is, by normalised lerp, taken the short way round.
     * It is left unnormalised: turning by {@code q} scales by {@code |q|²}, which changes no sign. {@code local} is in
     * the sphere's space before it is squashed, so the patches stay put on the sphere however it is pressed.
     */
    private static Expr turned(Body b, LocalVar hit, LocalVar alpha, LocalVar[] local) {
        LocalVar base = b.let("tb", mul(toInt(v(hit)), i(Spheres.SHOWN_STRIDE)));
        LocalVar[] before = new LocalVar[4];
        LocalVar[] after = new LocalVar[4];
        for (int a = 0; a < 4; a++) {
            before[a] = b.let("q0", load(SHOWN, add(v(base), i(Spheres.TURNED_BEFORE + a))));
            after[a] = b.let("q1", load(SHOWN, add(v(base), i(Spheres.TURNED_AFTER + a))));
        }
        LocalVar way = b.let("way", f(1));
        Expr aligned = add(dot(before, after), mul(v(before[3]), v(after[3])));
        b.when(lt(aligned, f(0)), back -> back.set(way, f(-1)));
        // The inverse turn: the conjugate, −(qx, qy, qz) and qw.
        LocalVar[] q = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            q[a] = b.let("q", neg(mix(mul(v(before[a]), v(way)), v(after[a]), v(alpha))));
        }
        LocalVar w = b.let("qw", mix(mul(v(before[3]), v(way)), v(after[3]), v(alpha)));
        // p + w t + q × t, with t = 2 q × p.
        LocalVar[] t = cross(b, q, local);
        for (int a = 0; a < 3; a++) {
            b.set(t[a], mul(f(2), v(t[a])));
        }
        LocalVar[] qt = cross(b, q, t);
        LocalVar[] p = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            p[a] = b.let("p", add(v(local[a]), add(mul(v(w), v(t[a])), v(qt[a]))));
        }
        return mul(mul(v(p[0]), v(p[1])), v(p[2]));
    }

    private static LocalVar[] cross(Body b, LocalVar[] x, LocalVar[] y) {
        return new LocalVar[] {
                b.let("cx", sub(mul(v(x[1]), v(y[2])), mul(v(x[2]), v(y[1])))),
                b.let("cy", sub(mul(v(x[2]), v(y[0])), mul(v(x[0]), v(y[2])))),
                b.let("cz", sub(mul(v(x[0]), v(y[1])), mul(v(x[1]), v(y[0]))))};
    }

    /** Word {@code word} of sphere {@code base}'s pressing, blended between the steps as its centre is. */
    private static Expr pressing(LocalVar base, int word, LocalVar alpha) {
        return mix(load(SHOWN, add(v(base), i(Spheres.PRESSED_BEFORE + word))),
                load(SHOWN, add(v(base), i(Spheres.PRESSED_AFTER + word))), v(alpha));
    }

    /** The symmetric {@code m}, as xx, yy, zz, xy, xz, yz, times {@code x}. */
    private static LocalVar[] times(Body b, LocalVar[] m, LocalVar[] x) {
        int[][] row = {{0, 3, 4}, {3, 1, 5}, {4, 5, 2}};
        LocalVar[] y = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            y[a] = b.let("mx", add(mul(v(m[row[a][0]]), v(x[0])), add(mul(v(m[row[a][1]]), v(x[1])),
                    mul(v(m[row[a][2]]), v(x[2])))));
        }
        return y;
    }

    private static Expr dot(LocalVar[] a, LocalVar[] b) {
        return add(add(mul(v(a[0]), v(b[0])), mul(v(a[1]), v(b[1]))), mul(v(a[2]), v(b[2])));
    }
}

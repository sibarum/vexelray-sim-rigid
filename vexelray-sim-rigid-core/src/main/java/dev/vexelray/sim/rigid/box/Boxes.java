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
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.neg;
import static dev.supirvast.vastir.build.Body.sqrt;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.v;

/**
 * Boxes: solid cuboids, free so far. The first stage of {@code docs/boxes.md}: a box's state, its inertia, and its
 * motion with nothing touching it.
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

    /** {@code [h, gx, gy, gz, sx, sy, sz]}, see {@link #params}. */
    public static final int PARAM_COUNT = 7;

    static final int H = 0;
    static final int GX = 1;
    static final int GY = 2;
    static final int GZ = 3;
    static final int SX = 4;
    static final int SY = 5;
    static final int SZ = 6;

    /** Every per-box field, in the order {@link BoxStep} holds them. */
    static final String[] BOX = {"x", "y", "z", "u", "v", "w", "lx", "ly", "lz", "qx", "qy", "qz", "qw", "im", "hx",
            "hy", "hz"};

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
        if (!(h > 0) || !(sx > 0) || !(sy > 0) || !(sz > 0)) {
            throw new IllegalArgumentException("h > 0 and a box of positive size, got h " + h + ", box " + sx + " × "
                    + sy + " × " + sz);
        }
        return new int[] {bits(h), bits(gx), bits(gy), bits(gz), bits(sx), bits(sy), bits(sz)};
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
            "qz", "qw", "hx", "hy", "hz"};
    static final List<Buffer> PREDICT_BUFFERS = bind(PREDICT_NAMES);

    /**
     * One invocation per box: gravity into the velocity, and the box moved along it, by symplectic Euler as a sphere
     * is; and the box turned, its angular momentum kept ({@link Boxes above}). The box is loaded once and stored once.
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
        int[] gravity = {GX, GY, GZ};

        Body b = new Body();
        LocalVar s = b.let("s", new Expr.InvocationId());
        b.when(lt(v(s), new Expr.InvocationCount()), t -> {
            LocalVar inverseMass = t.let("im", load(im, v(s)));
            t.when(gt(v(inverseMass), f(0)), free -> {
                LocalVar h = free.let("h", load(params, i(H)));
                for (int a = 0; a < 3; a++) {
                    LocalVar vel = free.let("vel", add(load(velocity[a], v(s)),
                            mul(v(h), load(params, i(gravity[a])))));
                    free.store(velocity[a], v(s), v(vel));
                    free.store(at[a], v(s), add(load(at[a], v(s)), mul(v(h), v(vel))));
                }

                LocalVar[] q = new LocalVar[4];
                for (int k = 0; k < 4; k++) {
                    q[k] = free.let("q", load(turn[k], v(s)));
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

    // --- building blocks -----------------------------------------------------------------------------------

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

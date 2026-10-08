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
 * <p>Coordinates are the simulation's: metres, {@code y} up, the box from the origin to {@code (sx, sy, sz)}. The
 * camera arrives as an eye and three axes, which the host works out from its orbit.
 */
final class SphereShader {

    /** Binding of the shown buffer at set 0; the host's binding must agree. */
    static final int SHOWN_BINDING = 0;

    /** The push-constant block, all f32, in this order. */
    private static final String[] MEMBERS = {"ex", "ey", "ez", "rx", "ry", "rz", "ux", "uy", "uz", "fx", "fy", "fz",
            "aspect", "alpha", "count", "sx", "sy", "sz"};

    static final int PUSH_BYTES = MEMBERS.length * Float.BYTES;

    /** Half the picture's height at unit distance: a vertical field of view of about 53°. */
    static final double TAN_HALF = 0.5;

    /** The floor's checker, in metres. */
    private static final double CHECKER = 0.05;

    /** From the light's side. Normalised by hand: (0.35, 0.85, 0.4) / |…|. */
    private static final double[] LIGHT = normalise(0.35, 0.85, 0.4);

    private static final PushConstants PUSH = block();
    private static final Buffer SHOWN = new Buffer("shown", SHOWN_BINDING, F32);
    private static final InterfaceVar UV = InterfaceVar.input("vUv", 0, VEC2);
    private static final InterfaceVar COLOR = InterfaceVar.output("fragColor", 0, VEC4);

    private static final double[] SKY_LOW = {0.16, 0.17, 0.19};
    private static final double[] SKY_HIGH = {0.05, 0.06, 0.08};
    private static final double[] FLOOR_A = {0.30, 0.31, 0.33};
    private static final double[] FLOOR_B = {0.24, 0.25, 0.27};
    private static final double[] BALL_A = {0.90, 0.55, 0.22};
    private static final double[] BALL_B = {0.30, 0.62, 0.86};

    private SphereShader() {
    }

    /** The fragment stage as SPIR-V. Pair it with {@code Fullscreen.triangleVertexWithUvSpirv()}. */
    static byte[] fragmentSpirv() {
        Body b = new Body();
        LocalVar uv = b.let("uv", input(UV));
        LocalVar[] eye = {b.let("ex", pushed(PUSH, 0)), b.let("ey", pushed(PUSH, 1)), b.let("ez", pushed(PUSH, 2))};
        LocalVar aspect = b.let("aspect", pushed(PUSH, 12));
        LocalVar alpha = b.let("alpha", pushed(PUSH, 13));
        LocalVar count = b.let("count", toInt(pushed(PUSH, 14)));
        LocalVar sx = b.let("sx", pushed(PUSH, 15));
        LocalVar sz = b.let("sz", pushed(PUSH, 17));

        // The ray: forward, plus right and up by where the pixel is. The picture's top row is uv.y = 0.
        LocalVar px = b.let("px", mul(mul(sub(mul(component(v(uv), 0), f(2)), f(1)), v(aspect)), f(TAN_HALF)));
        LocalVar py = b.let("py", mul(sub(f(1), mul(component(v(uv), 1), f(2))), f(TAN_HALF)));
        LocalVar[] d = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            d[a] = b.let("d", add(pushed(PUSH, 9 + a), add(mul(pushed(PUSH, 3 + a), v(px)), mul(pushed(PUSH, 6 + a), v(py)))));
        }
        LocalVar inverse = b.let("inverse", div(f(1), sqrt(dot(d, d))));
        for (int a = 0; a < 3; a++) {
            b.set(d[a], mul(v(d[a]), v(inverse)));
        }

        // The nearest sphere the ray meets: |e + t d − c|² = r², the smaller root, in front of the eye.
        LocalVar best = b.let("best", f(1e30));
        LocalVar hit = b.let("hit", f(-1));
        LocalVar[] centre = {b.let("hx", f(0)), b.let("hy", f(0)), b.let("hz", f(0))};
        LocalVar k = b.let("k", i(0));
        b.loop(lt(v(k), v(count)), pass -> {
            LocalVar base = pass.let("base", mul(v(k), i(Spheres.SHOWN_STRIDE)));
            LocalVar[] c = new LocalVar[3];
            LocalVar[] o = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                c[a] = pass.let("c", mix(load(SHOWN, add(v(base), i(a))), load(SHOWN, add(v(base), i(4 + a))), v(alpha)));
                o[a] = pass.let("o", sub(v(eye[a]), v(c[a])));
            }
            LocalVar r = pass.let("r", load(SHOWN, add(v(base), i(3))));
            LocalVar half = pass.let("half", dot(o, d));
            LocalVar disc = pass.let("disc", sub(mul(v(half), v(half)), sub(dot(o, o), mul(v(r), v(r)))));
            pass.when(gt(v(disc), f(0)), meets -> {
                LocalVar t = meets.let("t", sub(neg(v(half)), sqrt(v(disc))));
                meets.when(and(gt(v(t), f(0)), lt(v(t), v(best))), nearer -> {
                    nearer.set(best, v(t));
                    nearer.set(hit, toFloat(v(k)));
                    for (int a = 0; a < 3; a++) {
                        nearer.set(centre[a], v(c[a]));
                    }
                });
            });
            pass.set(k, add(v(k), i(1)));
        });

        // Sky, darker overhead.
        LocalVar up = b.let("up", clamp(add(mul(v(d[1]), f(0.5)), f(0.5)), f(0), f(1)));
        LocalVar[] rgb = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            rgb[a] = b.let("rgb", mix(f(SKY_LOW[a]), f(SKY_HIGH[a]), v(up)));
        }

        // The floor, inside the box, where the ray reaches it before any sphere.
        b.when(lt(v(d[1]), f(-1e-4)), down -> {
            LocalVar t = down.let("tf", div(neg(v(eye[1])), v(d[1])));
            down.when(lt(v(t), v(best)), first -> {
                LocalVar fx = first.let("fx", add(v(eye[0]), mul(v(t), v(d[0]))));
                LocalVar fz = first.let("fz", add(v(eye[2]), mul(v(t), v(d[2]))));
                Expr inside = and(and(gt(v(fx), f(0)), lt(v(fx), v(sx))), and(gt(v(fz), f(0)), lt(v(fz), v(sz))));
                first.when(inside, ground -> {
                    LocalVar s = ground.let("s", add(floor(div(v(fx), f(CHECKER))), floor(div(v(fz), f(CHECKER)))));
                    LocalVar odd = ground.let("odd", sub(v(s), mul(f(2), floor(div(v(s), f(2))))));
                    for (int a = 0; a < 3; a++) {
                        ground.set(rgb[a], mix(f(FLOOR_A[a]), f(FLOOR_B[a]), v(odd)));
                    }
                });
            });
        });

        // A sphere: lit from one side, with a little from everywhere, and a highlight.
        b.when(gt(v(hit), f(-0.5)), ball -> {
            LocalVar[] n = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                n[a] = ball.let("n", sub(add(v(eye[a]), mul(v(best), v(d[a]))), v(centre[a])));
            }
            LocalVar nl = ball.let("nl", div(f(1), sqrt(dot(n, n))));
            for (int a = 0; a < 3; a++) {
                ball.set(n[a], mul(v(n[a]), v(nl)));
            }
            LocalVar diffuse = ball.let("diffuse", max(f(0), add(add(mul(v(n[0]), f(LIGHT[0])), mul(v(n[1]), f(LIGHT[1]))),
                    mul(v(n[2]), f(LIGHT[2])))));
            // Blinn's half vector between the light and the eye, which is −d.
            LocalVar[] h = new LocalVar[3];
            for (int a = 0; a < 3; a++) {
                h[a] = ball.let("h", sub(f(LIGHT[a]), v(d[a])));
            }
            LocalVar hl = ball.let("hl", div(f(1), sqrt(dot(h, h))));
            LocalVar nh = ball.let("nh", max(f(0), mul(dot(n, h), v(hl))));
            LocalVar nh2 = ball.let("nh2", mul(v(nh), v(nh)));
            LocalVar nh8 = ball.let("nh8", mul(mul(v(nh2), v(nh2)), mul(v(nh2), v(nh2))));
            LocalVar spec = ball.let("spec", mul(f(0.35), mul(v(nh8), mul(v(nh8), v(nh8)))));
            // Two colours, mixed by the golden ratio of the index, so neighbours differ and nothing is random.
            LocalVar g = ball.let("g", mul(v(hit), f(0.618034)));
            LocalVar tint = ball.let("tint", sub(v(g), floor(v(g))));
            LocalVar light = ball.let("light", add(f(0.22), mul(f(0.78), v(diffuse))));
            for (int a = 0; a < 3; a++) {
                ball.set(rgb[a], add(mul(mix(f(BALL_A[a]), f(BALL_B[a]), v(tint)), v(light)), v(spec)));
            }
        });

        b.write(COLOR, vec4(vec3(v(rgb[0]), v(rgb[1]), v(rgb[2])), f(1)));
        Function main = new Function("main", new Type.FunctionType(Type.VOID, List.of()), b.finish());
        return new CoreToSpirv().lower(new CoreModule().addEntryPoint(EntryPoint.of(main, ShaderStage.FRAGMENT)))
                .toByteArray();
    }

    /**
     * The push constants for one frame, as the bytes the pipeline takes.
     *
     * @param eye     where the eye is, in the simulation's coordinates
     * @param right   the camera's right, unit length; likewise {@code up} and {@code forward}
     * @param alpha   how far between the last two steps to show the spheres, 0 to 1
     * @param extent  the box, {@code sx, sy, sz}
     */
    static byte[] push(double[] eye, double[] right, double[] up, double[] forward, double aspect, float alpha,
                       int count, double[] extent) {
        ByteBuffer bytes = ByteBuffer.allocate(PUSH_BYTES).order(ByteOrder.LITTLE_ENDIAN);
        for (double[] vector : new double[][] {eye, right, up, forward}) {
            for (double component : vector) {
                bytes.putFloat((float) component);
            }
        }
        bytes.putFloat((float) aspect).putFloat(alpha).putFloat(count);
        for (double e : extent) {
            bytes.putFloat((float) e);
        }
        return bytes.array();
    }

    private static Expr dot(LocalVar[] a, LocalVar[] b) {
        return add(add(mul(v(a[0]), v(b[0])), mul(v(a[1]), v(b[1]))), mul(v(a[2]), v(b[2])));
    }

    private static double[] normalise(double x, double y, double z) {
        double length = Math.sqrt(x * x + y * y + z * z);
        return new double[] {x / length, y / length, z / length};
    }

    private static PushConstants block() {
        List<PushConstants.Member> members = new ArrayList<>();
        for (String name : MEMBERS) {
            members.add(new PushConstants.Member(name, F32));
        }
        return new PushConstants(members);
    }
}

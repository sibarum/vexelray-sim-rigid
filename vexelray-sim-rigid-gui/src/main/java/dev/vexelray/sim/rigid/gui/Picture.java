package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.build.Body;
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
import static dev.supirvast.vastir.build.Body.input;
import static dev.supirvast.vastir.build.Body.lt;
import static dev.supirvast.vastir.build.Body.max;
import static dev.supirvast.vastir.build.Body.mix;
import static dev.supirvast.vastir.build.Body.mul;
import static dev.supirvast.vastir.build.Body.neg;
import static dev.supirvast.vastir.build.Body.pushed;
import static dev.supirvast.vastir.build.Body.sqrt;
import static dev.supirvast.vastir.build.Body.sub;
import static dev.supirvast.vastir.build.Body.toInt;
import static dev.supirvast.vastir.build.Body.v;
import static dev.supirvast.vastir.build.Body.vec3;
import static dev.supirvast.vastir.build.Body.vec4;

/**
 * What every body's picture shares, whatever its bodies are: the camera's ray, the sky and the checkered floor, the
 * light, and the push constants that carry them. {@link SphereShader} and {@link BoxShader} each add how a ray meets
 * their bodies.
 */
final class Picture {

    /** The push-constant block, all f32, in this order. */
    private static final String[] MEMBERS = {"ex", "ey", "ez", "rx", "ry", "rz", "ux", "uy", "uz", "fx", "fy", "fz",
            "aspect", "alpha", "count", "sx", "sy", "sz", "squash"};

    static final int PUSH_BYTES = MEMBERS.length * Float.BYTES;

    /** Half the picture's height at unit distance: a vertical field of view of about 53°. */
    static final double TAN_HALF = 0.5;

    static final PushConstants PUSH = block();
    private static final InterfaceVar UV = InterfaceVar.input("vUv", 0, VEC2);
    private static final InterfaceVar COLOR = InterfaceVar.output("fragColor", 0, VEC4);

    /** The floor's checker, in metres. */
    private static final double CHECKER = 0.05;
    /** From the light's side. Normalised by hand: (0.35, 0.85, 0.4) / |…|. */
    private static final double[] LIGHT = normalise(0.35, 0.85, 0.4);

    private static final double[] SKY_LOW = {0.16, 0.17, 0.19};
    private static final double[] SKY_HIGH = {0.05, 0.06, 0.08};
    private static final double[] FLOOR_A = {0.30, 0.31, 0.33};
    private static final double[] FLOOR_B = {0.24, 0.25, 0.27};
    private static final double[] BODY_A = {0.90, 0.55, 0.22};
    private static final double[] BODY_B = {0.30, 0.62, 0.86};

    /** One frame's camera and settings, as a fragment reads them from the push constants. */
    static final class Frame {
        final LocalVar[] eye;
        /** The ray through this pixel, unit length. */
        final LocalVar[] d = new LocalVar[3];
        final LocalVar alpha;
        final LocalVar count;
        final LocalVar sx;
        final LocalVar sz;

        Frame(Body b) {
            LocalVar uv = b.let("uv", input(UV));
            eye = new LocalVar[] {b.let("ex", pushed(PUSH, 0)), b.let("ey", pushed(PUSH, 1)),
                    b.let("ez", pushed(PUSH, 2))};
            LocalVar aspect = b.let("aspect", pushed(PUSH, 12));
            alpha = b.let("alpha", pushed(PUSH, 13));
            count = b.let("count", toInt(pushed(PUSH, 14)));
            sx = b.let("sx", pushed(PUSH, 15));
            sz = b.let("sz", pushed(PUSH, 17));
            // Forward, plus right and up by where the pixel is. The picture's top row is uv.y = 0.
            LocalVar px = b.let("px", mul(mul(sub(mul(component(v(uv), 0), f(2)), f(1)), v(aspect)), f(TAN_HALF)));
            LocalVar py = b.let("py", mul(sub(f(1), mul(component(v(uv), 1), f(2))), f(TAN_HALF)));
            for (int a = 0; a < 3; a++) {
                d[a] = b.let("d", add(pushed(PUSH, 9 + a), add(mul(pushed(PUSH, 3 + a), v(px)),
                        mul(pushed(PUSH, 6 + a), v(py)))));
            }
            LocalVar inverse = b.let("inverse", div(f(1), sqrt(dot(d, d))));
            for (int a = 0; a < 3; a++) {
                b.set(d[a], mul(v(d[a]), v(inverse)));
            }
        }

        /** The push constant {@code squash}, which only spheres read. */
        Expr squash() {
            return pushed(PUSH, 18);
        }
    }

    private Picture() {
    }

    /**
     * What the ray sees past every body: sky, darker overhead, and the checkered floor inside the box where the ray
     * reaches it before {@code best}.
     */
    static LocalVar[] background(Body b, Frame frame, LocalVar best) {
        LocalVar[] d = frame.d;
        LocalVar[] eye = frame.eye;
        LocalVar up = b.let("up", clamp(add(mul(v(d[1]), f(0.5)), f(0.5)), f(0), f(1)));
        LocalVar[] rgb = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            rgb[a] = b.let("rgb", mix(f(SKY_LOW[a]), f(SKY_HIGH[a]), v(up)));
        }
        b.when(lt(v(d[1]), f(-1e-4)), down -> {
            LocalVar t = down.let("tf", div(neg(v(eye[1])), v(d[1])));
            down.when(lt(v(t), v(best)), first -> {
                LocalVar fx = first.let("fx", add(v(eye[0]), mul(v(t), v(d[0]))));
                LocalVar fz = first.let("fz", add(v(eye[2]), mul(v(t), v(d[2]))));
                Expr inside = and(and(gt(v(fx), f(0)), lt(v(fx), v(frame.sx))),
                        and(gt(v(fz), f(0)), lt(v(fz), v(frame.sz))));
                first.when(inside, ground -> {
                    LocalVar s = ground.let("s", add(floor(div(v(fx), f(CHECKER))), floor(div(v(fz), f(CHECKER)))));
                    LocalVar odd = ground.let("odd", sub(v(s), mul(f(2), floor(div(v(s), f(2))))));
                    for (int a = 0; a < 3; a++) {
                        ground.set(rgb[a], mix(f(FLOOR_A[a]), f(FLOOR_B[a]), v(odd)));
                    }
                });
            });
        });
        return rgb;
    }

    /**
     * Body {@code hit} where the ray met it, with outward normal {@code normal} (normalised here): lit from one side,
     * with a little from everywhere, and a highlight. Its colour is one of two, mixed by the golden ratio of its index
     * so that neighbours differ and nothing is random, then {@code pale} of the way toward white.
     */
    static void shade(Body b, LocalVar[] rgb, Frame frame, LocalVar hit, LocalVar[] normal, Expr pale) {
        LocalVar[] n = normal;
        LocalVar[] d = frame.d;
        LocalVar nl = b.let("nl", div(f(1), sqrt(dot(n, n))));
        for (int a = 0; a < 3; a++) {
            b.set(n[a], mul(v(n[a]), v(nl)));
        }
        LocalVar diffuse = b.let("diffuse", max(f(0), add(add(mul(v(n[0]), f(LIGHT[0])), mul(v(n[1]), f(LIGHT[1]))),
                mul(v(n[2]), f(LIGHT[2])))));
        // Blinn's half vector between the light and the eye, which is −d.
        LocalVar[] h = new LocalVar[3];
        for (int a = 0; a < 3; a++) {
            h[a] = b.let("h", sub(f(LIGHT[a]), v(d[a])));
        }
        LocalVar hl = b.let("hl", div(f(1), sqrt(dot(h, h))));
        LocalVar nh = b.let("nh", max(f(0), mul(dot(n, h), v(hl))));
        LocalVar nh2 = b.let("nh2", mul(v(nh), v(nh)));
        LocalVar nh8 = b.let("nh8", mul(mul(v(nh2), v(nh2)), mul(v(nh2), v(nh2))));
        LocalVar spec = b.let("spec", mul(f(0.35), mul(v(nh8), mul(v(nh8), v(nh8)))));
        LocalVar g = b.let("g", mul(v(hit), f(0.618034)));
        LocalVar tint = b.let("tint", sub(v(g), floor(v(g))));
        LocalVar light = b.let("light", add(f(0.22), mul(f(0.78), v(diffuse))));
        LocalVar toward = b.let("toward", pale);
        for (int a = 0; a < 3; a++) {
            Expr colour = mix(mix(f(BODY_A[a]), f(BODY_B[a]), v(tint)), f(1), v(toward));
            b.set(rgb[a], add(mul(colour, v(light)), v(spec)));
        }
    }

    /** The colour written, and the fragment stage as SPIR-V. Pair it with {@code Fullscreen.triangleVertexWithUvSpirv()}. */
    static byte[] finish(Body b, LocalVar[] rgb) {
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
     * @param alpha   how far between the last two steps to show the bodies, 0 to 1
     * @param count   how many bodies
     * @param extent  the box, {@code sx, sy, sz}
     * @param squash  how much a sphere is squashed by what presses it: 0 for not at all, 1.5 for its overlaps just
     *                closed, more to exaggerate. Boxes are not squashed.
     */
    static byte[] push(double[] eye, double[] right, double[] up, double[] forward, double aspect, float alpha,
                       int count, double[] extent, double squash) {
        ByteBuffer bytes = ByteBuffer.allocate(PUSH_BYTES).order(ByteOrder.LITTLE_ENDIAN);
        for (double[] vector : new double[][] {eye, right, up, forward}) {
            for (double value : vector) {
                bytes.putFloat((float) value);
            }
        }
        bytes.putFloat((float) aspect).putFloat(alpha).putFloat(count);
        for (double e : extent) {
            bytes.putFloat((float) e);
        }
        bytes.putFloat((float) squash);
        return bytes.array();
    }

    static Expr dot(LocalVar[] a, LocalVar[] b) {
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

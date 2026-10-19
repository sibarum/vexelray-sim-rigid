package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.pass.BufferSpec;
import dev.supirvast.vastir.pass.Buffered;
import dev.supirvast.vastir.pass.Pass;
import dev.supirvast.vastir.tools.Accelerator;
import dev.supirvast.vastir.tools.PassRunner;
import dev.vexelray.sim.rigid.box.BoxScene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * The narrow phase against cases worked out by hand: a pair's manifold, its normal, which axis won, and each point on
 * each box in the world's frame, read back from the anchors.
 *
 * <p>The boxes throughout: a slab 0.6 × 0.2 × 0.6 m, half-extents (0.3, 0.1, 0.3), at the origin, its top at
 * {@code y = 0.1}; and cubes of half-extent 0.1.
 */
class BoxContactTest {

    private static final double[] SLAB = {0.3, 0.1, 0.3};
    private static final double[] CUBE = {0.1, 0.1, 0.1};
    private static final double[] IDENTITY = {0, 0, 0, 1};
    private static final double R2 = Math.sqrt(2);
    /** f32's rounding of positions near 0.3 m, through a few products. */
    private static final double CLOSE = 2e-6;

    /** A cube beside another, 5 cm apart: no contact. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void boxesApartHaveNoContact(Backend backend) {
        float[] m = collide(backend, 0, box(0, 0, 0, CUBE, IDENTITY), box(0.25, 0, 0, CUBE, IDENTITY));
        assertEquals(0, m[BoxContact.COUNT], "count");
    }

    /**
     * A cube resting 1 cm into the slab: the slab's top face is the reference, and the four points are the cube's
     * bottom corners, each 1 cm deep, the normal up from the slab to the cube.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aCubeOnASlabTouchesAtItsFourCorners(Backend backend) {
        float[] m = collide(backend, 0, box(0, 0, 0, SLAB, IDENTITY), box(0, 0.19, 0, CUBE, IDENTITY));
        Contact c = new Contact(m, box(0, 0, 0, SLAB, IDENTITY), box(0, 0.19, 0, CUBE, IDENTITY));
        c.face(1, 0, 1, 0);
        c.points(0.01, corners(0.1, 0.1, 0.1, 0.09));
    }

    /**
     * A cube on a cube the same size: every corner of each face lies exactly on the other's edge, the case a column
     * of boxes is made of. Still the four corners.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aCubeOnACubeTheSameSizeTouchesAtItsFourCorners(Backend backend) {
        Solid below = box(0, 0, 0, CUBE, IDENTITY);
        Solid above = box(0, 0.19, 0, CUBE, IDENTITY);
        Contact c = new Contact(collide(backend, 0, below, above), below, above);
        c.face(1, 0, 1, 0);
        c.points(0.01, corners(0.1, 0.1, 0.1, 0.09));
    }

    /** A cube turned 45° about the vertical on the slab: its bottom corners, a diamond, each 1 cm deep. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aTurnedCubeTouchesAtItsCorners(Backend backend) {
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid cube = box(0, 0.19, 0, CUBE, axisAngle(0, 1, 0, Math.PI / 4));
        Contact c = new Contact(collide(backend, 0, slab, cube), slab, cube);
        c.face(1, 0, 1, 0);
        double d = 0.1 * R2;
        c.points(0.01, new double[][] {{d, 0.09, 0}, {-d, 0.09, 0}, {0, 0.09, d}, {0, 0.09, -d}});
    }

    /**
     * A cube hanging over the slab's edge, its bottom from x = 0.15 to 0.35 where the slab ends at 0.3: the region
     * is clipped there, and two of the points are where the cube's bottom edges cross the slab's.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aCubeOverAnEdgeIsClippedThere(Backend backend) {
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid cube = box(0.25, 0.19, 0, CUBE, IDENTITY);
        Contact c = new Contact(collide(backend, 0, slab, cube), slab, cube);
        c.face(1, 0, 1, 0);
        c.points(0.01, new double[][] {{0.15, 0.09, 0.1}, {0.15, 0.09, -0.1}, {0.3, 0.09, 0.1}, {0.3, 0.09, -0.1}});
    }

    /**
     * The turned cube centred over the slab's edge: half its diamond is clipped away, leaving a triangle, so three
     * points: its far corner, and where its two edges cross the slab's edge.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aTurnedCubeOverAnEdgeTouchesAtThreePoints(Backend backend) {
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid cube = box(0.3, 0.19, 0, CUBE, axisAngle(0, 1, 0, Math.PI / 4));
        Contact c = new Contact(collide(backend, 0, slab, cube), slab, cube);
        c.face(1, 0, 1, 0);
        double d = 0.1 * R2;
        c.points(0.01, new double[][] {{0.3 - d, 0.09, 0}, {0.3, 0.09, d}, {0.3, 0.09, -d}});
    }

    /**
     * The cube on the slab with the pair the other way round: the cube is first, its bottom face the reference, the
     * normal down from it to the slab, and the points are the cube's face's corners, seen on the slab's top.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void thePairTheOtherWayRoundTurnsTheNormal(Backend backend) {
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid cube = box(0, 0.19, 0, CUBE, IDENTITY);
        Contact c = new Contact(collide(backend, 0, cube, slab), cube, slab);
        c.face(1, 0, -1, 0);
        c.pointsOn(0.01, corners(0.1, 0.1, 0.1, 0.09), corners(0.1, 0.1, 0.1, 0.1));
    }

    /**
     * A cube turned 45° about z, an edge along z on top, and above it one turned 45° about x, an edge along x below,
     * 1 cm into each other: the edges cross, and the one point is where, on each.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void crossedEdgesTouchAtOnePoint(Backend backend) {
        double top = 0.1 * R2;
        Solid below = box(0, 0, 0, CUBE, axisAngle(0, 0, 1, Math.PI / 4));
        Solid above = box(0, 2 * top - 0.01, 0, CUBE, axisAngle(1, 0, 0, Math.PI / 4));
        float[] m = collide(backend, 0, below, above);
        Contact c = new Contact(m, below, above);
        assertEquals(6 + 3 * 2, m[BoxContact.AXIS], "the cross of the first's z edge and the second's x edge");
        c.normal(0, 1, 0);
        c.pointsOn(0.01, new double[][] {{0, top, 0}}, new double[][] {{0, top - 0.01, 0}});
    }

    /** A cube corner-down, 1 cm into the slab: one point, the corner. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aCornerIntoAFaceTouchesAtTheCorner(Backend backend) {
        double tilt = Math.acos(1 / Math.sqrt(3));
        double reach = 0.1 * Math.sqrt(3);
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid cube = box(0, 0.1 + reach - 0.01, 0, CUBE, axisAngle(-1, 0, 1, tilt));
        Contact c = new Contact(collide(backend, 0, slab, cube), slab, cube);
        c.face(1, 0, 1, 0);
        c.pointsOn(0.01, new double[][] {{0, 0.1, 0}}, new double[][] {{0, 0.09, 0}});
    }

    /**
     * A cube beside another, turned 45° about z and set off diagonally: their shadows overlap along each of the
     * first's axes, but the second's axis (1, 1, 0) / √2 parts them, by 0.098 m. No contact.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void anAxisOfTheSecondPartsThem(Backend backend) {
        float[] m = collide(backend, 0, box(0, 0, 0, CUBE, IDENTITY),
                box(0.24, 0.24, 0, CUBE, axisAngle(0, 0, 1, Math.PI / 4)));
        assertEquals(0, m[BoxContact.COUNT], "count");
    }

    /**
     * The cube 5 mm above the slab: no contact with no margin, and with a margin of 1 cm, its four corners at a depth
     * of −5 mm.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aMarginFindsBoxesAboutToTouch(Backend backend) {
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid cube = box(0, 0.205, 0, CUBE, IDENTITY);
        assertEquals(0, collide(backend, 0, slab, cube)[BoxContact.COUNT], "count with no margin");
        Contact c = new Contact(collide(backend, 0.01, slab, cube), slab, cube);
        c.face(1, 0, 1, 0);
        c.points(-0.005, corners(0.1, 0.1, 0.1, 0.105));
    }

    /**
     * 500 pairs of boxes of random sizes, turns and places, near enough that most touch: wherever a manifold is
     * found, each point's anchor lies on its own box's surface, inside every slab and on at least one face, to
     * 1e-5 m; its depth is the two anchors' gap along the normal, to 1e-5 m; and no two points of a manifold are one.
     * And a pair whose inner spheres overlap, which must touch, has at least one point.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void everyPointLiesOnBothBoxesAtItsDepth(Backend backend) {
        java.util.Random random = new java.util.Random(19);
        int pairs = 500;
        Solid[] boxes = new Solid[2 * pairs];
        for (int k = 0; k < boxes.length; k++) {
            double[] half = {0.03 + 0.12 * random.nextDouble(), 0.03 + 0.12 * random.nextDouble(),
                    0.03 + 0.12 * random.nextDouble()};
            double spread = k % 2 == 0 ? 0 : 0.3;
            boxes[k] = box(spread * (random.nextDouble() - 0.5), spread * (random.nextDouble() - 0.5),
                    spread * (random.nextDouble() - 0.5), half, axisAngle(random.nextDouble() - 0.5,
                            random.nextDouble() - 0.5, random.nextDouble() - 0.5, 4 * random.nextDouble()));
        }
        float[] all = collide(backend, 0, boxes);
        int touching = 0;
        int points = 0;
        for (int p = 0; p < pairs; p++) {
            float[] m = java.util.Arrays.copyOfRange(all, p * BoxContact.MANIFOLD_WORDS,
                    (p + 1) * BoxContact.MANIFOLD_WORDS);
            Solid first = boxes[2 * p];
            Solid second = boxes[2 * p + 1];
            int count = (int) m[BoxContact.COUNT];
            assertTrue(count <= BoxContact.MOST_POINTS, "pair " + p + ": " + count + " points");
            double inner = Math.min(first.half[0], Math.min(first.half[1], first.half[2]))
                    + Math.min(second.half[0], Math.min(second.half[1], second.half[2]));
            double apartCentres = Math.hypot(Math.hypot(second.centre[0] - first.centre[0],
                    second.centre[1] - first.centre[1]), second.centre[2] - first.centre[2]);
            assertTrue(apartCentres >= inner || count > 0, "pair " + p + ": the spheres inside the boxes overlap,"
                    + " and no point was found");
            touching += count > 0 ? 1 : 0;
            points += count;
            double[] n = {m[BoxContact.NORMAL], m[BoxContact.NORMAL + 1], m[BoxContact.NORMAL + 2]};
            double[][] seen = new double[count][];
            for (int k = 0; k < count; k++) {
                int at = BoxContact.POINTS + k * BoxContact.POINT_WORDS;
                onSurface(m, at + BoxContact.ANCHOR_FIRST, first.half, "pair " + p + ", point " + k + " on the first");
                onSurface(m, at + BoxContact.ANCHOR_SECOND, second.half,
                        "pair " + p + ", point " + k + " on the second");
                double[] a = first.world(m, at + BoxContact.ANCHOR_FIRST);
                double[] b = second.world(m, at + BoxContact.ANCHOR_SECOND);
                double gap = (a[0] - b[0]) * n[0] + (a[1] - b[1]) * n[1] + (a[2] - b[2]) * n[2];
                assertEquals(gap, m[at + BoxContact.DEPTH], 1e-5, "pair " + p + ", point " + k + "'s depth");
                for (int j = 0; j < k; j++) {
                    double apart = Math.hypot(Math.hypot(b[0] - seen[j][0], b[1] - seen[j][1]), b[2] - seen[j][2]);
                    assertTrue(apart > 1e-6, "pair " + p + ": points " + j + " and " + k + " are one");
                }
                seen[k] = b;
            }
        }
        System.out.printf("random pairs on %s: %d of %d touch, %d points%n", backend, touching, pairs, points);
        assertTrue(touching > pairs / 2, "most pairs touch, so the test is not of empty manifolds: " + touching);
    }

    /** An anchor within its box's half-extents on every axis, and on one face, to 1e-5 m. */
    private static void onSurface(float[] m, int at, double[] half, String what) {
        double closest = Double.MAX_VALUE;
        for (int a = 0; a < 3; a++) {
            double inside = half[a] - Math.abs(m[at + a]);
            assertTrue(inside > -1e-5, what + ": outside along " + a + " by " + -inside);
            closest = Math.min(closest, inside);
        }
        assertTrue(closest < 1e-5, what + ": not on a face, " + closest + " m inside");
    }

    // --- spheres --------------------------------------------------------------------------------------------

    /** A ball of 5 cm resting 1 cm into the slab's top: one point, straight up, under its centre. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBallOnASlabTouchesUnderItsCentre(Backend backend) {
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid ball = ball(0, 0.14, 0, 0.05);
        Contact c = new Contact(collide(backend, 0, slab, ball), slab, ball);
        c.face(BoxContact.SPHERE_BOX, 0, 1, 0);
        c.pointsOn(0.01, new double[][] {{0, 0.1, 0}}, new double[][] {{0, 0.09, 0}});
    }

    /** The same pair the other way round: the normal down, from the ball to the slab. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBallFirstTurnsTheNormal(Backend backend) {
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid ball = ball(0, 0.14, 0, 0.05);
        Contact c = new Contact(collide(backend, 0, ball, slab), ball, slab);
        c.face(BoxContact.SPHERE_BOX, 0, -1, 0);
        c.pointsOn(0.01, new double[][] {{0, 0.09, 0}}, new double[][] {{0, 0.1, 0}});
    }

    /**
     * A ball over the slab's edge, its centre 3 cm out and 3 cm up from it: the nearest point is on the edge, the
     * normal at 45°, the depth the radius less √0.0018.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBallOverAnEdgeTouchesTheEdge(Backend backend) {
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid ball = ball(0.33, 0.13, 0, 0.05);
        Contact c = new Contact(collide(backend, 0, slab, ball), slab, ball);
        double d = 1 / R2;
        c.face(BoxContact.SPHERE_BOX, d, d, 0);
        double depth = 0.05 - 0.03 * R2;
        c.pointsOn(depth, new double[][] {{0.3, 0.1, 0}}, new double[][] {{0.33 - 0.05 * d, 0.13 - 0.05 * d, 0}});
    }

    /** A ball whose centre is 1 cm inside the slab's top: pushed out through the top, 6 cm deep. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBallWithItsCentreInsideIsPushedOutTheNearestFace(Backend backend) {
        Solid slab = box(0, 0, 0, SLAB, IDENTITY);
        Solid ball = ball(0.1, 0.09, 0.05, 0.05);
        Contact c = new Contact(collide(backend, 0, slab, ball), slab, ball);
        c.face(BoxContact.SPHERE_BOX, 0, 1, 0);
        c.pointsOn(0.06, new double[][] {{0.1, 0.1, 0.05}}, new double[][] {{0.1, 0.04, 0.05}});
    }

    /** Two balls, 5 and 4 cm, their centres 8 cm apart: 1 cm deep, along the line between them. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void twoBallsTouchOnTheLineBetweenThem(Backend backend) {
        Solid first = ball(0, 0, 0, 0.05);
        Solid second = ball(0.08, 0, 0, 0.04);
        Contact c = new Contact(collide(backend, 0, first, second), first, second);
        c.face(BoxContact.SPHERE_SPHERE, 1, 0, 0);
        c.pointsOn(0.01, new double[][] {{0.05, 0, 0}}, new double[][] {{0.04, 0, 0}});
    }

    /** A ball 5 cm above the slab: no contact with no margin. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBallApartHasNoContact(Backend backend) {
        assertEquals(0, collide(backend, 0, box(0, 0, 0, SLAB, IDENTITY), ball(0, 0.2, 0, 0.05))[BoxContact.COUNT],
                "count");
    }

    private static Solid ball(double x, double y, double z, double r) {
        return new Solid(new double[] {x, y, z}, new double[] {r, r, r}, IDENTITY, Boxes.SPHERE_SHAPE);
    }

    // --- the cases' working --------------------------------------------------------------------------------

    /** A box: centre, half-extents and orientation. */
    record Solid(double[] centre, double[] half, double[] q, float shape) {

        Solid(double[] centre, double[] half, double[] q) {
            this(centre, half, q, Boxes.BOX_SHAPE);
        }

        /** {@code anchor}, in this box's frame, in the world's. */
        double[] world(float[] m, int at) {
            double[] own = {m[at], m[at + 1], m[at + 2]};
            double[] turned = BoxDiagnostics.rotate(q, own);
            return new double[] {centre[0] + turned[0], centre[1] + turned[1], centre[2] + turned[2]};
        }
    }

    private static Solid box(double x, double y, double z, double[] half, double[] q) {
        return new Solid(new double[] {x, y, z}, half, q);
    }

    /** A manifold read back, and the checks a case makes of it. */
    private static final class Contact {
        private final float[] m;
        private final Solid first;
        private final Solid second;

        Contact(float[] m, Solid first, Solid second) {
            this.m = m;
            this.first = first;
            this.second = second;
        }

        /** The winning axis was face {@code axis} of the first box, and the normal is {@code (nx, ny, nz)}. */
        void face(int axis, double nx, double ny, double nz) {
            assertEquals(axis, m[BoxContact.AXIS], "the axis that won");
            normal(nx, ny, nz);
        }

        void normal(double nx, double ny, double nz) {
            double[] n = {nx, ny, nz};
            for (int a = 0; a < 3; a++) {
                assertEquals(n[a], m[BoxContact.NORMAL + a], CLOSE, "normal " + a);
            }
        }

        /** The points are {@code onSecond} on the second box, each {@code depth} deep, and raised by it on the first. */
        void points(double depth, double[][] onSecond) {
            double[][] onFirst = new double[onSecond.length][];
            double[] n = {m[BoxContact.NORMAL], m[BoxContact.NORMAL + 1], m[BoxContact.NORMAL + 2]};
            for (int k = 0; k < onSecond.length; k++) {
                onFirst[k] = new double[3];
                for (int a = 0; a < 3; a++) {
                    onFirst[k][a] = onSecond[k][a] + depth * n[a];
                }
            }
            pointsOn(depth, onFirst, onSecond);
        }

        /** The points, in any order, are {@code onFirst} on the first box and {@code onSecond} on the second. */
        void pointsOn(double depth, double[][] onFirst, double[][] onSecond) {
            int count = (int) m[BoxContact.COUNT];
            List<String> found = new ArrayList<>();
            for (int k = 0; k < count; k++) {
                int at = BoxContact.POINTS + k * BoxContact.POINT_WORDS;
                found.add(text(first.world(m, at + BoxContact.ANCHOR_FIRST)) + " / "
                        + text(second.world(m, at + BoxContact.ANCHOR_SECOND)) + " at " + m[at + BoxContact.DEPTH]);
            }
            assertEquals(onFirst.length, count, "points: " + found);
            boolean[] used = new boolean[count];
            for (int e = 0; e < onFirst.length; e++) {
                boolean matched = false;
                for (int k = 0; k < count && !matched; k++) {
                    int at = BoxContact.POINTS + k * BoxContact.POINT_WORDS;
                    if (!used[k] && near(first.world(m, at + BoxContact.ANCHOR_FIRST), onFirst[e])
                            && near(second.world(m, at + BoxContact.ANCHOR_SECOND), onSecond[e])) {
                        used[k] = true;
                        matched = true;
                        assertEquals(depth, m[at + BoxContact.DEPTH], CLOSE, "depth of " + text(onSecond[e]));
                    }
                }
                assertTrue(matched, "no point at " + text(onFirst[e]) + " / " + text(onSecond[e]) + " in " + found);
            }
        }

        private static boolean near(double[] p, double[] q) {
            return Math.abs(p[0] - q[0]) < 1e-5 && Math.abs(p[1] - q[1]) < 1e-5 && Math.abs(p[2] - q[2]) < 1e-5;
        }

        private static String text(double[] p) {
            return String.format("(%.5f, %.5f, %.5f)", p[0], p[1], p[2]);
        }
    }

    /** The four corners {@code (±hx, y, ±hz)}; {@code hy} is unused, for symmetry with a box's extents. */
    private static double[][] corners(double hx, double hy, double hz, double y) {
        return new double[][] {{hx, y, hz}, {hx, y, -hz}, {-hx, y, hz}, {-hx, y, -hz}};
    }

    private static double[] axisAngle(double x, double y, double z, double angle) {
        double length = Math.sqrt(x * x + y * y + z * z);
        double sin = Math.sin(angle / 2) / length;
        return new double[] {x * sin, y * sin, z * sin, Math.cos(angle / 2)};
    }

    /** The manifold of the one pair {@code (first, second)}, found on {@code backend} with {@code margin}. */
    private static float[] collide(Backend backend, double margin, Solid first, Solid second) {
        return collide(backend, margin, new Solid[] {first, second});
    }

    /** The manifolds of the pairs {@code (boxes[0], boxes[1])}, {@code (boxes[2], boxes[3])} and so on. */
    private static float[] collide(Backend backend, double margin, Solid... boxes) {
        int pairs = boxes.length / 2;
        Program program = new Program(boxes.length, pairs);
        Accelerator accelerator = null;
        PassRunner runner;
        if (backend == Backend.CPU) {
            runner = PassRunner.cpu(program, Boxes.WORKGROUP, PassRunner.NO_SUBGROUP);
        } else {
            accelerator = new Accelerator();
            if (!accelerator.capabilities().gpuAvailable()) {
                accelerator.close();
                assumeTrue(Boolean.getBoolean("supirvast.requireGpu"), "no Vulkan device");
                throw new IllegalStateException("-Dsupirvast.requireGpu=true but no Vulkan device");
            }
            runner = PassRunner.gpu(accelerator, program, Boxes.WORKGROUP, PassRunner.NO_SUBGROUP);
            runner.clear();
        }
        try {
            String[] names = {"x", "y", "z", "qx", "qy", "qz", "qw", "hx", "hy", "hz", "shape"};
            for (int k = 0; k < names.length; k++) {
                float[] values = new float[boxes.length];
                for (int s = 0; s < boxes.length; s++) {
                    values[s] = k == 10 ? boxes[s].shape : (float) (k < 3 ? boxes[s].centre[k] : k < 7
                            ? boxes[s].q[k - 3] : boxes[s].half[k - 7]);
                }
                runner.write(names[k], values);
            }
            runner.write("params", Boxes.params(1e-3, 0, 0, 0, 1, 1, 1, 0, 0, margin));
            int[] pairList = new int[2 * pairs];
            for (int k = 0; k < pairList.length; k++) {
                pairList[k] = k;
            }
            runner.write("pairs", pairList);
            runner.run(program.passes).await();
            return runner.floats("manifold");
        } finally {
            runner.close();
            if (accelerator != null) {
                accelerator.close();
            }
        }
    }

    /** The collide pass alone, over {@code boxes} and {@code pairs}. */
    private static final class Program implements Buffered {
        final Map<String, BufferSpec> buffers = new LinkedHashMap<>();
        final List<Pass> passes;

        Program(int boxes, int pairs) {
            for (String name : List.of("x", "y", "z", "qx", "qy", "qz", "qw", "hx", "hy", "hz", "shape")) {
                buffers.put(name, new BufferSpec(name, Body.F32, boxes));
            }
            buffers.put("params", new BufferSpec("params", Body.F32, Boxes.PARAM_COUNT));
            buffers.put("pairs", new BufferSpec("pairs", Body.I32, 2 * pairs));
            buffers.put("manifold", new BufferSpec("manifold", Body.F32, BoxContact.MANIFOLD_WORDS * pairs));
            buffers.put("candidates", new BufferSpec("candidates", Body.F32, BoxContact.SCRATCH_WORDS * pairs));
            passes = List.of(new Pass("collide", BoxContact.collide(), BoxContact.BUFFERS,
                    List.of(BoxContact.NAMES), pairs));
        }

        @Override
        public Map<String, BufferSpec> buffers() {
            return buffers;
        }
    }
}

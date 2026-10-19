package dev.vexelray.sim.rigid.box;

import dev.vexelray.sim.rigid.box.BoxScene.Backend;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Spheres among boxes, on the boxes' soft step: a sphere is a body whose shape says so, its radius in its
 * half-extents. Where the answer is known, as the spheres' own tests know it.
 */
class BoxSphereTest {

    private static final double G = 9.81;

    /** A ball of 5 cm on a free cube of 10 cm on the floor: after 2 s it rests on the cube's top, centred, still. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBallRestsOnABox(Backend backend) {
        try (BoxScene scene = new BoxScene(2).uniform(0.05, 0.05, 0.05, 1)) {
            scene.sphere(1, 0.05);
            scene.im[1] = 2;
            scene.muStatic = scene.muKinetic = 0.5;
            scene.x[0] = scene.z[0] = scene.x[1] = scene.z[1] = 0.5f;
            scene.y[0] = 0.05f;
            scene.y[1] = 0.151f;
            scene.start(backend).advanceSeconds(2);
            BoxDiagnostics state = scene.diagnostics();
            System.out.printf("ball on box, %s: height %.5f, off centre %.2e, energy %.2e J%n", backend, scene.y[1],
                    Math.hypot(scene.x[1] - 0.5, scene.z[1] - 0.5), state.kinetic());
            assertEquals(0.15, scene.y[1], 5e-4, "on the box's top");
            assertTrue(Math.hypot(scene.x[1] - 0.5, scene.z[1] - 0.5) < 1e-4, "still over the box's centre");
            assertTrue(state.kinetic() < 1e-6, "still moving: " + state.kinetic() + " J");
        }
    }

    /**
     * A ball of 5 cm sliding at 2 m/s on a fixed slab, μ = 0.3: friction slows it and spins it up until it rolls, at
     * {@code 5/7 v₀} with {@code ω = v / r}, as a sliding sphere does on the floor. After 1 s, to 1%.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aSlidingBallSettlesIntoRollingOnABox(Backend backend) {
        try (BoxScene scene = new BoxScene(2).uniform(1.8, 0.05, 0.3, 1)) {
            scene.sphere(1, 0.05);
            scene.im[0] = 0;
            scene.muStatic = scene.muKinetic = 0.3;
            scene.sx = 4;
            scene.sz = 1;
            scene.x[0] = 2;
            scene.y[0] = 0.05f;
            scene.z[0] = scene.z[1] = 0.5f;
            scene.x[1] = 0.4f;
            scene.y[1] = 0.15f;
            scene.u[1] = 2;
            scene.start(backend).advanceSeconds(1);
            scene.read();
            double rolling = 2 * 5.0 / 7;
            System.out.printf("ball rolls on a box, %s: speed %.4f m/s, expected %.4f; spin %.3f rad/s, expected"
                    + " %.3f%n", backend, scene.u[1], rolling, scene.az[1], -rolling / 0.05);
            assertEquals(rolling, scene.u[1], 0.01 * rolling, "rolling speed");
            assertEquals(-rolling / 0.05, scene.az[1], 0.01 * rolling / 0.05, "spin, rolling without slipping");
            assertEquals(0.15, scene.y[1], 5e-4, "on the slab");
        }
    }

    /**
     * Rolling resistance: a ball of 5 cm rolling at 1 m/s with {@code μr = 0.1} slows at {@code 5/7 μr g}, as the
     * spheres' own does, to 0.650 m/s after 0.5 s, still rolling; on the floor, which is the walls' contact, and on a
     * fixed slab, which is a pair's. To 1e-2 m/s.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aRollingBallSlowsAtFiveSeventhsMuRollingG(Backend backend) {
        for (boolean onSlab : new boolean[] {false, true}) {
            int n = onSlab ? 2 : 1;
            try (BoxScene scene = new BoxScene(n).uniform(1.8, 0.05, 0.3, 1)) {
                int ball = n - 1;
                scene.sphere(ball, 0.05);
                scene.muStatic = scene.muKinetic = 0.5;
                scene.muRolling = 0.1;
                scene.sx = 4;
                scene.sz = 1;
                if (onSlab) {
                    scene.im[0] = 0;
                    scene.x[0] = 2;
                    scene.y[0] = 0.05f;
                    scene.z[0] = 0.5f;
                }
                scene.x[ball] = 0.4f;
                scene.y[ball] = onSlab ? 0.15f : 0.05f;
                scene.z[ball] = 0.5f;
                scene.u[ball] = 1;
                scene.az[ball] = -20;
                scene.start(backend).advanceSeconds(0.5);
                scene.read();
                double expected = 1 - 0.5 * 5.0 / 7 * 0.1 * G;
                System.out.printf("rolling resistance %s, %s: %.4f m/s, expected %.4f; spin %.3f, rolling %.3f%n",
                        onSlab ? "on a slab" : "on the floor", backend, scene.u[ball], expected, scene.az[ball],
                        -scene.u[ball] / 0.05);
                assertEquals(expected, scene.u[ball], 1e-2, onSlab ? "on the slab" : "on the floor");
                assertEquals(-scene.u[ball] / 0.05, scene.az[ball], 0.2, "still rolling");
            }
        }
    }

    /** A ball of 1 kg at 2 m/s into a free box of 1 kg, no gravity: momentum is kept to 1e-6. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBallIntoABoxKeepsMomentum(Backend backend) {
        try (BoxScene scene = new BoxScene(2).uniform(0.05, 0.05, 0.05, 1)) {
            scene.sphere(0, 0.05);
            scene.gy = 0;
            scene.sx = scene.sy = scene.sz = 2;
            scene.x[0] = 0.7f;
            scene.x[1] = 1;
            scene.y[0] = scene.y[1] = scene.z[0] = scene.z[1] = 1;
            scene.u[0] = 2;
            scene.start(backend).advanceSeconds(0.5);
            BoxDiagnostics state = scene.diagnostics();
            System.out.printf("ball into box, %s: momentum %.6f, speeds %.4f %.4f%n", backend, state.momentum()[0],
                    scene.u[0], scene.u[1]);
            assertEquals(2, state.momentum()[0], 1e-6, "momentum");
            assertEquals(1, scene.u[0], 1e-2, "inelastic: moving together");
            assertEquals(1, scene.u[1], 1e-2, "inelastic: moving together");
        }
    }

    /** A box of 20 × 10 × 20 cm on four fixed balls of 5 cm at its corners: it rests level. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBoxRestsOnFourBalls(Backend backend) {
        try (BoxScene scene = new BoxScene(5).uniform(0.05, 0.05, 0.05, 1)) {
            double[][] at = {{0.42, 0.58}, {0.58, 0.42}, {0.42, 0.42}, {0.58, 0.58}};
            for (int k = 0; k < 4; k++) {
                scene.sphere(k, 0.05);
                scene.im[k] = 0;
                scene.x[k] = (float) at[k][0];
                scene.z[k] = (float) at[k][1];
                scene.y[k] = 0.05f;
            }
            scene.half[0][4] = scene.half[2][4] = 0.1f;
            scene.x[4] = scene.z[4] = 0.5f;
            scene.y[4] = 0.151f;
            scene.muStatic = scene.muKinetic = 0.5;
            scene.start(backend).advanceSeconds(2);
            BoxDiagnostics state = scene.diagnostics();
            double[] up = BoxDiagnostics.rotate(new double[] {scene.q[0][4], scene.q[1][4], scene.q[2][4],
                    scene.q[3][4]}, new double[] {0, 1, 0});
            double tilt = Math.hypot(up[0], up[2]);
            System.out.printf("box on balls, %s: height %.5f, tilt %.2e, energy %.2e J%n", backend, scene.y[4], tilt,
                    state.kinetic());
            assertEquals(0.15, scene.y[4], 5e-4, "on the balls' tops");
            assertTrue(tilt < 1e-4, "tilted by " + tilt);
            assertTrue(state.kinetic() < 1e-6, "still moving: " + state.kinetic() + " J");
        }
    }

    /**
     * 64 balls and 64 cubes, 8 cm, in a loose block of alternating bodies, dropped, friction 0.5 and rolling resistance
     * 0.3: after 4 s it rests, under 1e-5 J, no point deeper than 2 mm. Without rolling resistance the boxes rest and
     * the balls roll on, at 3 cm/s after 4 s: friction alone does not stop a ball rolling.
     */
    @Test
    void aMixedPileRests() {
        int side = 4;
        int n = 2 * side * side * side;
        try (BoxScene scene = new BoxScene(n).uniform(0.04, 0.04, 0.04, 0.5)) {
            scene.muStatic = scene.muKinetic = 0.5;
            scene.muRolling = 0.3;
            scene.sx = scene.sz = 1;
            scene.sy = 2.5;
            Random random = new Random(3);
            int k = 0;
            for (int i = 0; i < side; i++) {
                for (int j = 0; j < 2 * side; j++) {
                    for (int l = 0; l < side; l++, k++) {
                        if (k % 2 == 0) {
                            scene.sphere(k, 0.04);
                        }
                        scene.x[k] = (float) (0.35 + 0.1 * i + 0.02 * random.nextDouble());
                        scene.y[k] = (float) (0.3 + 0.1 * j);
                        scene.z[k] = (float) (0.35 + 0.1 * l + 0.02 * random.nextDouble());
                    }
                }
            }
            scene.start(Backend.GPU).advanceSeconds(4);
            BoxDiagnostics state = scene.diagnostics();
            float[] manifold = scene.floats("manifold");
            double deepest = 0;
            for (int p = 0; p < scene.slots(); p++) {
                int at = p * BoxContact.MANIFOLD_WORDS;
                for (int c = 0; c < (int) manifold[at + BoxContact.COUNT]; c++) {
                    deepest = Math.max(deepest, manifold[at + BoxContact.POINTS + c * BoxContact.POINT_WORDS
                            + BoxContact.DEPTH]);
                }
            }
            System.out.printf("mixed pile of %d: energy %.2e J, deepest %.2f mm, %d touching, rounds %d%n", n,
                    state.kinetic(), 1000 * deepest, scene.last.touching(), scene.mostRounds);
            assertTrue(state.kinetic() < 1e-5, "still moving: " + state.kinetic() + " J");
            assertTrue(deepest < 2e-3, "a point " + deepest + " m deep");
            assertEquals(0, scene.unplaced, "pairs left unsolved");
            assertEquals(0, scene.unlisted, "partners with no slot");
        }
    }
}

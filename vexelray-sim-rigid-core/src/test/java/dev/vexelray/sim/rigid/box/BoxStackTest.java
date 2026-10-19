package dev.vexelray.sim.rigid.box;

import dev.vexelray.sim.rigid.box.BoxScene.Backend;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Boxes against boxes, judged the way the spheres' column is: what they overlap by, what is still moving, and how
 * far the top of a stack has drifted.
 */
class BoxStackTest {

    /**
     * Two cubes meeting head on at 1 m/s each, with no gravity: momentum is kept to 1e-6, and the contact, having no
     * restitution, leaves them moving together, here not at all.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aHeadOnCollisionKeepsMomentum(Backend backend) {
        try (BoxScene scene = new BoxScene(2).uniform(0.05, 0.05, 0.05, 1)) {
            scene.gy = 0;
            scene.sx = scene.sy = scene.sz = 2;
            scene.x[0] = 0.8f;
            scene.x[1] = 1.2f;
            scene.y[0] = scene.y[1] = scene.z[0] = scene.z[1] = 1;
            scene.u[0] = 1;
            scene.u[1] = -1;
            scene.start(backend).advanceSeconds(0.5);
            BoxDiagnostics state = scene.diagnostics();
            System.out.printf("head on, %s: momentum %.2e, speeds %.2e %.2e, gap %.5f%n", backend,
                    state.momentum()[0], scene.u[0], scene.u[1], scene.x[1] - scene.x[0]);
            assertEquals(0, state.momentum()[0], 1e-6, "momentum");
            assertEquals(0, scene.u[0], 1e-3, "stopped");
            assertEquals(0, scene.u[1], 1e-3, "stopped");
            assertEquals(0.1, scene.x[1] - scene.x[0], 1e-3, "touching");
        }
    }

    /**
     * A column of ten cubes of 10 cm, each set 1 mm above the one below, with friction 0.5: after 3 s it rests, its top
     * cube within 1 mm of over the bottom one, its moving energy under 1e-6 J. Measured, 0.2 mm and 2e-10 J.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aColumnOfTenRests(Backend backend) {
        column(backend, 10, 1e-3, 1e-6);
    }

    /**
     * Twenty: it stands, but sways a little on its soft contacts and is not yet still after 3 s. Its top within 2 cm
     * and its moving energy under 1e-4 J; measured, 9 to 12 mm and 1e-6 J. {@code docs/boxes.md} has the sway.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aColumnOfTwentyStands(Backend backend) {
        column(backend, 20, 2e-2, 1e-4);
    }

    /**
     * The harmonic overhang: six blocks 40 cm long, 5 cm high, each stuck out past the one below it by
     * {@code f · L / 2k}, {@code k} counted from the top, so that at {@code f = 1} the blocks above each block have
     * their centre of mass exactly over its edge. At 0.95 the stack stands: after 3 s no block has dropped by 2 mm or
     * slid by 1 mm (measured, 1.5 mm, the top tilting onto its edge, and 0.4 mm), and it is still. At 1.05 it falls:
     * a block has dropped by more than its own height. A manifold that lost its points at the edges would tip early.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void theHarmonicOverhangStandsShortOfItsLimitAndFallsPastIt(Backend backend) {
        for (double f : new double[] {0.95, 1.05}) {
            int n = 6;
            double length = 0.4;
            try (BoxScene scene = new BoxScene(n).uniform(length / 2, 0.025, 0.1, 1)) {
                scene.muStatic = scene.muKinetic = 0.5;
                scene.sx = 1.6;
                scene.sz = 0.5;
                scene.sy = 1;
                for (int i = 0; i < n; i++) {
                    scene.x[i] = i == 0 ? 0.3f : (float) (scene.x[i - 1] + f * length / (2 * (n - i)));
                    scene.y[i] = (float) (0.025 + 0.05 * i);
                    scene.z[i] = 0.25f;
                }
                float[] x0 = scene.x.clone();
                float[] y0 = scene.y.clone();
                scene.start(backend).advanceSeconds(3);
                BoxDiagnostics state = scene.diagnostics();
                double drop = 0;
                double slide = 0;
                for (int i = 0; i < n; i++) {
                    drop = Math.max(drop, y0[i] - scene.y[i]);
                    slide = Math.max(slide, Math.abs(scene.x[i] - x0[i]));
                }
                System.out.printf("overhang %.2f, %s: dropped %.2e m, slid %.2e m, energy %.2e J%n", f, backend, drop,
                        slide, state.kinetic());
                if (f < 1) {
                    assertTrue(drop < 2e-3, "at " + f + " a block dropped by " + drop + " m");
                    assertTrue(slide < 1e-3, "at " + f + " a block slid by " + slide + " m");
                    assertTrue(state.kinetic() < 1e-6, "at " + f + " still moving: " + state.kinetic() + " J");
                } else {
                    assertTrue(drop > 0.05, "at " + f + " it stands: the most a block dropped is " + drop + " m");
                }
            }
        }
    }

    /**
     * A pyramid of cubes, ten at its base and 55 in all, each row set on the one below half a cube along, with 0.5 mm
     * between neighbours: after 3 s no cube has moved by 1 mm, and nothing is moving. On the GPU in every build; on the
     * CPU, which takes a minute and a half, with {@code -Pphysics}.
     */
    @Test
    void aPyramidOfTenStands() {
        pyramid(Backend.GPU);
    }

    @Test
    @Tag("physics")
    void aPyramidOfTenStandsOnTheCpu() {
        pyramid(Backend.CPU);
    }

    private static void pyramid(Backend backend) {
        int base = 10;
        int n = base * (base + 1) / 2;
        try (BoxScene scene = new BoxScene(n).uniform(0.05, 0.05, 0.05, 1)) {
            scene.muStatic = scene.muKinetic = 0.5;
            scene.sx = 1.6;
            scene.sz = 0.5;
            scene.sy = 1.5;
            int k = 0;
            for (int row = 0; row < base; row++) {
                for (int c = 0; c < base - row; c++, k++) {
                    scene.x[k] = (float) (0.3 + 0.1005 * (c + row / 2.0));
                    scene.y[k] = (float) (0.05 + 0.1 * row);
                    scene.z[k] = 0.25f;
                }
            }
            float[] x0 = scene.x.clone();
            float[] y0 = scene.y.clone();
            long start = System.nanoTime();
            scene.start(backend).advanceSeconds(3);
            double ms = (System.nanoTime() - start) / 1e6 / 180;
            BoxDiagnostics state = scene.diagnostics();
            double moved = 0;
            for (int i = 0; i < n; i++) {
                moved = Math.max(moved, Math.hypot(scene.x[i] - x0[i], scene.y[i] - y0[i]));
            }
            System.out.printf("pyramid of %d, %s: moved %.2e m, energy %.2e J, rounds %d, points %d, %.2f ms a step%n",
                    base, backend, moved, state.kinetic(), scene.mostRounds, scene.last.points(), ms);
            assertTrue(moved < 1e-3, "a cube moved by " + moved + " m");
            assertTrue(state.kinetic() < 1e-6, "still moving: " + state.kinetic() + " J");
            assertEquals(0, scene.unplaced, "pairs left unsolved");
        }
    }

    private static void column(Backend backend, int n, double mostDrift, double mostEnergy) {
        try (BoxScene scene = new BoxScene(n).uniform(0.05, 0.05, 0.05, 1)) {
            scene.sx = scene.sz = 1;
            scene.sy = 0.11 * n + 0.5;
            for (int k = 0; k < n; k++) {
                scene.x[k] = scene.z[k] = 0.5f;
                scene.y[k] = (float) (0.05 + 0.101 * k);
            }
            scene.muStatic = scene.muKinetic = 0.5;
            long start = System.nanoTime();
            scene.start(backend).advanceSeconds(3);
            double ms = (System.nanoTime() - start) / 1e6 / 180;
            BoxDiagnostics state = scene.diagnostics();
            double overlap = 0;
            for (int k = 1; k < n; k++) {
                overlap = Math.max(overlap, 0.1 - (scene.y[k] - scene.y[k - 1]));
            }
            overlap = Math.max(overlap, 0.05 - scene.y[0]);
            double drift = Math.hypot(scene.x[n - 1] - 0.5, scene.z[n - 1] - 0.5);
            System.out.printf("column of %d, %s: overlap %.2f%% of a half, drift %.2e m, energy %.2e J, rounds %d,"
                    + " %.2f ms a step%n", n, backend, 100 * overlap / 0.05, drift, state.kinetic(), scene.mostRounds,
                    ms);
            assertTrue(drift < mostDrift, "the top drifted " + drift + " m");
            assertTrue(state.kinetic() < mostEnergy, "still moving: " + state.kinetic() + " J");
            assertEquals(0, scene.unplaced, "pairs left unsolved");
        }
    }
}

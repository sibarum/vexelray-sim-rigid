package dev.vexelray.sim.rigid.box;

import dev.vexelray.sim.rigid.box.BoxScene.Backend;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A box with nothing touching it, where the answer is known: a free fall, a spin about a stable principal axis, a
 * tumble whose angular momentum and energy stay what they were, and a spin about the intermediate axis, which turns
 * over at the period the closed form gives.
 *
 * <p>The box throughout is 0.1 × 0.2 × 0.4 m, half-extents 0.05, 0.1 and 0.2, so its largest moment is about x, the
 * intermediate about y and the smallest about z.
 */
class BoxFreeTest {

    private static final double HX = 0.05;
    private static final double HY = 0.1;
    private static final double HZ = 0.2;

    /** A dropped box falls as symplectic Euler says, to f32's rounding, and does not turn. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aFreeFallIsSymplecticEulers(Backend backend) {
        try (BoxScene scene = new BoxScene(1).uniform(HX, HY, HZ, 1)) {
            scene.x[0] = scene.z[0] = 0.5f;
            scene.y[0] = 8;
            scene.start(backend).advanceSeconds(0.5).read();

            int substeps = 30 * scene.substeps;
            double h = scene.dt / scene.substeps;
            double y = 8;
            double vy = 0;
            for (int k = 0; k < substeps; k++) {
                vy += h * scene.gy;
                y += h * vy;
            }
            assertEquals(vy, scene.v[0], 1e-5, "speed");
            assertEquals(y, scene.y[0], 1e-5, "height");
            assertEquals(1, scene.q[3][0], 0, "not turned");
        }
    }

    /**
     * A spin about the largest or the smallest principal axis is kept, and turns the box about that axis at its rate:
     * 5 rad/s for 10 s. The first-order turn is short by {@code θ³ / 12} a substep, 3e-4 rad over the 6000 here.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aSpinAboutAStableAxisIsKept(Backend backend) {
        for (int axis : new int[] {0, 2}) {
            try (BoxScene scene = new BoxScene(1).uniform(HX, HY, HZ, 1)) {
                scene.gy = 0;
                scene.x[0] = scene.y[0] = scene.z[0] = 0.5f;
                float[][] spin = {scene.ax, scene.ay, scene.az};
                spin[axis][0] = 5;
                scene.start(backend).advanceSeconds(10).read();

                double[] own = scene.ownSpin(0);
                for (int a = 0; a < 3; a++) {
                    assertEquals(a == axis ? 5 : 0, own[a], 1e-4, "axis " + axis + ", spin component " + a);
                }
                double angle = 2 * Math.atan2(scene.q[axis][0], scene.q[3][0]);
                double expected = Math.IEEEremainder(50, 2 * Math.PI);
                assertEquals(0, Math.IEEEremainder(angle - expected, 2 * Math.PI), 1e-3, "axis " + axis + ", turned");
            }
        }
    }

    /**
     * Nudged off a stable axis, a spin stays near it: 5 rad/s about the largest or the smallest axis, with 0.1 rad/s
     * about each other, for 10 s. The angle between the spin and the axis starts at 0.028 rad and stays below twice
     * that.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aNudgedSpinAboutAStableAxisStaysNearIt(Backend backend) {
        for (int axis : new int[] {0, 2}) {
            try (BoxScene scene = new BoxScene(1).uniform(HX, HY, HZ, 1)) {
                scene.gy = 0;
                scene.x[0] = scene.y[0] = scene.z[0] = 0.5f;
                float[][] spin = {scene.ax, scene.ay, scene.az};
                for (int a = 0; a < 3; a++) {
                    spin[a][0] = a == axis ? 5 : 0.1f;
                }
                scene.start(backend);
                double widest = 0;
                for (int frame = 0; frame < 600; frame++) {
                    scene.advance(1).read();
                    double[] own = scene.ownSpin(0);
                    double length = Math.sqrt(own[0] * own[0] + own[1] * own[1] + own[2] * own[2]);
                    widest = Math.max(widest, Math.acos(Math.min(1, Math.abs(own[axis]) / length)));
                }
                assertTrue(widest < 0.056, "axis " + axis + ": strayed to " + widest + " rad");
            }
        }
    }

    /**
     * A box tumbling about no principal axis, at {@code (1, 2, 3)} rad/s and moving at 1 m/s with no gravity, for
     * 10 s: the angular momentum about the origin is kept to the bit, since nothing writes it, and the energy to
     * 1e-6. Measured, 1.3e-7 on the CPU and 3.5e-8 on the GPU, where the same sums in doubles leave 3.7e-7, the
     * integration's own error.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aTumbleKeepsItsAngularMomentumAndEnergy(Backend backend) {
        try (BoxScene scene = tumbling()) {
            BoxDiagnostics before = scene.start(backend).diagnostics();
            scene.advanceSeconds(10);
            BoxDiagnostics after = scene.diagnostics();

            double length = norm(before.angular());
            double drift = norm(difference(after.angular(), before.angular())) / length;
            double energy = Math.abs(after.kinetic() - before.kinetic()) / before.kinetic();
            System.out.printf("tumble on %s: angular momentum %.2e, energy %.2e%n", backend, drift, energy);
            assertEquals(0, drift, 0, "angular momentum moved by " + drift + " of itself");
            assertTrue(energy < 1e-6, "energy moved by " + energy + " of itself");
            double[] own = scene.ownSpin(0);
            assertTrue(Math.abs(own[1]) > 0.1, "it tumbles, so the test is not of a still box");
        }
    }

    /**
     * Why {@link Boxes#predict} keeps the angular momentum rather than integrating the gyroscopic term explicitly,
     * {@code ω += h I⁻¹ (−ω × I ω)}: the same tumble, on the host in doubles, at the same substep. Each explicit step
     * adds a step at right angles to {@code L}, so {@code |L|} only grows, and the energy with it.
     */
    @Test
    void theExplicitGyroscopicTermGrowsTheAngularMomentum() {
        double[] moment = Boxes.inertia(2, HX, HY, HZ);
        double h = 1.0 / 600;
        double[] omega = {1, 2, 3};
        double[] start = momentum(moment, omega);
        for (int k = 0; k < 6000; k++) {
            double[] l = momentum(moment, omega);
            double[] torque = cross(omega, l);
            for (int a = 0; a < 3; a++) {
                omega[a] -= h * torque[a] / moment[a];
            }
        }
        double grown = norm(momentum(moment, omega)) / norm(start) - 1;
        System.out.printf("explicit gyroscopic term, 10 s: |L| grew by %.2e%n", grown);
        assertTrue(grown > 1e-3, "explicit integration kept |L| to " + grown);
    }

    /**
     * The Dzhanibekov effect: a spin of 5 rad/s about the intermediate axis, nudged by 0.1 rad/s about the smallest,
     * turns over and back. The spin about that axis changes sign every half period of the closed form (Landau and
     * Lifshitz, <i>Mechanics</i>, §37), {@code 2K(k) √(I₁ I₂ I₃ / ((I₃ − I₂)(M² − 2E I₁)))}, 3.403 s here. Over
     * 12 s the crossings, read at every frame and placed between frames by linear interpolation, are that far apart
     * to 0.5%; measured, within 0.2% on both backends.
     *
     * <p>The test is a sharp one. The nudge leaves the spin 8e-5 of {@code M²} from the separatrix, the motion that
     * never turns over, and the period goes as the logarithm of that distance: an energy error of 1e-4, which an
     * earlier form of the predict made in 10 s, moved the period by 2%.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aSpinAboutTheIntermediateAxisTurnsOverAtItsPeriod(Backend backend) {
        double[] moment = Boxes.inertia(1, HX, HY, HZ);
        double[] omega = {0, 5, 0.1};
        double expected = halfPeriod(moment, omega);
        try (BoxScene scene = new BoxScene(1).uniform(HX, HY, HZ, 1)) {
            scene.gy = 0;
            scene.x[0] = scene.y[0] = scene.z[0] = 0.5f;
            scene.ay[0] = (float) omega[1];
            scene.az[0] = (float) omega[2];
            scene.start(backend);

            List<Double> crossings = new ArrayList<>();
            double was = omega[1];
            for (int frame = 1; frame <= 720; frame++) {
                scene.advance(1).read();
                double now = scene.ownSpin(0)[1];
                if (Math.signum(now) != Math.signum(was)) {
                    crossings.add((frame - 1 + was / (was - now)) * scene.dt);
                }
                was = now;
            }
            assertTrue(crossings.size() >= 3, "turned over " + crossings.size() + " times in 12 s, expected half"
                    + " a period of " + expected + " s");
            for (int k = 1; k < crossings.size(); k++) {
                double gap = crossings.get(k) - crossings.get(k - 1);
                System.out.printf("flip on %s: %.4f s apart, closed form %.4f s%n", backend, gap, expected);
                assertEquals(expected, gap, 0.005 * expected,"crossings " + (k - 1) + " and " + k);
            }
        }
    }

    // --- the closed form, and the host's own sums -----------------------------------------------------------

    /**
     * Half the period of a torque-free body's spin about its intermediate axis, from the moments about its axes and
     * its angular velocity in its own frame. Landau and Lifshitz write it for {@code I₁ < I₂ < I₃} and
     * {@code M² > 2E I₂}; for {@code M² < 2E I₂} the first and third are swapped.
     */
    static double halfPeriod(double[] moment, double[] omega) {
        Integer[] order = {0, 1, 2};
        java.util.Arrays.sort(order, (a, b) -> Double.compare(moment[a], moment[b]));
        double m2 = 0;
        double twoE = 0;
        for (int a = 0; a < 3; a++) {
            double l = moment[a] * omega[a];
            m2 += l * l;
            twoE += l * omega[a];
        }
        double i1 = moment[order[0]];
        double i2 = moment[order[1]];
        double i3 = moment[order[2]];
        if (m2 < twoE * i2) {
            double swap = i1;
            i1 = i3;
            i3 = swap;
        }
        double k2 = (i2 - i1) * (twoE * i3 - m2) / ((i3 - i2) * (m2 - twoE * i1));
        double scale = Math.sqrt(i1 * i2 * i3 / ((i3 - i2) * (m2 - twoE * i1)));
        return 2 * completeEllipticK(k2) * scale;
    }

    /** {@code K(k)} for {@code k² < 1}, by the arithmetic–geometric mean: {@code π / (2 AGM(1, √(1 − k²)))}. */
    static double completeEllipticK(double k2) {
        double a = 1;
        double b = Math.sqrt(1 - k2);
        for (int k = 0; k < 64 && Math.abs(a - b) > 1e-15 * a; k++) {
            double mean = (a + b) / 2;
            b = Math.sqrt(a * b);
            a = mean;
        }
        return Math.PI / (2 * a);
    }

    @Test
    void theEllipticIntegralIsRight() {
        assertEquals(Math.PI / 2, completeEllipticK(0), 1e-15);
        assertEquals(1.8540746773013719, completeEllipticK(0.5), 1e-14, "K at k² = ½, from tables");
    }

    private static BoxScene tumbling() {
        BoxScene scene = new BoxScene(1).uniform(HX, HY, HZ, 2);
        scene.gy = 0;
        scene.sx = scene.sy = scene.sz = 100;
        scene.x[0] = scene.y[0] = scene.z[0] = 50;
        scene.u[0] = 1;
        scene.ax[0] = 1;
        scene.ay[0] = 2;
        scene.az[0] = 3;
        return scene;
    }

    private static double[] momentum(double[] moment, double[] omega) {
        return new double[] {moment[0] * omega[0], moment[1] * omega[1], moment[2] * omega[2]};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double[] difference(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double norm(double[] a) {
        return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    }
}

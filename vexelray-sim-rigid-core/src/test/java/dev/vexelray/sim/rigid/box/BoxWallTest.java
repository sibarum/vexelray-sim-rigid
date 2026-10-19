package dev.vexelray.sim.rigid.box;

import dev.vexelray.sim.rigid.box.BoxScene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A box against the walls, where the answer is known: dropped flat, it rests flat; dropped on a corner, it settles on
 * a face; on a slope it rests while {@code μ ≥ tan θ} and slides at {@code g (sin θ − μ cos θ)} below it; and a tall
 * box with friction enough not to slide stands until {@code tan θ} passes its width over its height, and tips past it.
 *
 * <p>A slope is the floor with gravity tilted by {@code θ} along x, as the spheres' friction tests make one.
 */
class BoxWallTest {

    private static final double G = 9.81;

    /** A flat box, 0.2 × 0.1 × 0.2 m, dropped 1 cm onto the floor with μ = 0.5: after 2 s it lies flat and still. */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBoxDroppedFlatRestsFlat(Backend backend) {
        try (BoxScene scene = new BoxScene(1).uniform(0.1, 0.05, 0.1, 1)) {
            scene.x[0] = scene.z[0] = 0.5f;
            scene.y[0] = 0.06f;
            scene.muStatic = scene.muKinetic = 0.5;
            scene.start(backend).advanceSeconds(2);
            BoxDiagnostics state = scene.diagnostics();

            double tilt = tilt(axis(scene, 0, 1));
            System.out.printf("flat on %s: tilt %.2e rad, height %.6f, drift %.2e %.2e, energy %.2e J%n", backend,
                    tilt, scene.y[0], scene.x[0] - 0.5, scene.z[0] - 0.5, state.kinetic());
            assertTrue(tilt < 1e-4, "tilted by " + tilt + " rad");
            assertEquals(0.05, scene.y[0], 1e-4, "resting on its face");
            assertEquals(0.5, scene.x[0], 1e-4, "no drift along x");
            assertEquals(0.5, scene.z[0], 1e-4, "no drift along z");
            assertTrue(state.kinetic() < 1e-6, "still moving: " + state.kinetic() + " J");
        }
    }

    /**
     * A box of 0.1 × 0.2 × 0.4 m, turned by 0.8 rad about {@code (1, 0.3, 0.7)} and dropped from 0.3 m onto a corner,
     * μ = 0.5: after 3 s one of its axes is upright, and it rests at that half-extent's height.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBoxDroppedOnACornerSettlesOnAFace(Backend backend) {
        try (BoxScene scene = new BoxScene(1).uniform(0.05, 0.1, 0.2, 1)) {
            scene.sx = scene.sz = 2;
            scene.x[0] = scene.z[0] = 1;
            scene.y[0] = 0.3f;
            double[] turn = axisAngle(new double[] {1, 0.3, 0.7}, 0.8);
            for (int k = 0; k < 4; k++) {
                scene.q[k][0] = (float) turn[k];
            }
            scene.muStatic = scene.muKinetic = 0.5;
            scene.start(backend).advanceSeconds(3);
            BoxDiagnostics state = scene.diagnostics();

            int up = 0;
            for (int k = 1; k < 3; k++) {
                if (Math.abs(axis(scene, 0, k)[1]) > Math.abs(axis(scene, 0, up)[1])) {
                    up = k;
                }
            }
            double tilt = tilt(axis(scene, 0, up));
            System.out.printf("corner on %s: axis %d up, tilt %.2e rad, height %.6f, energy %.2e J%n", backend, up,
                    tilt, scene.y[0], state.kinetic());
            assertTrue(tilt < 1e-3, "axis " + up + " tilted by " + tilt + " rad");
            assertEquals(scene.half[up][0], scene.y[0], 1e-3, "resting on the face across axis " + up);
            assertTrue(state.kinetic() < 1e-5, "still moving: " + state.kinetic() + " J");
        }
    }

    /**
     * The flat box on a 20° slope with μ = 0.5, more than {@code tan 20° = 0.36}: it settles within 1e-5 m of where it
     * was put, and from 1 s to 3 s moves by less than 1e-5 m. Measured, it moves 7e-7 m on the CPU and none on the
     * GPU, where with the slip read from two positions it crept 3e-5 m.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBoxRestsOnASlopeItsFrictionHolds(Backend backend) {
        try (BoxScene scene = onASlope(20, 0.5)) {
            scene.start(backend).advanceSeconds(1).read();
            double settled = scene.x[0];
            scene.advanceSeconds(2).read();
            System.out.printf("rests on %s: settled %.2e m down, then crept %.2e m, speed %.2e m/s%n", backend,
                    settled - 1, scene.x[0] - settled, scene.u[0]);
            assertEquals(1, settled, 1e-5, "slid down the slope");
            assertEquals(settled, scene.x[0], 1e-5, "crept down the slope");
            assertEquals(0, scene.u[0], 1e-5, "moving down the slope");
        }
    }

    /**
     * The flat box on a 30° slope with μ = 0.3, less than {@code tan 30° = 0.58}: it slides, speeding up at
     * {@code g (sin θ − μ cos θ)}, measured between 0.5 s and 1 s, to 1%, and does not turn.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aBoxSlidesDownASlopeItsFrictionCannotHold(Backend backend) {
        double theta = Math.toRadians(30);
        try (BoxScene scene = onASlope(30, 0.3)) {
            scene.start(backend).advanceSeconds(0.5).read();
            double early = scene.u[0];
            scene.advanceSeconds(0.5).read();
            double rate = (scene.u[0] - early) / 0.5;
            double expected = G * (Math.sin(theta) - 0.3 * Math.cos(theta));
            double tilt = tilt(axis(scene, 0, 1));
            System.out.printf("slides on %s: %.4f m/s², expected %.4f; tilt %.2e rad%n", backend, rate, expected,
                    tilt);
            assertEquals(expected, rate, 0.01 * expected, "acceleration down the slope");
            assertTrue(tilt < 1e-3, "tilted by " + tilt + " rad");
        }
    }

    /**
     * A tall box, 0.1 × 0.4 × 0.1 m, on a slope with μ = 1, enough that it cannot slide below 45°. It tips when its
     * weight passes outside its base, at {@code tan θ = 0.1 / 0.4}, 14.04°: at 12° it stands, and at 16° it falls.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void aTallBoxTipsWhereItsWeightLeavesItsBase(Backend backend) {
        for (double degrees : new double[] {12, 16}) {
            try (BoxScene scene = new BoxScene(1).uniform(0.05, 0.2, 0.05, 1)) {
                tilt(scene, degrees);
                scene.sx = scene.sz = 2;
                scene.x[0] = scene.z[0] = 1;
                scene.y[0] = 0.2f;
                scene.muStatic = scene.muKinetic = 1;
                scene.start(backend).advanceSeconds(2);
                BoxDiagnostics state = scene.diagnostics();
                double upright = axis(scene, 0, 1)[1];
                System.out.printf("tall on %s at %.0f°: upright %.6f, energy %.2e J%n", backend, degrees, upright,
                        state.kinetic());
                if (degrees < 14) {
                    assertTrue(upright > 1 - 1e-6, "at " + degrees + "° it leans: " + upright);
                    assertTrue(state.kinetic() < 1e-6, "at " + degrees + "° still moving: " + state.kinetic());
                } else {
                    assertTrue(upright < 0.5, "at " + degrees + "° it still stands: " + upright);
                }
            }
        }
    }

    // --- building blocks -----------------------------------------------------------------------------------

    /** The flat box resting on a floor tilted by {@code degrees}, with friction {@code mu}, room to slide along x. */
    private static BoxScene onASlope(double degrees, double mu) {
        BoxScene scene = new BoxScene(1).uniform(0.1, 0.05, 0.1, 1);
        tilt(scene, degrees);
        scene.sx = 10;
        scene.x[0] = 1;
        scene.z[0] = 0.5f;
        scene.y[0] = 0.05f;
        scene.muStatic = scene.muKinetic = mu;
        return scene;
    }

    private static void tilt(BoxScene scene, double degrees) {
        double theta = Math.toRadians(degrees);
        scene.gx = G * Math.sin(theta);
        scene.gy = -G * Math.cos(theta);
    }

    /**
     * How far a unit vector leans from upright, from its horizontal part: {@code acos} of its upright part would turn
     * f32's rounding of a vector near upright, 1e-7, into a lean of 4e-4 rad.
     */
    private static double tilt(double[] unit) {
        return Math.asin(Math.min(1, Math.hypot(unit[0], unit[2])));
    }

    /** Box {@code s}'s own axis {@code k} in the world's frame, as the state was last read. */
    private static double[] axis(BoxScene scene, int s, int k) {
        double[] unit = new double[3];
        unit[k] = 1;
        return BoxDiagnostics.rotate(new double[] {scene.q[0][s], scene.q[1][s], scene.q[2][s], scene.q[3][s]}, unit);
    }

    private static double[] axisAngle(double[] axis, double angle) {
        double length = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        double sin = Math.sin(angle / 2) / length;
        return new double[] {axis[0] * sin, axis[1] * sin, axis[2] * sin, Math.cos(angle / 2)};
    }
}

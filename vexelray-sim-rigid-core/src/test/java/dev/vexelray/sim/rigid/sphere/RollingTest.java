package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Rolling resistance ({@link Spheres#roll}), where the answer is known: a torque of at most {@code μr r N} against a
 * sphere's rolling. Gauss–Seidel over the contact list, on each backend.
 */
class RollingTest {

    private static final double G = 9.81;

    /** A sphere on the floor, rolling along x at {@code v0} without slipping, friction enough to keep it so. */
    private static Scene rolling(double v0, double muRolling) {
        Scene scene = new Scene(1).uniform(0.1, 1);
        scene.sx = 6;
        scene.x[0] = 0.5f;
        scene.y[0] = 0.1f;
        scene.z[0] = 0.5f;
        scene.u[0] = (float) v0;
        scene.az[0] = (float) (-v0 / 0.1);
        scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
        scene.muStatic = scene.muKinetic = 0.5;
        scene.muRolling = muRolling;
        return scene;
    }

    /**
     * Rolling resistance takes {@code μr r N} of torque, and friction keeps the contact point still, so the sphere
     * slows as a whole: {@code (I/r² + m) a = −μr N}, which for a solid sphere is {@code a = −5/7 μr g}. From 1 m/s at
     * μr = 0.1, 0.70 m/s² for 1.43 s, and then it stops and stays stopped.
     */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void aRollingSphereSlowsAtFiveSeventhsOfMuGAndStops(Backend backend) {
        double mu = 0.1;
        try (Scene scene = rolling(1, mu)) {
            scene.start(backend).advanceSeconds(0.5);
            scene.read();
            double expected = 1 - 5.0 / 7 * mu * G * 0.5;
            assertEquals(expected, scene.u[0], 1e-2, "slowed at 5/7 μr g");
            assertEquals(-scene.u[0] / 0.1, scene.az[0], 5e-2, "still rolling, not sliding");

            scene.advanceSeconds(1.5);
            scene.read();
            assertTrue(Math.abs(scene.u[0]) < 1e-4, "stopped: " + scene.u[0] + " m/s");
            assertTrue(Math.abs(scene.az[0]) < 1e-3, "not turning: " + scene.az[0] + " rad/s");
            double stopped = scene.x[0];
            scene.advanceSeconds(1);
            scene.read();
            assertEquals(stopped, scene.x[0], 1e-5, "and stays where it stopped");
        }
    }

    /** Without rolling resistance the same sphere rolls on, as it did before there was any. */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void withoutItASphereRollsOn(Backend backend) {
        try (Scene scene = rolling(1, 0)) {
            scene.start(backend).advanceSeconds(1);
            scene.read();
            assertEquals(1, scene.u[0], 1e-3, "rolling on at 1 m/s");
            assertEquals(-10, scene.az[0], 1e-2);
        }
    }

    /**
     * A sphere spinning in place on the floor about the vertical, which friction at a point cannot touch: rolling
     * resistance takes twisting too, so it stops, at {@code μr r N / I}, here 24.5 rad/s² from 5 rad/s.
     */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void aSphereSpinningOnTheFloorStops(Backend backend) {
        try (Scene scene = rolling(0, 0.1)) {
            scene.ay[0] = 5;
            scene.start(backend).advanceSeconds(0.1);
            scene.read();
            assertEquals(5 - 0.1 * G / (Spheres.INERTIA * 0.1) * 0.1, scene.ay[0], 5e-2, "slowed at μr g / ⅖ r");
            scene.advanceSeconds(0.5);
            scene.read();
            assertTrue(Math.abs(scene.ay[0]) < 1e-3, "stopped: " + scene.ay[0] + " rad/s");
        }
    }
}

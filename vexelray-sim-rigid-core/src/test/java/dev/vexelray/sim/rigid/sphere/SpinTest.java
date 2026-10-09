package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A sphere's spin, before anything can change it: nothing applies a torque yet, so a spin is kept exactly, and turns
 * the orientation at the rate it says, under every solve and on each backend.
 */
class SpinTest {

    /**
     * A sphere spun at half a turn a second about {@code (1, 2, 2) / 3}, with no gravity, after one second is turned
     * by π about that axis: the quaternion {@code (1, 2, 2) / 3, 0}. The first-order turn is short by {@code θ³ / 12}
     * a substep, 3e-5 rad over the 600 here, so the orientation is judged to 2e-4.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.sphere.SpheresTest#everySolve")
    void aSpinIsKeptAndTurnsTheSphereAtItsRate(Backend backend, SphereStep.Solve solve) {
        try (Scene scene = new Scene(1).uniform(0.1, 1)) {
            scene.gy = 0;
            scene.x[0] = scene.y[0] = scene.z[0] = 0.5f;
            float rate = (float) (Math.PI / 3);
            scene.ax[0] = rate;
            scene.ay[0] = 2 * rate;
            scene.az[0] = 2 * rate;
            scene.solve = solve;
            scene.start(backend).advanceSeconds(1);
            scene.read();

            assertEquals(rate, scene.ax[0], 0, "nothing changes the spin");
            assertEquals(2 * rate, scene.ay[0], 0, "nothing changes the spin");
            assertEquals(2 * rate, scene.az[0], 0, "nothing changes the spin");
            float[] q = scene.orientation(0);
            double[] expected = {1 / 3.0, 2 / 3.0, 2 / 3.0, 0};
            for (int k = 0; k < 4; k++) {
                assertEquals(expected[k], q[k], 2e-4, "orientation component " + k);
            }
            assertEquals(0.5, scene.x[0], 0, "spinning does not move it");
        }
    }

    /**
     * A sphere spinning on the floor, with no friction, keeps its spin and rests where a still one would; and the
     * diagnostics count the spin's energy, {@code ½ · ⅖ m r² ω²}.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.sphere.SpheresTest#everySolve")
    void aSpinningSphereRestsOnTheFloorStillSpinning(Backend backend, SphereStep.Solve solve) {
        try (Scene scene = new Scene(1).uniform(0.1, 2)) {
            scene.x[0] = scene.z[0] = 0.5f;
            scene.y[0] = 0.3f;
            scene.ax[0] = 10;
            scene.solve = solve;
            scene.start(backend).advanceSeconds(1);
            SphereDiagnostics state = scene.diagnostics();

            assertEquals(10, scene.ax[0], 0, "no friction, so the floor does not touch the spin");
            assertEquals(0.1, scene.y[0], 1e-6, "resting on the floor");
            assertEquals(0.5 * 0.4 * 2 * 0.1 * 0.1 * 100, state.rotational(), 1e-6, "the spin's energy");
            assertTrue(state.kinetic() - state.rotational() < 1e-6, "nothing else moves: " + state);
        }
    }
}

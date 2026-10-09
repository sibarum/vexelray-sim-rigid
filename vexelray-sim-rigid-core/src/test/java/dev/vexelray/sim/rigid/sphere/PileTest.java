package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A pile: what must hold however well or badly the solver settles it. How well it settles is
 * {@link SphereSweepTest}'s to measure; this only judges that it does not go wrong.
 */
@Tag("physics")
class PileTest {

    /**
     * A hundred and twenty-five spheres dropped as a loose lattice into a box, for three seconds, with every pair
     * tested, with the grid, and by Gauss–Seidel. Nothing goes non-finite, nothing leaves the box, no sphere passes
     * into another by more than a fifth of a radius, and the pile ends with less energy than it started with:
     * contact only ever takes energy away.
     */
    @ParameterizedTest
    @CsvSource({"CPU, false, JACOBI", "CPU, true, JACOBI",
            "CPU, true, GAUSS_SEIDEL_BY_CONTACT", "GPU, false, JACOBI", "GPU, true, JACOBI",
            "GPU, true, GAUSS_SEIDEL_BY_CONTACT"})
    void aDroppedPileStaysInsideApartAndLosesEnergy(Backend backend, boolean grid, SphereStep.Solve solve) {
        int side = 5;
        try (Scene scene = new Scene(side * side * side).uniform(0.03, 0.1)) {
            scene.sx = scene.sz = 0.4;
            scene.sy = 1;
            scene.substeps = 20;
            scene.grid = grid;
            scene.solve = solve;
            Random random = new Random(11);
            int k = 0;
            for (int i = 0; i < side; i++) {
                for (int j = 0; j < side; j++) {
                    for (int l = 0; l < side; l++, k++) {
                        scene.x[k] = (float) (0.07 + 0.065 * i + 0.005 * random.nextDouble());
                        scene.y[k] = (float) (0.2 + 0.07 * j);
                        scene.z[k] = (float) (0.07 + 0.065 * l + 0.005 * random.nextDouble());
                    }
                }
            }
            SphereDiagnostics start = SphereDiagnostics.of(scene.x, scene.y, scene.z, scene.u, scene.v, scene.w,
                    scene.r, scene.im, scene.gy, scene.sx, scene.sy, scene.sz);
            scene.start(backend);
            double worst = 0;
            for (int second = 0; second < 3; second++) {
                SphereDiagnostics state = scene.advanceSeconds(1).diagnostics();
                assertFalse(state.broken(), "non-finite after " + (second + 1) + " s");
                assertTrue(state.maxWall() < 1e-5, "outside the box: " + state);
                worst = Math.max(worst, state.maxOverlap());
                assertTrue(state.kinetic() + state.potential() < start.kinetic() + start.potential(),
                        "energy grew: " + state + " from " + start);
            }
            assertTrue(worst < 0.2, "spheres passed into each other by " + worst + " of a radius");
        }
    }
}

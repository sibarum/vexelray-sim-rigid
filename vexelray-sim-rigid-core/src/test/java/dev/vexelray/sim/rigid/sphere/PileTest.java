package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;

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
     * contact only ever takes energy away. Gauss–Seidel also with friction, which only takes energy too, spin
     * counted.
     */
    @ParameterizedTest
    @CsvSource({"CPU, false, JACOBI, 0", "CPU, true, JACOBI, 0",
            "CPU, true, GAUSS_SEIDEL_BY_CONTACT, 0", "CPU, true, GAUSS_SEIDEL_BY_CONTACT, 0.5",
            "GPU, false, JACOBI, 0", "GPU, true, JACOBI, 0",
            "GPU, true, GAUSS_SEIDEL_BY_CONTACT, 0", "GPU, true, GAUSS_SEIDEL_BY_CONTACT, 0.5"})
    void aDroppedPileStaysInsideApartAndLosesEnergy(Backend backend, boolean grid, SphereStep.Solve solve,
                                                    double mu) {
        int side = 5;
        try (Scene scene = new Scene(side * side * side).uniform(0.03, 0.1)) {
            scene.sx = scene.sz = 0.4;
            scene.sy = 1;
            scene.substeps = 20;
            scene.grid = grid;
            scene.solve = solve;
            scene.muStatic = scene.muKinetic = mu;
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

    /**
     * Three hundred and forty-three spheres dropped with friction, μ = 0.5, at the demo's 10 substeps: once settled,
     * the pile rests. Measured before contacts kept their impulses from substep to substep and before friction waited
     * for every contact to be parted: from 4 s to 6 s it still lost 0.25–0.4 J of potential energy a second, with 0.02
     * J moving, most of it spin. Since, 1e-4 J a second, and 3e-7 J moving.
     */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void aPileWithFrictionComesToRest(Backend backend) {
        int side = 7;
        try (Scene scene = new Scene(side * side * side).uniform(0.03, 0.1)) {
            scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
            scene.muStatic = scene.muKinetic = 0.5;
            scene.sx = scene.sz = 0.08 + 0.06 * side;
            scene.sy = 0.5 + 0.07 * side;
            Random random = new Random(7);
            int k = 0;
            for (int i = 0; i < side; i++) {
                for (int j = 0; j < side; j++) {
                    for (int l = 0; l < side; l++, k++) {
                        scene.x[k] = (float) (0.07 + 0.06 * i + 0.005 * random.nextDouble());
                        scene.y[k] = (float) (0.3 + 0.07 * j);
                        scene.z[k] = (float) (0.07 + 0.06 * l + 0.005 * random.nextDouble());
                    }
                }
            }
            scene.start(backend).advanceSeconds(4);
            double settled = scene.diagnostics().potential();
            SphereDiagnostics later = scene.advanceSeconds(2).diagnostics();
            double creep = (settled - later.potential()) / 2;
            assertTrue(creep < 5e-3, "still creeping, " + creep + " J a second: " + later);
            assertTrue(later.kinetic() < 1e-4, "at rest: " + later);
        }
    }
}

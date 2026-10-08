package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Gauss–Seidel over the contact list, run until every contact is solved: the same answer as running enough rounds,
 * and nothing left unsolved.
 */
class UntilDoneTest {

    /** Rounds enough for the pile below, with room; the fixed solve's last round checks and does not claim. */
    private static final int ENOUGH = SphereStep.CYCLE - 1;

    /**
     * A crowded pile of mixed sizes and masses, a quarter of a second, both ways. The rounds run until done are the
     * fixed solve's first rounds, on the same dispatches, and those after the last contact find nothing to do; so
     * while no pass needs more than {@link #ENOUGH}, the two leave the same state, to the bit.
     */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void untilDoneIsEnoughRoundsAndLeavesNothingUnsolved(Backend backend) {
        float[][] fixed;
        try (Scene scene = pile()) {
            scene.fixedRounds = true;
            scene.rounds = ENOUGH;
            scene.start(backend).advanceSeconds(0.25).read();
            fixed = new float[][] {scene.x.clone(), scene.y.clone(), scene.z.clone(), scene.v.clone()};
            assertEquals(0, scene.contactCount()[Spheres.MISSED], "the fixed solve had enough rounds");
        }
        try (Scene scene = pile()) {
            scene.start(backend).advanceSeconds(0.25).read();
            System.out.printf("[until done] %s: %.1f waits and %.1f rounds a step, at most %d in a pass%n", backend,
                    scene.waits / 15.0, scene.roundsRun / 15.0, scene.mostRounds);
            assertTrue(scene.mostRounds <= ENOUGH, "a pass needed " + scene.mostRounds + " rounds; the comparison "
                    + "needs a pile that fits the fixed solve's");
            assertEquals(0, scene.unlisted, "every contact fitted the list");
            assertArrayEquals(fixed[0], scene.x, 0f);
            assertArrayEquals(fixed[1], scene.y, 0f);
            assertArrayEquals(fixed[2], scene.z, 0f);
            assertArrayEquals(fixed[3], scene.v, 0f);
        }
    }

    private static Scene pile() {
        int n = 300;
        Scene scene = new Scene(n);
        scene.sx = scene.sz = 0.5;
        scene.sy = 1;
        scene.substeps = 10;
        scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
        Random random = new Random(5);
        for (int k = 0; k < n; k++) {
            scene.r[k] = (float) (0.02 + 0.015 * random.nextDouble());
            scene.im[k] = (float) (1 / (0.05 + 0.2 * random.nextDouble()));
            scene.x[k] = (float) (0.05 + 0.4 * random.nextDouble());
            scene.y[k] = (float) (0.05 + 0.3 * random.nextDouble());
            scene.z[k] = (float) (0.05 + 0.4 * random.nextDouble());
        }
        return scene;
    }
}

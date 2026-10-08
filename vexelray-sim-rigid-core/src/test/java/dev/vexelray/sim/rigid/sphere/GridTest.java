package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** The broad phase: the grid finds the contacts every pair finds, and nothing else changes. */
class GridTest {

    /**
     * Six hundred spheres of mixed sizes, crowded into a box so that most overlap, some past its walls, stepped
     * once by each search from the same state. The grid sums each sphere's contacts in another order, so the two
     * agree to rounding, not to the bit; a missed contact would leave a sphere a sizeable fraction of a radius
     * off.
     */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void theGridFindsWhatEveryPairFinds(Backend backend) {
        Scene pairs = crowd();
        Scene grid = crowd();
        grid.grid = true;
        try (pairs; grid) {
            pairs.start(backend).advance(1).read();
            grid.start(backend).advance(1).read();
            double worst = 0;
            for (int k = 0; k < pairs.n; k++) {
                worst = Math.max(worst, Math.abs(pairs.x[k] - grid.x[k]));
                worst = Math.max(worst, Math.abs(pairs.y[k] - grid.y[k]));
                worst = Math.max(worst, Math.abs(pairs.z[k] - grid.z[k]));
            }
            assertTrue(worst < 1e-5, "the searches disagree by " + worst + " m");
        }
    }

    private static Scene crowd() {
        int n = 600;
        Scene scene = new Scene(n);
        scene.sx = 0.6;
        scene.sy = 0.4;
        scene.sz = 0.5;
        Random random = new Random(5);
        for (int k = 0; k < n; k++) {
            scene.r[k] = (float) (0.02 + 0.03 * random.nextDouble());
            scene.im[k] = (float) (1 / (0.1 + random.nextDouble()));
            scene.x[k] = (float) (-0.05 + 0.7 * random.nextDouble());
            scene.y[k] = (float) (0.4 * random.nextDouble());
            scene.z[k] = (float) (0.5 * random.nextDouble());
        }
        return scene;
    }

    @Test
    void aGridCoversTheBoxWithCellsAsWideAsTheWidestSphere() {
        SphereGrid grid = SphereGrid.covering(1, 0.45, 0.1, 0.1);
        assertEquals(10, grid.nx());
        assertEquals(5, grid.ny());
        assertEquals(1, grid.nz());
        assertEquals(50, grid.cells());
        assertThrows(IllegalArgumentException.class, () -> SphereGrid.covering(1, 1, 1, 0));
    }
}

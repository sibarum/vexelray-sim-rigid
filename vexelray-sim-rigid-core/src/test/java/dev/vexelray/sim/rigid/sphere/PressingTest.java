package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * The pressing {@link Spheres#show} leaves for a picture: each sphere's flattening and lean, from the overlaps the step
 * ended with, against the same sums worked out on the host.
 */
class PressingTest {

    /**
     * Fixed spheres, which nothing parts, overlapping each other and the walls: a sphere of 0.1 in the low x wall by
     * half its radius, one beside it, one of 0.15 above that, and one of 0.1 alone in the ceiling. The pressing is
     * the overlaps', shares divided at the radii, to f32's rounding; and the step before's is kept in the "before"
     * slot.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.sphere.SpheresTest#everySolve")
    void thePressingIsTheOverlapsTheStepEndedWith(Backend backend, SphereStep.Solve solve) {
        try (Scene scene = new Scene(4)) {
            float[][] at = {{0.05f, 0.5f, 0.5f}, {0.2f, 0.55f, 0.5f}, {0.25f, 0.72f, 0.5f}, {0.5f, 0.95f, 0.5f}};
            float[] radii = {0.1f, 0.1f, 0.15f, 0.1f};
            for (int s = 0; s < 4; s++) {
                scene.x[s] = at[s][0];
                scene.y[s] = at[s][1];
                scene.z[s] = at[s][2];
                scene.r[s] = radii[s];
                scene.im[s] = 0;
            }
            scene.solve = solve;
            scene.start(backend).advance(1);
            float[] earlier = scene.shown().clone();
            scene.advance(1).read();
            float[] shown = scene.shown();

            double[][] expected = new double[4][];
            for (int s = 0; s < 4; s++) {
                int base = s * Spheres.SHOWN_STRIDE;
                expected[s] = pressing(scene, s);
                for (int k = 0; k <= Spheres.LEAN_LENGTH; k++) {
                    assertEquals(expected[s][k], shown[base + Spheres.PRESSED_AFTER + k], 1e-6,
                            "sphere " + s + ", word " + k);
                    assertEquals(earlier[base + Spheres.PRESSED_AFTER + k], shown[base + Spheres.PRESSED_BEFORE + k],
                            0, "sphere " + s + ", word " + k + " kept from the step before");
                }
            }
            // That the scene presses as described, so the comparison above is not of zeros.
            assertEquals(0.7094, expected[0][Spheres.TRACE], 1e-3, "half from the wall, and the neighbour's share");
            assertEquals(-0.3013, expected[0][Spheres.LEAN], 1e-3, "pressed more from the wall's side");
            assertEquals(0.2912, expected[2][Spheres.TRACE], 1e-3, "the larger sphere's share of its overlap");
            assertEquals(0.5, expected[3][Spheres.LEAN + 1], 1e-3, "in the ceiling by half its radius");
        }
    }

    /** Sphere {@code s}'s pressing as {@link Spheres#show} sums it, in doubles from the positions read back. */
    private static double[] pressing(Scene scene, int s) {
        double[] p = new double[Spheres.PRESSING_WORDS];
        double[] c = {scene.x[s], scene.y[s], scene.z[s]};
        double r = scene.r[s];
        double[] extent = {scene.sx, scene.sy, scene.sz};
        for (int a = 0; a < 3; a++) {
            double[] depths = {r - c[a], c[a] + r - extent[a]};
            for (int side = 0; side < 2; side++) {
                if (depths[side] > 0) {
                    double[] n = new double[3];
                    n[a] = side == 0 ? -1 : 1;
                    add(p, depths[side] / r, n);
                }
            }
        }
        for (int o = 0; o < scene.n; o++) {
            double[] d = {scene.x[o] - c[0], scene.y[o] - c[1], scene.z[o] - c[2]};
            double dist = Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            double reach = r + scene.r[o];
            if (o != s && dist < reach) {
                add(p, (reach - dist) / reach, new double[] {d[0] / dist, d[1] / dist, d[2] / dist});
            }
        }
        p[Spheres.TRACE] = p[0] + p[1] + p[2];
        p[Spheres.LEAN_LENGTH] = Math.sqrt(p[6] * p[6] + p[7] * p[7] + p[8] * p[8]);
        return p;
    }

    /** {@code f n nᵀ} into the flattening and {@code f n} into the lean. */
    private static void add(double[] p, double f, double[] n) {
        int[][] pairs = {{0, 0}, {1, 1}, {2, 2}, {0, 1}, {0, 2}, {1, 2}};
        for (int k = 0; k < 6; k++) {
            p[Spheres.FLATTENING + k] += f * n[pairs[k][0]] * n[pairs[k][1]];
        }
        for (int a = 0; a < 3; a++) {
            p[Spheres.LEAN + a] += f * n[a];
        }
    }
}

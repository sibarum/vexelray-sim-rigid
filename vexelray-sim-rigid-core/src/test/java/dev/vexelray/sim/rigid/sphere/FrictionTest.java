package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Friction, where the answer is known: a solid sphere sliding into rolling, rolling or sliding down a slope, and a
 * glancing collision that must keep its momentum, and its angular momentum to first order. Gauss–Seidel over the
 * contact list, the solve that has friction, on each backend.
 */
class FrictionTest {

    private static final double G = 9.81;

    private static Scene onTheFloor(double mu) {
        Scene scene = new Scene(1).uniform(0.1, 1);
        scene.sx = 4;
        scene.x[0] = 0.5f;
        scene.y[0] = 0.1f;
        scene.z[0] = 0.5f;
        scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
        scene.muStatic = scene.muKinetic = mu;
        return scene;
    }

    /**
     * A sphere set sliding along the floor at {@code v₀} with no spin. Kinetic friction slows it by {@code μ g} and
     * spins it up by {@code 5 μ g / 2r} until its contact point stops, at {@code t = 2 v₀ / 7 μ g}; from then it
     * rolls at {@code 5/7 v₀} with {@code ω = −v / r}, whatever {@code μ} is. Here 2 m/s at μ = 0.3: rolling from
     * 0.194 s at 1.4286 m/s.
     */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void aSlidingSphereSettlesIntoRollingAtFiveSevenths(Backend backend) {
        double mu = 0.3;
        double v0 = 2;
        try (Scene scene = onTheFloor(mu)) {
            scene.u[0] = (float) v0;
            scene.start(backend).advanceSeconds(0.1);
            scene.read();
            double t = 0.1;
            assertEquals(v0 - mu * G * t, scene.u[0], 1e-2, "slowed by μ g while it slides");
            assertEquals(-2.5 * mu * G * t / 0.1, scene.az[0], 0.1, "spun up by 5 μ g / 2r while it slides");

            scene.advanceSeconds(0.4);
            SphereDiagnostics state = scene.diagnostics();
            assertEquals(5 * v0 / 7, scene.u[0], 5e-3, "rolling at 5/7 of where it started: " + state);
            assertEquals(-scene.u[0] / 0.1, scene.az[0], 1e-2, "its contact point at rest: ω = −v / r");
            assertEquals(0.1, scene.y[0], 1e-5, "on the floor");
            assertEquals(0, scene.ax[0], 1e-6, "turned about no other axis");
            assertEquals(0, scene.ay[0], 1e-6, "turned about no other axis");
        }
    }

    /**
     * Gravity tilted by 20° along x makes the floor a slope. With enough friction, {@code μ ≥ 2/7 tan θ} or 0.104, the
     * sphere rolls down it at {@code 5/7 g sin θ}; with less it slides, at {@code g (sin θ − μ cos θ)}, and spins up
     * at {@code 5 μ g cos θ / 2r}. Half a second from rest, each.
     */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void onASlopeASphereRollsWithEnoughFrictionAndSlidesWithout(Backend backend) {
        double theta = Math.toRadians(20);
        double t = 0.5;
        try (Scene rolling = onTheFloor(0.5)) {
            rolling.x[0] = 0.2f;
            rolling.gx = G * Math.sin(theta);
            rolling.gy = -G * Math.cos(theta);
            rolling.start(backend).advanceSeconds(t);
            rolling.read();
            assertEquals(5.0 / 7 * G * Math.sin(theta) * t, rolling.u[0], 1e-2, "rolling at 5/7 g sin θ");
            assertEquals(-rolling.u[0] / 0.1, rolling.az[0], 1e-2, "without slipping");
        }
        double mu = 0.05;
        try (Scene sliding = onTheFloor(mu)) {
            sliding.x[0] = 0.2f;
            sliding.gx = G * Math.sin(theta);
            sliding.gy = -G * Math.cos(theta);
            sliding.start(backend).advanceSeconds(t);
            sliding.read();
            assertEquals(G * (Math.sin(theta) - mu * Math.cos(theta)) * t, sliding.u[0], 1e-2,
                    "sliding at g (sin θ − μ cos θ)");
            assertEquals(-2.5 * mu * G * Math.cos(theta) * t / 0.1, sliding.az[0], 0.05,
                    "spun up at 5 μ g cos θ / 2r");
        }
    }

    /**
     * A sphere of 1 kg at 2 m/s glancing off one of 3 kg at rest, a quarter of the radii's sum off centre, with no
     * gravity and μ = 0.5. Friction spins them both and takes energy; the momentum is kept as the solve keeps it.
     *
     * <p>The angular momentum about the origin is kept to first order in the substep, and friction keeps it no worse
     * than the contact does without it. Measured: a frictionless pair changes {@code L} by 1.2e-3 of 1 at 10 substeps,
     * 6e-4 at 20 and 3.5e-4 at 40, and with friction 8e-4, 3e-4 and 1.7e-4. Friction acts at one point both spheres
     * share, so it adds nothing of its own; what is left is XPBD's. A correction is directed by the predicted
     * positions and changes {@code L} by {@code Σ m x₀ × Δx / h}, with {@code x₀} the substep's start, which is not
     * along the correction for a pair moving across each other.
     */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void aGlancingCollisionKeepsMomentumAndAngularMomentumToFirstOrder(Backend backend) {
        double[] frictionless = new double[2];
        double[] withFriction = new double[2];
        for (int run = 0; run < 2; run++) {
            for (double mu : new double[] {0, 0.5}) {
                try (Scene scene = glancing(mu)) {
                    scene.substeps = 10 << run;
                    SphereDiagnostics start = SphereDiagnostics.of(scene.x, scene.y, scene.z, scene.u, scene.v,
                            scene.w, scene.ax, scene.ay, scene.az, scene.r, scene.im, 0, scene.sx, scene.sy, scene.sz);
                    scene.start(backend).advanceSeconds(0.5);
                    SphereDiagnostics end = scene.diagnostics();
                    for (int k = 0; k < 3; k++) {
                        assertEquals(start.momentum()[k], end.momentum()[k], 1e-5, "momentum " + k + ": " + end);
                    }
                    double change = Math.abs(end.angular()[2] - start.angular()[2]);
                    assertEquals(0, end.angular()[0], 1e-6, "no angular momentum out of the plane: " + end);
                    assertEquals(start.angular()[1], end.angular()[1], 1e-6, "none out of the plane: " + end);
                    if (mu == 0) {
                        frictionless[run] = change;
                        assertEquals(0, end.rotational(), 0, "without friction nothing spins: " + end);
                    } else {
                        withFriction[run] = change;
                        assertTrue(Math.abs(scene.az[0]) > 1 && Math.abs(scene.az[1]) > 0.1,
                                "friction spun them: " + scene.az[0] + ", " + scene.az[1]);
                        assertTrue(end.kinetic() < start.kinetic(), "friction only takes energy: " + end);
                    }
                }
            }
            assertTrue(withFriction[run] <= frictionless[run], "friction no worse than the contact: "
                    + withFriction[run] + " against " + frictionless[run]);
        }
        assertTrue(withFriction[1] < 0.6 * withFriction[0], "first order: halved with the substep, "
                + withFriction[0] + " then " + withFriction[1]);
        assertTrue(withFriction[0] < 1.5e-3, "small at 10 substeps: " + withFriction[0]);
    }

    private static Scene glancing(double mu) {
        Scene scene = new Scene(2).uniform(0.1, 1);
        scene.gy = 0;
        scene.sx = 4;
        scene.im[1] = 1 / 3f;
        scene.x[0] = 1;
        scene.x[1] = 1.5f;
        scene.y[0] = 0.5f;
        scene.y[1] = 0.55f;
        scene.z[0] = scene.z[1] = 0.5f;
        scene.u[0] = 2;
        scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
        scene.muStatic = scene.muKinetic = mu;
        return scene;
    }

    /** A column of ten with friction rests as it does without: friction must not stir what is still. */
    @ParameterizedTest
    @EnumSource(Backend.class)
    void aColumnStillRestsWithFriction(Backend backend) {
        try (Scene scene = new Scene(10).uniform(0.05, 1)) {
            for (int k = 0; k < 10; k++) {
                scene.x[k] = scene.z[k] = 0.5f;
                scene.y[k] = 0.05f + 0.1f * k;
            }
            scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
            scene.muStatic = scene.muKinetic = 0.5;
            scene.start(backend).advanceSeconds(2);
            SphereDiagnostics state = scene.diagnostics();
            assertTrue(state.kinetic() < 1e-6, "at rest: " + state);
            assertTrue(state.maxOverlap() < 0.05, "standing: " + state);
            assertTrue(Math.abs(scene.x[9] - 0.5) < 1e-4, "upright: the top at x " + scene.x[9]);
        }
    }
}

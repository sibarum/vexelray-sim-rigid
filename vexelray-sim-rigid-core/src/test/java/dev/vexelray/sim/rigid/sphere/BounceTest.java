package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.Arrays;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Restitution ({@link Spheres#bounce}): a ball rebounds to {@code e²} of its drop, a head-on collision of equal masses
 * at {@code e = 1} swaps their velocities and keeps momentum, and a ball resting on the floor stays at rest however
 * springy it is.
 */
class BounceTest {

    /** Every backend under every solve. */
    static Stream<Arguments> everySolve() {
        return Arrays.stream(Backend.values()).flatMap(backend -> Arrays.stream(SphereStep.Solve.values())
                .map(solve -> Arguments.of(backend, solve)));
    }

    private static final double RADIUS = 0.05;
    private static final double DROP = 1.0;

    /**
     * A ball dropped {@link #DROP} onto the floor, from rest. It strikes at {@code √(2gH)} and leaves at {@code e} of
     * that, so it rises to {@code e² H}. The discrete fall and the substep that the contact lands in cost a little
     * either way; to within a hundredth of the drop, and measured at a thousandth.
     */
    @ParameterizedTest
    @MethodSource("everySolve")
    void aBallReboundsToTheSquareOfItsRestitution(Backend backend, SphereStep.Solve solve) {
        for (double e : new double[] {0.5, 0.9}) {
            double rose = rebound(backend, solve, e);
            System.out.printf("[bounce] %s %s: e = %.1f rose %.3f m of %.1f, e^2 is %.3f%n", backend, solve, e, rose, DROP,
                    e * e);
            assertEquals(e * e * DROP, rose, 0.01 * DROP, "e = " + e + " on " + backend + " " + solve);
        }
    }

    @ParameterizedTest
    @MethodSource("everySolve")
    void withoutRestitutionABallDoesNotBounce(Backend backend, SphereStep.Solve solve) {
        assertTrue(rebound(backend, solve, 0) < 0.01 * DROP, "it should stay down");
    }

    /** The highest the ball gets, above where it rests, after it first hits the floor. */
    private static double rebound(Backend backend, SphereStep.Solve solve, double e) {
        try (Scene scene = new Scene(1).uniform(RADIUS, 1)) {
            scene.sx = scene.sz = 0.4;
            scene.sy = 2;
            scene.substeps = 20;
            scene.solve = solve;
            scene.restitution = e;
            scene.x[0] = scene.z[0] = 0.2f;
            scene.y[0] = (float) (RADIUS + DROP);
            scene.start(backend);
            boolean hit = false;
            double highest = 0;
            for (int frame = 0; frame < 150; frame++) {
                scene.advance(1).read();
                double above = scene.y[0] - RADIUS;
                if (!hit && scene.v[0] > 0) {
                    hit = true;
                }
                if (hit) {
                    highest = Math.max(highest, above);
                    if (scene.v[0] < 0 && highest > 0) {
                        break;      // on the way down again
                    }
                }
            }
            return highest;
        }
    }

    /**
     * Two spheres of one mass meet head on, with no gravity, at {@code e = 1}: the one moving stops and the one at rest
     * leaves with its speed. Momentum is kept exactly as the solve keeps it, and the energy within a few percent, the
     * part the position solve's own push gives the pair before restitution sets it.
     */
    @ParameterizedTest
    @MethodSource("everySolve")
    void anElasticCollisionSwapsTheVelocities(Backend backend, SphereStep.Solve solve) {
        try (Scene scene = new Scene(2).uniform(RADIUS, 1)) {
            scene.gy = 0;
            scene.sx = 2;
            scene.sy = scene.sz = 0.4;
            scene.substeps = 20;
            scene.solve = solve;
            scene.restitution = 1;
            scene.x[0] = 0.5f;
            scene.x[1] = 0.8f;
            scene.y[0] = scene.y[1] = scene.z[0] = scene.z[1] = 0.2f;
            scene.u[0] = 1;
            scene.start(backend).advanceSeconds(0.5).read();
            assertEquals(1, scene.u[0] + scene.u[1], 1e-5, "momentum");
            assertEquals(0, scene.u[0], 0.03, "the one that was moving has stopped");
            assertEquals(1, scene.u[1], 0.03, "the one that was at rest has its speed");
        }
    }

    /**
     * A springy ball set down on the floor: gravity gives it {@code g h} into the floor each substep, which is below
     * restitution's threshold, so the contact is a resting one and nothing bounces.
     */
    @ParameterizedTest
    @MethodSource("everySolve")
    void aBallAtRestStaysAtRest(Backend backend, SphereStep.Solve solve) {
        try (Scene scene = new Scene(1).uniform(RADIUS, 1)) {
            scene.sx = scene.sy = scene.sz = 0.4;
            scene.substeps = 10;
            scene.solve = solve;
            scene.restitution = 0.9;
            scene.x[0] = scene.z[0] = 0.2f;
            scene.y[0] = (float) RADIUS;
            scene.start(backend).advanceSeconds(2).read();
            assertTrue(Math.abs(scene.v[0]) < 1e-3, "speed " + scene.v[0]);
            assertEquals(RADIUS, scene.y[0], 1e-4);
        }
    }
}

package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.Arrays;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The cases with an answer known in advance: a fall, a landing, and a collision, under each solve. Each backend is
 * judged on its own.
 */
class SpheresTest {

    /** Every backend under every solve. */
    static Stream<Arguments> everySolve() {
        return Arrays.stream(Backend.values()).flatMap(backend -> Arrays.stream(SphereStep.Solve.values())
                .map(solve -> Arguments.of(backend, solve)));
    }

    /**
     * Unconstrained, a substep is symplectic Euler, whose discrete answer is exact: after {@code k} substeps of
     * {@code h}, {@code v = −g h k} and {@code y = y₀ − g h² k(k + 1) / 2}.
     */
    @ParameterizedTest
    @MethodSource("everySolve")
    void aFallingSphereFollowsSymplecticEulerExactly(Backend backend, SphereStep.Solve solve) {
        try (Scene scene = new Scene(1).uniform(0.1, 1)) {
            scene.sx = scene.sy = scene.sz = 10;
            scene.x[0] = scene.z[0] = 5;
            scene.y[0] = 8;
            scene.solve = solve;
            scene.start(backend).advance(30);
            scene.read();

            int k = 30 * scene.substeps;
            double h = scene.dt / scene.substeps;
            assertEquals(-9.81 * h * k, scene.v[0], 1e-4, "velocity");
            assertEquals(8 - 9.81 * h * h * k * (k + 1) / 2, scene.y[0], 1e-4, "height");
            assertEquals(5, scene.x[0], 0, "nothing moves it sideways");
        }
    }

    /** A dropped sphere lands, does not bounce (contact is inelastic), and rests on the floor rather than in it. */
    @ParameterizedTest
    @MethodSource("everySolve")
    void aDroppedSphereComesToRestOnTheFloor(Backend backend, SphereStep.Solve solve) {
        try (Scene scene = new Scene(1).uniform(0.1, 1)) {
            scene.x[0] = scene.z[0] = 0.5f;
            scene.y[0] = 0.8f;
            scene.solve = solve;
            scene.start(backend).advanceSeconds(1);
            SphereDiagnostics state = scene.diagnostics();

            assertEquals(0.1, scene.y[0], 1e-6, "resting on the floor");
            assertTrue(state.maxSpeed() < 1e-4, "at rest: " + state);
            assertTrue(state.maxWall() < 1e-5, "not in the floor: " + state);
        }
    }

    /**
     * A sphere of 1 kg at 2 m/s into one of 3 kg at rest, with no gravity. A perfectly inelastic collision: the momentum
     * of 2 kg·m/s is kept, and the two leave together at 0.5 m/s, touching and not passing through each other.
     */
    @ParameterizedTest
    @MethodSource("everySolve")
    void aCollisionKeepsMomentumAndTheSpheresLeaveTogether(Backend backend, SphereStep.Solve solve) {
        try (Scene scene = new Scene(2).uniform(0.1, 1)) {
            scene.gy = 0;
            scene.sx = 4;
            scene.y[0] = scene.y[1] = scene.z[0] = scene.z[1] = 0.5f;
            scene.x[0] = 1;
            scene.x[1] = 2;
            scene.u[0] = 2;
            scene.im[1] = 1 / 3f;
            scene.solve = solve;
            scene.start(backend).advanceSeconds(1);
            SphereDiagnostics state = scene.diagnostics();

            assertEquals(2, state.momentum()[0], 1e-6, "momentum: " + state);
            assertEquals(0.5, scene.u[0], 1e-3, "the light one");
            assertEquals(0.5, scene.u[1], 1e-3, "the heavy one");
            assertTrue(scene.x[0] < scene.x[1], "they did not pass through each other");
            assertTrue(state.maxOverlap() < 1e-3, "touching, not overlapping: " + state);
            assertFalse(state.broken());
        }
    }
}

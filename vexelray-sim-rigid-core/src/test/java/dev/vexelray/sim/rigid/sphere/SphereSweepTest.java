package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;

import java.util.Random;

/**
 * The experiment's measurements, not assertions: a column and a pile, under each relaxation and several substep
 * counts, with how deep the spheres sink into each other, how much the stack jitters, and what a step costs.
 * {@code -Drigid.sweep=true} runs it; {@code -Drigid.backend=CPU} runs it on the CPU instead of the GPU.
 */
@EnabledIfSystemProperty(named = "rigid.sweep", matches = "true")
class SphereSweepTest {

    private static final Backend BACKEND = Backend.valueOf(System.getProperty("rigid.backend", "GPU"));

    private record Solver(String name, double omega, boolean averaged, int substeps, int iterations) {
    }

    private static final Solver[] SOLVERS = {
            new Solver("constant omega=1", 1, false, 10, 1),
            new Solver("constant omega=0.5", 0.5, false, 10, 1),
            new Solver("averaged omega=1", 1, true, 10, 1),
            new Solver("averaged omega=1", 1, true, 5, 1),
            new Solver("averaged omega=1", 1, true, 20, 1),
            new Solver("averaged omega=1 2 it", 1, true, 5, 2),
            new Solver("averaged omega=1 10 it", 1, true, 1, 10),
    };

    /** Twenty spheres in a column a sphere wide, touching, under gravity for three seconds. */
    @Test
    void column() {
        System.out.println("column of 20, r = 5 cm, 3 s at 60 Hz, " + BACKEND);
        System.out.println(header());
        for (Solver solver : SOLVERS) {
            int n = 20;
            try (Scene scene = configure(new Scene(n).uniform(0.05, 1), solver)) {
                scene.sx = scene.sz = 0.1;
                scene.sy = 2 * 0.05 * n + 0.5;
                for (int k = 0; k < n; k++) {
                    scene.x[k] = scene.z[k] = 0.05f;
                    scene.y[k] = (float) (0.05 + 0.1 * k);
                }
                float top = scene.y[n - 1];
                run(scene, solver, 180, s -> String.format("top sank %.2e m", top - s.y[n - 1]));
            }
        }
    }

    /** Three hundred and forty-three spheres dropped as a loose lattice into a box, settled for four seconds. */
    @Test
    void pile() {
        System.out.println("pile of 343, r = 3 cm, 4 s at 60 Hz, " + BACKEND);
        System.out.println(header());
        for (Solver solver : SOLVERS) {
            int side = 7;
            int n = side * side * side;
            try (Scene scene = configure(new Scene(n).uniform(0.03, 0.1), solver)) {
                scene.sx = scene.sz = 0.5;
                scene.sy = 1.5;
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
                run(scene, solver, 240, s -> String.format("top %.3f m", max(s.y)));
            }
        }
    }

    private static Scene configure(Scene scene, Solver solver) {
        scene.omega = solver.omega();
        scene.averaged = solver.averaged();
        scene.substeps = solver.substeps();
        scene.iterations = solver.iterations();
        return scene;
    }

    private static String header() {
        return String.format("%-20s %4s %3s  %-10s %-10s %-10s %-10s %-9s  %s", "solver", "sub", "it", "overlap",
                "wall", "KE end", "vmax end", "ms/step", "");
    }

    private static void run(Scene scene, Solver solver, int frames,
                            java.util.function.Function<Scene, String> extra) {
        scene.start(BACKEND).advance(2);   // compilation and first-use costs, out of the timing
        scene.read();
        long start = System.nanoTime();
        scene.advance(frames - 2);
        SphereDiagnostics state = scene.diagnostics();
        double ms = (System.nanoTime() - start) / 1e6 / (frames - 2);
        System.out.println(String.format("%-20s %4d %3d  %-10.2e %-10.2e %-10.2e %-10.2e %-9.3f  %s%s",
                solver.name(), solver.substeps(), solver.iterations(), state.maxOverlap(), state.maxWall(),
                state.kinetic(), state.maxSpeed(), ms, extra.apply(scene), state.broken() ? "  BROKEN" : ""));
    }

    private static float max(float[] values) {
        float m = Float.NEGATIVE_INFINITY;
        for (float value : values) {
            m = Math.max(m, value);
        }
        return m;
    }
}

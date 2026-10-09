package dev.vexelray.sim.rigid.sphere;

import dev.supirvast.vastir.tools.Accelerator;
import dev.supirvast.vastir.tools.PassRunner;

import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * A set of spheres in a box, put on a backend and stepped: what each test builds, then reads back.
 *
 * <p>Set the fields, then {@link #start}; after that {@link #advance} steps on the backend and {@link #read} brings the
 * state back into the same fields.
 */
final class Scene implements AutoCloseable {

    /** Which backend a scene runs on. Each is judged on its own; the two are not compared. */
    enum Backend { CPU, GPU }

    final int n;
    final float[] x;
    final float[] y;
    final float[] z;
    final float[] u;
    final float[] v;
    final float[] w;
    final float[] r;
    final float[] im;

    double gx;
    double gy = -9.81;
    double gz;
    double sx = 1;
    double sy = 1;
    double sz = 1;
    /** The frame: a step is this long, cut into {@link #substeps}. */
    double dt = 1.0 / 60;
    int substeps = 10;
    int iterations = 1;
    double omega = 1;
    boolean averaged = true;
    /** Whether the solve searches a grid, its cells as wide as the widest sphere, or tests every pair. */
    boolean grid;
    /** How a pass solves; Gauss–Seidel searches the grid whatever {@link #grid} says. */
    SphereStep.Solve solve = SphereStep.Solve.JACOBI;
    /** Rounds a pass, for Gauss–Seidel over the contact list when {@link #fixedRounds}. */
    int rounds = SphereStep.ROUNDS;
    /**
     * Whether Gauss–Seidel over the contact list runs {@link #rounds} rounds a pass, as it was first measured, rather
     * than until every contact is solved.
     */
    boolean fixedRounds;

    /** What the steps run until done have taken, added up: see {@link SphereStepper.Report}. */
    long waits;
    long roundsRun;
    int mostRounds;
    long unlisted;

    private Accelerator accelerator;
    private PassRunner runner;
    private SphereStep step;
    private SphereStepper stepper;

    Scene(int n) {
        this.n = n;
        x = new float[n];
        y = new float[n];
        z = new float[n];
        u = new float[n];
        v = new float[n];
        w = new float[n];
        r = new float[n];
        im = new float[n];
        java.util.Arrays.fill(im, 1);
    }

    /** Every sphere this radius and mass. */
    Scene uniform(double radius, double mass) {
        java.util.Arrays.fill(r, (float) radius);
        java.util.Arrays.fill(im, (float) (1 / mass));
        return this;
    }

    /** The widest sphere's diameter. */
    double widest() {
        double widest = 0;
        for (float radius : r) {
            widest = Math.max(widest, 2 * radius);
        }
        return widest;
    }

    Scene start(Backend backend) {
        boolean gridded = grid || solve != SphereStep.Solve.JACOBI;
        step = new SphereStep(n, substeps, iterations, gridded ? SphereGrid.covering(sx, sy, sz, widest()) : null,
                solve, rounds, SphereStep.CONTACTS_PER_SPHERE);
        stepper = new SphereStepper(step);
        if (backend == Backend.CPU) {
            runner = PassRunner.cpu(step, Spheres.WORKGROUP, PassRunner.NO_SUBGROUP);
        } else {
            accelerator = new Accelerator();
            if (!accelerator.capabilities().gpuAvailable()) {
                accelerator.close();
                accelerator = null;
                assumeTrue(Boolean.getBoolean("supirvast.requireGpu"), "no Vulkan device");
                throw new IllegalStateException("-Dsupirvast.requireGpu=true but no Vulkan device");
            }
            runner = PassRunner.gpu(accelerator, step, Spheres.WORKGROUP, PassRunner.NO_SUBGROUP);
            runner.clear();
        }
        step.constants().forEach(runner::write);
        runner.write("params", Spheres.params(dt / substeps, gx, gy, gz, sx, sy, sz, omega, averaged));
        runner.write("x", x);
        runner.write("y", y);
        runner.write("z", z);
        runner.write("u", u);
        runner.write("v", v);
        runner.write("w", w);
        runner.write("r", r);
        runner.write("im", im);
        return this;
    }

    /** {@code frames} steps of {@link #dt}. */
    Scene advance(int frames) {
        SphereStepper.Runner run = new SphereStepper.Runner() {
            @Override
            public void run(java.util.List<dev.supirvast.vastir.pass.Pass> passes) {
                runner.run(passes);
            }

            @Override
            public int[] runAndRead(java.util.List<dev.supirvast.vastir.pass.Pass> passes) {
                runner.run(passes).await();
                return runner.peek("readout");
            }
        };
        for (int k = 0; k < frames; k++) {
            if (fixedRounds || !step.untilDone()) {
                runner.run(step.step());
                continue;
            }
            SphereStepper.Report report = stepper.step(run);
            waits += report.waits();
            roundsRun += report.rounds();
            mostRounds = Math.max(mostRounds, report.mostRounds());
            unlisted += report.unlisted();
        }
        return this;
    }

    /** {@link #advance} for as many frames as {@code seconds} takes. */
    Scene advanceSeconds(double seconds) {
        return advance((int) Math.round(seconds / dt));
    }

    Scene read() {
        copy(runner.floats("x"), x);
        copy(runner.floats("y"), y);
        copy(runner.floats("z"), z);
        copy(runner.floats("u"), u);
        copy(runner.floats("v"), v);
        copy(runner.floats("w"), w);
        return this;
    }


    /** The contact list's counts ({@link Spheres#CONTACT_COUNT_WORDS}), or null where the solve keeps no list. */
    int[] contactCount() {
        return step.buffers().containsKey("contactCount") ? runner.read("contactCount") : null;
    }
    SphereDiagnostics diagnostics() {
        read();
        return SphereDiagnostics.of(x, y, z, u, v, w, r, im, gy, sx, sy, sz);
    }

    private static void copy(float[] from, float[] to) {
        System.arraycopy(from, 0, to, 0, to.length);
    }

    @Override
    public void close() {
        if (runner != null) {
            runner.close();
        }
        if (accelerator != null) {
            accelerator.close();
        }
    }
}

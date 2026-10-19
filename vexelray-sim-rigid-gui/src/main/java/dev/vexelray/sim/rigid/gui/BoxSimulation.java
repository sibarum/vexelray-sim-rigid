package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.tools.Accelerator;
import dev.supirvast.vastir.tools.Completion;
import dev.supirvast.vastir.tools.GpuContext;
import dev.supirvast.vastir.tools.PassRunner;
import dev.supirvast.vastir.tools.ResidentBuffer;
import dev.vexelray.sim.rigid.box.BoxDiagnostics;
import dev.vexelray.sim.rigid.box.BoxStep;
import dev.vexelray.sim.rigid.box.BoxStepper;
import dev.vexelray.sim.rigid.box.Boxes;
import dev.vexelray.sim.rigid.sphere.SphereStepper;

/**
 * Boxes in a walled box, stepped on resident buffers: {@link BoxStep} on a {@link PassRunner}, as
 * {@link SphereSimulation} steps spheres. A box step is one list, recorded once, which waits for nothing until the
 * end; so far boxes meet only the walls.
 *
 * <p>Made in two halves, as the spheres' is: the constructor allocates and registers, which is slow and may run on any
 * thread; everything after is the owning thread's.
 */
public final class BoxSimulation implements RigidSimulation {

    /** A box step solves no list, so it waits for nothing and has no rounds to report. */
    private static final SphereStepper.Report SOLVED = new SphereStepper.Report(0, 0, 0, 0);

    private final Accelerator accelerator;
    private final BoxStep step;
    private final PassRunner runner;
    private final BoxStepper stepper;
    private BoxStepper.Report last = new BoxStepper.Report(0, 0, 0, 0, 0);

    /** {@code step} on {@code context}, or on a device of its own if that is null. */
    public BoxSimulation(GpuContext context, BoxStep step) {
        this(context == null ? new Accelerator() : Accelerator.on(context), step);
    }

    /** {@code step} on the CPU, through Truffle. */
    public static BoxSimulation onCpu(BoxStep step) {
        return new BoxSimulation((Accelerator) null, step);
    }

    private BoxSimulation(Accelerator accelerator, BoxStep step) {
        this.accelerator = accelerator;
        this.step = step;
        this.runner = accelerator == null ? PassRunner.cpu(step, Boxes.WORKGROUP, PassRunner.NO_SUBGROUP)
                : PassRunner.gpu(accelerator, step, Boxes.WORKGROUP, PassRunner.NO_SUBGROUP);
        this.stepper = new BoxStepper(step);
        runner.prepare(step.step(0));
    }

    @Override
    public ShownRing.Shape shape() {
        return ShownRing.Shape.BOXES;
    }

    @Override
    public int bodies() {
        return step.boxes;
    }

    @Override
    public int shownWords() {
        return Boxes.SHOWN_STRIDE * step.boxes;
    }

    public int substeps() {
        return step.substeps;
    }

    /**
     * Starts from this state, each array as long as the box count: centres, velocities, angular velocities in the
     * world's frame, orientations ({@code qx, qy, qz, qw}), half-extents ({@code hx, hy, hz}) and inverse masses. The
     * angular velocities are written as the angular momentum they make, which is what a box keeps.
     */
    public void start(float[] x, float[] y, float[] z, float[] u, float[] v, float[] w, float[][] spin, float[][] q,
                      float[][] half, float[] im, int[] params) {
        runner.clear();
        step.constants().forEach(runner::write);
        float[][] momentum = new float[3][step.boxes];
        for (int s = 0; s < step.boxes; s++) {
            if (im[s] <= 0) {
                continue;
            }
            double[] l = BoxDiagnostics.momentum(new double[] {spin[0][s], spin[1][s], spin[2][s]},
                    new double[] {q[0][s], q[1][s], q[2][s], q[3][s]},
                    Boxes.inertia(1 / im[s], half[0][s], half[1][s], half[2][s]));
            for (int a = 0; a < 3; a++) {
                momentum[a][s] = (float) l[a];
            }
        }
        String[][] names = {{"x", "y", "z"}, {"u", "v", "w"}, {"lx", "ly", "lz"}, {"hx", "hy", "hz"}};
        float[][][] values = {{x, y, z}, {u, v, w}, momentum, half};
        for (int f = 0; f < names.length; f++) {
            for (int a = 0; a < 3; a++) {
                runner.write(names[f][a], values[f][a]);
            }
        }
        String[] turn = {"qx", "qy", "qz", "qw"};
        for (int k = 0; k < 4; k++) {
            runner.write(turn[k], q[k]);
        }
        runner.write("im", im);
        runner.write("params", params);
        runner.write("shown", Boxes.shownAtStart(x, y, z, q, half));
    }

    /** New parameters ({@link Boxes#params}), from the next step on. */
    public void params(int[] params) {
        runner.write("params", params);
    }

    @Override
    public SphereSimulation.StepReport stepAndWait() {
        long start = System.nanoTime();
        Completion[] done = new Completion[1];
        int[] submissions = new int[1];
        last = stepper.step(new BoxStepper.Runner() {
            @Override
            public void run(java.util.List<dev.supirvast.vastir.pass.Pass> passes) {
                done[0] = runner.run(passes);
                submissions[0]++;
            }

            @Override
            public int[] runAndRead(java.util.List<dev.supirvast.vastir.pass.Pass> passes, String buffer) {
                run(passes);
                done[0].await();
                return runner.read(buffer);
            }

            @Override
            public void write(String buffer, int[] values) {
                runner.write(buffer, values);
            }
        });
        runner.finish();
        return new SphereSimulation.StepReport(SOLVED, submissions[0], System.nanoTime() - start,
                done[0].gpuNanos().orElse(0));
    }

    /** What the last step took: the touching pairs, their points, and the rounds they were sorted into. */
    public BoxStepper.Report last() {
        return last;
    }

    /**
     * The state, read back: {@code x, y, z, u, v, w}, the angular momentum {@code lx, ly, lz}, and the orientation
     * {@code qx, qy, qz, qw}.
     */
    public float[][] state() {
        String[] names = {"x", "y", "z", "u", "v", "w", "lx", "ly", "lz", "qx", "qy", "qz", "qw"};
        float[][] out = new float[names.length][];
        for (int k = 0; k < names.length; k++) {
            out[k] = runner.floats(names[k]);
        }
        return out;
    }

    @Override
    public int[] readShown() {
        return runner.read("shown");
    }

    @Override
    public String where() {
        return accelerator == null ? "the CPU (Truffle)" : accelerator.capabilities().deviceName();
    }

    @Override
    public ResidentBuffer shown() {
        return runner.onDevice() ? runner.resident("shown") : null;
    }

    @Override
    public Accelerator accelerator() {
        return accelerator;
    }

    @Override
    public void close() {
        runner.close();
        if (accelerator != null) {
            accelerator.close();
        }
    }
}

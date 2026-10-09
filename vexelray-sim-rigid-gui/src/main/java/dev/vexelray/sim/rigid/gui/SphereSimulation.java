package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.pass.Pass;
import dev.supirvast.vastir.tools.Accelerator;
import dev.supirvast.vastir.tools.Completion;
import dev.supirvast.vastir.tools.DispatchSequence;
import dev.supirvast.vastir.tools.GpuContext;
import dev.supirvast.vastir.tools.KernelColumn;
import dev.supirvast.vastir.tools.KernelHandle;
import dev.supirvast.vastir.tools.KernelSpec;
import dev.supirvast.vastir.tools.PassRunner;
import dev.supirvast.vastir.tools.ResidentBuffer;
import dev.vexelray.sim.rigid.sphere.SphereGrid;
import dev.vexelray.sim.rigid.sphere.SphereStep;
import dev.vexelray.sim.rigid.sphere.SphereStepper;
import dev.vexelray.sim.rigid.sphere.Spheres;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * Spheres in a box, stepped on resident buffers: {@link SphereStep} on a {@link PassRunner}, run by a
 * {@link SphereStepper}. Jacobi's step is one recorded submission however many substeps it holds; Gauss–Seidel over
 * the contact list is run pass by pass until every contact is solved, which waits for the GPU after each batch of
 * rounds and so belongs on a thread of its own, never the one that draws.
 *
 * <p>Made in two halves, as the accelerator allows: the constructor allocates and registers, which is slow (every
 * kernel lowered, validated and compiled) and may run on any thread; everything after is the owning thread's.
 *
 * <p>On a context lent from the application's device ({@code AppCompute.lend}, or {@code AppCompute.on} a lent
 * compute queue), the buffers are ones the window can draw from, and a picture is handed its steps by {@link ShownSlots}.
 * Without one, it runs on a device of its own.
 */
public final class SphereSimulation implements AutoCloseable {

    /**
     * What a step took.
     *
     * @param solve       what running it until done took: waits, rounds, and contacts that did not fit the list
     * @param submissions how many submissions it was, after {@linkplain #slice slicing}
     * @param wallNanos   from the first submission to knowing the step was done, for {@link #stepAndWait}; else from
     *                    the first submission to the last
     * @param gpuNanos    the step's time on the GPU, every submission whose time could be read, added up; zero where
     *                    the device writes no timestamps
     */
    public record StepReport(SphereStepper.Report solve, int submissions, long wallNanos, long gpuNanos) {
    }

    private final Accelerator accelerator;
    private final SphereStep step;
    private final PassRunner runner;
    private final SphereStepper stepper;
    /** Lists cut to the slice, by the identity of the list they were cut from. */
    private final Map<List<Pass>, List<List<Pass>>> sliced = new IdentityHashMap<>();
    private int slice;

    // What the step in progress has taken, for its report.
    private int submissions;
    private long gpuNanos;
    private final List<Completion> unread = new ArrayList<>();

    /**
     * Jacobi over {@code grid}, or every pair without one: what the demo has run from the start.
     *
     * @param context the application's device, or null for one of the simulation's own
     * @param grid    the broad phase's grid, or null to test every pair
     */
    public SphereSimulation(GpuContext context, int spheres, int substeps, int iterations, SphereGrid grid) {
        this(context, new SphereStep(spheres, substeps, iterations, grid));
    }

    /** {@code step}, whatever it solves with, on {@code context}, or on a device of its own if that is null. */
    public SphereSimulation(GpuContext context, SphereStep step) {
        this(context == null ? new Accelerator() : Accelerator.on(context), step);
    }

    /**
     * {@code step} on the CPU, through Truffle: the same passes, lowered for the host rather than the GPU. Its
     * state is arrays, so a picture is handed it through the host ({@link ShownSlots#on}).
     */
    public static SphereSimulation onCpu(SphereStep step) {
        return new SphereSimulation((Accelerator) null, step);
    }

    private SphereSimulation(Accelerator accelerator, SphereStep step) {
        this.accelerator = accelerator;
        this.step = step;
        this.runner = accelerator == null ? PassRunner.cpu(step, Spheres.WORKGROUP, PassRunner.NO_SUBGROUP)
                : PassRunner.gpu(accelerator, step, Spheres.WORKGROUP, PassRunner.NO_SUBGROUP);
        this.stepper = new SphereStepper(step);
        runner.prepare(step.step());
        if (step.untilDone()) {
            runner.prepare(step.opening(0, 0, SphereStep.BATCH));
        }
    }

    public int spheres() {
        return step.spheres;
    }

    public int substeps() {
        return step.substeps;
    }

    public int iterations() {
        return step.iterations;
    }

    public SphereStep.Solve solve() {
        return step.solve;
    }

    /** Whether the state is on a GPU, as a picture of it needs. */
    public boolean onDevice() {
        return runner.onDevice();
    }

    /**
     * At most {@code dispatches} dispatches in any one submission, so that on a GPU shared with drawing no single
     * submission holds it for long; zero for no bound. A list longer than that is submitted in pieces, in order, with
     * no wait between them. What size keeps a frame's time flat is a measurement, not a guess.
     */
    public void slice(int dispatches) {
        if (dispatches < 0) {
            throw new IllegalArgumentException("a slice is some dispatches, or zero for no bound; got " + dispatches);
        }
        if (dispatches != slice) {
            slice = dispatches;
            sliced.clear();
        }
    }

    /**
     * Starts from this state: positions, velocities, radii and inverse masses, each as long as the sphere count. The
     * picture's "before" and "after" are both this state, so the first frame does not blend from nowhere.
     */
    public void start(float[] x, float[] y, float[] z, float[] u, float[] v, float[] w, float[] r, float[] im,
                      int[] params) {
        runner.clear();
        runner.write("x", x);
        runner.write("y", y);
        runner.write("z", z);
        runner.write("u", u);
        runner.write("v", v);
        runner.write("w", w);
        runner.write("r", r);
        runner.write("im", im);
        step.constants().forEach(runner::write);
        runner.write("params", params);
        float[] shown = new float[Spheres.SHOWN_STRIDE * step.spheres];
        for (int s = 0; s < step.spheres; s++) {
            int base = Spheres.SHOWN_STRIDE * s;
            shown[base] = shown[base + 4] = x[s];
            shown[base + 1] = shown[base + 5] = y[s];
            shown[base + 2] = shown[base + 6] = z[s];
            shown[base + 3] = r[s];
        }
        runner.write("shown", shown);
    }

    /** New parameters ({@link Spheres#params}), from the next step on. */
    public void params(int[] params) {
        runner.write("params", params);
    }

    /**
     * One step: every substep, every contact solved, and the picture's buffer updated. Jacobi's returns without
     * waiting; Gauss–Seidel over the list waits after each batch of rounds, and returns once its last pass is known
     * to be done, with the closing passes still running.
     */
    public StepReport step() {
        return run(false);
    }

    /** {@link #step}, and waits for all of it: what a thread that runs steps back to back on its own queue does. */
    public StepReport stepAndWait() {
        return run(true);
    }

    private StepReport run(boolean wait) {
        long start = System.nanoTime();
        submissions = 0;
        gpuNanos = 0;
        SphereStepper.Report solve = stepper.step(new SphereStepper.Runner() {
            @Override
            public void run(List<Pass> passes) {
                submit(passes);
            }

            @Override
            public int[] runAndRead(List<Pass> passes) {
                submit(passes).await();
                return runner.peek("readout");
            }
        });
        if (wait) {
            runner.finish();
        }
        long wall = System.nanoTime() - start;
        unread.removeIf(c -> {
            if (!c.done()) {
                return false;
            }
            gpuNanos += c.gpuNanos().orElse(0);
            return true;
        });
        return new StepReport(solve, submissions, wall, gpuNanos);
    }

    /** Submits {@code passes}, a slice at a time; returns the last submission's completion. */
    private Completion submit(List<Pass> passes) {
        List<List<Pass>> pieces = slice == 0 || passes.size() <= slice ? List.of(passes)
                : sliced.computeIfAbsent(passes, list -> {
                    List<List<Pass>> out = new ArrayList<>();
                    for (int k = 0; k < list.size(); k += slice) {
                        out.add(List.copyOf(list.subList(k, Math.min(list.size(), k + slice))));
                    }
                    return List.copyOf(out);
                });
        Completion last = null;
        for (List<Pass> piece : pieces) {
            last = runner.run(piece);
            unread.add(last);
            submissions++;
        }
        return last;
    }

    /** Waits for every step run so far: what orders the steps before a draw that reads their buffers. */
    public void finish() {
        runner.finish();
    }

    /** The state, read back: {@code x, y, z, u, v, w}. */
    public float[][] state() {
        return new float[][] {runner.floats("x"), runner.floats("y"), runner.floats("z"), runner.floats("u"),
                runner.floats("v"), runner.floats("w")};
    }

    /**
     * The last step's {@code shown}, read back to the host: what a picture on another device is handed, through
     * {@link ShownSlots}. On the CPU, a copy of the array; on a GPU, a staged read that waits for the step.
     */
    public int[] readShown() {
        return runner.read("shown");
    }

    /** Where this simulation runs, for whoever reports it: the device's name, or the CPU. */
    public String where() {
        return accelerator == null ? "the CPU (Truffle)" : accelerator.capabilities().deviceName();
    }

    /** The resident {@code shown}, for {@link ShownSlots} to copy from on the same device; null on the CPU. */
    ResidentBuffer shown() {
        return runner.onDevice() ? runner.resident("shown") : null;
    }

    /** The accelerator the simulation runs on; null on the CPU. */
    Accelerator accelerator() {
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

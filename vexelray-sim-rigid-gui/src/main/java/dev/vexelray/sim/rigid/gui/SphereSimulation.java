package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.tools.Accelerator;
import dev.supirvast.vastir.tools.GpuContext;
import dev.supirvast.vastir.tools.PassRunner;
import dev.vexelray.sim.rigid.sphere.SphereGrid;
import dev.vexelray.sim.rigid.sphere.SphereStep;
import dev.vexelray.sim.rigid.sphere.Spheres;

/**
 * Spheres in a box, stepped on resident buffers: {@link SphereStep} on a {@link PassRunner}, so a step is one
 * recorded submission however many substeps it holds.
 *
 * <p>Made in two halves, as the accelerator allows: the constructor allocates and registers, which is slow (every
 * kernel lowered, validated and compiled) and may run on any thread; everything after is the owning thread's.
 *
 * <p>On a context lent from the application's device ({@code AppCompute.lend}), the buffers are ones the window can
 * draw from, and {@link #shownBuffer} is what a picture binds. Without one, it runs on a device of its own.
 */
public final class SphereSimulation implements AutoCloseable {

    private final Accelerator accelerator;
    private final SphereStep step;
    private final PassRunner runner;

    /**
     * @param context the application's device, or null for one of the simulation's own
     * @param grid    the broad phase's grid, or null to test every pair
     */
    public SphereSimulation(GpuContext context, int spheres, int substeps, int iterations, SphereGrid grid) {
        this.accelerator = context == null ? new Accelerator() : Accelerator.on(context);
        this.step = new SphereStep(spheres, substeps, iterations, grid);
        this.runner = PassRunner.gpu(accelerator, step, Spheres.WORKGROUP, PassRunner.NO_SUBGROUP);
        runner.prepare(step.step());
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

    /** Whether the state is on a GPU, as a picture of it needs. */
    public boolean onDevice() {
        return runner.onDevice();
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

    /** One step: every substep, and the picture's buffer updated. Returns without waiting. */
    public void step() {
        runner.run(step.step());
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

    /** The Vulkan buffer a picture binds: {@link Spheres#SHOWN_STRIDE} floats a sphere. */
    public long shownBuffer() {
        return runner.resident("shown").vkBuffer();
    }

    @Override
    public void close() {
        runner.close();
        accelerator.close();
    }
}

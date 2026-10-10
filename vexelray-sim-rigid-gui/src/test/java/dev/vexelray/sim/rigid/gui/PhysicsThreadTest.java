package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.tools.GpuContext;
import dev.vexelray.gui.core.Gui;
import dev.vexelray.gui.core.app.ComputeQueue;
import dev.vexelray.gui.core.app.GuiApp;
import dev.vexelray.gui.harness.HarnessApp;
import dev.vexelray.os.WindowConfig;
import dev.vexelray.sim.core.gui.AppCompute;
import dev.vexelray.sim.rigid.sphere.SphereGrid;
import dev.vexelray.sim.rigid.sphere.SphereStep;
import dev.vexelray.sim.rigid.sphere.Spheres;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import java.util.Random;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * Physics on a thread of its own, on the compute queue a live application lent it, while the application's loop goes
 * on drawing: every step solves every contact, steps run back to back, and a step cut into slices is the same step.
 *
 * <p>Needs a Vulkan device with a compute-only queue family; this is an integration test, not a unit test.
 * It opens a window, so it runs only with {@code -Pphysics}.
 */
@Tag("physics")
class PhysicsThreadTest {

    private static final int SIDE = 10;
    private static final int STEPS = 60;
    private static final double DT = 1.0 / 60;
    private static final int SUBSTEPS = 10;

    @Test
    void stepsRunBackToBackOnTheLentQueueAndASliceIsTheSameStep() throws Exception {
        Gui gui = new Gui();
        try (HarnessApp harness = HarnessApp.start(gui, WindowConfig.of("physics", 320, 200),
                GuiApp.Compute.OWN_QUEUE)) {
            ComputeQueue queue = onLoop(harness, () -> harness.app().lendComputeQueue());
            assumeTrue(queue.available(), "no compute queue to lend: " + queue.why());

            Run whole = run(queue, gui, harness, 0);
            Run sliced = run(queue, gui, harness, 12);

            for (Run run : new Run[] {whole, sliced}) {
                System.out.printf("[physics thread] slice %d: %.3f ms a step on the wall, %.3f on the GPU, %.1f waits,"
                                + " %.1f submissions; %d frames drawn meanwhile%n", run.slice,
                        run.wallNanos / 1e6 / STEPS, run.gpuNanos / 1e6 / STEPS, run.waits / (double) STEPS,
                        run.submissions / (double) STEPS, run.frames);
                assertEquals(0, run.unlisted, "every contact fitted the list, so every contact was solved");
                assertTrue(run.frames > 0, "the loop went on drawing while physics ran on its own queue");
            }
            assertTrue(sliced.submissions > whole.submissions, "the slices are more submissions");
            for (int axis = 0; axis < 6; axis++) {
                assertArrayEquals(whole.state[axis], sliced.state[axis], 0f, "a sliced step is the same step");
            }
        }
    }

    private record Run(int slice, float[][] state, long wallNanos, long gpuNanos, long waits, long submissions,
                       long unlisted, long frames) {
    }

    /**
     * {@link #STEPS} steps of a dropped pile on a platform thread of its own, every one waited for, as a physics
     * component would run them; frames requested from the test's thread all the while.
     */
    private static Run run(ComputeQueue queue, Gui gui, HarnessApp harness, int slice) throws Exception {
        CompletableFuture<Run> result = new CompletableFuture<>();
        AtomicBoolean running = new AtomicBoolean(true);
        long framesBefore = harness.frames();
        Thread physics = Thread.ofPlatform().name("vexel-component-physics").start(() -> {
            try (GpuContext context = AppCompute.on(queue).orElseThrow();
                 SphereSimulation sim = new SphereSimulation(context, step())) {
                sim.slice(slice);
                start(sim);
                long wall = 0;
                long gpu = 0;
                long waits = 0;
                long submissions = 0;
                long unlisted = 0;
                for (int k = 0; k < STEPS; k++) {
                    SphereSimulation.StepReport report = sim.stepAndWait();
                    wall += report.wallNanos();
                    gpu += report.gpuNanos();
                    waits += report.solve().waits();
                    submissions += report.submissions();
                    unlisted += report.solve().unlisted();
                }
                result.complete(new Run(slice, sim.state(), wall, gpu, waits, submissions, unlisted, 0));
            } catch (Throwable t) {
                result.completeExceptionally(t);
            } finally {
                running.set(false);
            }
        });
        while (running.get()) {
            gui.requestFrame();
            Thread.sleep(2);
        }
        physics.join();
        Run run = result.get(60, TimeUnit.SECONDS);
        return new Run(run.slice, run.state, run.wallNanos, run.gpuNanos, run.waits, run.submissions, run.unlisted,
                harness.frames() - framesBefore);
    }

    private static SphereStep step() {
        int n = SIDE * SIDE * SIDE;
        return new SphereStep(n, SUBSTEPS, 1, SphereGrid.covering(box(), 1.2, box(), 0.06),
                SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT);
    }

    private static double box() {
        return 0.08 + 0.06 * SIDE;
    }

    private static void start(SphereSimulation sim) {
        int n = SIDE * SIDE * SIDE;
        float[] x = new float[n];
        float[] y = new float[n];
        float[] z = new float[n];
        float[] r = new float[n];
        float[] im = new float[n];
        Random random = new Random(7);
        int k = 0;
        for (int i = 0; i < SIDE; i++) {
            for (int j = 0; j < SIDE; j++) {
                for (int l = 0; l < SIDE; l++, k++) {
                    x[k] = (float) (0.07 + 0.06 * i + 0.005 * random.nextDouble());
                    y[k] = (float) (0.3 + 0.07 * j);
                    z[k] = (float) (0.07 + 0.06 * l + 0.005 * random.nextDouble());
                    r[k] = 0.03f;
                    im[k] = 10f;
                }
            }
        }
        sim.start(x, y, z, new float[n], new float[n], new float[n], r, im,
                Spheres.params(DT / SUBSTEPS, 0, -9.81, 0, box(), 1.2, box(), 1, false));
    }

    /** Run {@code work} on the loop thread and hand back what it returned. */
    private static <T> T onLoop(HarnessApp harness, java.util.function.Supplier<T> work) throws Exception {
        CompletableFuture<T> done = new CompletableFuture<>();
        harness.app().post(() -> {
            try {
                done.complete(work.get());
            } catch (RuntimeException | Error e) {
                done.completeExceptionally(e);
            }
        });
        return done.get(10, TimeUnit.SECONDS);
    }
}

package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.tools.GpuContext;
import dev.vexelray.gui.core.Gui;
import dev.vexelray.gui.core.Node;
import dev.vexelray.gui.core.app.GuiApp;
import dev.vexelray.gui.core.layout.Length;
import dev.vexelray.gui.harness.HarnessApp;
import dev.vexelray.os.WindowConfig;
import dev.vexelray.sim.core.gui.AppCompute;
import dev.vexelray.sim.core.gui.Orbit;
import dev.vexelray.sim.rigid.gui.SphereRunner.Backend;
import dev.vexelray.sim.rigid.sphere.SphereGrid;
import dev.vexelray.sim.rigid.sphere.SphereStep;
import dev.vexelray.sim.rigid.sphere.Spheres;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import sibarum.kronometer.Dilated;
import sibarum.kronometer.Dur;
import sibarum.kronometer.Kron;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

/**
 * The timing plan's measurement: one scene on each backend, run as the demo runs it, and what each does to the
 * frame. A real application with a real frame loop draws the picture every frame from the ring; physics runs on a
 * thread of its own, as fast as the world's clock allows, on the application's compute queue, on the integrated GPU,
 * or on the CPU. What is recorded:
 *
 * <ul>
 *   <li>a step's wall time on the physics thread, its time on the GPU, and the hand-back to the picture's device;</li>
 *   <li>the frames drawn while it runs, and how long the longest of them took;</li>
 *   <li>the dilation: steps finished over steps due.</li>
 * </ul>
 *
 * A measurement, not an assertion: {@code -Drigid.profile=true} runs it. It needs a GPU with a compute-only queue
 * family, and the integrated GPU for that row.
 */
@EnabledIfSystemProperty(named = "rigid.profile", matches = "true")
class BackendProfileTest {

    private static final double DT = 1.0 / 60;
    private static final int SUBSTEPS = 10;
    private static final long WARM_MILLIS = 1500;
    private static final long MEASURE_MILLIS = 4000;

    /** One run: a backend, a pile of {@code side}³ spheres, and the slice its submissions are cut to (0 for none). */
    private record Case(String name, Backend backend, int side, int slice, boolean held, int substeps, int iterations) {

        Case(String name, Backend backend, int side, int slice, boolean held) {
            this(name, backend, side, slice, held, SUBSTEPS, 1);
        }
    }

    /** What a run measured. */
    private record Result(Case c, int steps, double stepMs, double gpuMs, double handBackMs, int frames, double p50,
                          double p99, double worst, double dilation) {
    }

    @Test
    void eachBackendAgainstTheFrame() throws Exception {
        List<Case> cases = new ArrayList<>();
        cases.add(new Case("picture only", Backend.QUEUE, 10, 0, true));
        cases.add(new Case("CPU", Backend.CPU, 5, 0, false));
        for (int side : new int[] {7, 10, 16}) {
            cases.add(new Case("own queue", Backend.QUEUE, side, 0, false));
            cases.add(new Case("own queue, sliced 8", Backend.QUEUE, side, 8, false));
            cases.add(new Case("integrated GPU", Backend.INTEGRATED, side, 0, false));
            if (side <= 10) {
                cases.add(new Case("CPU", Backend.CPU, side, 0, false));
            }
        }
        // Heavy: a step of tens of milliseconds on the GPU, which the frame has to share it with.
        for (int slice : new int[] {0, 32, 8}) {
            cases.add(new Case("heavy, sliced " + slice, Backend.QUEUE, 16, slice, false, 40, 5));
        }
        cases.add(new Case("heavy, integrated", Backend.INTEGRATED, 16, 0, false, 40, 5));

        Gui gui = new Gui();
        Node box = gui.box().width(Length.dp(512)).height(Length.dp(512));
        gui.root().children(box);
        SphereView view = new SphereView(box, 512, new Orbit());
        ShownRing ring = new ShownRing();
        Kron kron = Kron.driven();
        long origin = System.nanoTime();
        Dilated[] world = new Dilated[1];
        AtomicBoolean recording = new AtomicBoolean();
        AtomicBoolean drawing = new AtomicBoolean(true);
        List<Long> frameTimes = new ArrayList<>();
        GuiApp[] application = new GuiApp[1];

        Runnable eachFrame = () -> {
            long now = System.nanoTime();
            kron.tick(now - origin);
            ShownRing.Frame frame = ring.take();
            if (frame != null && world[0] != null && drawing.get()) {
                view.show(application[0], frame, world[0].phase(now), 0);
            }
            if (recording.get()) {
                synchronized (frameTimes) {
                    frameTimes.add(System.nanoTime());
                }
            }
            gui.requestFrame();
        };

        List<Result> results = new ArrayList<>();
        try (HarnessApp harness = HarnessApp.start(gui, WindowConfig.of("profile", 640, 600),
                GuiApp.Compute.OWN_QUEUE, eachFrame)) {
            application[0] = harness.app();
            GpuContext render = onLoop(harness, () -> AppCompute.on(harness.app().lendComputeQueue()).orElseThrow());
            SphereRunner runner = new SphereRunner(render, ring);
            try {
                for (Case c : cases) {
                    results.add(run(c, harness, kron, world, runner, recording, frameTimes));
                    System.out.println("[profile] " + line(results.getLast()));
                }
            } finally {
                // The frame stops drawing the ring first: its buffers are about to be freed, and a draw of a freed
                // buffer is a lost device. Then the simulations and rings go, then the context they were on.
                onLoop(harness, () -> {
                    drawing.set(false);
                    view.close();
                    if (world[0] != null) {
                        world[0].close();
                    }
                    return null;
                });
                Thread closer = Thread.ofPlatform().start(runner::close);
                closer.join();
                onLoop(harness, () -> {
                    render.close();
                    return null;
                });
            }
        }
        System.out.println();
        System.out.println(String.format("%-22s %6s  %6s %8s %8s %9s  %6s %7s %7s %7s  %8s", "backend", "spheres",
                "steps", "step ms", "GPU ms", "hand ms", "frames", "p50 ms", "p99 ms", "max ms", "dilation"));
        for (Result r : results) {
            System.out.println(line(r));
        }
    }

    private static String line(Result r) {
        return String.format("%-22s %6d   %6d %8.2f %8.2f %9.2f  %6d %7.2f %7.2f %7.1f  %7.0f%%", r.c.name,
                r.c.side * r.c.side * r.c.side, r.steps, r.stepMs, r.gpuMs, r.handBackMs, r.frames, r.p50, r.p99,
                r.worst, 100 * r.dilation);
    }

    /** One case: built on its backend, warmed, then measured while the frame loop draws it. */
    private static Result run(Case c, HarnessApp harness, Kron kron, Dilated[] world, SphereRunner runner,
                              AtomicBoolean recording, List<Long> frameTimes) throws Exception {
        Semaphore due = new Semaphore(0);
        Dilated clock = onLoop(harness, () -> {
            if (world[0] != null) {
                world[0].close();
            }
            Dilated made = kron.tempo().fixed("physics " + c, Dur.hz(1 / DT)).dilated(2);
            made.onDue(due::release);
            made.hold(c.held);
            world[0] = made;
            return made;
        });
        CountDownLatch built = new CountDownLatch(1);
        AtomicBoolean stop = new AtomicBoolean();
        AtomicBoolean measuring = new AtomicBoolean();
        long[] sums = new long[4];   // steps, wall, gpu, hand-back
        CompletableFuture<Void> failed = new CompletableFuture<>();
        Thread physics = Thread.ofPlatform().name("vexel-component-physics").start(() -> {
            try {
                int n = c.side * c.side * c.side;
                double width = 0.08 + 0.06 * c.side;
                double height = 0.5 + 0.07 * c.side;
                SphereStep step = new SphereStep(n, c.substeps, c.iterations,
                        SphereGrid.covering(width, height, width, 0.06));
                SphereSimulation sim = runner.install(step, c.backend, new double[] {width, height, width},
                        s -> start(s, c.side, width, height, c.substeps));
                sim.slice(c.slice);
                clock.discard();
                built.countDown();
                while (!stop.get()) {
                    if (!clock.take()) {
                        due.tryAcquire(20, TimeUnit.MILLISECONDS);
                        continue;
                    }
                    SphereRunner.Kept kept = runner.step();
                    clock.finished(kept.finishedNanos());
                    if (measuring.get()) {
                        sums[0]++;
                        sums[1] += kept.step().wallNanos();
                        sums[2] += kept.step().gpuNanos();
                        sums[3] += kept.handBackNanos();
                    }
                }
                failed.complete(null);
            } catch (Throwable t) {
                built.countDown();
                failed.completeExceptionally(t);
            }
        });
        built.await();
        Thread.sleep(WARM_MILLIS);
        synchronized (frameTimes) {
            frameTimes.clear();
        }
        long dueBefore = clock.due();
        long finishedBefore = clock.finished();
        measuring.set(true);
        recording.set(true);
        Thread.sleep(MEASURE_MILLIS);
        recording.set(false);
        measuring.set(false);
        long dueAfter = clock.due();
        long finishedAfter = clock.finished();
        stop.set(true);
        physics.join();
        failed.get();

        long[] times;
        synchronized (frameTimes) {
            times = frameTimes.stream().mapToLong(Long::longValue).toArray();
        }
        double[] intervals = new double[Math.max(0, times.length - 1)];
        for (int k = 1; k < times.length; k++) {
            intervals[k - 1] = (times[k] - times[k - 1]) / 1e6;
        }
        Arrays.sort(intervals);
        int steps = (int) sums[0];
        double dilation = dueAfter > dueBefore ? (finishedAfter - finishedBefore) / (double) (dueAfter - dueBefore)
                : 0;
        return new Result(c, steps, steps == 0 ? 0 : sums[1] / 1e6 / steps, steps == 0 ? 0 : sums[2] / 1e6 / steps,
                steps == 0 ? 0 : sums[3] / 1e6 / steps, times.length, percentile(intervals, 0.5),
                percentile(intervals, 0.99), intervals.length == 0 ? 0 : intervals[intervals.length - 1], dilation);
    }

    private static double percentile(double[] sorted, double p) {
        if (sorted.length == 0) {
            return 0;
        }
        return sorted[Math.min(sorted.length - 1, (int) Math.floor(p * sorted.length))];
    }

    /** A loose block of {@code side}³ spheres above the floor, as the demo's piles are. */
    private static void start(SphereSimulation sim, int side, double width, double height, int substeps) {
        int n = side * side * side;
        float[] x = new float[n];
        float[] y = new float[n];
        float[] z = new float[n];
        float[] r = new float[n];
        float[] im = new float[n];
        Random random = new Random(7);
        int k = 0;
        for (int i = 0; i < side; i++) {
            for (int j = 0; j < side; j++) {
                for (int l = 0; l < side; l++, k++) {
                    x[k] = (float) (0.07 + 0.06 * i + 0.005 * random.nextDouble());
                    y[k] = (float) (0.3 + 0.07 * j);
                    z[k] = (float) (0.07 + 0.06 * l + 0.005 * random.nextDouble());
                    r[k] = 0.03f;
                    im[k] = 10f;
                }
            }
        }
        sim.start(x, y, z, new float[n], new float[n], new float[n], r, im,
                Spheres.params(DT / substeps, 0, -9.81, 0, width, height, width, 1, true));
    }

    /** Run {@code work} on the loop thread and hand back what it returned. */
    private static <T> T onLoop(HarnessApp harness, Supplier<T> work) throws Exception {
        CompletableFuture<T> done = new CompletableFuture<>();
        harness.app().post(() -> {
            try {
                done.complete(work.get());
            } catch (RuntimeException | Error e) {
                done.completeExceptionally(e);
            }
        });
        return done.get(30, TimeUnit.SECONDS);
    }
}

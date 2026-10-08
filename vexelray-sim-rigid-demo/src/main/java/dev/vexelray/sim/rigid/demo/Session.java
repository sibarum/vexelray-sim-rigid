package dev.vexelray.sim.rigid.demo;

import dev.supirvast.vastir.tools.GpuContext;
import dev.vexelray.framework.shell.Shell;
import dev.vexelray.gui.core.app.GuiApp;
import dev.vexelray.sim.core.gui.AppCompute;
import dev.vexelray.sim.core.gui.Builds;
import dev.vexelray.sim.rigid.demo.Controls.Solver;
import dev.vexelray.sim.rigid.gui.SphereSimulation;
import dev.vexelray.sim.rigid.gui.SphereView;
import dev.vexelray.sim.rigid.sphere.SphereDiagnostics;
import dev.vexelray.sim.rigid.sphere.Spheres;
import sibarum.kronometer.Dur;
import sibarum.kronometer.Handoff;
import sibarum.kronometer.Kron;
import sibarum.kronometer.Rate;
import sibarum.kronometer.Ratio;
import sibarum.kronometer.Tempo;

/**
 * One frame's work, on the main thread: take what the user asked for, run the physics steps that came due, draw.
 *
 * <h2>The timing is Kronometer's</h2>
 *
 * The physics is a fixed 60 Hz {@link Rate} inside a {@link Tempo} the playback speed scales, so a quarter speed is
 * the same steps a quarter as often, and nothing here counts time. The steps run on the GPU, which only this thread
 * may submit to, so the rate's steps are {@linkplain Handoff handed off}: the timeline says a step is due, and this
 * frame, in {@code APP} after the framework has ticked the clock, runs what came due. The picture blends each sphere
 * between the last two steps that ran by {@link Handoff#phase}, so it moves smoothly at any refresh rate, one step
 * behind. A pause {@linkplain Handoff#hold holds} the handoff, which parks the rate, so a paused window costs nothing.
 *
 * <h2>Nothing stops</h2>
 *
 * A simulation of a new shape (another scenario, another number of substeps) is its kernels lowered, validated and
 * compiled, which takes a good part of a second. It is made on the offload lane ({@link Builds}), and the running one
 * goes on until it lands. A change of substeps carries the state over, so the same pile can be watched under each.
 */
final class Session implements AutoCloseable {

    /** The physics rate: a step is this long in the playback tempo's own time. */
    static final double STEP_SECONDS = 1.0 / 60;

    private static final double GRAVITY = -9.81;

    /** The readings are read back every this many frames: a readback is a wait, and they need not be live to the frame. */
    private static final int READ_EVERY = 10;

    /** A simulation and what it was made for, as the offload lane hands it over. */
    private record Built(SphereSimulation sim, Scenario scenario, Scenario.State state, boolean carried)
            implements AutoCloseable {
        @Override
        public void close() {
            sim.close();
        }
    }

    private final GuiApp app;
    private final Controls controls;
    private final SphereView view;
    private final Readings readings;
    private final Kron kron;
    private final Tempo playback;
    private final Handoff steps;
    private final GpuContext context;
    private final Builds<Built> builds;

    private SphereSimulation sim;
    private Scenario scenario;
    private Scenario.State state;
    private Solver solver;
    private Ratio speed = Ratio.of(1, 1);
    private boolean paused;
    private int wantedSubsteps;
    private int wantedIterations;

    private double simulated;
    private long frames;
    private double stepMillis;
    private double drawMillis;
    private int lastDrained;

    Session(Shell shell, Controls controls, SphereView view, Readings readings) {
        this.app = shell.app();
        this.controls = controls;
        this.view = view;
        this.readings = readings;
        this.kron = shell.krono().kron();
        this.playback = kron.tempo().child("playback", speed);
        Rate physics = playback.fixed("physics", Dur.hz(1 / STEP_SECONDS)).maxCatchUp(4);
        this.steps = physics.handoff();
        this.context = AppCompute.lend(app).orElse(null);
        this.builds = new Builds<>(app);
        steps.hold(true);                         // nothing to run until the first simulation lands
    }

    /** One frame. Main thread, in {@code FrameStage.APP}, after the clock has ticked. */
    void frame() {
        settle();
        if (builds.landed()) {
            install(builds.take());
        }
        if (sim == null) {
            readings.waiting(controls.scenario());
            return;
        }
        long start = System.nanoTime();
        lastDrained = steps.drain(step -> sim.step());
        simulated += lastDrained * STEP_SECONDS;
        if (lastDrained > 0) {
            // Waited for here rather than by the draw, so the time is the steps' alone: submitted, run, finished.
            sim.finish();
            double ms = (System.nanoTime() - start) / 1e6 / lastDrained;
            stepMillis = stepMillis == 0 ? ms : 0.9 * stepMillis + 0.1 * ms;
        }
        if (context != null) {
            long drawStart = System.nanoTime();
            view.show(app, sim, steps.phase(kron.now()), state.extent);
            double ms = (System.nanoTime() - drawStart) / 1e6;
            drawMillis = drawMillis == 0 ? ms : 0.9 * drawMillis + 0.1 * ms;
        }
        if (frames++ % READ_EVERY == 0) {
            read();
        }
    }

    /** What the controls ask for, against what is running: settings applied at once, new shapes sent to be built. */
    private void settle() {
        Ratio wantedSpeed = controls.speed();
        if (!wantedSpeed.equals(speed)) {
            speed = wantedSpeed;
            kron.onTimeline(() -> playback.rescale(wantedSpeed));
        }
        boolean wantedPause = controls.paused();
        if (wantedPause != paused) {
            paused = wantedPause;
            if (sim != null) {
                steps.hold(paused);
            }
        }
        Solver wantedSolver = controls.solver();
        if (wantedSolver != solver) {
            solver = wantedSolver;
            if (sim != null) {
                sim.params(params(sim.substeps(), state.extent));
            }
        }
        boolean reset = controls.takeReset();
        Scenario wantedScenario = controls.scenario();
        int substeps = controls.substeps();
        int iterations = controls.iterations();
        if (reset || wantedScenario != scenario || substeps != wantedSubsteps || iterations != wantedIterations) {
            boolean carry = !reset && wantedScenario == scenario && sim != null;
            Scenario.State from = carry ? current() : wantedScenario.start();
            scenario = wantedScenario;
            wantedSubsteps = substeps;
            wantedIterations = iterations;
            builds.start(() -> new Built(new SphereSimulation(context, from.n, substeps, iterations,
                    from.grid()), wantedScenario,
                    from, carry));
        }
    }

    /** A landed simulation takes over: started from its state, and the steps owed the old one forgotten. */
    private void install(Built built) {
        if (built == null) {
            return;
        }
        if (sim != null) {
            sim.close();
        }
        sim = built.sim();
        state = built.state();
        if (!built.carried()) {
            simulated = 0;
        }
        sim.start(state.x, state.y, state.z, state.u, state.v, state.w, state.r, state.im,
                params(sim.substeps(), state.extent));
        steps.discard();
        steps.hold(paused);
        read();
    }

    /** The running simulation's state now, as a start for one of another shape. */
    private Scenario.State current() {
        float[][] now = sim.state();
        Scenario.State s = new Scenario.State(state.n, state.extent[0], state.extent[1], state.extent[2]);
        System.arraycopy(now[0], 0, s.x, 0, s.n);
        System.arraycopy(now[1], 0, s.y, 0, s.n);
        System.arraycopy(now[2], 0, s.z, 0, s.n);
        System.arraycopy(now[3], 0, s.u, 0, s.n);
        System.arraycopy(now[4], 0, s.v, 0, s.n);
        System.arraycopy(now[5], 0, s.w, 0, s.n);
        System.arraycopy(state.r, 0, s.r, 0, s.n);
        System.arraycopy(state.im, 0, s.im, 0, s.n);
        return s;
    }

    private int[] params(int substeps, double[] extent) {
        return Spheres.params(STEP_SECONDS / substeps, 0, GRAVITY, 0, extent[0], extent[1], extent[2], solver.omega,
                solver.averaged);
    }

    private void read() {
        float[][] now = sim.state();
        SphereDiagnostics d = SphereDiagnostics.of(now[0], now[1], now[2], now[3], now[4], now[5], state.r, state.im,
                GRAVITY, state.extent[0], state.extent[1], state.extent[2]);
        readings.show(scenario, d, simulated, sim, stepMillis, drawMillis, lastDrained, steps.dropped(), paused,
                context != null);
    }

    @Override
    public void close() {
        steps.close();
        builds.close();
        if (sim != null) {
            sim.close();
        }
        if (context != null) {
            context.close();
        }
    }
}

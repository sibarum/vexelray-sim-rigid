package dev.vexelray.sim.rigid.demo;

import dev.vexelray.framework.api.BeforeFrame;
import dev.vexelray.framework.api.MainThread;
import dev.vexelray.gui.core.app.GuiApp;
import dev.vexelray.sim.rigid.demo.Messages.Build;
import dev.vexelray.sim.rigid.demo.Messages.Relax;
import dev.vexelray.sim.rigid.gui.ShownRing;
import dev.vexelray.sim.rigid.gui.SphereRunner;
import dev.vexelray.sim.rigid.sphere.SphereStep;
import sibarum.atchung.Atchung;
import sibarum.kronometer.Dilated;
import sibarum.kronometer.Kron;
import sibarum.kronometer.Ratio;
import sibarum.kronometer.Tempo;

/**
 * The frame's side of the demo, on the main thread: what the user asked for told to {@link Physics}, and the newest
 * finished step drawn.
 *
 * <h2>Two clocks</h2>
 *
 * The world runs on its own time ({@link Dilated}): a fixed 60 Hz grid inside the playback tempo says when a step is
 * due, and the physics lane counts the steps that finish. When steps get slow the world slows, and its time falls
 * behind the wall's; this reads how far as the <i>dilation</i>. Everything else here, the panel and the camera among
 * it, is on the wall's time and never slows with the world.
 *
 * <h2>The frame never waits for a step</h2>
 *
 * It draws the ring's newest finished step, blended from the one before by how far the frame is towards the next,
 * as the world's clock predicts it ({@link Dilated#phase}). A step later than predicted leaves the picture on the
 * newest until it lands; the frame rate does not move.
 */
@MainThread
final class Session implements AutoCloseable {

    /** The readings are refreshed every this many frames: they need not be live to the frame. */
    private static final int READ_EVERY = 10;

    private final GuiApp app;
    private final Ui ui;
    private final Controls controls;
    private final ShownRing ring;
    private final PhysicsNews news;
    private final Atchung bus;
    private final Kron kron;
    private final Tempo playback;
    private final Dilated world;

    private Scenario scenario;
    private Controls.Solver solver;
    private int substeps;
    private int iterations;
    private SphereRunner.Backend backend;
    private SphereStep.Solve solve;
    private double restitution;
    private Ratio speed = Ratio.of(1, 1);
    private boolean paused;
    private long frames;
    private double drawMillis;

    Session(GuiApp app, Ui ui, Controls controls, ShownRing ring, PhysicsNews news, Atchung bus, Kron kron,
            Tempo playback, Dilated world) {
        this.app = app;
        this.ui = ui;
        this.controls = controls;
        this.ring = ring;
        this.news = news;
        this.bus = bus;
        this.kron = kron;
        this.playback = playback;
        this.world = world;
    }

    /** One frame, after the clock has ticked. */
    @BeforeFrame
    public void frame() {
        ui.panel().sync();
        settle();
        ShownRing.Frame frame = ring.take();
        if (frame != null) {
            long start = System.nanoTime();
            ui.view().show(app, frame, world.phase(start));
            double ms = (System.nanoTime() - start) / 1e6;
            drawMillis = drawMillis == 0 ? ms : 0.9 * drawMillis + 0.1 * ms;
        }
        if (frames++ % READ_EVERY == 0) {
            ui.readings().show(news.latest(), world, drawMillis, paused);
        }
    }

    /** What the controls ask for, against what was last said: settings sent at once, new shapes sent to be built. */
    private void settle() {
        Ratio wantedSpeed = controls.speed();
        if (!wantedSpeed.equals(speed)) {
            speed = wantedSpeed;
            kron.onTimeline(() -> playback.rescale(wantedSpeed));
        }
        boolean wantedPause = controls.paused();
        if (wantedPause != paused) {
            paused = wantedPause;
            world.hold(paused);
        }
        Controls.Solver wantedSolver = controls.solver();
        double wantedRestitution = controls.restitution();
        if (solver != null && (wantedSolver != solver || wantedRestitution != restitution)) {
            bus.publish(Messages.RELAX_TOPIC, new Relax(wantedSolver.omega, wantedSolver.averaged,
                    wantedRestitution));
        }
        solver = wantedSolver;
        restitution = wantedRestitution;
        boolean reset = controls.takeReset();
        Scenario wantedScenario = controls.scenario();
        int wantedSubsteps = controls.substeps();
        int wantedIterations = controls.iterations();
        SphereRunner.Backend wantedBackend = controls.backend();
        SphereStep.Solve wantedSolve = controls.solve();
        if (reset || wantedScenario != scenario || wantedSubsteps != substeps || wantedIterations != iterations
                || wantedBackend != backend || wantedSolve != solve) {
            boolean carry = !reset && wantedScenario == scenario;
            scenario = wantedScenario;
            substeps = wantedSubsteps;
            iterations = wantedIterations;
            backend = wantedBackend;
            solve = wantedSolve;
            bus.publish(Messages.BUILD_TOPIC, new Build(scenario, solve, substeps, iterations, carry, solver.omega,
                    solver.averaged, restitution, backend));
        }
    }

    /** The view too: it is drawn here, on the window's device, and must go before the device does. */
    @Override
    public void close() {
        world.close();
        ui.view().close();
    }
}

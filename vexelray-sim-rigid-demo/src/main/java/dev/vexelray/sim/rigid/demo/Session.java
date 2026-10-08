package dev.vexelray.sim.rigid.demo;

import dev.vexelray.framework.api.BeforeFrame;
import dev.vexelray.framework.api.MainThread;
import dev.vexelray.gui.core.app.GuiApp;
import dev.vexelray.gui.krono.KronoGui;
import dev.vexelray.sim.rigid.demo.Messages.Allow;
import dev.vexelray.sim.rigid.demo.Messages.Build;
import dev.vexelray.sim.rigid.demo.Messages.Relax;
import dev.vexelray.sim.rigid.gui.ShownRing;
import sibarum.atchung.Atchung;
import sibarum.kronometer.Dur;
import sibarum.kronometer.Handoff;
import sibarum.kronometer.Kron;
import sibarum.kronometer.Rate;
import sibarum.kronometer.Ratio;
import sibarum.kronometer.Tempo;

/**
 * The frame's side of the demo, on the main thread: what the user asked for told to {@link Physics}, the steps the
 * clock has made due counted for it, and the newest finished step drawn.
 *
 * <h2>The timing is Kronometer's</h2>
 *
 * The physics is a fixed 60 Hz {@link Rate} inside a {@link Tempo} the playback speed scales, so a quarter speed is the
 * same steps a quarter as often. The frame does not run them: it counts the steps the rate made due and tells the
 * physics lane how many there have been ({@link Allow}), and the physics lane runs them. A pause {@linkplain
 * Handoff#hold holds} the handoff, which parks the rate.
 *
 * <h2>The frame never waits for a step</h2>
 *
 * It draws whatever the ring's newest finished step is, blended from the step before it by how far the frame is into
 * the step after, measured by how long the last step took. A step that takes longer than a frame leaves the picture
 * on the last finished step until it lands; the frame rate does not move.
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
    private final Handoff steps;

    private Scenario scenario;
    private Controls.Solver solver;
    private int substeps;
    private int iterations;
    private Ratio speed = Ratio.of(1, 1);
    private boolean paused;
    private long due;
    private long told = -1;
    private boolean toldHeld;
    private long frames;
    private double drawMillis;

    Session(GuiApp app, Ui ui, Controls controls, ShownRing ring, PhysicsNews news, Atchung bus, KronoGui krono) {
        this.app = app;
        this.ui = ui;
        this.controls = controls;
        this.ring = ring;
        this.news = news;
        this.bus = bus;
        this.kron = krono.kron();
        this.playback = kron.tempo().child("playback", speed);
        Rate physics = playback.fixed("physics", Dur.hz(1 / Physics.STEP_SECONDS)).maxCatchUp(4);
        this.steps = physics.handoff();
    }

    /** One frame, after the clock has ticked. */
    @BeforeFrame
    public void frame() {
        ui.panel().sync();
        settle();
        due += steps.drain(step -> { });
        if (due != told || paused != toldHeld) {
            told = due;
            toldHeld = paused;
            bus.publish(Messages.ALLOW_TOPIC, new Allow(due, paused));
        }
        ShownRing.Frame frame = ring.take();
        if (frame != null) {
            long start = System.nanoTime();
            ui.view().show(app, frame, frame.alpha(start));
            double ms = (System.nanoTime() - start) / 1e6;
            drawMillis = drawMillis == 0 ? ms : 0.9 * drawMillis + 0.1 * ms;
        }
        if (frames++ % READ_EVERY == 0) {
            ui.readings().show(news.latest(), drawMillis, paused);
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
            steps.hold(paused);
        }
        Controls.Solver wantedSolver = controls.solver();
        if (wantedSolver != solver && solver != null) {
            bus.publish(Messages.RELAX_TOPIC, new Relax(wantedSolver.omega, wantedSolver.averaged));
        }
        solver = wantedSolver;
        boolean reset = controls.takeReset();
        Scenario wantedScenario = controls.scenario();
        int wantedSubsteps = controls.substeps();
        int wantedIterations = controls.iterations();
        if (reset || wantedScenario != scenario || wantedSubsteps != substeps || wantedIterations != iterations) {
            boolean carry = !reset && wantedScenario == scenario;
            scenario = wantedScenario;
            substeps = wantedSubsteps;
            iterations = wantedIterations;
            bus.publish(Messages.BUILD_TOPIC, new Build(scenario, substeps, iterations, carry, solver.omega,
                    solver.averaged));
        }
    }

    /** The view too: it is drawn here, on the window's device, and must go before the device does. */
    @Override
    public void close() {
        steps.close();
        ui.view().close();
    }
}

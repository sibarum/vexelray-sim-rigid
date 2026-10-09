package dev.vexelray.sim.rigid.demo;

import dev.supirvast.vastir.tools.GpuContext;
import dev.vexelray.framework.api.Component;
import dev.vexelray.framework.api.Overflow;
import dev.vexelray.framework.api.Subscribe;
import dev.vexelray.gui.core.app.ComputeQueue;
import dev.vexelray.sim.core.gui.AppCompute;
import dev.vexelray.sim.rigid.demo.Messages.Build;
import dev.vexelray.sim.rigid.demo.Messages.Next;
import dev.vexelray.sim.rigid.demo.Messages.Relax;
import dev.vexelray.sim.rigid.gui.ShownRing;
import dev.vexelray.sim.rigid.gui.SphereRunner;
import dev.vexelray.sim.rigid.gui.SphereSimulation;
import dev.vexelray.sim.rigid.sphere.SphereDiagnostics;
import dev.vexelray.sim.rigid.sphere.SphereStep;
import dev.vexelray.sim.rigid.sphere.Spheres;
import sibarum.atchung.Atchung;
import sibarum.kronometer.Dilated;

/**
 * The physics, on a lane of its own: it makes the simulation, on whichever backend the user chose, runs its steps back
 * to back, and keeps each finished step in the {@link ShownRing} for the frame to draw ({@link SphereRunner}).
 *
 * <h2>The frame never waits for it</h2>
 *
 * Nothing here is on the main thread, and nothing the main thread does waits for anything here. A step publishes its
 * slot only once it is done, so the frame's draw finds its wait already met; a slow step means the frame keeps drawing
 * the last one, and the world's time slows instead of the frame rate falling.
 *
 * <h2>The clock decides, not this</h2>
 *
 * The world's clock ({@link Dilated}) says when a step is due, on its grid in the playback tempo, and wakes this lane
 * when one is ({@link Next}); this runs steps while it says so, one a delivery, so the lane's other mail is read
 * between them, and tells it each one that finishes. That count is the world's time. When steps are slower than the
 * grid, the clock forgives what is owed past {@link #MOST_BEHIND} rather than have it run in a burst: the world
 * slows, which is the dilation the timing plan asks for.
 *
 * <h2>Backends</h2>
 *
 * The compute queue the application lent is where every ring lives, because it is on the device the picture is drawn
 * on. A simulation runs there too, or on the integrated GPU, or on the CPU; from those, each step comes to the ring
 * through the host, and what that costs is a reading.
 */
@Component(lane = "physics")
final class Physics implements AutoCloseable {

    /** The world's step, in its own time. */
    static final double STEP_SECONDS = 1.0 / 60;

    private static final double GRAVITY = -9.81;

    /** Steps owed past which the world's clock forgives them, and the world slows, rather than catch up in a burst. */
    static final int MOST_BEHIND = 2;

    /** The state is read back for the readings every this many steps: a readback is a wait, and they need not be live. */
    private static final int READ_EVERY = 10;

    private final PhysicsNews news;
    private final Atchung bus;
    private final Dilated world;
    private final GpuContext context;
    private final SphereRunner runner;
    private final String noQueue;

    private SphereSimulation sim;
    private Scenario.State state;
    private Scenario scenario = Scenario.COLUMN;
    private long steps;
    private String problem = "";

    private double omega = 1;
    private boolean averaged = true;
    private double simulated;
    private double stepMillis;
    private double gpuMillis;
    private double handBackMillis;
    private SphereDiagnostics read;

    Physics(ComputeQueue queue, ShownRing ring, PhysicsNews news, Atchung bus, Dilated world) {
        this.news = news;
        this.bus = bus;
        this.world = world;
        // On the timeline, as a step comes due: a sample, so a burst of them is one wake.
        world.onDue(() -> bus.publish(Messages.NEXT_TOPIC, new Next()));
        this.context = AppCompute.on(queue).orElse(null);
        this.runner = context == null ? null : new SphereRunner(context, ring);
        this.noQueue = context == null
                ? "No compute queue of its own (" + queue.why() + "), so there is no physics to draw." : "";
        if (context == null) {
            news.report(new PhysicsNews.Report(scenario, false, 0, 0, 0, 0, null, 0, 0, 0, "", noQueue));
        }
    }

    /** A simulation of a new shape, or on another backend, made here; the world waits while it is. */
    @Subscribe(topic = Messages.BUILD, capacity = 8)
    public void build(Build b) {
        if (runner == null) {
            return;
        }
        boolean carry = b.carry() && sim != null && b.scenario() == scenario;
        Scenario.State from = carry ? current() : b.scenario().start();
        scenario = b.scenario();
        omega = b.omega();
        averaged = b.averaged();
        news.report(PhysicsNews.Report.waiting(scenario));
        SphereStep step = new SphereStep(from.n, b.substeps(), b.iterations(), from.grid());
        SphereRunner.Backend backend = b.backend();
        try {
            sim = runner.install(step, backend, from.extent, s -> s.start(from.x, from.y, from.z, from.u, from.v,
                    from.w, from.r, from.im, params(b.substeps(), from)));
            problem = "";
        } catch (IllegalStateException none) {
            // No such device here: say so, and run where the picture is instead.
            problem = backend.label + " is not here (" + none.getMessage() + "), so this runs on this GPU.";
            sim = runner.install(step, SphereRunner.Backend.QUEUE, from.extent, s -> s.start(from.x, from.y,
                    from.z, from.u, from.v, from.w, from.r, from.im, params(b.substeps(), from)));
        }
        state = from;
        if (!carry) {
            simulated = 0;
        }
        read = null;
        stepMillis = gpuMillis = handBackMillis = 0;
        world.discard();                // the steps owed while it was made are not this one's to run
        report();
        step(new Next());
    }

    @Subscribe(topic = Messages.RELAX, overflow = Overflow.COALESCE_LATEST)
    public void relax(Relax r) {
        omega = r.omega();
        averaged = r.averaged();
        if (sim != null) {
            sim.params(params(sim.substeps(), state));
        }
    }

    /**
     * One step, if the world's clock says one is due, and the next asked for in case another is: one a delivery, so the
     * lane's other mail, a new build say, is read between them.
     */
    @Subscribe(topic = Messages.NEXT, overflow = Overflow.COALESCE_LATEST)
    public void step(Next n) {
        if (sim != null && world.take()) {
            SphereRunner.Kept kept = runner.step();
            world.finished(kept.finishedNanos());
            steps++;
            simulated += STEP_SECONDS;
            stepMillis = average(stepMillis, kept.step().wallNanos() / 1e6);
            gpuMillis = average(gpuMillis, kept.step().gpuNanos() / 1e6);
            handBackMillis = average(handBackMillis, kept.handBackNanos() / 1e6);
            if (steps % READ_EVERY == 0) {
                float[][] now = sim.state();
                read = SphereDiagnostics.of(now[0], now[1], now[2], now[3], now[4], now[5], state.r, state.im,
                        GRAVITY, state.extent[0], state.extent[1], state.extent[2]);
            }
            report();
            bus.publish(Messages.NEXT_TOPIC, new Next());
        }
    }

    private static double average(double was, double now) {
        return was == 0 ? now : 0.9 * was + 0.1 * now;
    }

    private void report() {
        news.report(new PhysicsNews.Report(scenario, false, sim.spheres(), sim.substeps(), sim.iterations(),
                simulated, read, stepMillis, gpuMillis, handBackMillis, sim.where(), problem));
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

    private int[] params(int substeps, Scenario.State s) {
        return Spheres.params(STEP_SECONDS / substeps, 0, GRAVITY, 0, s.extent[0], s.extent[1], s.extent[2], omega,
                averaged);
    }

    /** After the lane has stopped: every simulation, then the context. The application closes the device after. */
    @Override
    public void close() {
        if (runner != null) {
            runner.close();
        }
        if (context != null) {
            context.close();
        }
    }
}

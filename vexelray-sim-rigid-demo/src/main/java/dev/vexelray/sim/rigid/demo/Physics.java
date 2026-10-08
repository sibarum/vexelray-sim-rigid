package dev.vexelray.sim.rigid.demo;

import dev.supirvast.vastir.tools.GpuContext;
import dev.vexelray.framework.api.Component;
import dev.vexelray.framework.api.Overflow;
import dev.vexelray.framework.api.Subscribe;
import dev.vexelray.gui.core.app.ComputeQueue;
import dev.vexelray.sim.core.gui.AppCompute;
import dev.vexelray.sim.rigid.demo.Messages.Allow;
import dev.vexelray.sim.rigid.demo.Messages.Build;
import dev.vexelray.sim.rigid.demo.Messages.Next;
import dev.vexelray.sim.rigid.demo.Messages.Relax;
import dev.vexelray.sim.rigid.gui.ShownRing;
import dev.vexelray.sim.rigid.gui.SphereSimulation;
import dev.vexelray.sim.rigid.sphere.SphereDiagnostics;
import dev.vexelray.sim.rigid.sphere.Spheres;
import sibarum.atchung.Atchung;

import java.util.HashMap;
import java.util.Map;

/**
 * The physics, on a lane of its own: it makes the simulation, runs its steps back to back on the compute queue the
 * application lent it, and keeps each finished step in the {@link ShownRing} for the frame to draw.
 *
 * <h2>The frame never waits for it</h2>
 *
 * Nothing here is on the main thread, and nothing the main thread does waits for anything here. A step publishes its
 * slot only once it is done, so the frame's draw finds its wait already met; a slow step means the frame keeps drawing
 * the last one, and the world's time slows instead of the frame rate falling.
 *
 * <h2>The clock decides, not this</h2>
 *
 * The frame counts the steps its clock has made due and says so ({@link Allow}); this runs steps until it has run as
 * many, one a delivery ({@link Next}), so the lane's other mail is read between them. When it falls more than
 * {@link #MOST_BEHIND} steps behind, the steps it owes are dropped rather than run in a burst: the world slows, which
 * is the dilation the timing plan asks for. The dilated clock that counts game time is the next stage's.
 *
 * <h2>Generations</h2>
 *
 * A new simulation, of another scenario or shape, is made here, which takes a good part of a second while the world
 * waits. It brings its own ring buffers; the old simulation is closed only once the frame has moved past its buffers,
 * which the ring says.
 */
@Component(lane = "physics")
final class Physics implements AutoCloseable {

    /** The world's step, in its own time. */
    static final double STEP_SECONDS = 1.0 / 60;

    private static final double GRAVITY = -9.81;

    /** Steps owed past which they are dropped, and the world slows, rather than run back to back to catch up. */
    static final int MOST_BEHIND = 2;

    /** The state is read back for the readings every this many steps: a readback is a wait, and they need not be live. */
    private static final int READ_EVERY = 10;

    private final ShownRing ring;
    private final PhysicsNews news;
    private final Atchung bus;
    private final GpuContext context;
    private final String problem;

    /** Every simulation not yet closed, by its ring generation, the running one included. */
    private final Map<Long, SphereSimulation> sims = new HashMap<>();
    private SphereSimulation sim;
    private Scenario.State state;
    private Scenario scenario = Scenario.COLUMN;
    private long generation;
    private long kept;

    private double omega = 1;
    private boolean averaged = true;
    private long due;
    private long ran;
    private boolean held = true;
    private long dropped;
    private double simulated;
    private double stepMillis;
    private double gpuMillis;
    private SphereDiagnostics read;

    Physics(ComputeQueue queue, ShownRing ring, PhysicsNews news, Atchung bus) {
        this.ring = ring;
        this.news = news;
        this.bus = bus;
        this.context = AppCompute.on(queue).orElse(null);
        this.problem = context == null
                ? "No compute queue of its own (" + queue.why() + "), so there is no physics to draw." : "";
        if (context == null) {
            news.report(new PhysicsNews.Report(scenario, false, 0, 0, 0, 0, null, 0, 0, 0, problem));
        }
    }

    /** A simulation of a new shape, made here; the world waits while it is. */
    @Subscribe(topic = Messages.BUILD, capacity = 8)
    public void build(Build b) {
        if (context == null) {
            return;
        }
        boolean carry = b.carry() && sim != null && b.scenario() == scenario;
        Scenario.State from = carry ? current() : b.scenario().start();
        scenario = b.scenario();
        omega = b.omega();
        averaged = b.averaged();
        news.report(PhysicsNews.Report.waiting(scenario));
        SphereSimulation made = new SphereSimulation(context, from.n, b.substeps(), b.iterations(), from.grid());
        made.start(from.x, from.y, from.z, from.u, from.v, from.w, from.r, from.im, params(made.substeps(), from));
        sim = made;
        state = from;
        if (!carry) {
            simulated = 0;
        }
        read = null;
        // The start, kept as the generation's first step, so the frame has a picture before the first step is run.
        generation++;
        kept = 1;
        made.keep(0, kept).await();
        sims.put(generation, made);
        ring.install(made.generation(generation, from.extent));
        ring.publish(kept, System.nanoTime());
        ran = due;                      // the steps owed the old simulation are not this one's to run
        report();
        next();
    }

    @Subscribe(topic = Messages.RELAX, overflow = Overflow.COALESCE_LATEST)
    public void relax(Relax r) {
        omega = r.omega();
        averaged = r.averaged();
        if (sim != null) {
            sim.params(params(sim.substeps(), state));
        }
    }

    @Subscribe(topic = Messages.ALLOW, overflow = Overflow.COALESCE_LATEST)
    public void allow(Allow a) {
        due = a.due();
        held = a.held();
        next();
    }

    /** One step, if one is due, and the next asked for if there is another. */
    @Subscribe(topic = Messages.NEXT, overflow = Overflow.COALESCE_LATEST)
    public void step(Next n) {
        if (sim == null || held || ran >= due) {
            return;
        }
        if (due - ran > MOST_BEHIND) {
            dropped += due - ran - MOST_BEHIND;
            ran = due - MOST_BEHIND;
        }
        SphereSimulation.StepReport report = sim.stepAndWait();
        kept++;
        sim.keep(ring.back(), kept).await();
        ring.publish(kept, System.nanoTime());
        ran++;
        simulated += STEP_SECONDS;
        double wall = report.wallNanos() / 1e6;
        double gpu = report.gpuNanos() / 1e6;
        stepMillis = stepMillis == 0 ? wall : 0.9 * stepMillis + 0.1 * wall;
        gpuMillis = gpuMillis == 0 ? gpu : 0.9 * gpuMillis + 0.1 * gpu;
        for (ShownRing.Generation old : ring.retired()) {
            SphereSimulation done = sims.remove(old.id());
            if (done != null) {
                done.close();
            }
        }
        if (kept % READ_EVERY == 0) {
            float[][] now = sim.state();
            read = SphereDiagnostics.of(now[0], now[1], now[2], now[3], now[4], now[5], state.r, state.im, GRAVITY,
                    state.extent[0], state.extent[1], state.extent[2]);
        }
        report();
        next();
    }

    private void next() {
        if (sim != null && !held && ran < due) {
            bus.publish(Messages.NEXT_TOPIC, new Next());
        }
    }

    private void report() {
        news.report(new PhysicsNews.Report(scenario, false, sim.spheres(), sim.substeps(), sim.iterations(),
                simulated, read, stepMillis, gpuMillis, dropped, problem));
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
        for (SphereSimulation s : sims.values()) {
            s.close();
        }
        sims.clear();
        if (context != null) {
            context.close();
        }
    }
}

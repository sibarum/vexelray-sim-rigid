package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.tools.GpuContext;
import dev.vexelray.sim.rigid.sphere.SphereStep;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

/**
 * The stepping thread's side of a picture: a simulation on whichever backend was asked for, its steps kept in a
 * {@link ShownRing} on the device the picture is drawn on, and each simulation freed once the picture has moved past
 * it. One thread, the one that steps; nothing here is the frame's.
 *
 * <h2>Backends</h2>
 *
 * <ul>
 *   <li>{@link Backend#QUEUE}: on the picture's own device, on its compute queue. A step is kept by a copy on the GPU.</li>
 *   <li>{@link Backend#INTEGRATED}: on the integrated GPU, a device of its own, whose steps come to the picture through
 *       the host.</li>
 *   <li>{@link Backend#CPU}: through Truffle, on the host, whose steps are already there to be written over.</li>
 * </ul>
 *
 * The same passes run on each, which is what lets one be measured against another.
 */
public final class SphereRunner implements AutoCloseable {

    /** Where a simulation runs. */
    public enum Backend {
        QUEUE("This GPU, on a queue of its own"),
        INTEGRATED("The integrated GPU"),
        CPU("The CPU, through Truffle");

        public final String label;

        Backend(String label) {
            this.label = label;
        }
    }

    /**
     * A step, kept.
     *
     * @param step          what the step took
     * @param handBackNanos from the step being done to its being in the ring: a copy on the GPU beside it, or a read
     *                      to the host and a write to the picture's device from anywhere else
     * @param finishedNanos when it was published, on {@link System#nanoTime}'s clock
     */
    public record Kept(SphereSimulation.StepReport step, long handBackNanos, long finishedNanos) {
    }

    private final GpuContext render;
    private final ShownRing ring;
    private final Map<Long, SphereSimulation> sims = new HashMap<>();
    private final Map<Long, ShownSlots> slotsOf = new HashMap<>();
    private GpuContext integrated;
    private SphereSimulation sim;
    private ShownSlots slots;
    private long generation;
    private long kept;

    /**
     * @param render the picture's device, on the queue this thread was lent: where every ring lives, and where
     *               {@link Backend#QUEUE} simulations run
     */
    public SphereRunner(GpuContext render, ShownRing ring) {
        this.render = render;
        this.ring = ring;
    }

    /**
     * A new simulation of {@code step} on {@code backend}, started by {@code start} and made the ring's current one:
     * its start kept as its first step, so the picture has something before the first step is run. Slow: every kernel
     * is lowered and compiled.
     *
     * @param extent the box, {@code sx, sy, sz}, in metres, for the picture
     * @throws IllegalStateException if the backend has no device here, naming the devices there are
     */
    public SphereSimulation install(SphereStep step, Backend backend, double[] extent,
                                    Consumer<SphereSimulation> start) {
        SphereSimulation made = switch (backend) {
            case QUEUE -> new SphereSimulation(render, step);
            case INTEGRATED -> new SphereSimulation(integrated(), step);
            case CPU -> SphereSimulation.onCpu(step);
        };
        ShownSlots madeSlots;
        try {
            start.accept(made);
            madeSlots = backend == Backend.QUEUE ? ShownSlots.beside(made) : ShownSlots.on(render, step.spheres);
        } catch (RuntimeException | Error e) {
            made.close();
            throw e;
        }
        sim = made;
        slots = madeSlots;
        generation++;
        kept = 1;
        slots.keep(0, kept, sim).await();
        sims.put(generation, sim);
        slotsOf.put(generation, slots);
        ring.install(slots.generation(generation, extent));
        ring.publish(kept, System.nanoTime());
        return made;
    }

    /** The integrated GPU's context, opened the first time it is asked for and kept. */
    private GpuContext integrated() {
        if (integrated == null) {
            integrated = GpuContext.open("integrated");
        }
        return integrated;
    }

    /** The running simulation, or null before the first {@link #install}. */
    public SphereSimulation sim() {
        return sim;
    }

    /**
     * One step of the running simulation, waited for, kept in the ring and published for the picture; and any
     * simulation the picture has moved past, freed.
     */
    public Kept step() {
        SphereSimulation.StepReport report = sim.stepAndWait();
        long done = System.nanoTime();
        kept++;
        slots.keep(ring.back(), kept, sim).await();
        long published = System.nanoTime();
        ring.publish(kept, published);
        for (ShownRing.Generation old : ring.retired()) {
            free(old.id());
        }
        return new Kept(report, published - done, published);
    }

    private void free(long id) {
        ShownSlots oldSlots = slotsOf.remove(id);
        if (oldSlots != null) {
            oldSlots.close();
        }
        SphereSimulation oldSim = sims.remove(id);
        if (oldSim != null) {
            oldSim.close();
        }
    }

    /** Every simulation and ring, then the integrated GPU's context. The picture's device is the caller's. */
    @Override
    public void close() {
        for (Long id : java.util.List.copyOf(sims.keySet())) {
            free(id);
        }
        if (integrated != null) {
            integrated.close();
        }
    }
}

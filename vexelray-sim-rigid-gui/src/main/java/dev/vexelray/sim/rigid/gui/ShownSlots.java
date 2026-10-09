package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.tools.Accelerator;
import dev.supirvast.vastir.tools.Completion;
import dev.supirvast.vastir.tools.DispatchSequence;
import dev.supirvast.vastir.tools.GpuContext;
import dev.supirvast.vastir.tools.KernelColumn;
import dev.supirvast.vastir.tools.KernelHandle;
import dev.supirvast.vastir.tools.KernelSpec;
import dev.supirvast.vastir.tools.ResidentBuffer;
import dev.vexelray.sim.rigid.sphere.Spheres;

import java.util.List;

/**
 * A ring's buffers ({@link ShownRing}), on the device the picture is drawn on: {@link ShownRing#SLOTS} copies of a
 * simulation's {@code shown}, and the timeline a kept step signals.
 *
 * <p>Where the simulation runs decides how a step gets here:
 *
 * <ul>
 *   <li><b>Beside it</b> ({@link #beside}): the simulation computes on the device the picture is drawn on, so a step
 *       is kept by a copy on the GPU, from its {@code shown} to a slot. Nothing crosses the bus.</li>
 *   <li><b>Elsewhere</b> ({@link #on}): on another GPU, or on the CPU. A step's {@code shown} is read back to the host
 *       and written here, eight floats a sphere, then copied into the slot on this device. That round trip is what
 *       running elsewhere costs, and it is measured.</li>
 * </ul>
 *
 * <p>Either way the copy into the slot signals the timeline at the step's value, on the GPU, so the picture's wait for
 * it is the same. The stepping thread's, as the simulation is.
 */
public final class ShownSlots implements AutoCloseable {

    private final Accelerator accelerator;
    private final boolean ownsAccelerator;
    private final int spheres;
    /** What a slot is copied from: the simulation's {@code shown}, or the buffer the host writes it into. */
    private final ResidentBuffer source;
    private final ResidentBuffer incoming;
    private final ResidentBuffer[] slots = new ResidentBuffer[ShownRing.SLOTS];
    private final DispatchSequence[] keeps = new DispatchSequence[ShownRing.SLOTS];
    private final KernelHandle keeper;
    private final GpuContext.Timeline timeline;

    private ShownSlots(Accelerator accelerator, boolean ownsAccelerator, int spheres, ResidentBuffer shown) {
        this.accelerator = accelerator;
        this.ownsAccelerator = ownsAccelerator;
        this.spheres = spheres;
        int words = Spheres.SHOWN_STRIDE * spheres;
        this.incoming = shown == null ? accelerator.allocate(Body.F32, words) : null;
        this.source = shown != null ? shown : incoming;
        this.keeper = accelerator.register(new KernelSpec(Spheres.keep(), List.of(
                KernelColumn.output("shown", 0, Spheres.KEEP_BUFFERS.get(0).element()).withLength(words),
                KernelColumn.output("slot", 1, Spheres.KEEP_BUFFERS.get(1).element()).withLength(words)))
                .withWorkgroupSize(Spheres.WORKGROUP)).orElseThrow();
        this.timeline = accelerator.timeline(0);
        for (int k = 0; k < slots.length; k++) {
            slots[k] = accelerator.allocate(Body.F32, words);
            keeps[k] = accelerator.sequence().dispatch(keeper, List.of(source, slots[k]), words).build();
        }
    }

    /** Slots on the device {@code sim} computes on, kept by a copy there. {@code sim} must be on a GPU. */
    public static ShownSlots beside(SphereSimulation sim) {
        if (sim.shown() == null) {
            throw new IllegalArgumentException("a simulation on the CPU has no device to keep its steps beside; "
                    + "use ShownSlots.on the device the picture is drawn on");
        }
        return new ShownSlots(sim.accelerator(), false, sim.spheres(), sim.shown());
    }

    /**
     * Slots on {@code render}, the device the picture is drawn on, for a simulation of {@code spheres} spheres that
     * runs anywhere else: each step is handed through the host.
     */
    public static ShownSlots on(GpuContext render, int spheres) {
        return new ShownSlots(Accelerator.on(render), true, spheres, null);
    }

    /** Whether a step gets here through the host, as it does from another device or the CPU. */
    public boolean throughHost() {
        return incoming != null;
    }

    /**
     * Keeps {@code sim}'s last step in slot {@code slot}, and sets the timeline to {@code value} once it is there, on
     * the GPU. From another device or the CPU, the step is read back and written here first, which waits.
     *
     * @return the copy's completion: the step is the picture's to take once it is done
     */
    public Completion keep(int slot, long value, SphereSimulation sim) {
        if (incoming != null) {
            incoming.write(sim.readShown());
        }
        return keeps[slot].run(List.of(), List.of(timeline.at(value)));
    }

    /**
     * The slots as a picture binds them.
     *
     * @param id     the generation's number, which the ring counts
     * @param extent the box, {@code sx, sy, sz}, in metres
     */
    public ShownRing.Generation generation(long id, double[] extent) {
        long[] handles = new long[slots.length];
        for (int k = 0; k < slots.length; k++) {
            handles[k] = slots[k].vkBuffer();
        }
        return new ShownRing.Generation(id, handles, timeline.handle(), spheres, extent);
    }

    /** Waits for what is in flight, then frees the slots and the timeline. Before the simulation it is beside. */
    @Override
    public void close() {
        accelerator.finish();
        for (DispatchSequence keep : keeps) {
            keep.close();
        }
        for (ResidentBuffer slot : slots) {
            slot.close();
        }
        if (incoming != null) {
            incoming.close();
        }
        accelerator.release(keeper);
        timeline.close();
        if (ownsAccelerator) {
            accelerator.close();
        }
    }
}

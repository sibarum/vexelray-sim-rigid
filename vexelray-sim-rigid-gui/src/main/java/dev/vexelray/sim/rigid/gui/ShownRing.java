package dev.vexelray.sim.rigid.gui;

import java.util.ArrayList;
import java.util.List;

/**
 * Finished steps handed from the thread that steps to the thread that draws, with neither ever waiting for the other:
 * a ring of three copies of a simulation's {@code shown}, and which of them is which.
 *
 * <h2>Three slots, three roles</h2>
 *
 * At any moment one slot is the <b>front</b>, the one the frame reads; one is <b>ready</b>, the newest finished
 * step the frame has not taken yet, or none; and the rest are free, one of which the next step is kept in. The writer
 * keeps a step only in a free slot and the reader reads only the front, so physics can keep step {@code n + 1} while a
 * frame reads step {@code n}, and three slots are always enough: with one front and one ready there is a third. A
 * step the frame never took is simply overwritten: the frame always shows the newest finished step, not every step.
 *
 * <h2>What this relies on</h2>
 *
 * <ul>
 *   <li><b>A step is published only once it is done.</b> The frame's draw waits for the step's timeline value inside
 *       the GPU, which makes the other queue's writes visible to it; because the value has already been reached,
 *       the wait costs nothing and never holds the frame.</li>
 *   <li><b>A frame is done with its front before it takes the next.</b> {@code SampledColorTarget.renderInto} returns
 *       only once its draw has finished, so when the frame takes a new front, the old one is free at once. A draw that
 *       stopped waiting would need to signal when it had finished reading, and the writer to wait for that.</li>
 * </ul>
 *
 * <h2>Generations</h2>
 *
 * A new simulation brings new buffers: {@link #install} starts a {@link Generation}. The frame goes on reading the old
 * one's front until it takes from the new, and only then is the old one {@linkplain #retired retired}, for the writer
 * to free. So a buffer is never freed while a frame may still read it.
 *
 * <p>Thread-safe, and the one thing the two threads share: every method holds the ring's lock for a few field writes.
 */
public final class ShownRing {

    /** Slots in a ring. */
    public static final int SLOTS = 3;

    /**
     * One simulation's ring.
     *
     * @param id       counts up from one, a generation per simulation
     * @param slots    the {@code VkBuffer} of each slot, {@link #SLOTS} of them, on the application's device
     * @param timeline the {@code VkSemaphore}, of timeline type, a kept step signals
     * @param spheres  how many spheres a slot holds
     * @param extent   the box, {@code sx, sy, sz}, in metres
     */
    public record Generation(long id, long[] slots, long timeline, int spheres, double[] extent) {

        public Generation {
            if (slots.length != SLOTS) {
                throw new IllegalArgumentException("a ring has " + SLOTS + " slots, got " + slots.length);
            }
            slots = slots.clone();
            extent = extent.clone();
        }

        /** The {@code VkBuffer} of slot {@code k}. */
        public long slot(int k) {
            return slots[k];
        }

        /** The box's size along axis {@code a}. */
        public double extent(int a) {
            return extent[a];
        }
    }

    /**
     * A finished step, as the frame reads it.
     *
     * @param generation    whose slot it is in
     * @param slot          which slot
     * @param step          its number in the generation, counting from one; the timeline reached this value with it
     * @param finishedNanos when it was published, on {@link System#nanoTime}'s clock
     */
    public record Frame(Generation generation, int slot, long step, long finishedNanos) {
    }

    private Generation generation;
    private final List<Generation> retired = new ArrayList<>();
    /** The generation the front belongs to, which may be older than {@link #generation}. */
    private Generation frontGeneration;
    private int front = -1;
    private int ready = -1;
    private int back;
    private final long[] steps = new long[SLOTS];
    private final long[] finished = new long[SLOTS];
    private Frame taken;

    /**
     * The writer: a new simulation's ring, from now on the one steps are kept in. The last generation is retired
     * once the frame has taken from this one, or at once if it never took from it.
     */
    public synchronized void install(Generation next) {
        if (generation != null && generation != frontGeneration) {
            retired.add(generation);
        }
        generation = next;
        ready = -1;
        back = 0;
        java.util.Arrays.fill(steps, 0);
    }

    /** The writer: the slot the next finished step is to be kept in. */
    public synchronized int back() {
        return back;
    }

    /**
     * The writer: the step just kept in {@link #back()} is finished, its timeline value reached. It is now the newest
     * the frame can take, and the next step goes in a slot the frame is not reading and has not been offered.
     */
    public synchronized void publish(long step, long finishedNanos) {
        steps[back] = step;
        finished[back] = finishedNanos;
        ready = back;
        for (int k = 0; k < SLOTS; k++) {
            if (k != ready && !(frontGeneration == generation && k == front)) {
                back = k;
                break;
            }
        }
    }

    /**
     * The writer: generations the frame has moved past, to free, each once. Their buffers are no longer read.
     */
    public synchronized List<Generation> retired() {
        List<Generation> out = List.copyOf(retired);
        retired.clear();
        return out;
    }

    /**
     * The reader: the newest finished step, taken as the front if it is newer than the one there; or the front as it
     * was, or null before any step has been published. The front taken before is free from this call on, so the frame
     * must be done drawing it.
     */
    public synchronized Frame take() {
        if (ready >= 0) {
            if (frontGeneration != null && frontGeneration != generation) {
                retired.add(frontGeneration);
            }
            front = ready;
            frontGeneration = generation;
            ready = -1;
            taken = new Frame(generation, front, steps[front], finished[front]);
        }
        return taken;
    }

    /** Every generation still live, the current one included: for whoever closes the ring last to free. */
    public synchronized List<Generation> all() {
        List<Generation> out = new ArrayList<>(retired);
        retired.clear();
        if (frontGeneration != null && frontGeneration != generation) {
            out.add(frontGeneration);
        }
        if (generation != null) {
            out.add(generation);
        }
        return out;
    }
}

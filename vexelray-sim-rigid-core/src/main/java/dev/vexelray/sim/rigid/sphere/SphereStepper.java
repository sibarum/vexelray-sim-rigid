package dev.vexelray.sim.rigid.sphere;

import dev.supirvast.vastir.pass.Pass;

import java.util.List;

/**
 * Runs a {@link SphereStep}: as its one list, or, for Gauss–Seidel over a contact list, pass by pass until every
 * contact is solved. That is the rule this exists for: a step advances the same time and solves every contact,
 * however long that takes.
 *
 * <p>A pass opens with a batch of rounds and a {@linkplain Spheres#report report}, and the runner waits for it and
 * reads the report where it is. While a contact is still open, another {@link SphereStep#BATCH} rounds go, and
 * another report. The opening batch is sized from what the passes before needed, so that most passes are one
 * submission and one wait: a wait costs far more than the empty rounds a generous batch runs at the end, each of
 * which finds nothing to do. The batch grows at once when a pass needs more, and shrinks a batch at a time after a
 * long run of passes that needed no more than it.
 *
 * <p>Not thread-safe: one stepper, one thread, as the runner it drives.
 */
public final class SphereStepper {

    /** What runs the passes: a {@code PassRunner}, by another name, since this module only describes them. */
    public interface Runner {

        /** Submits {@code passes}, without waiting. */
        void run(List<Pass> passes);

        /** Submits {@code passes}, waits for them, and returns the step's {@code readout} as they left it. */
        int[] runAndRead(List<Pass> passes);
    }

    /**
     * What a step took.
     *
     * @param waits      how many times the host waited for the GPU to say whether to go on
     * @param rounds     the rounds run, every pass of every substep together, empty ones at the end of a batch included
     * @param mostRounds the most rounds any one pass ran
     * @param unlisted   contacts that did not fit the list and so were not solved, every substep together: the one way
     *                   a step can still leave a contact unsolved, which a larger capacity answers
     */
    public record Report(int waits, int rounds, int mostRounds, int unlisted) {

        /** A step run as one list, which waits for nothing and reports nothing. */
        static final Report NONE = new Report(0, 0, 0, 0);
    }

    /** Passes in a row that needed no more than their opening batch before it is tried a batch smaller. */
    static final int SHRINK_AFTER = 120;

    private final SphereStep step;
    /** Per pass of a substep: the rounds its opening batch runs, and how many passes in a row it has been enough. */
    private final int[] opening;
    private final int[] enough;

    public SphereStepper(SphereStep step) {
        this.step = step;
        this.opening = new int[step.iterations];
        this.enough = new int[step.iterations];
        java.util.Arrays.fill(opening, SphereStep.BATCH);
    }

    /** One step. Returns once the last pass is known to be done; the step's closing passes may still be running. */
    public Report step(Runner runner) {
        if (!step.untilDone()) {
            runner.run(step.step());
            return Report.NONE;
        }
        int waits = 0;
        int rounds = 0;
        int most = 0;
        int unlisted = 0;
        // Each round solves at least the open contact of highest priority, so a pass needs no more rounds than the
        // list is long; past that, something is wrong with the rounds rather than the scene.
        int limit = step.capacity + 2 * SphereStep.CYCLE;
        for (int sub = 0; sub < step.substeps; sub++) {
            for (int it = 0; it < step.iterations; it++) {
                int k = opening[it];
                int[] read = runner.runAndRead(step.opening(sub, it, k));
                waits++;
                while (read[Spheres.READ_OPEN] != 0) {
                    if (k > limit) {
                        throw new IllegalStateException("a pass is still open after " + k + " rounds, on a list of "
                                + read[Spheres.READ_LISTED]);
                    }
                    read = runner.runAndRead(step.more(it, k, SphereStep.BATCH));
                    k += SphereStep.BATCH;
                    waits++;
                }
                rounds += k;
                most = Math.max(most, k);
                unlisted += Math.max(0, read[Spheres.READ_LISTED] - step.capacity);
                adapt(it, k);
            }
        }
        runner.run(step.closing());
        return new Report(waits, rounds, most, unlisted);
    }

    /** The rounds pass {@code iteration}'s next opening batch runs. */
    int opening(int iteration) {
        return opening[iteration];
    }

    private void adapt(int iteration, int ran) {
        if (ran > opening[iteration]) {
            opening[iteration] = Math.min(ran, SphereStep.CYCLE);
            enough[iteration] = 0;
        } else if (++enough[iteration] >= SHRINK_AFTER && opening[iteration] > SphereStep.BATCH) {
            opening[iteration] -= SphereStep.BATCH;
            enough[iteration] = 0;
        }
    }
}

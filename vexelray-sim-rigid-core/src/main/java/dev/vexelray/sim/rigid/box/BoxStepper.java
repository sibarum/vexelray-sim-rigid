package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.pass.Pass;

import java.util.Arrays;
import java.util.List;

/**
 * Runs a {@link BoxStep}: the broad phase and the collide, which pairs touch read back, those sorted on the host into
 * rounds of pairs that share no box, and the step's substeps run with them.
 *
 * <p>The rounds are made once a step, because the manifolds are: they do not change within it. Each touching pair goes
 * into the first round that has neither of its boxes yet, in the list's order, which is a greedy colouring of the
 * contact graph: as many rounds as the most crowded box has contacts, or one more. A column needs two; a pyramid,
 * six. The cost is one wait a step, for {@code contacts} to be read, and one write, of {@code rounds}. A pair that
 * finds all {@link BoxStep#MOST_ROUNDS} rounds taken is left unsolved for the step, and counted; so are partners a box
 * had no slot for.
 */
public final class BoxStepper {

    /** What the stepper is given to run with: a backend's runner, by another name. */
    public interface Runner {
        void run(List<Pass> passes);

        /** Runs {@code passes}, waits for them, and reads the int buffer {@code buffer} back. */
        int[] runAndRead(List<Pass> passes, String buffer);

        void write(String buffer, int[] values);
    }

    /**
     * What a step took.
     *
     * @param touching the pairs whose manifolds had points
     * @param points   the points of all of them
     * @param rounds   the rounds they were sorted into
     * @param unplaced touching pairs left unsolved, every round having one of their boxes
     * @param unlisted partners the broad phase found and had no slot for
     */
    public record Report(int touching, int points, int rounds, int unplaced, int unlisted) {

        static final Report NONE = new Report(0, 0, 0, 0, 0);
    }

    private final BoxStep step;
    private final int[] rounds;
    private final boolean[][] taken;

    public BoxStepper(BoxStep step) {
        this.step = step;
        this.rounds = new int[BoxStep.MOST_ROUNDS * step.width];
        this.taken = new boolean[BoxStep.MOST_ROUNDS][step.boxes];
    }

    /** One step. */
    public Report step(Runner runner) {
        if (step.slots == 0) {
            runner.run(step.step(0));
            return Report.NONE;
        }
        int[] contacts = runner.runAndRead(step.collide(), "contacts");
        int[] filled = new int[BoxStep.MOST_ROUNDS];
        for (boolean[] round : taken) {
            Arrays.fill(round, false);
        }
        Arrays.fill(rounds, -1);
        int used = 0;
        int touching = 0;
        int points = 0;
        int unplaced = 0;
        for (int p = 0; p < step.slots; p++) {
            int b = contacts[p];
            if (b < 0) {
                continue;
            }
            int a = p / BoxBroad.SLOTS;
            touching++;
            points += contacts[step.slots + p];
            int r = 0;
            while (r < BoxStep.MOST_ROUNDS && (taken[r][a] || taken[r][b])) {
                r++;
            }
            if (r == BoxStep.MOST_ROUNDS) {
                unplaced++;
            } else {
                taken[r][a] = taken[r][b] = true;
                rounds[r * step.width + filled[r]++] = p;
                used = Math.max(used, r + 1);
            }
        }
        int unlisted = 0;
        for (int s = 0; s < step.boxes; s++) {
            unlisted += Math.max(0, contacts[2 * step.slots + s] - BoxBroad.SLOTS);
        }
        if (used > 0) {
            runner.write("rounds", rounds);
        }
        runner.run(step.step(used));
        return new Report(touching, points, used, unplaced, unlisted);
    }
}

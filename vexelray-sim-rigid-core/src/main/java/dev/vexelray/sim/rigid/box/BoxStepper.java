package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.pass.Pass;

import java.util.Arrays;
import java.util.List;

/**
 * Runs a {@link BoxStep}: the pairs collided, which of them touch read back, those sorted on the host into rounds of
 * pairs that share no box, and the step's substeps run with them.
 *
 * <p>The rounds are made once a step, because the manifolds are: they do not change within it. Each touching pair goes
 * into the first round that has neither of its boxes yet, in the list's order, which is a greedy colouring of the
 * contact graph: as many rounds as the most crowded box has contacts, or one more. A column needs two; a pile, the
 * most contacts any box has. The cost is one wait a step, for the manifolds to be read. A pair that finds all
 * {@link BoxStep#MOST_ROUNDS} rounds taken is left unsolved for the step, and counted.
 */
public final class BoxStepper {

    /** What the stepper is given to run with: a backend's runner, by another name. */
    public interface Runner {
        void run(List<Pass> passes);

        /** Runs {@code passes}, waits for them, and reads {@code buffer} back. */
        float[] runAndRead(List<Pass> passes, String buffer);

        void write(String buffer, int[] values);
    }

    /**
     * What a step took.
     *
     * @param touching the pairs whose manifolds had points
     * @param points   the points of all of them
     * @param rounds   the rounds they were sorted into
     * @param unplaced touching pairs left unsolved, every round having one of their boxes
     */
    public record Report(int touching, int points, int rounds, int unplaced) {

        static final Report NONE = new Report(0, 0, 0, 0);
    }

    private final BoxStep step;
    private final int[][] rounds;
    private final boolean[][] taken;

    public BoxStepper(BoxStep step) {
        this.step = step;
        this.rounds = new int[BoxStep.MOST_ROUNDS][step.width];
        this.taken = new boolean[BoxStep.MOST_ROUNDS][step.boxes];
    }

    /** One step. */
    public Report step(Runner runner) {
        if (step.pairs == 0) {
            runner.run(step.step(0));
            return Report.NONE;
        }
        float[] manifold = runner.runAndRead(step.collide(), "manifold");
        int[] filled = new int[BoxStep.MOST_ROUNDS];
        for (boolean[] round : taken) {
            Arrays.fill(round, false);
        }
        int used = 0;
        int touching = 0;
        int points = 0;
        int unplaced = 0;
        for (int p = 0, a = 0, b = 1; p < step.pairs; p++) {
            int count = (int) manifold[p * BoxContact.MANIFOLD_WORDS + BoxContact.COUNT];
            if (count > 0) {
                touching++;
                points += count;
                int r = 0;
                while (r < BoxStep.MOST_ROUNDS && (taken[r][a] || taken[r][b])) {
                    r++;
                }
                if (r == BoxStep.MOST_ROUNDS) {
                    unplaced++;
                } else {
                    taken[r][a] = taken[r][b] = true;
                    rounds[r][filled[r]++] = p;
                    used = Math.max(used, r + 1);
                }
            }
            if (++b == step.boxes) {
                a++;
                b = a + 1;
            }
        }
        for (int r = 0; r < used; r++) {
            Arrays.fill(rounds[r], filled[r], step.width, -1);
            runner.write(BoxStep.round(r), rounds[r]);
        }
        runner.run(step.step(used));
        return new Report(touching, points, used, unplaced);
    }
}

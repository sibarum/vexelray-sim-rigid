package dev.vexelray.sim.rigid.sphere;

import dev.supirvast.vastir.pass.Pass;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * What a step run until done submits, against a runner that says how many rounds each pass needs, with no backend:
 * the segments, in order, the rounds they hold, and how the opening batch follows what passes need.
 */
class SphereStepperTest {

    /** Says a pass is open until it has run {@code needs} rounds, and records every segment it was given. */
    private static final class Needs implements SphereStepper.Runner {
        int needs;
        int listed = 10;
        int ran;
        final List<List<Pass>> segments = new ArrayList<>();
        final List<String> rounds = new ArrayList<>();

        Needs(int needs) {
            this.needs = needs;
        }

        @Override
        public void run(List<Pass> passes) {
            segments.add(passes);
        }

        @Override
        public int[] runAndRead(List<Pass> passes) {
            segments.add(passes);
            if (passes.stream().anyMatch(p -> p.name().equals("predict") || p.name().equals("walls"))) {
                ran = 0;   // a pass opens
            }
            for (Pass pass : passes) {
                if (pass.name().startsWith("round")) {
                    ran++;
                    rounds.add(pass.buffers().get(16));
                }
            }
            int[] readout = new int[Spheres.READOUT_WORDS];
            readout[Spheres.READ_OPEN] = ran < needs ? 1 : 0;
            readout[Spheres.READ_LISTED] = listed;
            return readout;
        }
    }

    private static SphereStep step(int substeps, int iterations) {
        return new SphereStep(64, substeps, iterations, SphereGrid.covering(1, 1, 1, 0.1),
                SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT);
    }

    @Test
    void aPassThatNeedsNoMoreThanItsOpeningIsOneWait() {
        SphereStep step = step(3, 1);
        Needs runner = new Needs(5);
        SphereStepper.Report report = new SphereStepper(step).step(runner);
        assertEquals(3, report.waits(), "one a substep");
        assertEquals(3 * SphereStep.BATCH, report.rounds());
        assertEquals(0, report.unlisted());
        assertEquals(4, runner.segments.size(), "three openings and the closing");
        assertSame(step.closing(), runner.segments.get(3));
    }

    @Test
    void aPassGoesOnABatchAtATimeUntilNothingIsOpen() {
        SphereStep step = step(1, 1);
        Needs runner = new Needs(3 * SphereStep.BATCH + 1);
        SphereStepper.Report report = new SphereStepper(step).step(runner);
        assertEquals(4, report.waits());
        assertEquals(4 * SphereStep.BATCH, report.mostRounds());
    }

    @Test
    void roundsPastTheCycleGoOnAroundItInOrder() {
        SphereStep step = step(1, 1);
        int needs = 2 * SphereStep.CYCLE + 5;
        Needs runner = new Needs(needs);
        new SphereStepper(step).step(runner);
        assertTrue(runner.rounds.size() >= needs);
        for (int k = 0; k < runner.rounds.size(); k++) {
            assertEquals("cycle0." + SphereStep.position(k), runner.rounds.get(k), "round " + k);
        }
        assertEquals(SphereStep.CYCLE, SphereStep.position(SphereStep.CYCLE));
        assertEquals(1, SphereStep.position(SphereStep.CYCLE + 1));
    }

    @Test
    void theOpeningGrowsToWhatAPassNeededAndShrinksSlowly() {
        SphereStep step = step(1, 2);
        Needs runner = new Needs(20);
        SphereStepper stepper = new SphereStepper(step);
        stepper.step(runner);
        assertEquals(24, stepper.opening(0), "the rounds the first pass ran, a batch at a time");
        assertEquals(24, stepper.opening(1));
        runner.needs = 3;
        for (int k = 0; k < SphereStepper.SHRINK_AFTER - 1; k++) {
            stepper.step(runner);
        }
        assertEquals(24, stepper.opening(0), "not before a long run of passes that needed less");
        stepper.step(runner);
        assertEquals(16, stepper.opening(0), "then a batch smaller");
    }

    @Test
    void aLaterPassOpensWithTheWallsOfTheOneBefore() {
        SphereStep step = step(2, 2);
        Needs runner = new Needs(1);
        new SphereStepper(step).step(runner);
        List<String> firsts = runner.segments.stream().map(s -> s.get(0).name()).toList();
        assertEquals(List.of("predict", "walls", "walls", "walls", "walls"), firsts);
        assertEquals("velocity", runner.segments.get(2).get(1).name(), "a new substep closes the last one");
        assertEquals("predict", runner.segments.get(2).get(2).name());
    }

    @Test
    void contactsTheListHadNoRoomForAreReported() {
        SphereStep step = step(2, 1);
        Needs runner = new Needs(1);
        runner.listed = step.capacity + 7;
        assertEquals(14, new SphereStepper(step).step(runner).unlisted());
    }

    @Test
    void aPassThatNeverEndsIsAnErrorNotAHang() {
        SphereStep step = step(1, 1);
        assertThrows(IllegalStateException.class, () -> new SphereStepper(step).step(new Needs(Integer.MAX_VALUE)));
    }
}

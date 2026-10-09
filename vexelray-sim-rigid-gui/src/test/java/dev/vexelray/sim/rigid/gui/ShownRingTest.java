package dev.vexelray.sim.rigid.gui;

import org.junit.jupiter.api.Test;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The ring's three roles, with no device: the writer never keeps a step in the slot the frame reads or the one it has
 * been offered, the frame always takes the newest finished step, and a generation is retired only once the frame has
 * moved past it.
 */
class ShownRingTest {

    private static ShownRing.Generation generation(long id) {
        return new ShownRing.Generation(id, new long[] {10 * id, 10 * id + 1, 10 * id + 2}, 99, 4,
                new double[] {1, 2, 3});
    }

    @Test
    void nothingIsTakenBeforeAStepIsPublished() {
        ShownRing ring = new ShownRing();
        ring.install(generation(1));
        assertNull(ring.take());
    }

    @Test
    void theWriterNeverKeepsAStepWhereTheFrameReadsOrHasBeenOffered() {
        ShownRing ring = new ShownRing();
        ring.install(generation(1));
        ShownRing.Frame front = null;
        for (long step = 1; step <= 200; step++) {
            int back = ring.back();
            if (front != null) {
                assertNotEquals(front.slot(), back, "step " + step + " would be kept in the slot the frame reads");
            }
            ring.publish(step, step * 1_000_000L);
            assertNotEquals(back, ring.back(), "nor in the slot just offered");
            if (step % 3 == 0 || step % 7 == 0) {   // frames now and then, at no fixed rate
                front = ring.take();
                assertEquals(step, front.step(), "the frame takes the newest finished step");
            }
        }
    }

    @Test
    void aFrameWithNothingNewKeepsWhatItHas() {
        ShownRing ring = new ShownRing();
        ring.install(generation(1));
        ring.publish(1, 1);
        ShownRing.Frame first = ring.take();
        assertSame(first, ring.take());
    }

    @Test
    void anOldGenerationIsRetiredOnlyOnceTheFrameHasMovedPastIt() {
        ShownRing ring = new ShownRing();
        ShownRing.Generation one = generation(1);
        ShownRing.Generation two = generation(2);
        ring.install(one);
        ring.publish(1, 1);
        ring.take();
        ring.install(two);
        ring.publish(1, 2);
        assertEquals(List.of(), ring.retired(), "the frame still reads the first generation's slot");
        assertSame(two, ring.take().generation());
        assertEquals(List.of(one), ring.retired());
        assertEquals(List.of(), ring.retired(), "each once");
    }

    @Test
    void aGenerationTheFrameNeverTookIsRetiredAtTheNext() {
        ShownRing ring = new ShownRing();
        ShownRing.Generation one = generation(1);
        ring.install(one);
        ring.publish(1, 1);
        ring.take();
        ShownRing.Generation two = generation(2);
        ring.install(two);
        ring.publish(1, 2);
        ring.install(generation(3));
        assertEquals(List.of(two), ring.retired(), "never read, so free at once");
        Set<Long> live = new HashSet<>();
        ring.all().forEach(g -> live.add(g.id()));
        assertTrue(live.containsAll(Set.of(1L, 3L)), "the front's and the current one are still live: " + live);
    }
}

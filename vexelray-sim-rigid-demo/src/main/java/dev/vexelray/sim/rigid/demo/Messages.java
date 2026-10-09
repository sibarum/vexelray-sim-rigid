package dev.vexelray.sim.rigid.demo;

import dev.vexelray.sim.rigid.gui.SphereRunner;
import dev.vexelray.sim.rigid.sphere.SphereStep;
import sibarum.atchung.Topic;

/**
 * What the frame tells {@link Physics}: the topics, named once so that an annotation and a publisher cannot disagree
 * by a typo, and what each carries. Every payload is a record of values, so it crosses to the physics lane as if it
 * had crossed a wire.
 */
final class Messages {

    static final String BUILD = "rigid.build";
    static final String RELAX = "rigid.relax";
    static final String NEXT = "rigid.next";

    static final Topic<Build> BUILD_TOPIC = Topic.of(BUILD, Build.class);
    static final Topic<Relax> RELAX_TOPIC = Topic.of(RELAX, Relax.class);
    static final Topic<Next> NEXT_TOPIC = Topic.of(NEXT, Next.class);

    private Messages() {
    }

    /**
     * A simulation of this shape, made on the physics lane and started from the scenario's start, or from where the
     * running one has got to if {@code carry} and the scenario is the same, solved by {@code solve}, on
     * {@code backend}. An edge: every one is
     * acted on.
     */
    record Build(Scenario scenario, SphereStep.Solve solve, int substeps, int iterations, boolean carry, double omega,
                 boolean averaged, SphereRunner.Backend backend) {
    }

    /** How the solve relaxes, from the next step on. A sample: only the newest matters. */
    record Relax(double omega, boolean averaged) {
    }

    /**
     * Run the next step, if the world's clock says one is due: published by the clock, on the timeline, as a step
     * comes due, and by physics to itself after a step. One a delivery, so the lane's other mail goes between.
     */
    record Next() {
    }
}

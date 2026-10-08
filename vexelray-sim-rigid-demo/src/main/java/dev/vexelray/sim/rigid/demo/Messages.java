package dev.vexelray.sim.rigid.demo;

import sibarum.atchung.Topic;

/**
 * What the frame tells {@link Physics}: the topics, named once so that an annotation and a publisher cannot disagree
 * by a typo, and what each carries. Every payload is a record of values, so it crosses to the physics lane as if it
 * had crossed a wire.
 */
final class Messages {

    static final String BUILD = "rigid.build";
    static final String RELAX = "rigid.relax";
    static final String ALLOW = "rigid.allow";
    static final String NEXT = "rigid.next";

    static final Topic<Build> BUILD_TOPIC = Topic.of(BUILD, Build.class);
    static final Topic<Relax> RELAX_TOPIC = Topic.of(RELAX, Relax.class);
    static final Topic<Allow> ALLOW_TOPIC = Topic.of(ALLOW, Allow.class);
    static final Topic<Next> NEXT_TOPIC = Topic.of(NEXT, Next.class);

    private Messages() {
    }

    /**
     * A simulation of this shape, made on the physics lane and started from the scenario's start, or from where the
     * running one has got to if {@code carry} and the scenario is the same. An edge: every one is acted on.
     */
    record Build(Scenario scenario, int substeps, int iterations, boolean carry, double omega, boolean averaged) {
    }

    /** How the solve relaxes, from the next step on. A sample: only the newest matters. */
    record Relax(double omega, boolean averaged) {
    }

    /**
     * The steps the clock has made due since the application started, counted, and whether the world is held. A
     * sample: the count only grows, so the newest says everything the older did. Physics runs steps back to back
     * until it has run as many, and never decides for itself that one is due.
     */
    record Allow(long due, boolean held) {
    }

    /** Physics to itself: run the next step, if one is due. One a delivery, so the lane's other mail goes between. */
    record Next() {
    }
}

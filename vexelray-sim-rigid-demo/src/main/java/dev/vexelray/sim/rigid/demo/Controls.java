package dev.vexelray.sim.rigid.demo;

import sibarum.kronometer.Ratio;

import java.util.List;

/**
 * What the user has asked for, held until the next frame takes it.
 *
 * <p>Key and widget handlers run on worker threads and the simulation lives on the main thread, so a handler only
 * records a request here and the frame acts on it: no handler touches the GPU. Settings are values the frame reads;
 * a reset is a flag it clears. Every change moves {@link #version}, which is how the panel knows to read the settings
 * back after a key changed one. Every launch starts from the defaults.
 */
final class Controls {

    /** How the solve relaxes; see {@code Spheres}. */
    enum Solver {
        AVERAGED("Averaged, ω = 1", 1, true),
        CONSTANT("Constant, ω = 1", 1, false),
        HALF("Constant, ω = 0.5", 0.5, false);

        final String label;
        final double omega;
        final boolean averaged;

        Solver(String label, double omega, boolean averaged) {
            this.label = label;
            this.omega = omega;
            this.averaged = averaged;
        }
    }

    /** The playback speeds: exact ratios, as a Kronometer tempo takes them. */
    static final List<Ratio> SPEEDS = List.of(Ratio.of(1, 8), Ratio.of(1, 4), Ratio.of(1, 2), Ratio.of(1, 1),
            Ratio.of(2, 1));

    static final List<Integer> SUBSTEPS = List.of(1, 2, 5, 10, 20, 40);
    static final List<Integer> ITERATIONS = List.of(1, 2, 5, 10);

    private Scenario scenario = Scenario.COLUMN;
    private Solver solver = Solver.AVERAGED;
    private int substeps = 10;
    private int iterations = 1;
    private Ratio speed = Ratio.of(1, 1);
    private boolean paused;
    private boolean reset;
    private long version;

    synchronized Scenario scenario() {
        return scenario;
    }

    synchronized void scenario(Scenario s) {
        if (s != scenario) {
            scenario = s;
            reset = true;
            version++;
        }
    }

    synchronized void nextScenario() {
        Scenario[] all = Scenario.values();
        scenario(all[(scenario.ordinal() + 1) % all.length]);
    }

    synchronized Solver solver() {
        return solver;
    }

    synchronized void solver(Solver s) {
        solver = s;
        version++;
    }

    synchronized int substeps() {
        return substeps;
    }

    synchronized void substeps(int n) {
        substeps = n;
        version++;
    }

    synchronized int iterations() {
        return iterations;
    }

    synchronized void iterations(int n) {
        iterations = n;
        version++;
    }

    synchronized Ratio speed() {
        return speed;
    }

    synchronized void speed(Ratio r) {
        speed = r;
        version++;
    }

    synchronized void faster() {
        speed(SPEEDS.get(Math.min(SPEEDS.size() - 1, SPEEDS.indexOf(speed) + 1)));
    }

    synchronized void slower() {
        speed(SPEEDS.get(Math.max(0, SPEEDS.indexOf(speed) - 1)));
    }

    synchronized boolean paused() {
        return paused;
    }

    synchronized void togglePause() {
        paused = !paused;
        version++;
    }

    synchronized void reset() {
        reset = true;
        version++;
    }

    /** Whether a start over was asked for since the last call, which clears it. */
    synchronized boolean takeReset() {
        boolean was = reset;
        reset = false;
        return was;
    }

    synchronized long version() {
        return version;
    }

    static String speedLabel(Ratio r) {
        if (r.equals(Ratio.of(1, 1))) {
            return "Real time";
        }
        return (r.den() == 1 ? Long.toString(r.num()) : r.num() + "/" + r.den()) + "×";
    }
}

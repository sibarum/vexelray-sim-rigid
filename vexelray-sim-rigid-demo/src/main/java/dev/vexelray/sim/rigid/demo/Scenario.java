package dev.vexelray.sim.rigid.demo;

import java.util.Random;

/**
 * The scenarios: the ones the sweep measures, and one larger, so that what the solver costs can be felt.
 *
 * <p>A scenario is a box and a starting state. The solver's settings are not part of it; they are the user's, and they
 * carry from one scenario to the next.
 */
enum Scenario {

    COLUMN("Column of 20", "Twenty spheres touching, in a tube a sphere wide. They should rest. With Jacobi they sink"
            + " into each other and jitter, and more substeps is what helps.") {
        @Override
        State start() {
            int n = 20;
            State s = new State(n, 0.1, 2.5, 0.1);
            for (int k = 0; k < n; k++) {
                s.place(k, 0.05, 0.05 + 0.1 * k, 0.05, 0.05, 1);
            }
            return s;
        }
    },

    PILE("Pile of 343", "A loose block of spheres dropped into a box. With no friction the pile slumps flat; how far"
            + " the spheres sink into each other while it does is the solver's error.") {
        @Override
        State start() {
            return block(7, 0.03, 0.1, 0.5, 1.5);
        }
    },

    BIG_PILE("Pile of 1000", "The same, three times as many. Every pair is tested, so a step costs nine times as much:"
            + " the reason the solver will need a broad phase.") {
        @Override
        State start() {
            return block(10, 0.025, 0.05, 0.6, 1.8);
        }
    };

    final String label;
    final String about;

    Scenario(String label, String about) {
        this.label = label;
        this.about = about;
    }

    abstract State start();

    /** A cube of {@code side³} spheres, loosely spaced and jittered, above the floor of a square box. */
    private static State block(int side, double radius, double mass, double width, double height) {
        State s = new State(side * side * side, width, height, width);
        Random random = new Random(7);
        double pitch = 2.3 * radius;
        double margin = (width - pitch * (side - 1)) / 2;
        int k = 0;
        for (int i = 0; i < side; i++) {
            for (int j = 0; j < side; j++) {
                for (int l = 0; l < side; l++, k++) {
                    s.place(k, margin + pitch * i + 0.2 * radius * random.nextDouble(), 0.3 + pitch * j,
                            margin + pitch * l + 0.2 * radius * random.nextDouble(), radius, mass);
                }
            }
        }
        return s;
    }

    /** Where every sphere starts, and the box: all a simulation is started from. */
    static final class State {
        final int n;
        final double[] extent;
        final float[] x;
        final float[] y;
        final float[] z;
        final float[] u;
        final float[] v;
        final float[] w;
        final float[] r;
        final float[] im;

        State(int n, double sx, double sy, double sz) {
            this.n = n;
            extent = new double[] {sx, sy, sz};
            x = new float[n];
            y = new float[n];
            z = new float[n];
            u = new float[n];
            v = new float[n];
            w = new float[n];
            r = new float[n];
            im = new float[n];
        }

        void place(int k, double px, double py, double pz, double radius, double mass) {
            x[k] = (float) px;
            y[k] = (float) py;
            z[k] = (float) pz;
            r[k] = (float) radius;
            im[k] = (float) (1 / mass);
        }
    }
}

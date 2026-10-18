package dev.vexelray.sim.rigid.demo;

import dev.vexelray.sim.rigid.sphere.SphereGrid;

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

    HEAP("Heap on a bed", "A tall, narrow stack dropped on a floor of fixed spheres, as granular experiments glue"
            + " grains to theirs, so that a sphere cannot roll away without climbing. With friction, under"
            + " Gauss–Seidel, it stands as a tall, steep heap; with none, or under Jacobi, which has no friction, it"
            + " slumps into a lower, ragged mound. On a smooth floor it spreads flat either way: friction does not stop"
            + " a sphere rolling, and nothing here resists rolling yet.") {
        @Override
        State start() {
            int side = 5;
            int layers = 14;
            int bed = 20;
            double radius = 0.03;
            double width = bed * 2 * radius;
            double pitch = 2.2 * radius;
            State s = new State(bed * bed + side * side * layers, width, 1.6, width);
            int k = 0;
            for (int i = 0; i < bed; i++) {
                for (int l = 0; l < bed; l++, k++) {
                    s.place(k, radius + 2 * radius * i, radius, radius + 2 * radius * l, radius, 0.1);
                    s.im[k] = 0;
                }
            }
            Random random = new Random(11);
            double margin = (width - pitch * (side - 1)) / 2;
            for (int j = 0; j < layers; j++) {
                for (int i = 0; i < side; i++) {
                    for (int l = 0; l < side; l++, k++) {
                        s.place(k, margin + pitch * i + 0.2 * radius * random.nextDouble(), 3.2 * radius + pitch * j,
                                margin + pitch * l + 0.2 * radius * random.nextDouble(), radius, 0.1);
                    }
                }
            }
            return s;
        }
    },

    BALLS("Bouncing balls", "Twelve balls dropped from different heights. Each rises to e² of its fall; at no"
            + " restitution they land and stay.") {
        @Override
        State start() {
            int n = 12;
            State s = new State(n, 1.2, 2.5, 0.4);
            for (int k = 0; k < n; k++) {
                s.place(k, 0.07 + 0.096 * k, 0.4 + 0.15 * k, 0.2, 0.04, 0.2);
            }
            return s;
        }
    },

    BIG_PILE("Pile of 1000", "The same, three times as many. The solve searches a grid, so a step costs about three"
            + " times as much, not the nine that testing every pair would; the picture still tests every sphere.") {
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

        /** The broad phase's grid over the box, its cells as wide as the widest sphere. */
        SphereGrid grid() {
            float widest = 0;
            for (float radius : r) {
                widest = Math.max(widest, 2 * radius);
            }
            return SphereGrid.covering(extent[0], extent[1], extent[2], widest);
        }
    }
}

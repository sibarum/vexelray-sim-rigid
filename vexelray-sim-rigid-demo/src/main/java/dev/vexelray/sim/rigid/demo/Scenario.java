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

    HEAP("Heap of 343", "A block of spheres on a wide, smooth floor, let go. Friction alone does not hold it: a sphere"
            + " rolls, and friction does not stop it rolling, so the block spreads into a single layer. With rolling"
            + " resistance as well, under Gauss–Seidel, it settles into a heap, the steeper the more there is.") {
        @Override
        State start() {
            int side = 7;
            double radius = 0.03;
            double width = 1.2;
            double pitch = 2.05 * radius;
            State s = new State(side * side * side, width, 1.2, width);
            Random random = new Random(11);
            double margin = (width - pitch * (side - 1)) / 2;
            int k = 0;
            for (int j = 0; j < side; j++) {
                for (int i = 0; i < side; i++) {
                    for (int l = 0; l < side; l++, k++) {
                        s.place(k, margin + pitch * i + 0.2 * radius * random.nextDouble(), 1.1 * radius + pitch * j,
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
    },

    SPINNING_BOX("Spinning box", "A box with no gravity, spun at 5 rad/s about its middle axis and nudged at 0.1 rad/s"
            + " about its shortest. A spin about the middle axis is not stable: the box turns over and back, every"
            + " 3.4 s, the period the closed form gives. Boxes take only the substeps and friction controls.") {
        @Override
        State start() {
            State s = State.boxes(1, 1.2, 1.2, 1.2, 0);
            s.box(0, 0.6, 0.6, 0.6, 0.075, 0.15, 0.3, 1, IDENTITY, new double[] {0, 5, 0.1});
            return s;
        }
    },

    DROPPED_BOXES("Dropped boxes", "Six boxes of different shapes, each turned and dropped onto a corner."
            + " Each settles onto a face; with friction they stop where they land. Boxes take only the substeps and"
            + " friction controls.") {
        @Override
        State start() {
            double[][] halves = {{0.05, 0.05, 0.05}, {0.04, 0.08, 0.12}, {0.1, 0.03, 0.06}, {0.03, 0.12, 0.03},
                    {0.07, 0.07, 0.02}, {0.05, 0.09, 0.07}};
            State s = State.boxes(halves.length, 1.8, 1.2, 0.6, -9.81);
            Random random = new Random(5);
            for (int k = 0; k < halves.length; k++) {
                double[] axis = {random.nextDouble() - 0.5, random.nextDouble() - 0.5, random.nextDouble() - 0.5};
                s.box(k, 0.2 + 0.28 * k, 0.4 + 0.08 * k, 0.3, halves[k][0], halves[k][1], halves[k][2], 0.5,
                        axisAngle(axis, 0.6 + 0.3 * k), new double[3]);
            }
            return s;
        }
    },

    PYRAMID("Pyramid of 55", "Cubes of 10 cm, ten at the base, each row set half a cube along on the one below."
            + " Every pair of cubes is tested each step, and the touching ones are solved in rounds that share no"
            + " cube. Boxes take only the substeps and friction controls.") {
        @Override
        State start() {
            int base = 10;
            State s = State.boxes(base * (base + 1) / 2, 1.6, 1.4, 0.5, -9.81);
            int k = 0;
            for (int row = 0; row < base; row++) {
                for (int c = 0; c < base - row; c++, k++) {
                    s.box(k, 0.3 + 0.1005 * (c + row / 2.0), 0.05 + 0.1 * row, 0.25, 0.05, 0.05, 0.05, 1, IDENTITY,
                            new double[3]);
                }
            }
            return s;
        }
    };

    private static final double[] IDENTITY = {0, 0, 0, 1};

    final String label;
    final String about;

    Scenario(String label, String about) {
        this.label = label;
        this.about = about;
    }

    abstract State start();

    /** Whether its bodies are boxes, which take only the substeps and friction controls. */
    boolean boxes() {
        return this == SPINNING_BOX || this == DROPPED_BOXES || this == PYRAMID;
    }

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

    /** The quaternion that turns by {@code angle} about {@code axis}, {@code (x, y, z, w)}. */
    private static double[] axisAngle(double[] axis, double angle) {
        double length = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        double sin = Math.sin(angle / 2) / length;
        return new double[] {axis[0] * sin, axis[1] * sin, axis[2] * sin, Math.cos(angle / 2)};
    }

    /**
     * Where every body starts, the box they are in, and the gravity: all a simulation is started from. Spheres have
     * radii; boxes have half-extents, orientations and spins instead, and {@link #boxes} says which.
     */
    static final class State {
        final int n;
        final double[] extent;
        final boolean boxes;
        /** Gravity along y, in metres per second squared. */
        final double gravity;
        final float[] x;
        final float[] y;
        final float[] z;
        final float[] u;
        final float[] v;
        final float[] w;
        final float[] r;
        final float[] im;
        /** A box's half-extents, {@code hx, hy, hz}; its orientation, {@code qx, qy, qz, qw}; its angular velocity. */
        final float[][] half;
        final float[][] q;
        final float[][] spin;

        State(int n, double sx, double sy, double sz) {
            this(n, sx, sy, sz, false, -9.81);
        }

        private State(int n, double sx, double sy, double sz, boolean boxes, double gravity) {
            this.n = n;
            extent = new double[] {sx, sy, sz};
            this.boxes = boxes;
            this.gravity = gravity;
            x = new float[n];
            y = new float[n];
            z = new float[n];
            u = new float[n];
            v = new float[n];
            w = new float[n];
            r = new float[n];
            im = new float[n];
            half = new float[3][n];
            q = new float[4][n];
            spin = new float[3][n];
            java.util.Arrays.fill(q[3], 1);
        }

        /** {@code n} boxes in a box of this size, under {@code gravity} along y. */
        static State boxes(int n, double sx, double sy, double sz, double gravity) {
            return new State(n, sx, sy, sz, true, gravity);
        }

        void box(int k, double px, double py, double pz, double hx, double hy, double hz, double mass,
                 double[] turn, double[] omega) {
            x[k] = (float) px;
            y[k] = (float) py;
            z[k] = (float) pz;
            im[k] = (float) (1 / mass);
            double[] h = {hx, hy, hz};
            for (int a = 0; a < 3; a++) {
                half[a][k] = (float) h[a];
                spin[a][k] = (float) omega[a];
            }
            for (int j = 0; j < 4; j++) {
                q[j][k] = (float) turn[j];
            }
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

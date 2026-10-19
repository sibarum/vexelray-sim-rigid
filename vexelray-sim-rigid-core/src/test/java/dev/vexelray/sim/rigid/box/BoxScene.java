package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.tools.Accelerator;
import dev.supirvast.vastir.tools.PassRunner;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * A set of boxes, put on a backend and stepped: what each test builds, then reads back, as the spheres' {@code Scene}
 * is.
 *
 * <p>Set the fields, then {@link #start}; after that {@link #advance} steps on the backend and {@link #read} brings the
 * state back into the same fields.
 */
final class BoxScene implements AutoCloseable {

    /** Which backend a scene runs on. Each is judged on its own; the two are not compared. */
    enum Backend { CPU, GPU }

    static Stream<Backend> backends() {
        return Stream.of(Backend.values());
    }

    final int n;
    final float[] x;
    final float[] y;
    final float[] z;
    final float[] u;
    final float[] v;
    final float[] w;
    /**
     * The angular velocity in the world's frame, in radians a second: written as the angular momentum it makes, and
     * read back from it.
     */
    final float[] ax;
    final float[] ay;
    final float[] az;
    /** The angular momentum, {@code lx, ly, lz}, as the backend keeps it: set by {@link #start}, then {@link #read}. */
    final float[][] l;
    /** The orientation, {@code qx, qy, qz, qw}: the identity unless set. */
    final float[][] q;
    /** The half-extents along the box's own axes. */
    final float[][] half;
    /** Each body's shape: {@link Boxes#BOX_SHAPE}, the default, or {@link Boxes#SPHERE_SHAPE}. */
    final float[] shape;
    final float[] im;

    double gx;
    double gy = -9.81;
    double gz;
    double sx = 1;
    double sy = 1;
    double sz = 1;
    /** The frame: a step is this long, cut into {@link #substeps}. */
    double dt = 1.0 / 60;
    int substeps = 10;
    /** Friction at the walls, static and kinetic: {@link Boxes#walls}. */
    double muStatic;
    double muKinetic;
    /** Rolling resistance where a sphere is in a contact: {@link Boxes#roll}. */
    double muRolling;
    /** Sweeps of each kind a substep makes over its contacts, the walls' and the pairs'. */
    int passes = Boxes.WALL_PASSES;
    /** How far apart two boxes may be at the start of a step and still have their contact found. */
    double margin = 0.005;
    /** Whether every other sweep runs backwards. */
    boolean symmetric;

    /** What the steps have taken, added up: see {@link BoxStepper.Report}. */
    int mostRounds;
    long unplaced;
    long unlisted;
    BoxStepper.Report last;

    private Accelerator accelerator;
    private PassRunner runner;
    private BoxStep step;
    private BoxStepper stepper;

    BoxScene(int n) {
        this.n = n;
        x = new float[n];
        y = new float[n];
        z = new float[n];
        u = new float[n];
        v = new float[n];
        w = new float[n];
        ax = new float[n];
        ay = new float[n];
        az = new float[n];
        l = new float[3][n];
        q = new float[4][n];
        java.util.Arrays.fill(q[3], 1);
        half = new float[3][n];
        shape = new float[n];
        im = new float[n];
        java.util.Arrays.fill(im, 1);
    }

    /** Body {@code s} a sphere of radius {@code r}. */
    BoxScene sphere(int s, double r) {
        shape[s] = Boxes.SPHERE_SHAPE;
        for (int a = 0; a < 3; a++) {
            half[a][s] = (float) r;
        }
        return this;
    }

    /** Every box these half-extents and this mass. */
    BoxScene uniform(double hx, double hy, double hz, double mass) {
        java.util.Arrays.fill(half[0], (float) hx);
        java.util.Arrays.fill(half[1], (float) hy);
        java.util.Arrays.fill(half[2], (float) hz);
        java.util.Arrays.fill(im, (float) (1 / mass));
        return this;
    }

    BoxScene start(Backend backend) {
        double widest = 0;
        for (int s = 0; s < n; s++) {
            widest = Math.max(widest, Math.sqrt(half[0][s] * half[0][s] + half[1][s] * half[1][s]
                    + half[2][s] * half[2][s]));
        }
        step = new BoxStep(n, substeps, passes, symmetric, BoxBroad.grid(sx, sy, sz, widest, margin));
        stepper = new BoxStepper(step);
        if (backend == Backend.CPU) {
            runner = PassRunner.cpu(step, Boxes.WORKGROUP, PassRunner.NO_SUBGROUP);
        } else {
            accelerator = new Accelerator();
            if (!accelerator.capabilities().gpuAvailable()) {
                accelerator.close();
                accelerator = null;
                assumeTrue(Boolean.getBoolean("supirvast.requireGpu"), "no Vulkan device");
                throw new IllegalStateException("-Dsupirvast.requireGpu=true but no Vulkan device");
            }
            runner = PassRunner.gpu(accelerator, step, Boxes.WORKGROUP, PassRunner.NO_SUBGROUP);
            runner.clear();
        }
        step.constants().forEach(runner::write);
        runner.write("params", Boxes.params(dt / substeps, gx, gy, gz, sx, sy, sz, muStatic, muKinetic,
                margin, muRolling));
        for (int s = 0; s < n; s++) {
            if (im[s] <= 0) {
                continue;
            }
            double[] spun = BoxDiagnostics.momentum(new double[] {ax[s], ay[s], az[s]}, turn(s), moment(s));
            for (int a = 0; a < 3; a++) {
                l[a][s] = (float) spun[a];
            }
        }
        String[][] fields = {{"x", "y", "z"}, {"u", "v", "w"}, {"lx", "ly", "lz"}, {"hx", "hy", "hz"}};
        float[][][] values = {{x, y, z}, {u, v, w}, l, half};
        for (int f = 0; f < fields.length; f++) {
            for (int a = 0; a < 3; a++) {
                runner.write(fields[f][a], values[f][a]);
            }
        }
        String[] turn = {"qx", "qy", "qz", "qw"};
        for (int k = 0; k < 4; k++) {
            runner.write(turn[k], q[k]);
        }
        runner.write("im", im);
        runner.write("shape", shape);
        return this;
    }

    /** {@code frames} steps of {@link #dt}. */
    BoxScene advance(int frames) {
        BoxStepper.Runner run = new BoxStepper.Runner() {
            @Override
            public void run(java.util.List<dev.supirvast.vastir.pass.Pass> passes) {
                runner.run(passes);
            }

            @Override
            public int[] runAndRead(java.util.List<dev.supirvast.vastir.pass.Pass> passes, String buffer) {
                runner.run(passes).await();
                return runner.read(buffer);
            }

            @Override
            public void write(String buffer, int[] values) {
                runner.write(buffer, values);
            }
        };
        for (int k = 0; k < frames; k++) {
            last = stepper.step(run);
            mostRounds = Math.max(mostRounds, last.rounds());
            unplaced += last.unplaced();
            unlisted += last.unlisted();
        }
        return this;
    }

    /** {@link #advance} for as many frames as {@code seconds} takes. */
    BoxScene advanceSeconds(double seconds) {
        return advance((int) Math.round(seconds / dt));
    }

    BoxScene read() {
        String[] names = {"x", "y", "z", "u", "v", "w", "lx", "ly", "lz", "qx", "qy", "qz", "qw"};
        float[][] into = {x, y, z, u, v, w, l[0], l[1], l[2], q[0], q[1], q[2], q[3]};
        for (int k = 0; k < names.length; k++) {
            System.arraycopy(runner.floats(names[k]), 0, into[k], 0, n);
        }
        float[][] spin = {ax, ay, az};
        for (int s = 0; s < n; s++) {
            double[] omega = im[s] > 0 ? BoxDiagnostics.spin(new double[] {l[0][s], l[1][s], l[2][s]}, turn(s),
                    moment(s)) : new double[3];
            for (int a = 0; a < 3; a++) {
                spin[a][s] = (float) omega[a];
            }
        }
        return this;
    }

    /** The picture's buffer as it is on the backend now, {@link Boxes#SHOWN_STRIDE} floats a box. */
    float[] shown() {
        return runner.floats("shown");
    }

    /** Any float buffer of the step as it is on the backend now. */
    float[] floats(String name) {
        return runner.floats(name);
    }

    /** The step's slots, {@link BoxStep#slots}, for reading its manifolds. */
    int slots() {
        return step.slots;
    }

    BoxDiagnostics diagnostics() {
        read();
        return BoxDiagnostics.of(x, y, z, u, v, w, l, q, half, shape, im);
    }

    private double[] turn(int s) {
        return new double[] {q[0][s], q[1][s], q[2][s], q[3][s]};
    }

    private double[] moment(int s) {
        return Boxes.inertia(shape[s], 1 / im[s], half[0][s], half[1][s], half[2][s]);
    }

    /** Box {@code s}'s angular velocity in its own frame, as the state was last {@link #read}. */
    double[] ownSpin(int s) {
        double[] turn = {q[0][s], q[1][s], q[2][s], q[3][s]};
        return BoxDiagnostics.rotate(BoxDiagnostics.conjugate(turn), new double[] {ax[s], ay[s], az[s]});
    }

    @Override
    public void close() {
        if (runner != null) {
            runner.close();
        }
        if (accelerator != null) {
            accelerator.close();
        }
    }
}

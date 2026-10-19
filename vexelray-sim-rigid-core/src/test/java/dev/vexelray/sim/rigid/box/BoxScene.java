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
    /** Sweeps over its corners each box makes against the walls a substep. */
    int passes = Boxes.WALL_PASSES;

    private Accelerator accelerator;
    private PassRunner runner;
    private BoxStep step;

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
        im = new float[n];
        java.util.Arrays.fill(im, 1);
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
        step = new BoxStep(n, substeps);
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
                passes));
        for (int s = 0; s < n; s++) {
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
        return this;
    }

    /** {@code frames} steps of {@link #dt}. */
    BoxScene advance(int frames) {
        for (int k = 0; k < frames; k++) {
            runner.run(step.step());
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

    BoxDiagnostics diagnostics() {
        read();
        return BoxDiagnostics.of(x, y, z, u, v, w, l, q, half, im);
    }

    private double[] turn(int s) {
        return new double[] {q[0][s], q[1][s], q[2][s], q[3][s]};
    }

    private double[] moment(int s) {
        return Boxes.inertia(1 / im[s], half[0][s], half[1][s], half[2][s]);
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

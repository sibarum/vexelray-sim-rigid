package dev.vexelray.sim.rigid.box;

/**
 * What a state of boxes is, judged on the host in doubles: its moving energy, translational and rotational, and its
 * momentum and angular momentum. Fixed boxes count for nothing.
 *
 * @param kinetic    the whole moving energy, in joules
 * @param rotational the part of it in turning, {@code ½ Lᵀ R I⁻¹ Rᵀ L}
 * @param momentum   {@code Σ m v}
 * @param angular    the angular momentum about the origin, {@code Σ (m x × v + L)}
 */
public record BoxDiagnostics(double kinetic, double rotational, double[] momentum, double[] angular) {

    /**
     * The state's diagnostics, from per-box arrays as {@link BoxStep} holds them; {@code l} is {@code lx, ly, lz},
     * {@code q} {@code qx, qy, qz, qw}, {@code half} {@code hx, hy, hz} and {@code shape} each body's shape.
     */
    public static BoxDiagnostics of(float[] x, float[] y, float[] z, float[] u, float[] v, float[] w, float[][] l,
                                    float[][] q, float[][] half, float[] shape, float[] im) {
        double kinetic = 0;
        double rotational = 0;
        double[] momentum = new double[3];
        double[] angular = new double[3];
        for (int s = 0; s < x.length; s++) {
            if (im[s] <= 0) {
                continue;
            }
            double m = 1 / im[s];
            double[] at = {x[s], y[s], z[s]};
            double[] vel = {u[s], v[s], w[s]};
            double[] spun = {l[0][s], l[1][s], l[2][s]};
            double[] omega = spin(spun, new double[] {q[0][s], q[1][s], q[2][s], q[3][s]},
                    Boxes.inertia(shape[s], m, half[0][s], half[1][s], half[2][s]));
            double[] orbit = cross(at, vel);
            for (int a = 0; a < 3; a++) {
                kinetic += 0.5 * m * vel[a] * vel[a];
                rotational += 0.5 * spun[a] * omega[a];
                momentum[a] += m * vel[a];
                angular[a] += m * orbit[a] + spun[a];
            }
        }
        return new BoxDiagnostics(kinetic + rotational, rotational, momentum, angular);
    }

    /** The angular velocity, in the world's frame, of a box with angular momentum {@code l}, turned by {@code q}. */
    public static double[] spin(double[] l, double[] q, double[] moment) {
        double[] own = rotate(conjugate(q), l);
        return rotate(q, new double[] {own[0] / moment[0], own[1] / moment[1], own[2] / moment[2]});
    }

    /** The angular momentum, in the world's frame, of a box spinning at {@code omega}, turned by {@code q}. */
    public static double[] momentum(double[] omega, double[] q, double[] moment) {
        double[] own = rotate(conjugate(q), omega);
        return rotate(q, new double[] {own[0] * moment[0], own[1] * moment[1], own[2] * moment[2]});
    }

    /** {@code p} turned by the unit quaternion {@code q}, {@code (x, y, z, w)}. */
    public static double[] rotate(double[] q, double[] p) {
        double[] axis = {q[0], q[1], q[2]};
        double[] t = cross(axis, p);
        for (int a = 0; a < 3; a++) {
            t[a] *= 2;
        }
        double[] qt = cross(axis, t);
        return new double[] {p[0] + q[3] * t[0] + qt[0], p[1] + q[3] * t[1] + qt[1], p[2] + q[3] * t[2] + qt[2]};
    }

    public static double[] conjugate(double[] q) {
        return new double[] {-q[0], -q[1], -q[2], q[3]};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
}

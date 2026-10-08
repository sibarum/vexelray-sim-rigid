package dev.vexelray.sim.rigid.sphere;

/**
 * What a state of spheres says about the solver, read on the host in double precision.
 *
 * <p>The overlaps are what the solver failed to undo: the deepest one between two spheres, and the deepest one into a
 * wall, both as a share of the smaller radius involved, so they read the same at any scale. The energies and the
 * momentum are for the conservation tests and the at-rest ones: a stack that is resting has no kinetic energy, and one
 * that jitters does.
 *
 * @param maxOverlap     the deepest overlap between two spheres, over the smaller radius
 * @param maxWall        the deepest a sphere reaches past a wall, over its radius
 * @param kinetic        {@code Σ ½ m |v|²}, in joules; fixed spheres count nothing
 * @param potential      {@code Σ m · (−g) · y}, against the floor, in joules, for gravity along y
 * @param momentum       {@code Σ m v}, x, y and z, in kilogram metres per second
 * @param maxSpeed       the fastest sphere, in metres per second
 * @param broken         whether any position or velocity is not finite
 */
public record SphereDiagnostics(double maxOverlap, double maxWall, double kinetic, double potential,
                                double[] momentum, double maxSpeed, boolean broken) {

    /** Reads a state: positions, velocities, radii and inverse masses, all as long as the sphere count. */
    public static SphereDiagnostics of(float[] x, float[] y, float[] z, float[] u, float[] v, float[] w, float[] r,
                                       float[] im, double gy, double sx, double sy, double sz) {
        int n = x.length;
        double maxOverlap = 0;
        double maxWall = 0;
        double kinetic = 0;
        double potential = 0;
        double[] momentum = new double[3];
        double maxSpeed = 0;
        boolean broken = false;
        for (int a = 0; a < n; a++) {
            for (float value : new float[] {x[a], y[a], z[a], u[a], v[a], w[a]}) {
                broken |= !Float.isFinite(value);
            }
            for (int b = a + 1; b < n; b++) {
                double dx = x[a] - x[b];
                double dy = y[a] - y[b];
                double dz = z[a] - z[b];
                double overlap = r[a] + r[b] - Math.sqrt(dx * dx + dy * dy + dz * dz);
                maxOverlap = Math.max(maxOverlap, overlap / Math.min(r[a], r[b]));
            }
            double[] at = {x[a], y[a], z[a]};
            double[] extent = {sx, sy, sz};
            for (int axis = 0; axis < 3; axis++) {
                maxWall = Math.max(maxWall, (r[a] - at[axis]) / r[a]);
                maxWall = Math.max(maxWall, (at[axis] + r[a] - extent[axis]) / r[a]);
            }
            double speed2 = (double) u[a] * u[a] + (double) v[a] * v[a] + (double) w[a] * w[a];
            maxSpeed = Math.max(maxSpeed, Math.sqrt(speed2));
            if (im[a] > 0) {
                double m = 1.0 / im[a];
                kinetic += 0.5 * m * speed2;
                potential += m * -gy * y[a];
                momentum[0] += m * u[a];
                momentum[1] += m * v[a];
                momentum[2] += m * w[a];
            }
        }
        return new SphereDiagnostics(maxOverlap, maxWall, kinetic, potential, momentum, maxSpeed, broken);
    }

    @Override
    public String toString() {
        return String.format("overlap %.2e  wall %.2e  KE %.3e  PE %.3e  p (%.2e, %.2e, %.2e)  vmax %.3e%s",
                maxOverlap, maxWall, kinetic, potential, momentum[0], momentum[1], momentum[2], maxSpeed,
                broken ? "  BROKEN" : "");
    }
}

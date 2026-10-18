package dev.vexelray.sim.rigid.sphere;

import dev.vexelray.sim.rigid.sphere.Scene.Backend;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;

import java.util.Random;

/**
 * Measurements, not assertions: whether what should rest with friction does, and how fast it creeps when it does not.
 * The pile of 343 over six seconds, its energies every half second and its contacts' slip; and the few-sphere cases
 * that took the creep apart: a column, a sphere wedged on a slope, one in a pocket of three, and a free pyramid of
 * four. {@code -Drigid.creep=true} runs them; {@code -Drigid.backend=CPU} on the CPU; {@code -Drigid.mu},
 * {@code -Drigid.substeps} and {@code -Drigid.iterations} set the solve, and {@code -Drigid.dump=true} prints what each
 * contact took. {@code docs/TODO.md} has what they measured.
 */
@EnabledIfSystemProperty(named = "rigid.creep", matches = "true")
class CreepTest {

    private static final Backend BACKEND = Backend.valueOf(System.getProperty("rigid.backend", "GPU"));

    @Test
    void pileOf343WithFriction() {
        int substeps = Integer.getInteger("rigid.substeps", 10);
        int iterations = Integer.getInteger("rigid.iterations", 1);
        double mu = Double.parseDouble(System.getProperty("rigid.mu", "0.5"));
        int side = 7;
        int n = side * side * side;
        try (Scene scene = new Scene(n).uniform(0.03, 0.1)) {
            scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
            scene.substeps = substeps;
            scene.iterations = iterations;
            scene.omega = 1;
            scene.averaged = false;
            scene.muStatic = scene.muKinetic = mu;
            scene.sx = scene.sz = 0.08 + 0.06 * side;
            scene.sy = 0.5 + 0.07 * side;
            Random random = new Random(7);
            int k = 0;
            for (int i = 0; i < side; i++) {
                for (int j = 0; j < side; j++) {
                    for (int l = 0; l < side; l++, k++) {
                        scene.x[k] = (float) (0.07 + 0.06 * i + 0.005 * random.nextDouble());
                        scene.y[k] = (float) (0.3 + 0.07 * j);
                        scene.z[k] = (float) (0.07 + 0.06 * l + 0.005 * random.nextDouble());
                    }
                }
            }
            System.out.printf("pile of %d, mu %.2f, %d x %d, %s%n", n, mu, substeps, iterations, BACKEND);
            System.out.printf("%5s  %-10s %-10s %-10s %-10s %-10s %s%n", "t", "PE", "dPE/s", "KE move", "KE spin",
                    "overlap", "moving");
            scene.start(BACKEND);
            double last = Double.NaN;
            for (int half = 1; half <= 12; half++) {
                scene.advance(30);
                SphereDiagnostics state = scene.diagnostics();
                double pe = state.potential();
                int moving = 0;
                for (int s = 0; s < n; s++) {
                    if (scene.u[s] * scene.u[s] + scene.v[s] * scene.v[s] + scene.w[s] * scene.w[s] > 1e-4) {
                        moving++;
                    }
                }
                System.out.printf("%5.1f  %-10.4f %-10.2e %-10.2e %-10.2e %-10.2e %d  %s%n", half * 0.5, pe,
                        (last - pe) / 0.5, state.kinetic() - state.rotational(), state.rotational(),
                        state.maxOverlap(), moving, contacts(scene));
                last = pe;
            }
            // Whether the last pass's friction reached every contact in the rounds its parting took: done with the
            // rubbing's own pass, the step's last, 2 · iterations.
            int listed = Math.min(scene.ints("contactCount")[Spheres.LISTED], n * SphereStep.CONTACTS_PER_SPHERE);
            int[] done = scene.ints("contactDone");
            int unrubbed = 0;
            for (int c = 0; c < listed; c++) {
                if (done[c] != 2 * iterations) {
                    unrubbed++;
                }
            }
            System.out.printf("  %d of %d contacts not rubbed in the last pass%n", unrubbed, listed);
            Integer[] bySpin = new Integer[n];
            for (int s = 0; s < n; s++) {
                bySpin[s] = s;
            }
            java.util.Arrays.sort(bySpin, (p, q) -> Double.compare(spin2(scene, q), spin2(scene, p)));
            for (int top = 0; top < 8; top++) {
                int s = bySpin[top];
                int touching = 0;
                for (int o = 0; o < n; o++) {
                    double dx = scene.x[s] - scene.x[o];
                    double dy = scene.y[s] - scene.y[o];
                    double dz = scene.z[s] - scene.z[o];
                    if (o != s && Math.sqrt(dx * dx + dy * dy + dz * dz) < 0.06 * 1.01) {
                        touching++;
                    }
                }
                System.out.printf("  sphere %3d spin %.2e rad/s, speed %.2e m/s at (%.3f, %.3f, %.3f) of (%.2f, %.2f,"
                                + " %.2f), %d spheres touching%n", s, Math.sqrt(spin2(scene, s)),
                        Math.sqrt(scene.u[s] * scene.u[s] + scene.v[s] * scene.v[s] + scene.w[s] * scene.w[s]),
                        scene.x[s], scene.y[s], scene.z[s], scene.sx, scene.sy, scene.sz, touching);
            }
        }
    }

    /**
     * With {@code -Drigid.dump=true}, what every listed contact and every sphere's floor has taken in the last
     * substep: its normal impulse and its tangential one, in position terms.
     */
    private static void dump(Scene scene) {
        if (!Boolean.getBoolean("rigid.dump")) {
            return;
        }
        int listed = scene.ints("contactCount")[Spheres.LISTED];
        int[] first = scene.ints("contactA");
        int[] second = scene.ints("contactB");
        float[] impulse = scene.floats("contactImpulse");
        float[] walls = scene.floats("wallImpulse");
        int w = Spheres.IMPULSE_WORDS;
        for (int q = 0; q < listed; q++) {
            System.out.printf("    %d-%d  normal %.3e  tangent (%.2e, %.2e, %.2e)%n", first[q], second[q],
                    impulse[w * q], impulse[w * q + 1], impulse[w * q + 2], impulse[w * q + 3]);
        }
        for (int s = 0; s < scene.n; s++) {
            int floor = Spheres.WALL_IMPULSE_WORDS * s + w;
            if (walls[floor] > 0) {
                System.out.printf("    floor %d  normal %.3e  tangent (%.2e, %.2e, %.2e)%n", s, walls[floor],
                        walls[floor + 1], walls[floor + 2], walls[floor + 3]);
            }
        }
    }

    private static double spin2(Scene s, int k) {
        return s.ax[k] * s.ax[k] + s.ay[k] * s.ay[k] + s.az[k] * s.az[k];
    }

    /**
     * A tower stacked square, layer on layer, in a box a little wider than it: without friction each layer falls into
     * the hollows of the one below and the tower slumps; with it, the question is whether it stands.
     */
    @Test
    void towerInABox() {
        int side = Integer.getInteger("rigid.side", 4);
        int layers = Integer.getInteger("rigid.layers", 12);
        double r = 0.03;
        double gap = Double.parseDouble(System.getProperty("rigid.gap", "0.002"));
        try (Scene scene = new Scene(side * side * layers).uniform(r, 0.1)) {
            scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
            scene.muStatic = scene.muKinetic = Double.parseDouble(System.getProperty("rigid.mu", "0.5"));
            scene.muRolling = Double.parseDouble(System.getProperty("rigid.rolling", "0"));
            double width = side * 2 * r + Double.parseDouble(System.getProperty("rigid.slack", "0.01"));
            scene.sx = scene.sz = width;
            scene.sy = layers * 2 * r + 0.5;
            double pitch = 2 * r + gap;
            double jitter = Double.parseDouble(System.getProperty("rigid.jitter", "0.004"));
            Random random = new Random(3);
            double margin = (width - pitch * (side - 1)) / 2;
            int k = 0;
            for (int j = 0; j < layers; j++) {
                for (int i = 0; i < side; i++) {
                    for (int l = 0; l < side; l++, k++) {
                        scene.x[k] = (float) (margin + pitch * i + jitter * (random.nextDouble() - 0.5));
                        scene.y[k] = (float) (r + pitch * j);
                        scene.z[k] = (float) (margin + pitch * l + jitter * (random.nextDouble() - 0.5));
                    }
                }
            }
            scene.start(BACKEND);
            for (int t = 1; t <= 5; t++) {
                scene.advanceSeconds(1);
                SphereDiagnostics state = scene.diagnostics();
                float top = 0;
                for (float y : scene.y) {
                    top = Math.max(top, y);
                }
                System.out.printf("%d s  top %.3f of %.3f  KE %.2e%n", t, top, r + pitch * (layers - 1),
                        state.kinetic());
            }
        }
    }

    /**
     * A heap on a rough floor: a stack of free spheres dropped on a bed of fixed ones, as a granular experiment glues
     * grains to its floor, so that a sphere cannot roll away without climbing. Its height and how far it spreads.
     */
    @Test
    void heapOnABed() {
        double r = 0.03;
        int bed = Integer.getInteger("rigid.bed", 20);
        int side = Integer.getInteger("rigid.side", 5);
        int layers = Integer.getInteger("rigid.layers", 14);
        double width = bed > 0 ? bed * 2 * r : 1.2;
        int free = side * side * layers;
        try (Scene scene = new Scene(bed * bed + free).uniform(r, 0.1)) {
            scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
            scene.muStatic = scene.muKinetic = Double.parseDouble(System.getProperty("rigid.mu", "0.5"));
            scene.muRolling = Double.parseDouble(System.getProperty("rigid.rolling", "0"));
            scene.sx = scene.sz = width;
            scene.sy = 1.6;
            int k = 0;
            for (int i = 0; i < bed; i++) {
                for (int l = 0; l < bed; l++, k++) {
                    scene.x[k] = (float) (r + 2 * r * i);
                    scene.z[k] = (float) (r + 2 * r * l);
                    scene.y[k] = (float) r;
                    scene.im[k] = 0;
                }
            }
            Random random = new Random(11);
            double pitch = Double.parseDouble(System.getProperty("rigid.pitch", "2.2")) * r;
            double margin = (width - pitch * (side - 1)) / 2;
            for (int j = 0; j < layers; j++) {
                for (int i = 0; i < side; i++) {
                    for (int l = 0; l < side; l++, k++) {
                        scene.x[k] = (float) (margin + pitch * i + 0.2 * r * random.nextDouble());
                        scene.z[k] = (float) (margin + pitch * l + 0.2 * r * random.nextDouble());
                        scene.y[k] = (float) ((bed > 0 ? 3.2 : 1.1) * r + pitch * j);
                    }
                }
            }
            scene.start(BACKEND);
            for (int t = 1; t <= 6; t++) {
                scene.advanceSeconds(1);
                SphereDiagnostics state = scene.diagnostics();
                double top = 0;
                double spread = 0;
                for (int s = bed * bed; s < scene.n; s++) {
                    top = Math.max(top, scene.y[s]);
                    spread = Math.max(spread, Math.hypot(scene.x[s] - width / 2, scene.z[s] - width / 2));
                }
                System.out.printf("%d s  top %.3f  spread %.3f of %.3f  KE %.2e%n", t, top, spread, width / 2,
                        state.kinetic());
            }
        }
    }

    /** Twenty spheres in a column a sphere wide, touching, as the sweep has them: how still they come to rest. */
    @Test
    void columnOf20() {
        int n = 20;
        try (Scene scene = new Scene(n).uniform(0.05, 1)) {
            scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
            scene.substeps = Integer.getInteger("rigid.substeps", 10);
            scene.iterations = Integer.getInteger("rigid.iterations", 1);
            scene.omega = 1;
            scene.averaged = false;
            scene.muStatic = scene.muKinetic = Double.parseDouble(System.getProperty("rigid.mu", "0"));
            scene.sx = scene.sz = 0.1;
            scene.sy = 2 * 0.05 * n + 0.5;
            for (int k = 0; k < n; k++) {
                scene.x[k] = scene.z[k] = 0.05f;
                scene.y[k] = (float) (0.05 + 0.1 * k);
            }
            scene.start(BACKEND);
            for (int k = 1; k <= 6; k++) {
                scene.advance(30);
                SphereDiagnostics state = scene.diagnostics();
                System.out.printf("%4.1f s  overlap %.3e  KE %.3e  vmax %.3e%n", k * 0.5, state.maxOverlap(),
                        state.kinetic(), state.maxSpeed());
            }
        }
    }

    /**
     * A sphere on a 20° slope, against a fixed sphere below it on the slope: it rests only if friction at both its
     * contacts holds it, each substep, against gravity's new pull.
     */
    @Test
    void wedgedOnASlope() {
        double r = 0.03;
        double theta = Math.toRadians(Double.parseDouble(System.getProperty("rigid.slope", "20")));
        try (Scene scene = new Scene(2).uniform(r, 0.1)) {
            scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
            scene.substeps = Integer.getInteger("rigid.substeps", 10);
            scene.iterations = Integer.getInteger("rigid.iterations", 1);
            scene.omega = 1;
            scene.averaged = false;
            scene.muStatic = scene.muKinetic = Double.parseDouble(System.getProperty("rigid.mu", "0.5"));
            scene.muRolling = Double.parseDouble(System.getProperty("rigid.rolling", "0"));
            scene.sx = scene.sy = scene.sz = 0.5;
            scene.gx = 9.81 * Math.sin(theta);
            scene.gy = -9.81 * Math.cos(theta);
            scene.x[0] = 0.2f;
            scene.x[1] = (float) (0.2 + 2 * r);
            scene.y[0] = scene.y[1] = (float) r;
            scene.z[0] = scene.z[1] = 0.25f;
            scene.im[1] = 0;
            scene.start(BACKEND);
            for (int k = 0; k <= 10; k++) {
                scene.read();
                System.out.printf("%4.2f s  spin z %.3e  v (%.3e, %.3e)  at (%.6f, %.6f)%n", k * 0.1, scene.az[0],
                        scene.u[0], scene.v[0], scene.x[0], scene.y[0]);
                scene.advance(6);
            }
        }
    }

    /**
     * Three free spheres touching on the floor and one on them: the top pushes the three apart, and they stand only
     * if friction at the floor and between the spheres holds them, and stops them rolling outward.
     */
    @Test
    void pyramidOfFour() {
        double r = 0.03;
        try (Scene scene = new Scene(4).uniform(r, 0.1)) {
            scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
            scene.substeps = Integer.getInteger("rigid.substeps", 10);
            scene.iterations = Integer.getInteger("rigid.iterations", 1);
            scene.omega = 1;
            scene.averaged = false;
            scene.muStatic = scene.muKinetic = Double.parseDouble(System.getProperty("rigid.mu", "0.5"));
            scene.muRolling = Double.parseDouble(System.getProperty("rigid.rolling", "0"));
            scene.sx = scene.sy = scene.sz = 0.5;
            double c = 0.25;
            double side = 2 * r;
            for (int k = 0; k < 3; k++) {
                double angle = 2 * Math.PI * k / 3;
                scene.x[k] = (float) (c + side / Math.sqrt(3) * Math.cos(angle));
                scene.z[k] = (float) (c + side / Math.sqrt(3) * Math.sin(angle));
                scene.y[k] = (float) r;
            }
            scene.x[3] = scene.z[3] = (float) c;
            scene.y[3] = (float) (r + side * Math.sqrt(2.0 / 3));
            scene.start(BACKEND);
            for (int k = 0; k <= 10; k++) {
                scene.read();
                double spread = Math.hypot(scene.x[0] - c, scene.z[0] - c);
                System.out.printf("%4.2f s  top %.6f  spread %.6f  spins %.2e %.2e  speed %.2e  %s%n", k * 0.3,
                        scene.y[3], spread, Math.sqrt(spin2(scene, 0)), Math.sqrt(spin2(scene, 3)),
                        Math.hypot(scene.u[0], scene.w[0]), contacts(scene));
                dump(scene);
                scene.advance(Integer.getInteger("rigid.frames", 18));
            }
        }
    }

    /** A sphere resting in the pocket of three fixed ones, set spinning: friction at three points should stop it. */
    @Test
    void oneOnThree() {
        double r = 0.03;
        try (Scene scene = new Scene(4).uniform(r, 0.1)) {
            scene.solve = SphereStep.Solve.GAUSS_SEIDEL_BY_CONTACT;
            scene.substeps = Integer.getInteger("rigid.substeps", 10);
            scene.iterations = Integer.getInteger("rigid.iterations", 1);
            scene.omega = 1;
            scene.averaged = false;
            scene.muStatic = scene.muKinetic = Double.parseDouble(System.getProperty("rigid.mu", "0.5"));
            scene.muRolling = Double.parseDouble(System.getProperty("rigid.rolling", "0"));
            scene.sx = scene.sy = scene.sz = 0.5;
            double c = 0.25;
            double side = 2 * r;
            for (int k = 0; k < 3; k++) {
                double angle = 2 * Math.PI * k / 3;
                scene.x[k] = (float) (c + side / Math.sqrt(3) * Math.cos(angle));
                scene.z[k] = (float) (c + side / Math.sqrt(3) * Math.sin(angle));
                scene.y[k] = (float) r;
                scene.im[k] = 0;
            }
            scene.x[3] = scene.z[3] = (float) c;
            scene.y[3] = (float) (r + side * Math.sqrt(2.0 / 3));
            scene.ax[3] = 1;
            scene.ay[3] = 2;
            scene.az[3] = 3;
            scene.start(BACKEND);
            for (int k = 0; k <= 10; k++) {
                scene.read();
                System.out.printf("%4.2f s  spin (%.3e, %.3e, %.3e)  v (%.3e, %.3e, %.3e)  %s%n", k * 0.1,
                        scene.ax[3], scene.ay[3], scene.az[3], scene.u[3], scene.v[3], scene.w[3], contacts(scene));
                dump(scene);
                scene.advance(6);
            }
        }
    }

    private static final double LIMIT = Double.parseDouble(System.getProperty("rigid.limit", "1.01"));

    /**
     * Every pair touching, or within 1% of the radii of touching, and every sphere on the floor: the RMS speed at
     * which their contact points slip along the surface, and at which they twist about the normal, which friction at
     * a point cannot resist. And the RMS spin, against the RMS of what is left of it once its part along each
     * sphere's contact normals is taken out.
     */
    private static String contacts(Scene s) {
        int count = 0;
        double slip2 = 0;
        double twist2 = 0;
        double spin2 = 0;
        int n = s.n;
        double[][] normals = new double[n][];
        java.util.List<double[]>[] around = new java.util.List[n];
        for (int k = 0; k < n; k++) {
            around[k] = new java.util.ArrayList<>();
        }
        for (int a = 0; a < n; a++) {
            for (int o = a + 1; o < n; o++) {
                double[] d = {s.x[a] - s.x[o], s.y[a] - s.y[o], s.z[a] - s.z[o]};
                double dist = Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                double reach = s.r[a] + s.r[o];
                if (dist < reach * LIMIT && s.im[a] + s.im[o] > 0) {
                    double[] nn = {d[0] / dist, d[1] / dist, d[2] / dist};
                    double[] pa = point(s, a, -s.r[a], nn);
                    double[] po = point(s, o, s.r[o], nn);
                    double[] rel = {pa[0] - po[0], pa[1] - po[1], pa[2] - po[2]};
                    double along = dot(rel, nn);
                    double t2 = dot(rel, rel) - along * along;
                    double tw = (s.ax[a] - s.ax[o]) * nn[0] + (s.ay[a] - s.ay[o]) * nn[1]
                            + (s.az[a] - s.az[o]) * nn[2];
                    slip2 += t2;
                    twist2 += tw * tw * s.r[a] * s.r[a];
                    count++;
                    around[a].add(nn);
                    around[o].add(nn);
                }
            }
            if (s.y[a] < s.r[a] * 1.01) {
                double[] up = {0, 1, 0};
                double[] pa = point(s, a, -s.r[a], up);
                slip2 += pa[0] * pa[0] + pa[2] * pa[2];
                twist2 += s.ay[a] * s.ay[a] * s.r[a] * s.r[a];
                count++;
                around[a].add(up);
            }
        }
        double free2 = 0;
        for (int a = 0; a < n; a++) {
            double[] w = {s.ax[a], s.ay[a], s.az[a]};
            spin2 += dot(w, w);
            // What of the spin turns no contact about its normal: the least, over the sphere's contacts.
            double least = dot(w, w);
            for (double[] nn : around[a]) {
                double p = dot(w, nn);
                least = Math.min(least, dot(w, w) - p * p);
            }
            free2 += least;
        }
        return String.format("contacts %d, slip %.2e m/s, twist %.2e m/s, spin %.2e rad/s, off-normal %.2e",
                count, Math.sqrt(slip2 / count), Math.sqrt(twist2 / count), Math.sqrt(spin2 / n),
                Math.sqrt(free2 / n));
    }

    /** The velocity of the point {@code lever · n} from sphere {@code s}'s centre. */
    private static double[] point(Scene s, int k, double lever, double[] nn) {
        double[] w = {s.ax[k], s.ay[k], s.az[k]};
        double[] arm = {lever * nn[0], lever * nn[1], lever * nn[2]};
        return new double[] {s.u[k] + w[1] * arm[2] - w[2] * arm[1], s.v[k] + w[2] * arm[0] - w[0] * arm[2],
                s.w[k] + w[0] * arm[1] - w[1] * arm[0]};
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
}

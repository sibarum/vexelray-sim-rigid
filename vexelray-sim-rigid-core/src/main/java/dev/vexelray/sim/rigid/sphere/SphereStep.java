package dev.vexelray.sim.rigid.sphere;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.pass.BufferSpec;
import dev.supirvast.vastir.pass.Buffered;
import dev.supirvast.vastir.pass.CountingSort;
import dev.supirvast.vastir.pass.Pass;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A whole {@link Spheres} step as data: which kernel, over which named buffers, with how many invocations, in order.
 * Nothing here runs anything.
 *
 * <p>A sphere carries {@code x, y, z}, the velocity {@code u, v, w}, its radius {@code r} and inverse mass
 * {@code im}; {@code cx, cy, cz} are the Jacobi solve's scratch, and {@code dx, dy, dz} the substep's constraint
 * moves, zero between substeps. {@code shown} is for a picture, {@link Spheres#SHOWN_STRIDE} floats a sphere. The
 * parameters' {@code h} is the substep, so a step advances {@code substeps · h} seconds.
 *
 * <p>With a {@link SphereGrid}, every substep sorts the spheres into its cells after they are predicted, by
 * SupirVast's {@link CountingSort}, and the solve tests only neighbouring cells. The spheres themselves stay where
 * they are: the sort lists them by cell in {@code order}, with {@code keys, ranks, counts, starts} and
 * {@code sums} its working. Without one, the solve tests every pair.
 */
public final class SphereStep implements Buffered {

    /** How a pass solves the contacts. */
    public enum Solve {
        /** Every sphere against the state before the pass ({@link Spheres#solve()}), then all moved at once. */
        JACOBI,
        /** Contact by contact, both spheres at once, in colours that each see the last: {@link Spheres#solveContacts}. */
        GAUSS_SEIDEL
    }

    public final int spheres;
    public final int substeps;
    public final int iterations;
    /** The broad phase's grid, or null for every pair. */
    public final SphereGrid grid;
    public final Solve solve;

    private final Map<String, BufferSpec> buffers = new LinkedHashMap<>();
    private final List<Pass> step;

    /** {@code substeps} substeps of one solve each: XPBD's own recommendation, and the default. Every pair. */
    public SphereStep(int spheres, int substeps) {
        this(spheres, substeps, 1);
    }

    /** {@code substeps} substeps, each of {@code iterations} solve-and-apply pairs. Every pair. */
    public SphereStep(int spheres, int substeps, int iterations) {
        this(spheres, substeps, iterations, null);
    }

    /** {@code substeps} substeps, each of {@code iterations} solve-and-apply pairs, over {@code grid} if not null. */
    public SphereStep(int spheres, int substeps, int iterations, SphereGrid grid) {
        this(spheres, substeps, iterations, grid, Solve.JACOBI);
    }

    /**
     * {@code substeps} substeps, each of {@code iterations} passes of {@code solve}, over {@code grid} if not null.
     * {@link Solve#GAUSS_SEIDEL} colours the grid's cells, so it needs one.
     */
    public SphereStep(int spheres, int substeps, int iterations, SphereGrid grid, Solve solve) {
        if (spheres < 1 || substeps < 1 || iterations < 1) {
            throw new IllegalArgumentException("a step needs spheres, substeps and iterations, got " + spheres + ", "
                    + substeps + ", " + iterations);
        }
        if (solve == Solve.GAUSS_SEIDEL && grid == null) {
            throw new IllegalArgumentException("Gauss–Seidel colours the grid's cells, and there is no grid");
        }
        this.spheres = spheres;
        this.substeps = substeps;
        this.iterations = iterations;
        this.grid = grid;
        this.solve = solve;
        for (String field : Spheres.SPHERE) {
            buffers.put(field, new BufferSpec(field, Body.F32, spheres));
        }
        buffers.put("params", new BufferSpec("params", Body.F32, Spheres.PARAM_COUNT));
        buffers.put("shown", new BufferSpec("shown", Body.F32, Spheres.SHOWN_STRIDE * spheres));

        Pass predict = new Pass("predict", Spheres.predict(), Spheres.PREDICT_BUFFERS,
                List.of(Spheres.PREDICT_NAMES), spheres);
        Pass apply = new Pass("apply", Spheres.apply(), Spheres.APPLY_BUFFERS, List.of(Spheres.APPLY_NAMES), spheres);
        Pass velocity = new Pass("velocity", Spheres.velocity(), Spheres.VELOCITY_BUFFERS,
                List.of(Spheres.VELOCITY_NAMES), spheres);
        List<Pass> sort = new ArrayList<>();
        List<Pass> pass = new ArrayList<>();
        if (grid == null) {
            pass.add(new Pass("solve", Spheres.solve(), Spheres.SOLVE_BUFFERS, List.of(Spheres.SOLVE_NAMES), spheres));
            pass.add(apply);
        } else {
            int length = CountingSort.length(grid.cells());
            for (String name : List.of("keys", "ranks", "order")) {
                buffers.put(name, new BufferSpec(name, Body.I32, spheres));
            }
            buffers.put("counts", new BufferSpec("counts", Body.I32, length));
            buffers.put("starts", new BufferSpec("starts", Body.I32, length));
            buffers.put("sums", new BufferSpec("sums", Body.I32, CountingSort.blocks(length)));
            sort.add(new Pass("bin", Spheres.bin(grid), Spheres.BIN_BUFFERS, List.of(Spheres.BIN_NAMES), spheres));
            sort.addAll(CountingSort.scan(length, "counts", "starts", "sums"));
            sort.add(new Pass("order", CountingSort.order(), CountingSort.ORDER_BUFFERS,
                    List.of("keys", "ranks", "starts", "order"), spheres));
            if (solve == Solve.JACOBI) {
                pass.add(new Pass("solve", Spheres.solve(grid), Spheres.GRID_SOLVE_BUFFERS,
                        List.of(Spheres.GRID_SOLVE_NAMES), spheres));
                pass.add(apply);
            } else {
                for (int colour = 0; colour < Spheres.COLOURS; colour++) {
                    int cells = Spheres.cellsOf(grid, colour);
                    if (cells > 0) {
                        pass.add(new Pass("colour " + colour, Spheres.solveContacts(grid, colour),
                                Spheres.CONTACT_BUFFERS, List.of(Spheres.CONTACT_NAMES), cells));
                    }
                }
                pass.add(new Pass("walls", Spheres.walls(), Spheres.WALLS_BUFFERS, List.of(Spheres.WALLS_NAMES),
                        spheres));
            }
        }
        List<Pass> passes = new ArrayList<>();
        for (int sub = 0; sub < substeps; sub++) {
            passes.add(predict);
            passes.addAll(sort);
            for (int it = 0; it < iterations; it++) {
                passes.addAll(pass);
            }
            passes.add(velocity);
        }
        passes.add(new Pass("show", Spheres.show(), Spheres.SHOW_BUFFERS, List.of(Spheres.SHOW_NAMES), spheres));
        step = List.copyOf(passes);
    }

    @Override
    public Map<String, BufferSpec> buffers() {
        return buffers;
    }

    /**
     * One step: per substep, predict, then the sort if there is a grid, then {@link #iterations} passes of the
     * solve — solve and apply for Jacobi, the colours in turn and then the walls for Gauss–Seidel — then velocity;
     * and once at the end, {@link Spheres#show show}, for a picture.
     */
    public List<Pass> step() {
        return step;
    }
}

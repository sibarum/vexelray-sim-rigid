package dev.vexelray.sim.rigid.sphere;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.Function;
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
 *
 * <p>Some buffers hold constants the step needs and cannot compute: {@link #constants()} has them, to write once
 * after the buffers are cleared.
 */
public final class SphereStep implements Buffered {

    /** How a pass solves the contacts. */
    public enum Solve {
        /** Every sphere against the state before the pass ({@link Spheres#solve()}), then all moved at once. */
        JACOBI,
        /** Contact by contact, in 27 colours of the grid's cells, one serial invocation a cell. */
        GAUSS_SEIDEL_BY_CELL,
        /** Contact by contact from a list, in rounds of contacts sharing no sphere: {@link Spheres#contactRound}. */
        GAUSS_SEIDEL_BY_CONTACT
    }

    /**
     * The rounds a pass of {@link Solve#GAUSS_SEIDEL_BY_CONTACT} takes, unless told otherwise. Measured on a pile of
     * 4096: 24 left 16 contacts a step unsolved, 32 none, and 48 the same answer as 32 for more time.
     */
    public static final int ROUNDS = 32;
    /** The rounds a batch adds when a pass run until done ({@link #untilDone}) is still open after its last report. */
    public static final int BATCH = 8;
    /**
     * The positions a pass run until done cycles through after its first round, so that it can run any number of
     * rounds from a fixed set of them: a multiple of three, for the claim arrays' turns, of two, for the open
     * words', and of {@link #BATCH}, so batches line up with it. Also the most rounds an opening batch runs.
     */
    public static final int CYCLE = 48;

    /** The places in the contact list per sphere, unless told otherwise: twelve contacts each, counted once. */
    public static final int CONTACTS_PER_SPHERE = 6;

    public final int spheres;
    public final int substeps;
    public final int iterations;
    /** The broad phase's grid, or null for every pair. */
    public final SphereGrid grid;
    public final Solve solve;
    /** For {@link Solve#GAUSS_SEIDEL_BY_CONTACT}: the rounds a pass takes, and the places in the list. */
    public final int rounds;
    public final int capacity;

    private final Map<String, BufferSpec> buffers = new LinkedHashMap<>();
    private final Map<String, int[]> constants = new LinkedHashMap<>();
    private final List<Pass> step;

    // What the segments of a step run until done ({@link #opening}) are made of; null unless {@link #untilDone}.
    private final Pass predict;
    private final Pass velocity;
    private final Pass walls;
    private final Pass show;
    private final List<Pass> prepare = new ArrayList<>();
    /** Per pass of a substep, the rounds at each of the cycle's positions, and the report after each. */
    private final List<List<Pass>> cycle = new ArrayList<>();
    private final List<List<Pass>> reports = new ArrayList<>();
    /** Segments made so far, by what they are: a runner records each once and keeps it by identity. */
    private final Map<String, List<Pass>> segments = new java.util.HashMap<>();

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

    /** As below, with {@link #ROUNDS} rounds and {@link #CONTACTS_PER_SPHERE} places a sphere. */
    public SphereStep(int spheres, int substeps, int iterations, SphereGrid grid, Solve solve) {
        this(spheres, substeps, iterations, grid, solve, ROUNDS, CONTACTS_PER_SPHERE);
    }

    /**
     * {@code substeps} substeps, each of {@code iterations} passes of {@code solve}, over {@code grid} if not null.
     * Both Gauss–Seidels need the grid. {@code rounds} and {@code contactsPerSphere} size
     * {@link Solve#GAUSS_SEIDEL_BY_CONTACT}'s passes and list, and mean nothing to the others.
     */
    public SphereStep(int spheres, int substeps, int iterations, SphereGrid grid, Solve solve, int rounds,
                      int contactsPerSphere) {
        if (spheres < 1 || substeps < 1 || iterations < 1) {
            throw new IllegalArgumentException("a step needs spheres, substeps and iterations, got " + spheres + ", "
                    + substeps + ", " + iterations);
        }
        if (solve != Solve.JACOBI && grid == null) {
            throw new IllegalArgumentException("Gauss–Seidel works over the grid's cells, and there is no grid");
        }
        if (rounds < 1 || contactsPerSphere < 1) {
            throw new IllegalArgumentException("at least a round and a place a sphere, got " + rounds + " and "
                    + contactsPerSphere);
        }
        if (solve == Solve.GAUSS_SEIDEL_BY_CONTACT && spheres > Spheres.MOST_LISTED_SPHERES) {
            throw new IllegalArgumentException("a contact list orders at most " + Spheres.MOST_LISTED_SPHERES
                    + " spheres, got " + spheres);
        }
        this.spheres = spheres;
        this.substeps = substeps;
        this.iterations = iterations;
        this.grid = grid;
        this.solve = solve;
        this.rounds = rounds;
        this.capacity = spheres * contactsPerSphere;
        for (String field : Spheres.SPHERE) {
            buffers.put(field, new BufferSpec(field, Body.F32, spheres));
        }
        buffers.put("params", new BufferSpec("params", Body.F32, Spheres.PARAM_COUNT));
        buffers.put("shown", new BufferSpec("shown", Body.F32, Spheres.SHOWN_STRIDE * spheres));

        predict = new Pass("predict", Spheres.predict(), Spheres.PREDICT_BUFFERS,
                List.of(Spheres.PREDICT_NAMES), spheres);
        Pass apply = new Pass("apply", Spheres.apply(), Spheres.APPLY_BUFFERS, List.of(Spheres.APPLY_NAMES), spheres);
        velocity = new Pass("velocity", Spheres.velocity(), Spheres.VELOCITY_BUFFERS,
                List.of(Spheres.VELOCITY_NAMES), spheres);
        walls = new Pass("walls", Spheres.walls(), Spheres.WALLS_BUFFERS, List.of(Spheres.WALLS_NAMES), spheres);
        // Before every iteration of a substep, once: the sort, and what is made from it: {@link #prepare}.
        // An iteration's passes, which may differ by iteration.
        List<List<Pass>> iteration = new ArrayList<>();
        if (grid == null) {
            Pass solvePass = new Pass("solve", Spheres.solve(), Spheres.SOLVE_BUFFERS,
                    List.of(Spheres.SOLVE_NAMES), spheres);
            for (int it = 0; it < iterations; it++) {
                iteration.add(List.of(solvePass, apply));
            }
        } else {
            int length = CountingSort.length(grid.cells());
            for (String name : List.of("keys", "ranks", "order")) {
                buffers.put(name, new BufferSpec(name, Body.I32, spheres));
            }
            buffers.put("counts", new BufferSpec("counts", Body.I32, length));
            buffers.put("starts", new BufferSpec("starts", Body.I32, length));
            buffers.put("sums", new BufferSpec("sums", Body.I32, CountingSort.blocks(length)));
            prepare.add(new Pass("bin", Spheres.bin(grid), Spheres.BIN_BUFFERS, List.of(Spheres.BIN_NAMES), spheres));
            prepare.addAll(CountingSort.scan(length, "counts", "starts", "sums"));
            prepare.add(new Pass("order", CountingSort.order(), CountingSort.ORDER_BUFFERS,
                    List.of("keys", "ranks", "starts", "order"), spheres));
            switch (solve) {
                case JACOBI -> {
                    Pass solvePass = new Pass("solve", Spheres.solve(grid), Spheres.GRID_SOLVE_BUFFERS,
                            List.of(Spheres.GRID_SOLVE_NAMES), spheres);
                    for (int it = 0; it < iterations; it++) {
                        iteration.add(List.of(solvePass, apply));
                    }
                }
                case GAUSS_SEIDEL_BY_CELL -> {
                    List<Pass> colours = new ArrayList<>();
                    for (int colour = 0; colour < Spheres.COLOURS; colour++) {
                        int cells = Spheres.cellsOf(grid, colour);
                        if (cells > 0) {
                            colours.add(new Pass("colour " + colour, Spheres.solveContacts(grid, colour),
                                    Spheres.CONTACT_BUFFERS, List.of(Spheres.CONTACT_NAMES), cells));
                        }
                    }
                    colours.add(walls);
                    for (int it = 0; it < iterations; it++) {
                        iteration.add(colours);
                    }
                }
                case GAUSS_SEIDEL_BY_CONTACT -> byContact(prepare, iteration, walls);
            }
        }
        List<Pass> passes = new ArrayList<>();
        for (int sub = 0; sub < substeps; sub++) {
            passes.add(predict);
            passes.addAll(prepare);
            for (List<Pass> it : iteration) {
                passes.addAll(it);
            }
            passes.add(velocity);
        }
        show = new Pass("show", Spheres.show(), Spheres.SHOW_BUFFERS, List.of(Spheres.SHOW_NAMES), spheres);
        passes.add(show);
        step = List.copyOf(passes);
    }

    /**
     * The list made once a substep, after the sort; then per iteration {@link #rounds} rounds and one more to check
     * the last, and the walls. The dispatches of a substep take {@link Spheres#CLAIMS} in turn: the {@code g}-th
     * claims into array {@code g mod 3}, checks the one before it and clears the one after.
     */
    private void byContact(List<Pass> prepare, List<List<Pass>> iteration, Pass walls) {
        for (String claims : Spheres.CLAIMS) {
            buffers.put(claims, new BufferSpec(claims, Body.I32, spheres));
        }
        for (String name : List.of("contactA", "contactB", "contactDone")) {
            buffers.put(name, new BufferSpec(name, Body.I32, capacity));
        }
        buffers.put("contactCount", new BufferSpec("contactCount", Body.I32, Spheres.CONTACT_COUNT_WORDS));
        prepare.add(new Pass("clearContacts", Spheres.clearContacts(), Spheres.CLEAR_BUFFERS,
                List.of(Spheres.CLEAR_NAMES), spheres));
        prepare.add(new Pass("listContacts", Spheres.listContacts(grid, capacity), Spheres.LIST_BUFFERS,
                List.of(Spheres.LIST_NAMES), spheres));
        buffers.put("open", new BufferSpec("open", Body.I32, Spheres.OPEN_WORDS));
        buffers.put("readout", BufferSpec.readout("readout", Body.I32, Spheres.READOUT_WORDS));
        Function round = Spheres.contactRound(spheres, capacity);
        int g = 0;
        for (int it = 0; it < iterations; it++) {
            List<Pass> passes = new ArrayList<>();
            for (int k = 0; k <= rounds; k++, g++) {
                String kind = "round" + g;
                buffers.put(kind, new BufferSpec(kind, Body.I32, Spheres.ROUND_WORDS));
                constants.put(kind, new int[] {k == 0 ? 0 : 1, k == rounds ? 0 : 1, it + 1, g, g % 2, g - 1});
                passes.add(new Pass("round " + k, round, Spheres.ROUND_BUFFERS, roundNames(g, kind), capacity));
            }
            passes.add(walls);
            iteration.add(passes);
        }
        cycle(round);
    }

    /**
     * The rounds a pass run until done takes ({@link #opening}): per pass, one at each position of a cycle,
     * {@code 0} to {@link #CYCLE}, each with a report after it. Position 0 starts the pass and does not check; past
     * {@link #CYCLE}, a pass goes on at 1. A round at position {@code p} claims into array {@code p mod 3} and marks
     * word {@code p mod 2}, so the turns run on unbroken around the cycle, because the cycle's length is a multiple
     * of both; and it claims with dispatch {@code p mod CYCLE}, so the round at 1 checks against the same dispatch
     * whether it follows position 0 or position {@link #CYCLE}.
     */
    private void cycle(Function round) {
        Pass report = null;
        for (int it = 0; it < iterations; it++) {
            List<Pass> rounds = new ArrayList<>();
            List<Pass> after = new ArrayList<>();
            for (int p = 0; p <= CYCLE; p++) {
                String kind = "cycle" + it + "." + p;
                int dispatch = it * CYCLE + p % CYCLE;
                int checks = it * CYCLE + Math.max(p - 1, 0) % CYCLE;
                buffers.put(kind, new BufferSpec(kind, Body.I32, Spheres.ROUND_WORDS));
                constants.put(kind, new int[] {p == 0 ? 0 : 1, 1, it + 1, dispatch, p % 2, checks});
                rounds.add(new Pass("round " + p, round, Spheres.ROUND_BUFFERS, roundNames(p, kind), capacity));
                if (report == null) {
                    report = new Pass("report", Spheres.report(), Spheres.REPORT_BUFFERS,
                            List.of(Spheres.REPORT_NAMES), 1);
                }
                after.add(new Pass("report", report.kernel(), Spheres.REPORT_BUFFERS,
                        List.of("open", "contactCount", kind, "readout"), 1));
            }
            cycle.add(List.copyOf(rounds));
            reports.add(List.copyOf(after));
        }
    }

    /** A round's buffers, for the {@code g}-th dispatch of a pass, whose constants are {@code kind}. */
    private static List<String> roundNames(int g, String kind) {
        List<String> names = new ArrayList<>(List.of(Spheres.ROUND_NAMES));
        names.set(13, Spheres.CLAIMS[(g + 2) % 3]);
        names.set(14, Spheres.CLAIMS[g % 3]);
        names.set(15, Spheres.CLAIMS[(g + 1) % 3]);
        names.set(16, kind);
        return names;
    }

    /**
     * Whether a step is run until every contact is solved ({@link #opening}, {@link #more} and {@link #closing}),
     * rather than as the one list {@link #step()}: Gauss–Seidel over a contact list.
     */
    public boolean untilDone() {
        return solve == Solve.GAUSS_SEIDEL_BY_CONTACT;
    }

    /** Where in the cycle a pass's {@code k}-th round runs: {@code 0} first, then {@code 1} to {@link #CYCLE} over. */
    public static int position(int k) {
        return k == 0 ? 0 : 1 + (k - 1) % CYCLE;
    }

    /**
     * The segment that opens pass {@code iteration} of substep {@code substep}, ending in its first {@code rounds}
     * rounds and a report. Before them, what comes between the last segment and this one: for the first pass of a
     * substep, the substep before it closed (walls, velocity) unless this is the first, then predict and the sort and
     * the list; for a later pass, the walls of the one before.
     */
    public List<Pass> opening(int substep, int iteration, int rounds) {
        requireUntilDone();
        String key = "open " + Math.min(substep, 1) + " " + iteration + " " + rounds;
        return segments.computeIfAbsent(key, k -> {
            List<Pass> passes = new ArrayList<>();
            if (iteration > 0) {
                passes.add(walls);
            } else {
                if (substep > 0) {
                    passes.add(walls);
                    passes.add(velocity);
                }
                passes.add(predict);
                passes.addAll(prepare);
            }
            addRounds(passes, iteration, 0, rounds);
            return List.copyOf(passes);
        });
    }

    /** {@code rounds} more rounds of pass {@code iteration}, from its {@code from}-th, and a report. */
    public List<Pass> more(int iteration, int from, int rounds) {
        requireUntilDone();
        String key = "more " + iteration + " " + position(from) + " " + rounds;
        return segments.computeIfAbsent(key, k -> {
            List<Pass> passes = new ArrayList<>();
            addRounds(passes, iteration, from, rounds);
            return List.copyOf(passes);
        });
    }

    /** What ends a step once its last pass is done: the walls, velocity, and the picture's buffer. */
    public List<Pass> closing() {
        requireUntilDone();
        return segments.computeIfAbsent("close", k -> List.of(walls, velocity, show));
    }

    private void addRounds(List<Pass> passes, int iteration, int from, int rounds) {
        if (rounds < 1) {
            throw new IllegalArgumentException("a batch needs a round, got " + rounds);
        }
        for (int k = from; k < from + rounds; k++) {
            passes.add(cycle.get(iteration).get(position(k)));
        }
        passes.add(reports.get(iteration).get(position(from + rounds - 1)));
    }

    private void requireUntilDone() {
        if (!untilDone()) {
            throw new IllegalStateException(solve + " is run as one list, step()");
        }
    }

    @Override
    public Map<String, BufferSpec> buffers() {
        return buffers;
    }

    /** What to write into which buffers once, after they are cleared and before the first step. */
    public Map<String, int[]> constants() {
        return constants;
    }

    /**
     * One step: per substep, predict, then the sort if there is a grid, and the contact list if the solve wants
     * one; then {@link #iterations} passes of the solve — solve and apply for Jacobi, the colours or the rounds and
     * then the walls for Gauss–Seidel — then velocity; and once at the end, {@link Spheres#show show}, for a
     * picture.
     */
    public List<Pass> step() {
        return step;
    }
}

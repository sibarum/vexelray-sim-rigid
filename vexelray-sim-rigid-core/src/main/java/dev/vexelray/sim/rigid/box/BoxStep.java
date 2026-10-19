package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.Function;
import dev.supirvast.vastir.pass.BufferSpec;
import dev.supirvast.vastir.pass.Buffered;
import dev.supirvast.vastir.pass.CountingSort;
import dev.supirvast.vastir.pass.Pass;
import dev.vexelray.sim.rigid.sphere.SphereGrid;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A whole {@link Boxes} step as data, as {@code SphereStep} is for spheres: which kernel, over which named buffers,
 * with how many invocations, in order. Nothing here runs anything; {@link BoxStepper} runs it.
 *
 * <p>A step begins with {@link #collide}:
 *
 * <ol>
 *   <li>every slot's pair, manifold and impulses set aside ({@link BoxPairs#keep});</li>
 *   <li>the boxes sorted into the grid by their centres ({@link BoxBroad#bin} and SupirVast's counting sort), and each
 *       box's partners listed into its slots ({@link BoxBroad#list});</li>
 *   <li>every listed pair tested ({@link BoxContact#collide});</li>
 *   <li>each new point given the impulses of the point it was ({@link BoxPairs#carry});</li>
 *   <li>what the host needs to make the rounds written to {@code contacts} ({@link BoxBroad#counts}).</li>
 * </ol>
 *
 * Then {@link #step(int)}, for as many rounds as the stepper sorted the touching pairs into: each substep the soft
 * step — {@link Boxes#velocities}, the warm start, {@link #sweeps} sweeps of the solve, {@link Boxes#positions}, and
 * as many of the relax — each sweep every round of pairs ({@link BoxPairs#solve}) and then the walls
 * ({@link Boxes#walls}). Once at the end, {@link Boxes#show}.
 *
 * <p>{@code lambda} and {@code rub} hold each corner's normal and tangential impulse against each wall, and
 * {@code impulses} each slot's points'. {@code rounds} holds the rounds, {@link #width} places each, −1 where a place
 * is empty, written once a step; {@code which0} to {@code which31}, each one word, written once, hold where each round
 * begins. The parameters' {@code h} is the substep, so a step advances {@code substeps · h} seconds.
 */
public final class BoxStep implements Buffered {

    /** The most rounds a step's touching pairs can be sorted into; past them a pair waits for the next step. */
    public static final int MOST_ROUNDS = 32;

    public final int boxes;
    public final int substeps;
    /** Sweeps a substep makes of the solve, and as many of the relax. */
    public final int sweeps;
    /** Whether every other sweep runs backwards: the walls first, the rounds last to first, the points reversed. */
    public final boolean symmetric;
    /** The broad phase's grid; null for a lone box, which has no pairs. */
    public final SphereGrid grid;
    /** Slots in the pair list: {@link BoxBroad#SLOTS} for each box, or none for a lone box. */
    public final int slots;
    /** Places in a round: no more pairs than this can share no box. */
    public final int width;

    private final Map<String, BufferSpec> buffers = new LinkedHashMap<>();
    private final Map<String, int[]> constants = new LinkedHashMap<>();
    private final List<Pass> collide;
    private final Map<Integer, List<Pass>> steps = new HashMap<>();

    private final Pass velocities;
    private final Pass positions;
    private final Pass show;
    /** The walls' sweeps and the pairs' kernels, by kind and direction. */
    private final Map<String, Pass> walls = new HashMap<>();
    private final Map<String, Function> solve = new HashMap<>();

    /** A lone box, or boxes that never meet each other: {@code substeps} substeps of {@link Boxes#WALL_PASSES}. */
    public BoxStep(int boxes, int substeps) {
        this(boxes, substeps, Boxes.WALL_PASSES, false, null);
    }

    /**
     * {@code substeps} substeps of {@code sweeps} sweeps each of the solve and the relax, over {@code grid}
     * ({@link BoxBroad#grid}), which more than one box needs.
     */
    public BoxStep(int boxes, int substeps, int sweeps, boolean symmetric, SphereGrid grid) {
        if (boxes < 1 || substeps < 1 || sweeps < 1) {
            throw new IllegalArgumentException("a step needs boxes, substeps and sweeps, got " + boxes + ", "
                    + substeps + " and " + sweeps);
        }
        this.boxes = boxes;
        this.substeps = substeps;
        this.sweeps = sweeps;
        this.symmetric = symmetric;
        this.grid = boxes > 1 ? grid : null;
        this.slots = boxes > 1 && grid != null ? Math.multiplyExact(boxes, BoxBroad.SLOTS) : 0;
        this.width = Math.max(1, boxes / 2);
        for (String field : Boxes.BOX) {
            buffers.put(field, new BufferSpec(field, Body.F32, boxes));
        }
        int[] identity = new int[boxes];
        java.util.Arrays.fill(identity, Float.floatToRawIntBits(1));
        constants.put("qw", identity);
        buffers.put("params", new BufferSpec("params", Body.F32, Boxes.PARAM_COUNT));
        buffers.put("lambda", new BufferSpec("lambda", Body.F32, Boxes.LAMBDA_WORDS * boxes));
        buffers.put("rub", new BufferSpec("rub", Body.F32, Boxes.RUB_WORDS * boxes));
        buffers.put("shown", new BufferSpec("shown", Body.F32, Boxes.SHOWN_STRIDE * boxes));

        velocities = new Pass("velocities", Boxes.velocities(), Boxes.VELOCITIES_BUFFERS,
                List.of(Boxes.VELOCITIES_NAMES), boxes);
        positions = new Pass("positions", Boxes.positions(), Boxes.POSITIONS_BUFFERS, List.of(Boxes.POSITIONS_NAMES),
                boxes);
        show = new Pass("show", Boxes.show(), Boxes.SHOW_BUFFERS, List.of(Boxes.SHOW_NAMES), boxes);
        for (Boxes.Sweep kind : Boxes.Sweep.values()) {
            for (boolean reverse : new boolean[] {false, true}) {
                walls.put(key(kind, reverse), new Pass("walls " + kind, Boxes.walls(kind, reverse),
                        Boxes.WALLS_BUFFERS, List.of(Boxes.WALLS_NAMES), boxes));
            }
        }

        if (slots > 0) {
            int length = CountingSort.length(grid.cells());
            for (String name : List.of("keys", "ranks", "order")) {
                buffers.put(name, new BufferSpec(name, Body.I32, boxes));
            }
            buffers.put("counts", new BufferSpec("counts", Body.I32, length));
            buffers.put("starts", new BufferSpec("starts", Body.I32, length));
            buffers.put("sums", new BufferSpec("sums", Body.I32, CountingSort.blocks(length)));
            buffers.put("pairs", new BufferSpec("pairs", Body.I32, 2 * slots));
            buffers.put("pairsWas", new BufferSpec("pairsWas", Body.I32, 2 * slots));
            buffers.put("manifold", new BufferSpec("manifold", Body.F32, BoxContact.MANIFOLD_WORDS * slots));
            buffers.put("manifoldWas", new BufferSpec("manifoldWas", Body.F32, BoxContact.MANIFOLD_WORDS * slots));
            buffers.put("candidates", new BufferSpec("candidates", Body.F32, BoxContact.SCRATCH_WORDS * slots));
            buffers.put("impulses", new BufferSpec("impulses", Body.F32, BoxPairs.IMPULSE_WORDS * slots));
            buffers.put("impulsesWas", new BufferSpec("impulsesWas", Body.F32, BoxPairs.IMPULSE_WORDS * slots));
            buffers.put("contacts", new BufferSpec("contacts", Body.I32, 2 * slots + boxes));
            buffers.put("rounds", new BufferSpec("rounds", Body.I32, MOST_ROUNDS * width));
            for (int r = 0; r < MOST_ROUNDS; r++) {
                buffers.put(which(r), new BufferSpec(which(r), Body.I32, 1));
                constants.put(which(r), new int[] {r * width});
            }
            List<Pass> passes = new ArrayList<>();
            passes.add(new Pass("keep", BoxPairs.keep(), BoxPairs.CARRY_BUFFERS, List.of(BoxPairs.CARRY_NAMES),
                    slots));
            passes.add(new Pass("bin", BoxBroad.bin(grid), BoxBroad.BIN_BUFFERS, List.of(BoxBroad.BIN_NAMES), boxes));
            passes.addAll(CountingSort.scan(length, "counts", "starts", "sums"));
            passes.add(new Pass("order", CountingSort.order(), CountingSort.ORDER_BUFFERS,
                    List.of("keys", "ranks", "starts", "order"), boxes));
            passes.add(new Pass("list", BoxBroad.list(grid, slots), BoxBroad.LIST_BUFFERS,
                    List.of(BoxBroad.LIST_NAMES), boxes));
            passes.add(new Pass("collide", BoxContact.collide(), BoxContact.BUFFERS, List.of(BoxContact.NAMES),
                    slots));
            passes.add(new Pass("carry", BoxPairs.carry(), BoxPairs.CARRY_BUFFERS, List.of(BoxPairs.CARRY_NAMES),
                    slots));
            passes.add(new Pass("counts", BoxBroad.counts(slots), BoxBroad.COUNTS_BUFFERS,
                    List.of(BoxBroad.COUNTS_NAMES), slots));
            collide = List.copyOf(passes);
            for (Boxes.Sweep kind : Boxes.Sweep.values()) {
                for (boolean reverse : new boolean[] {false, true}) {
                    solve.put(key(kind, reverse), BoxPairs.solve(kind, reverse));
                }
            }
        } else {
            collide = List.of();
        }
    }

    private static String key(Boxes.Sweep kind, boolean reverse) {
        return kind + (reverse ? " back" : "");
    }

    /** The name of the one-word buffer that holds where round {@code r} begins in {@code rounds}. */
    public static String which(int r) {
        return "which" + r;
    }

    @Override
    public Map<String, BufferSpec> buffers() {
        return buffers;
    }

    /**
     * What to write into which buffers once, after they are cleared and before the first step: every orientation as
     * the identity, which a host that wants others writes after, and where each round begins.
     */
    public Map<String, int[]> constants() {
        return constants;
    }

    /** The passes that begin a step: the broad phase and the collide; none where no box can meet another. */
    public List<Pass> collide() {
        return collide;
    }

    /** A step's substeps, with the touching pairs in {@code rounds} rounds; one list for each count, kept. */
    public List<Pass> step(int rounds) {
        if (rounds < 0 || rounds > MOST_ROUNDS || (slots == 0 && rounds > 0)) {
            throw new IllegalArgumentException("0 to " + (slots == 0 ? 0 : MOST_ROUNDS) + " rounds, got " + rounds);
        }
        return steps.computeIfAbsent(rounds, this::build);
    }

    private List<Pass> build(int rounds) {
        List<Pass> passes = new ArrayList<>();
        for (int sub = 0; sub < substeps; sub++) {
            passes.add(velocities);
            sweep(passes, Boxes.Sweep.WARM, false, rounds);
            for (int sweep = 0; sweep < sweeps; sweep++) {
                sweep(passes, Boxes.Sweep.SOLVE, symmetric && sweep % 2 == 1, rounds);
            }
            passes.add(positions);
            for (int sweep = 0; sweep < sweeps; sweep++) {
                sweep(passes, Boxes.Sweep.RELAX, symmetric && sweep % 2 == 1, rounds);
            }
        }
        passes.add(show);
        return List.copyOf(passes);
    }

    /** One sweep of {@code kind}: every round of pairs, then the walls; or with {@code reverse}, backwards. */
    private void sweep(List<Pass> passes, Boxes.Sweep kind, boolean reverse, int rounds) {
        String key = key(kind, reverse);
        if (reverse) {
            passes.add(walls.get(key));
        }
        for (int order = 0; order < rounds; order++) {
            int r = reverse ? rounds - 1 - order : order;
            List<String> names = new ArrayList<>(List.of(BoxPairs.NAMES));
            names.set(names.indexOf("which"), which(r));
            passes.add(new Pass(kind + " " + r, solve.get(key), BoxPairs.BUFFERS, names, width));
        }
        if (!reverse) {
            passes.add(walls.get(key));
        }
    }
}

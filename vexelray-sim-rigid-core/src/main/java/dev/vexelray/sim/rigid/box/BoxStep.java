package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.core.Function;
import dev.supirvast.vastir.pass.BufferSpec;
import dev.supirvast.vastir.pass.Buffered;
import dev.supirvast.vastir.pass.Pass;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A whole {@link Boxes} step as data, as {@code SphereStep} is for spheres: which kernel, over which named buffers,
 * with how many invocations, in order. Nothing here runs anything; {@link BoxStepper} runs it.
 *
 * <p>A step begins with {@link #collide}: every pair's manifold and impulses set aside ({@link BoxPairs#keep}), every
 * pair of boxes tested ({@link BoxContact#collide}), and each new point given the impulses of the point it was
 * ({@link BoxPairs#carry}). Every pair of the boxes is in the list, {@code n (n − 1) / 2} of them: the broad phase is
 * stage 6. Then {@link #step(int)}, for as many rounds as the stepper sorted the touching pairs into: each substep the
 * soft step — {@link Boxes#velocities}, the warm start, {@link #sweeps} sweeps of the solve, {@link Boxes#positions},
 * and as many of the relax — each sweep every round of pairs ({@link BoxPairs#solve}) and then the walls
 * ({@link Boxes#walls}). Once at the end, {@link Boxes#show}.
 *
 * <p>{@code lambda} and {@code rub} hold each corner's normal and tangential impulse against each wall, and
 * {@code impulses} each pair's points'. {@code round0} to {@code round31} hold the rounds, {@link #width} places each,
 * −1 where a place is empty. The parameters' {@code h} is the substep, so a step advances {@code substeps · h} seconds.
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
    /** Pairs in the list: every pair of boxes. */
    public final int pairs;
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

    /** {@code substeps} substeps of {@link Boxes#WALL_PASSES} sweeps. */
    public BoxStep(int boxes, int substeps) {
        this(boxes, substeps, Boxes.WALL_PASSES);
    }

    /** {@code substeps} substeps of {@code sweeps} sweeps, every one forwards. */
    public BoxStep(int boxes, int substeps, int sweeps) {
        this(boxes, substeps, sweeps, false);
    }

    public BoxStep(int boxes, int substeps, int sweeps, boolean symmetric) {
        if (boxes < 1 || substeps < 1 || sweeps < 1) {
            throw new IllegalArgumentException("a step needs boxes, substeps and sweeps, got " + boxes + ", "
                    + substeps + " and " + sweeps);
        }
        this.boxes = boxes;
        this.substeps = substeps;
        this.sweeps = sweeps;
        this.symmetric = symmetric;
        this.pairs = boxes * (boxes - 1) / 2;
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

        if (pairs > 0) {
            buffers.put("pairs", new BufferSpec("pairs", Body.I32, 2 * pairs));
            buffers.put("manifold", new BufferSpec("manifold", Body.F32, BoxContact.MANIFOLD_WORDS * pairs));
            buffers.put("candidates", new BufferSpec("candidates", Body.F32, BoxContact.SCRATCH_WORDS * pairs));
            buffers.put("impulses", new BufferSpec("impulses", Body.F32, BoxPairs.IMPULSE_WORDS * pairs));
            buffers.put("manifoldWas", new BufferSpec("manifoldWas", Body.F32, BoxContact.MANIFOLD_WORDS * pairs));
            buffers.put("impulsesWas", new BufferSpec("impulsesWas", Body.F32, BoxPairs.IMPULSE_WORDS * pairs));
            for (int r = 0; r < MOST_ROUNDS; r++) {
                buffers.put(round(r), new BufferSpec(round(r), Body.I32, width));
            }
            int[] list = new int[2 * pairs];
            for (int a = 0, p = 0; a < boxes; a++) {
                for (int b = a + 1; b < boxes; b++, p++) {
                    list[2 * p] = a;
                    list[2 * p + 1] = b;
                }
            }
            constants.put("pairs", list);
            collide = List.of(
                    new Pass("keep", BoxPairs.keep(), BoxPairs.CARRY_BUFFERS, List.of(BoxPairs.CARRY_NAMES), pairs),
                    new Pass("collide", BoxContact.collide(), BoxContact.BUFFERS, List.of(BoxContact.NAMES), pairs),
                    new Pass("carry", BoxPairs.carry(), BoxPairs.CARRY_BUFFERS, List.of(BoxPairs.CARRY_NAMES),
                            pairs));
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

    /** The name of round {@code r}'s buffer. */
    public static String round(int r) {
        return "round" + r;
    }

    @Override
    public Map<String, BufferSpec> buffers() {
        return buffers;
    }

    /**
     * What to write into which buffers once, after they are cleared and before the first step: every orientation as
     * the identity, which a host that wants others writes after, and the list of pairs.
     */
    public Map<String, int[]> constants() {
        return constants;
    }

    /** The passes that begin a step, over every pair; none for a lone box. */
    public List<Pass> collide() {
        return collide;
    }

    /** A step's substeps, with the touching pairs in {@code rounds} rounds; one list for each count, kept. */
    public List<Pass> step(int rounds) {
        if (rounds < 0 || rounds > MOST_ROUNDS || (pairs == 0 && rounds > 0)) {
            throw new IllegalArgumentException("0 to " + (pairs == 0 ? 0 : MOST_ROUNDS) + " rounds, got " + rounds);
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
            names.set(names.indexOf("round"), round(r));
            passes.add(new Pass(kind + " " + r, solve.get(key), BoxPairs.BUFFERS, names, width));
        }
        if (!reverse) {
            passes.add(walls.get(key));
        }
    }
}

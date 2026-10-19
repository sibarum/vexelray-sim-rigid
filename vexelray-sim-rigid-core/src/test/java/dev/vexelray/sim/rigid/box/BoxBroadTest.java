package dev.vexelray.sim.rigid.box;

import dev.supirvast.vastir.build.Body;
import dev.supirvast.vastir.pass.BufferSpec;
import dev.supirvast.vastir.pass.Buffered;
import dev.supirvast.vastir.pass.Pass;
import dev.supirvast.vastir.tools.Accelerator;
import dev.supirvast.vastir.tools.PassRunner;
import dev.vexelray.sim.rigid.box.BoxScene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.Map;
import java.util.Random;
import java.util.TreeMap;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * The broad phase finds what testing every pair finds: the same pairs touching, each with the same points.
 */
class BoxBroadTest {

    private static final String[] FIELDS = {"x", "y", "z", "qx", "qy", "qz", "qw", "hx", "hy", "hz", "shape"};
    private static final double MARGIN = 0.005;

    /**
     * 150 boxes, half-extents from 2 to 8 cm, turned at random and packed into a metre cube so that many touch: the
     * pairs the grid lists and the narrow phase finds touching, with their numbers of points, are those that every
     * pair tested finds, and no box ran out of slots.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void theGridFindsWhatEveryPairFinds(Backend backend) {
        int n = 150;
        Random random = new Random(23);
        float[][] values = new float[FIELDS.length][n];
        double widest = 0;
        for (int s = 0; s < n; s++) {
            double[] half = {0.02 + 0.06 * random.nextDouble(), 0.02 + 0.06 * random.nextDouble(),
                    0.02 + 0.06 * random.nextDouble()};
            widest = Math.max(widest, Math.sqrt(half[0] * half[0] + half[1] * half[1] + half[2] * half[2]));
            double[] axis = {random.nextDouble() - 0.5, random.nextDouble() - 0.5, random.nextDouble() - 0.5};
            double length = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
            double angle = 4 * random.nextDouble();
            double sin = Math.sin(angle / 2) / length;
            double[] q = {axis[0] * sin, axis[1] * sin, axis[2] * sin, Math.cos(angle / 2)};
            for (int a = 0; a < 3; a++) {
                values[a][s] = (float) (0.1 + 0.8 * random.nextDouble());
                values[7 + a][s] = (float) half[a];
            }
            for (int k = 0; k < 4; k++) {
                values[3 + k][s] = (float) q[k];
            }
        }
        int[] params = Boxes.params(1e-3, 0, 0, 0, 1, 1, 1, 0, 0, MARGIN);

        // The grid: the step's broad phase and collide, and contacts read back.
        BoxStep step = new BoxStep(n, 1, 1, false, BoxBroad.grid(1, 1, 1, widest, MARGIN));
        Map<String, Integer> grid = new TreeMap<>();
        int[] unlisted = new int[1];
        run(backend, step, runner -> {
            step.constants().forEach(runner::write);
            write(runner, values, params);
            runner.run(step.collide()).await();
            int[] contacts = runner.read("contacts");
            for (int p = 0; p < step.slots; p++) {
                if (contacts[p] >= 0) {
                    grid.put((p / BoxBroad.SLOTS) + "-" + contacts[p], contacts[step.slots + p]);
                }
            }
            for (int s = 0; s < n; s++) {
                unlisted[0] += Math.max(0, contacts[2 * step.slots + s] - BoxBroad.SLOTS);
            }
        });

        // Every pair: the collide alone over the whole list.
        EveryPair every = new EveryPair(n);
        Map<String, Integer> all = new TreeMap<>();
        run(backend, every, runner -> {
            write(runner, values, params);
            runner.write("pairs", every.list);
            runner.run(every.passes).await();
            float[] manifold = runner.floats("manifold");
            for (int p = 0; p < every.pairs; p++) {
                int count = (int) manifold[p * BoxContact.MANIFOLD_WORDS + BoxContact.COUNT];
                if (count > 0) {
                    all.put(every.list[2 * p] + "-" + every.list[2 * p + 1], count);
                }
            }
        });

        System.out.printf("broad phase on %s: %d pairs touching of %d, %d by the grid, %d unlisted%n", backend,
                all.size(), every.pairs, grid.size(), unlisted[0]);
        assertTrue(all.size() > 100, "the test is of many contacts, not few: " + all.size());
        assertEquals(0, unlisted[0], "partners with no slot");
        assertEquals(all, grid, "the touching pairs and their points");
    }

    private static void write(PassRunner runner, float[][] values, int[] params) {
        for (int k = 0; k < FIELDS.length; k++) {
            runner.write(FIELDS[k], values[k]);
        }
        runner.write("params", params);
    }

    private static void run(Backend backend, Buffered program, Consumer<PassRunner> work) {
        Accelerator accelerator = null;
        PassRunner runner;
        if (backend == Backend.CPU) {
            runner = PassRunner.cpu(program, Boxes.WORKGROUP, PassRunner.NO_SUBGROUP);
        } else {
            accelerator = new Accelerator();
            if (!accelerator.capabilities().gpuAvailable()) {
                accelerator.close();
                assumeTrue(Boolean.getBoolean("supirvast.requireGpu"), "no Vulkan device");
                throw new IllegalStateException("-Dsupirvast.requireGpu=true but no Vulkan device");
            }
            runner = PassRunner.gpu(accelerator, program, Boxes.WORKGROUP, PassRunner.NO_SUBGROUP);
            runner.clear();
        }
        try {
            work.accept(runner);
        } finally {
            runner.close();
            if (accelerator != null) {
                accelerator.close();
            }
        }
    }

    /** The collide alone over every pair, as stage 5 ran it. */
    private static final class EveryPair implements Buffered {
        final int pairs;
        final int[] list;
        final Map<String, BufferSpec> buffers = new java.util.LinkedHashMap<>();
        final java.util.List<Pass> passes;

        EveryPair(int boxes) {
            pairs = boxes * (boxes - 1) / 2;
            list = new int[2 * pairs];
            for (int a = 0, p = 0; a < boxes; a++) {
                for (int b = a + 1; b < boxes; b++, p++) {
                    list[2 * p] = a;
                    list[2 * p + 1] = b;
                }
            }
            for (String name : FIELDS) {
                buffers.put(name, new BufferSpec(name, Body.F32, boxes));
            }
            buffers.put("params", new BufferSpec("params", Body.F32, Boxes.PARAM_COUNT));
            buffers.put("pairs", new BufferSpec("pairs", Body.I32, 2 * pairs));
            buffers.put("manifold", new BufferSpec("manifold", Body.F32, BoxContact.MANIFOLD_WORDS * pairs));
            buffers.put("candidates", new BufferSpec("candidates", Body.F32, BoxContact.SCRATCH_WORDS * pairs));
            passes = java.util.List.of(new Pass("collide", BoxContact.collide(), BoxContact.BUFFERS,
                    java.util.List.of(BoxContact.NAMES), pairs));
        }

        @Override
        public Map<String, BufferSpec> buffers() {
            return buffers;
        }
    }
}

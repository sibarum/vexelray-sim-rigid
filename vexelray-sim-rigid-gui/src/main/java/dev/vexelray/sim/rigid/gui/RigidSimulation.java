package dev.vexelray.sim.rigid.gui;

import dev.supirvast.vastir.tools.Accelerator;
import dev.supirvast.vastir.tools.ResidentBuffer;

/**
 * What a picture and the stepping thread need of a simulation, whatever its bodies are: a step, the step's
 * {@code shown}, and where it runs. {@link SphereSimulation} and {@link BoxSimulation} are the two.
 */
public interface RigidSimulation extends AutoCloseable {

    /** What its {@code shown} holds, which says how a picture draws it. */
    ShownRing.Shape shape();

    /** How many bodies. */
    int bodies();

    /** The words of its {@code shown}: the shape's stride times the bodies. */
    int shownWords();

    /** One step, waited for: what a thread that runs steps back to back on its own queue does. */
    SphereSimulation.StepReport stepAndWait();

    /** The last step's {@code shown}, read back to the host, for a picture on another device. */
    int[] readShown();

    /** The resident {@code shown}, for {@link ShownSlots} to copy from on the same device; null on the CPU. */
    ResidentBuffer shown();

    /** The accelerator it runs on; null on the CPU. */
    Accelerator accelerator();

    /** Where it runs, for whoever reports it: the device's name, or the CPU. */
    String where();

    @Override
    void close();
}

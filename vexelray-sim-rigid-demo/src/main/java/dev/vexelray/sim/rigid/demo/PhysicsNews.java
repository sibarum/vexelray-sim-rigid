package dev.vexelray.sim.rigid.demo;

import dev.vexelray.sim.rigid.sphere.SphereDiagnostics;
import dev.vexelray.sim.rigid.sphere.SphereStep;

import java.util.concurrent.atomic.AtomicReference;

/**
 * What {@link Physics} reports for the readings, the newest only: written on the physics lane, read by the frame.
 * The pictures themselves go through the {@code ShownRing}; this is the numbers beside them.
 */
final class PhysicsNews {

    /**
     * One report.
     *
     * @param scenario   what is running, or being built
     * @param building   whether a simulation is being made, the kernels lowered and compiled
     * @param spheres    how many, for the running one
     * @param substeps   its substeps a step
     * @param iterations its iterations a substep
     * @param simulated  seconds of world time run since the scenario started, counted by its finished steps
     * @param state      the readings of the state, as of the last time they were read back; null before then
     * @param stepMillis a step's wall time on the physics thread, averaged
     * @param gpuMillis  a step's time on the GPU, averaged; zero on the CPU
     * @param handBackMillis from a step being done to its being in the picture's ring, averaged
     * @param where      where the simulation runs
     * @param solve      how it solves its contacts
     * @param rounds     Gauss–Seidel's rounds a step, every pass of every substep together, averaged
     * @param waits      times a step waited to learn whether its contacts were all solved, averaged
     * @param unlisted   contacts that did not fit the list, and so were not solved, since the build
     * @param problem    why there is no physics, or an empty string
     */
    record Report(Scenario scenario, boolean building, int spheres, int substeps, int iterations, double simulated,
                  SphereDiagnostics state, double stepMillis, double gpuMillis, double handBackMillis,
                  String where, SphereStep.Solve solve, double rounds, double waits, long unlisted,
                  String problem) {

        static Report waiting(Scenario scenario) {
            return new Report(scenario, true, 0, 0, 0, 0, null, 0, 0, 0, "", SphereStep.Solve.JACOBI, 0, 0, 0, "");
        }
    }

    private final AtomicReference<Report> latest = new AtomicReference<>(Report.waiting(Scenario.COLUMN));

    void report(Report report) {
        latest.set(report);
    }

    Report latest() {
        return latest.get();
    }
}

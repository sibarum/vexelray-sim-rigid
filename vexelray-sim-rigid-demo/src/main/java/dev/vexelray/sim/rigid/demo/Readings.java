package dev.vexelray.sim.rigid.demo;

import dev.vexelray.gui.core.Gui;
import dev.vexelray.gui.core.Node;
import dev.vexelray.gui.core.layout.LayoutEnums.Direction;
import dev.vexelray.gui.core.layout.Length;
import dev.vexelray.gui.core.style.Role;
import dev.vexelray.sim.core.gui.DemoLook;
import sibarum.kronometer.Dilated;

import java.util.EnumMap;
import java.util.Map;

/**
 * The numbers beside the picture: the ones the sweep prints, live, in words someone who has not read the code can
 * follow. Every line exists from the start and keeps its place, so a reading changing never moves anything else, and
 * a line is rewritten only when its text changes.
 */
final class Readings {

    enum Line { ABOUT, TIME, OVERLAP, ENERGY, SPEED, COST, DILATION, PROBLEM, KEYS }

    private final Node panel;
    private final Map<Line, Node> nodes = new EnumMap<>(Line.class);
    private final Map<Line, String> shown = new EnumMap<>(Line.class);

    Readings(Gui gui) {
        panel = gui.column().direction(Direction.COLUMN)
                .width(Length.FILL)
                .gap(DemoLook.TIGHT);
        Node[] children = new Node[Line.values().length];
        for (Line line : Line.values()) {
            Role ink = line == Line.PROBLEM ? Role.DANGER : line == Line.KEYS || line == Line.ABOUT ? Role.DIM : Role.INK;
            Node node = gui.text(" ")
                    .width(Length.FILL)
                    .font(line == Line.ABOUT || line == Line.KEYS ? DemoLook.UI : DemoLook.MONO)
                    .textSize(line == Line.KEYS ? DemoLook.SMALL : DemoLook.LABEL)
                    .textColor(gui.theme().color(ink));
            gui.landmark("reading." + line.name().toLowerCase(java.util.Locale.ROOT), node);
            nodes.put(line, node);
            children[line.ordinal()] = node;
        }
        panel.children(children);
        set(Line.KEYS, "Space pause · R reset · N next scenario · = - speed · drag to turn · wheel to zoom");
    }

    Node node() {
        return panel;
    }

    /**
     * What the physics lane last reported, and what the picture costs the frame. While a simulation is being made,
     * only that it is.
     */
    void show(PhysicsNews.Report r, Dilated world, double drawMillis, boolean paused) {
        set(Line.ABOUT, r.scenario().about);
        // Read every time, building or not: a world that is waiting for its kernels is a world running slow.
        double dilation = world.dilation();
        set(Line.DILATION, String.format("World speed       %3.0f%% of the wall's · %d steps forgiven · a step"
                + " every %.1f ms", 100 * dilation, world.forgiven(), world.predicted().nanos() / 1e6));
        set(Line.PROBLEM, !r.problem().isEmpty() ? r.problem()
                : r.state() != null && r.state().broken() ? "A position or velocity is not a number." : "");
        if (r.building()) {
            set(Line.TIME, "Building the kernels…");
            return;
        }
        set(Line.TIME, String.format("%s %.2f s simulated · %d spheres · %d×%d per step", paused ? "Paused at" : "",
                r.simulated(), r.spheres(), r.substeps(), r.iterations()).trim());
        if (r.state() != null) {
            set(Line.OVERLAP, String.format("Deepest overlap   %5.1f%% of a radius", 100 * r.state().maxOverlap()));
            set(Line.ENERGY, String.format("Moving energy     %.2e J", r.state().kinetic()));
            set(Line.SPEED, String.format("Fastest sphere    %.3f m/s", r.state().maxSpeed()));
        }
        set(Line.COST, String.format("A step %.2f ms (%.2f on the GPU) on %s · to the picture %.2f ms · the picture"
                + " %.2f ms", r.stepMillis(), r.gpuMillis(), r.where(), r.handBackMillis(), drawMillis));
    }

    private void set(Line line, String text) {
        String value = text.isEmpty() ? " " : text;
        if (!value.equals(shown.get(line))) {
            shown.put(line, value);
            nodes.get(line).text(value);
        }
    }
}

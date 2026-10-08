package dev.vexelray.sim.rigid.demo;

import dev.vexelray.gui.core.Gui;
import dev.vexelray.gui.core.Node;
import dev.vexelray.gui.core.layout.LayoutEnums.Direction;
import dev.vexelray.gui.core.layout.Length;
import dev.vexelray.gui.core.style.Role;
import dev.vexelray.sim.core.gui.DemoLook;
import dev.vexelray.sim.rigid.gui.SphereSimulation;
import dev.vexelray.sim.rigid.sphere.SphereDiagnostics;

import java.util.EnumMap;
import java.util.Map;

/**
 * The numbers beside the picture: the ones the sweep prints, live, in words someone who has not read the code can
 * follow. Every line exists from the start and keeps its place, so a reading changing never moves anything else, and
 * a line is rewritten only when its text changes.
 */
final class Readings {

    enum Line { ABOUT, TIME, OVERLAP, ENERGY, SPEED, COST, DROPPED, PROBLEM, KEYS }

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

    /** Before the first simulation of a scenario has been made. */
    void waiting(Scenario scenario) {
        set(Line.ABOUT, scenario.about);
        set(Line.TIME, "Building the kernels…");
    }

    void show(Scenario scenario, SphereDiagnostics d, double simulated, SphereSimulation sim, double stepMillis,
              double drawMillis, int drained, long dropped, boolean paused, boolean drawn) {
        set(Line.ABOUT, scenario.about);
        set(Line.TIME, String.format("%s %.2f s simulated · %d spheres · %d×%d per step", paused ? "Paused at" : "",
                simulated, sim.spheres(), sim.substeps(), sim.iterations()).trim());
        set(Line.OVERLAP, String.format("Deepest overlap   %5.1f%% of a radius", 100 * d.maxOverlap()));
        set(Line.ENERGY, String.format("Moving energy     %.2e J", d.kinetic()));
        set(Line.SPEED, String.format("Fastest sphere    %.3f m/s", d.maxSpeed()));
        set(Line.COST, String.format("A step %.2f ms · the picture %.2f ms · %d steps this frame", stepMillis,
                drawMillis, drained));
        set(Line.DROPPED, String.format("Steps dropped     %d", dropped));
        String problem = d.broken() ? "A position or velocity is not a number."
                : !drawn ? "This GPU cannot compute where it draws, so there is no picture, only these numbers."
                : "";
        set(Line.PROBLEM, problem);
    }

    private void set(Line line, String text) {
        String value = text.isEmpty() ? " " : text;
        if (!value.equals(shown.get(line))) {
            shown.put(line, value);
            nodes.get(line).text(value);
        }
    }
}

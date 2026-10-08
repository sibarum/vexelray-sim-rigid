package dev.vexelray.sim.rigid.demo;

import dev.vexelray.gui.core.Gui;
import dev.vexelray.gui.core.Node;
import dev.vexelray.gui.core.layout.LayoutEnums.Direction;
import dev.vexelray.gui.core.layout.Length;
import dev.vexelray.gui.core.style.Role;
import dev.vexelray.gui.widget.Button;
import dev.vexelray.gui.widget.Inspector;
import dev.vexelray.sim.core.gui.DemoLook;
import dev.vexelray.sim.core.gui.Pick;
import dev.vexelray.sim.rigid.demo.Controls.Solver;

import java.util.List;

/**
 * The right-hand side: the transport, the settings, and the readings they move.
 *
 * <p>The settings are an {@link Inspector} of {@link Pick} rows, each a menu of the values worth comparing, so that
 * flipping between two is one click. Whatever a key changes the panel shows too: {@link #sync} reads the controls back
 * whenever their {@link Controls#version} has moved.
 */
final class Panel {

    private final Controls controls;
    private final Inspector settings;
    private final Button pause;
    private final Node node;
    private long version = -1;

    Panel(Gui gui, Controls controls, Readings readings) {
        this.controls = controls;
        Button reset = new Button(gui, "Reset").onPress(controls::reset);
        pause = new Button(gui, "Pause").onPress(controls::togglePause);
        for (Button b : new Button[] {reset, pause}) {
            b.node().width(Length.grow(1f));
        }
        gui.landmark("transport.pause", pause.node());
        Node transport = gui.row().width(Length.FILL).gap(DemoLook.GAP).children(reset.node(), pause.node());

        settings = new Inspector(gui).add(
                new Pick<>("Scenario", "Scenario", List.of(Scenario.values()), s -> s.label, controls::scenario,
                        controls::scenario),
                new Pick<>("Solver", "Relaxation", List.of(Solver.values()), s -> s.label, controls::solver,
                        controls::solver),
                new Pick<>("Solver", "Substeps", Controls.SUBSTEPS, String::valueOf, controls::substeps,
                        controls::substeps),
                new Pick<>("Solver", "Iterations", Controls.ITERATIONS, String::valueOf, controls::iterations,
                        controls::iterations),
                new Pick<>("Playback", "Speed", Controls.SPEEDS, Controls::speedLabel, controls::speed,
                        controls::speed));

        Node heading = gui.text("Readings").font(DemoLook.UI).textSize(DemoLook.HEADING)
                .textColor(gui.theme().color(Role.INK));
        node = gui.column().direction(Direction.COLUMN)
                .width(Length.grow(1f)).height(Length.FILL)
                .gap(DemoLook.GAP).padding(DemoLook.WIDE, DemoLook.WIDE)
                .background(gui.theme().color(Role.PANEL))
                .children(transport, settings.node(), heading, readings.node());
    }

    Node node() {
        return node;
    }

    /** Every frame: the settings read back if a key moved them, and the pause button's label. */
    void sync() {
        long now = controls.version();
        if (now != version) {
            version = now;
            settings.refresh();
            pause.label(controls.paused() ? "Resume" : "Pause");
        }
    }
}

package dev.vexelray.sim.rigid.demo;

import dev.vexelray.framework.api.VexelApp;
import dev.vexelray.framework.automation.AutomationStarter;

/**
 * The application declaration: {@link RigidDemo}'s facts plus {@link AutomationStarter}, the automation sockets, in
 * every build, the release one included.
 *
 * <p>The sockets are off unless the launch asks ({@code --automation}, {@code VEXELRAY_AUTOMATION}) or a person
 * switches them on from the title bar's plug, and loopback-only when they are on. The plug's lamps show each one's
 * port open or not and anything connected now or earlier; {@code --automation.available=none} leaves them out of a
 * run altogether. The processor generates {@code RigidDemoAppWiring} from this.
 */
@VexelApp(name = RigidDemo.APP, formerNames = RigidDemo.FORMER, title = RigidDemo.TITLE, width = RigidDemo.W,
        height = RigidDemo.H, starters = AutomationStarter.class)
final class RigidDemoApp {

    private RigidDemoApp() {
    }
}

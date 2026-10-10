package dev.vexelray.sim.rigid.demo;

import dev.vexelray.framework.api.VexelApp;
import dev.vexelray.framework.automation.AutomationStarter;

/**
 * The debug edition's application declaration: {@link RigidDemo}'s facts plus {@link AutomationStarter}, the driving
 * socket (off unless {@code --automation} or {@code -Dautomation} asks, and loopback-only when it is).
 *
 * <p>The processor generates {@code RigidDemoAppWiring} from this. The release edition ({@code src/edition-release})
 * declares the same application with no starters and no dependency on the automation module. The pom chooses which
 * one is compiled (property {@code edition.src}); keep the two annotations identical apart from {@code starters}.
 */
@VexelApp(name = RigidDemo.APP, title = RigidDemo.TITLE, width = RigidDemo.W, height = RigidDemo.H,
        starters = AutomationStarter.class)
final class RigidDemoApp {

    private RigidDemoApp() {
    }
}

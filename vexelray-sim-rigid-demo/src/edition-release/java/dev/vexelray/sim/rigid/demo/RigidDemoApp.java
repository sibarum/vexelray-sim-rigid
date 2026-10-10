package dev.vexelray.sim.rigid.demo;

import dev.vexelray.framework.api.VexelApp;

/**
 * The release edition's application declaration: {@link RigidDemo}'s facts and no starters. In particular no
 * {@code AutomationStarter}, and the pom drops {@code vexelray-framework-automation} from the classpath under
 * {@code -Pnative-release}, so a shipped binary cannot open a driving socket. The debug edition is
 * {@code src/edition-debug}; keep the two annotations identical apart from {@code starters}.
 */
@VexelApp(name = RigidDemo.APP, title = RigidDemo.TITLE, width = RigidDemo.W, height = RigidDemo.H)
final class RigidDemoApp {

    private RigidDemoApp() {
    }
}

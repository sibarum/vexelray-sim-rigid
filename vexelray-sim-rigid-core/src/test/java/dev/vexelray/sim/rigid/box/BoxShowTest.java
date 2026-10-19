package dev.vexelray.sim.rigid.box;

import dev.vexelray.sim.rigid.box.BoxScene.Backend;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** What {@link Boxes#show} leaves for a picture: each step's centre and orientation, with the step before's. */
class BoxShowTest {

    /**
     * Two boxes, one tumbling in free fall and one fixed: after three steps the "after" slots hold the state the step
     * ended on, the "before" slots what the step before left there, and the half-extents are each box's own.
     */
    @ParameterizedTest
    @MethodSource("dev.vexelray.sim.rigid.box.BoxScene#backends")
    void thePictureIsGivenEachStepAndTheOneBefore(Backend backend) {
        try (BoxScene scene = new BoxScene(2).uniform(0.05, 0.1, 0.15, 1)) {
            scene.x[0] = scene.y[0] = scene.z[0] = 0.5f;
            scene.ax[0] = 1;
            scene.ay[0] = 2;
            scene.az[0] = 3;
            scene.x[1] = 0.2f;
            scene.y[1] = 0.1f;
            scene.z[1] = 0.8f;
            scene.im[1] = 0;
            scene.half[2][1] = 0.05f;
            scene.start(backend).advance(2);
            float[] earlier = scene.shown().clone();
            scene.advance(1).read();
            float[] shown = scene.shown();

            float[][] at = {scene.x, scene.y, scene.z};
            for (int s = 0; s < 2; s++) {
                int base = s * Boxes.SHOWN_STRIDE;
                for (int a = 0; a < 3; a++) {
                    assertEquals(at[a][s], shown[base + Boxes.CENTRE_AFTER + a], 0, "box " + s + ", centre " + a);
                    assertEquals(earlier[base + Boxes.CENTRE_AFTER + a], shown[base + Boxes.CENTRE_BEFORE + a], 0,
                            "box " + s + ", centre " + a + " kept from the step before");
                    assertEquals(scene.half[a][s], shown[base + Boxes.HALF + a], 0, "box " + s + ", half " + a);
                }
                for (int k = 0; k < 4; k++) {
                    assertEquals(scene.q[k][s], shown[base + Boxes.TURNED_AFTER + k], 0, "box " + s + ", turn " + k);
                    assertEquals(earlier[base + Boxes.TURNED_AFTER + k], shown[base + Boxes.TURNED_BEFORE + k], 0,
                            "box " + s + ", turn " + k + " kept from the step before");
                }
            }
            assertTrue(shown[Boxes.CENTRE_AFTER + 1] < shown[Boxes.CENTRE_BEFORE + 1], "the first box is falling");
            assertTrue(shown[Boxes.TURNED_AFTER] != shown[Boxes.TURNED_BEFORE], "and turning");
        }
    }
}

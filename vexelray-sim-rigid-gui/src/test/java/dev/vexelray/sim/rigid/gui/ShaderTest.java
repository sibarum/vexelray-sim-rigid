package dev.vexelray.sim.rigid.gui;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Both pictures lower to SPIR-V, without a window: what a pipeline would be made from. */
class ShaderTest {

    private static final int MAGIC = 0x07230203;

    @Test
    void theSpheresPictureLowers() {
        lowers(SphereShader.fragmentSpirv());
    }

    @Test
    void theBoxesPictureLowers() {
        lowers(BoxShader.fragmentSpirv());
    }

    private static void lowers(byte[] spirv) {
        assertTrue(spirv.length > 20, "a module, not " + spirv.length + " bytes");
        int magic = (spirv[0] & 0xff) | (spirv[1] & 0xff) << 8 | (spirv[2] & 0xff) << 16 | (spirv[3] & 0xff) << 24;
        assertEquals(MAGIC, magic, "SPIR-V's magic number");
    }
}

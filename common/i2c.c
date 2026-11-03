/*
 * x6100-bbfw -- shared I2C master driver (see i2c.h). From Phase 1, following
 * i2c.md (FUN_08031e1c combined read, FUN_08031890 raw write, FUN_08031cfc
 * recovery).
 */
#include "bsp.h"
#include "i2c.h"

#define I2C_TMO 300000u

void i2c_pins_init(void)
{
    /* GPIOB clock is enabled by bsp_gpio_init_safe(). */
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_4);   /* I2C1_SCL */
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_4);   /* I2C1_SDA */
    M_AF(GPIOB, 6);  M_OD(GPIOB, 6);  SPD_HI(GPIOB, 6);  PULL_UP(GPIOB, 6);
    M_AF(GPIOB, 7);  M_OD(GPIOB, 7);  SPD_HI(GPIOB, 7);  PULL_UP(GPIOB, 7);

    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_10, LL_GPIO_AF_4); /* I2C2_SCL */
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_11, LL_GPIO_AF_4); /* I2C2_SDA */
    M_AF(GPIOB, 10); M_OD(GPIOB, 10); SPD_HI(GPIOB, 10); PULL_UP(GPIOB, 10);
    M_AF(GPIOB, 11); M_OD(GPIOB, 11); SPD_HI(GPIOB, 11); PULL_UP(GPIOB, 11);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1 | LL_APB1_GRP1_PERIPH_I2C2);
}

/* Standard mode @ PCLK1 = 62.5 MHz. SCL = 50 kHz (CCR=625), NOT the 100 kHz in
 * i2c.md: on hardware (2026-10-30, i2cdiag2) 100 kHz gives ARLO on I2C1 -- the
 * bus is rise-time limited and the master mis-samples SDA at 100 kHz; 50 kHz
 * ACKs cleanly. FREQ=50 vs 62 made no difference (so the over-spec FREQ is not
 * the issue, consistent with the OEM's 62.5 MHz I2C basis); TRISE=63 is the
 * standard-mode max. If matching the OEM's exact 100 kHz is ever needed, it
 * likely requires stronger pull-ups / a board change, not a register tweak. */
void i2c_master_init(I2C_TypeDef *i2c)
{
    /* Derived from PCLK1 so SCL stays ~50 kHz at either core clock (PCLK1 =
     * 62.5 MHz @250, 42 MHz @168): FREQ = PCLK1 MHz, CCR = PCLK1/(2*50kHz),
     * TRISE = FREQ+1 (standard-mode max). @62.5 MHz -> 62/625/63 as before. */
    LL_I2C_Disable(i2c);
    MODIFY_REG(i2c->CR2, I2C_CR2_FREQ, PCLK1_HZ / 1000000u);
    i2c->CCR   = PCLK1_HZ / (2u * 50000u);
    i2c->TRISE = (PCLK1_HZ / 1000000u) + 1u;
    LL_I2C_Enable(i2c);
}

int i2c_probe(I2C_TypeDef *i2c, uint8_t addr7)
{
    uint32_t t;

    LL_I2C_GenerateStartCondition(i2c);
    for (t = I2C_TMO; !LL_I2C_IsActiveFlag_SB(i2c); )
        if (!--t) { LL_I2C_GenerateStopCondition(i2c); return -1; }
    LL_I2C_TransmitData8(i2c, (uint8_t)(addr7 << 1));
    for (t = I2C_TMO; ; ) {
        if (LL_I2C_IsActiveFlag_ADDR(i2c)) {
            LL_I2C_ClearFlag_ADDR(i2c);
            LL_I2C_GenerateStopCondition(i2c);
            return 1;
        }
        if (LL_I2C_IsActiveFlag_AF(i2c)) {
            LL_I2C_ClearFlag_AF(i2c);
            LL_I2C_GenerateStopCondition(i2c);
            return 0;
        }
        if (!--t) { LL_I2C_GenerateStopCondition(i2c); return -1; }
    }
}

int i2c_read(I2C_TypeDef *i2c, uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t n)
{
    uint32_t t;

    LL_I2C_GenerateStartCondition(i2c);
    for (t = I2C_TMO; !LL_I2C_IsActiveFlag_SB(i2c); )
        if (!--t) goto fail;

    LL_I2C_TransmitData8(i2c, (uint8_t)(addr7 << 1));
    for (t = I2C_TMO; ; ) {
        if (LL_I2C_IsActiveFlag_ADDR(i2c)) break;
        if (LL_I2C_IsActiveFlag_AF(i2c)) { LL_I2C_ClearFlag_AF(i2c); goto fail; }
        if (!--t) goto fail;
    }
    LL_I2C_ClearFlag_ADDR(i2c);

    for (t = I2C_TMO; !LL_I2C_IsActiveFlag_TXE(i2c); )
        if (!--t) goto fail;
    LL_I2C_TransmitData8(i2c, reg);
    for (t = I2C_TMO; !(LL_I2C_IsActiveFlag_BTF(i2c) || LL_I2C_IsActiveFlag_TXE(i2c)); )
        if (!--t) goto fail;

    LL_I2C_GenerateStartCondition(i2c);
    for (t = I2C_TMO; !LL_I2C_IsActiveFlag_SB(i2c); )
        if (!--t) goto fail;
    LL_I2C_TransmitData8(i2c, (uint8_t)((addr7 << 1) | 1u));
    for (t = I2C_TMO; ; ) {
        if (LL_I2C_IsActiveFlag_ADDR(i2c)) break;
        if (LL_I2C_IsActiveFlag_AF(i2c)) { LL_I2C_ClearFlag_AF(i2c); goto fail; }
        if (!--t) goto fail;
    }

    if (n == 1u) {
        LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
        LL_I2C_ClearFlag_ADDR(i2c);
        LL_I2C_GenerateStopCondition(i2c);
        for (t = I2C_TMO; !LL_I2C_IsActiveFlag_RXNE(i2c); )
            if (!--t) goto fail;
        buf[0] = LL_I2C_ReceiveData8(i2c);
    } else {
        LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
        LL_I2C_ClearFlag_ADDR(i2c);
        for (uint32_t i = 0; i < n; i++) {
            if (i == n - 1u) {
                LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
                LL_I2C_GenerateStopCondition(i2c);
            }
            for (t = I2C_TMO; !LL_I2C_IsActiveFlag_RXNE(i2c); )
                if (!--t) goto fail;
            buf[i] = LL_I2C_ReceiveData8(i2c);
        }
    }
    return 0;

fail:
    LL_I2C_GenerateStopCondition(i2c);
    return -1;
}

int i2c_write_raw(I2C_TypeDef *i2c, uint8_t addr7, const uint8_t *buf, uint32_t n)
{
    uint32_t t;

    LL_I2C_GenerateStartCondition(i2c);
    for (t = I2C_TMO; !LL_I2C_IsActiveFlag_SB(i2c); )
        if (!--t) goto fail;

    LL_I2C_TransmitData8(i2c, (uint8_t)(addr7 << 1));
    for (t = I2C_TMO; ; ) {
        if (LL_I2C_IsActiveFlag_ADDR(i2c)) break;
        if (LL_I2C_IsActiveFlag_AF(i2c)) { LL_I2C_ClearFlag_AF(i2c); goto fail; }
        if (!--t) goto fail;
    }
    LL_I2C_ClearFlag_ADDR(i2c);

    for (uint32_t i = 0; i < n; i++) {
        for (t = I2C_TMO; !LL_I2C_IsActiveFlag_TXE(i2c); )
            if (!--t) goto fail;
        LL_I2C_TransmitData8(i2c, buf[i]);
    }
    for (t = I2C_TMO; !LL_I2C_IsActiveFlag_BTF(i2c); )
        if (!--t) goto fail;
    LL_I2C_GenerateStopCondition(i2c);
    return 0;

fail:
    LL_I2C_GenerateStopCondition(i2c);
    return -1;
}

void i2c1_bus_recover(void)
{
    LL_I2C_Disable(I2C1);

    M_OD(GPIOB, 6); SET_HI(GPIOB, 6); M_OUT(GPIOB, 6);
    M_OD(GPIOB, 7); SET_HI(GPIOB, 7); M_OUT(GPIOB, 7);

    /* OEM (VERIFIED disassembly): clock PB7 ten times while controlling PB6.
     * NOTE: textbook SMBus recovery clocks SCL (PB6); replicate OEM literally,
     * confirm on hardware. */
    for (int i = 0; i < 10; i++) {
        SET_LO(GPIOB, 7); bsp_delay_ms(1);
        SET_HI(GPIOB, 7); bsp_delay_ms(1);
    }

    M_AF(GPIOB, 6);
    M_AF(GPIOB, 7);
    i2c_master_init(I2C1);
    bsp_uprintf("i2c reseted\r\n");
}

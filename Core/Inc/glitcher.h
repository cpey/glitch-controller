#ifndef __GLITCHER_H
#define __GLITCHER_H

#include "controller.h"

#define PG_DAC_BYPASS

#define GLITCH_DELAY_MS         1000
#define GLITCH_DELAY_US         0
#define PG_VOLTAGE_FS           0xff
#define GLITCH_DELAY_US_LENGTH  3
#define GLITCH_DELAY_MS_LENGTH  4


void generate_glitch();

#define PG_MASK_BIT0    0x01
#define PG_MASK_BIT1    0x02
#define PG_MASK_BIT2    0x04
#define PG_MASK_BIT3    0x08
#define PG_MASK_BIT4    0x10
#define PG_MASK_BIT5    0x20
#define PG_MASK_BIT6    0x40
#define PG_MASK_BIT7    0x80

inline void pg_set_value(uint8_t value) {
    HAL_GPIO_WritePin(PG0_GPIO_Port, PG0_Pin, (GPIO_PinState)(value & PG_MASK_BIT0));
    HAL_GPIO_WritePin(PG1_GPIO_Port, PG1_Pin, (GPIO_PinState)(value & PG_MASK_BIT1));
    HAL_GPIO_WritePin(PG2_GPIO_Port, PG2_Pin, (GPIO_PinState)(value & PG_MASK_BIT2));
    HAL_GPIO_WritePin(PG3_GPIO_Port, PG3_Pin, (GPIO_PinState)(value & PG_MASK_BIT3));
    HAL_GPIO_WritePin(PG4_GPIO_Port, PG4_Pin, (GPIO_PinState)(value & PG_MASK_BIT4));
    HAL_GPIO_WritePin(PG5_GPIO_Port, PG5_Pin, (GPIO_PinState)(value & PG_MASK_BIT5));
    HAL_GPIO_WritePin(PG6_GPIO_Port, PG6_Pin, (GPIO_PinState)(value & PG_MASK_BIT6));
    HAL_GPIO_WritePin(PG7_GPIO_Port, PG7_Pin, (GPIO_PinState)(value & PG_MASK_BIT7));

    // Generate a  pulse on the PGCLK pin
    HAL_GPIO_TogglePin(PGCLK_GPIO_Port, PGCLK_Pin);
    HAL_GPIO_TogglePin(PGCLK_GPIO_Port, PGCLK_Pin);
}

inline void pg_set_value_fast(uint8_t value) {
    // Clk: PB7
    // Data: PB6, PB5, PB4, PB3, PD2, PC12, PC11, PC10
    *GPIOB_BSRR = GPIOB_VALUE_CFG(value) | (GPIOB_VALUE_CFG(~value) << 16);
    *GPIOD_BSRR = GPIOD_VALUE_CFG(value) | (GPIOD_VALUE_CFG(~value) << 16);
    *GPIOC_BSRR = GPIOC_VALUE_CFG(value) | (GPIOC_VALUE_CFG(~value) << 16);

    // Generate a  pulse on the PGCLK pin
    GPIOB->BSRR = (uint32_t)PGCLK_Pin;
    GPIOB->BRR  = (uint32_t)PGCLK_Pin;
}

inline void pg_sig_set_high() {
    PGSIG_GPIO_Port->BSRR = (uint32_t)PGSIG_Pin;
}

inline void pg_sig_set_low() {
    PGSIG_GPIO_Port->BRR  = (uint32_t)PGSIG_Pin;
}

#endif

#include "main.h"
#include "serial.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/**
 * cmd: '0x??' -> 4 chars
 * 
 */
void process_cmd_voltage(uint8_t *usb_msg, uint16_t usb_msg_len) {
    if (usb_msg_len < 4)
        return;

    if (usb_msg[0] != '0' || usb_msg[1] != 'x')
        return;

    if (!(IS_VALID_HEX(usb_msg[2]) && IS_VALID_HEX(usb_msg[3])))
        return;

    uint8_t pg_value = (HEX_TO_BYTE(usb_msg[2]) << 4) + HEX_TO_BYTE(usb_msg[3]);
    pg_set_value_fast(pg_value);
}

/**
 * cmd: '?' -> 1 chars
 * 
 */
void process_cmd_signal(uint8_t *usb_msg, uint16_t usb_msg_len) {
    if (usb_msg_len < 1)
        return;

    if (usb_msg[0] != '0' && usb_msg[0] != '1')
        return;

    if (usb_msg[0] == '0') {
        pg_sig_set_low();
    } else {
        pg_sig_set_high();
    }
}

/**
 * cmd: 
 * ms    -- '????'     -> up to 5 chars
 * ms.us -- '????.???' -> up to 9 chars
 */
void process_cmd_timer(uint8_t *usb_msg, uint16_t usb_msg_len) {
    char msg[64];
    uint32_t delay_us = 0;

    if (usb_msg[0] == '-')
        return;

    char *ptr = strchr((char *) usb_msg, '.');
    if (ptr != NULL) {
        uint8_t len = (char *) usb_msg + usb_msg_len - ptr - 2;
        if (len > GLITCH_DELAY_US_LENGTH) {
            sprintf(msg, "Invalid decimal fraction length.");
            send_to_usb(msg);
            return;
        }
        delay_us = strtol((char *) ptr + 1, NULL, 10);
    }

    uint32_t delay_ms = strtol((char *) usb_msg, NULL, 10);
    if (delay_ms > pow(10, GLITCH_DELAY_MS_LENGTH)) {
        sprintf(msg, "Invalid integer part length.");
        send_to_usb(msg);
        return;
    }
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    MX_TIM2_Init(delay_ms, delay_us);

    sprintf(msg, "Set timer period: %lu ms, %lu us", delay_ms, delay_us);
    send_to_usb(msg);
}

void process_cmd_set(uint8_t *usb_msg, uint16_t usb_msg_len) {

    uint8_t arg = usb_msg[1];
    
    switch (arg) {
        case 'v': /* ' v 0x??' -> 8 chars */
            if (usb_msg_len < 8)
                return;
            if (usb_msg[2] != ' ')
                return;
            process_cmd_voltage(usb_msg + 3, usb_msg_len - 3);
            break;
        case 's': /* ' s ?' -> 5 chars */
            if (usb_msg_len < 5)
                return;
            if (usb_msg[2] != ' ')
                return;
            process_cmd_signal(usb_msg + 3, usb_msg_len - 3);
            break;
        case 't': /* ' t ????.???' -> 12 chars (max) */
                  /* ' t ?'        -> 5 chars (min) */
            if (usb_msg_len < 5 || usb_msg_len > 12)
                return;
            if (usb_msg[2] != ' ')
                return;
            process_cmd_timer(usb_msg + 3, usb_msg_len - 3);
            break;
        default:
            break;
    }
}

void process_cmd_run() {
    reset_timer = true;
}

/**
 * cmd: 
 * 's v 0x??' -> 9 chars -- Set DAC input value
 * 's s ?'    -> 6 char  -- Set power-glitcher output voltage level (PG_DAC_BYPASS mode)
 * 'r'        -> 2 char  -- Run next test
 */
void process_cmd(uint8_t *usb_msg, uint16_t usb_msg_len) {
    if (!usb_msg_len) 
        return;
    uint8_t cmd = usb_msg[0];
    switch (cmd) {
        case 's':
            process_cmd_set(usb_msg + 1, usb_msg_len - 1);
            break;
        case 'r':
            process_cmd_run();
            break;
        default:
            break;
    }
}


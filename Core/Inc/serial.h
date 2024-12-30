#ifndef __SERIAL_H
#define __SERIAL_H

#include "usbd_cdc_if.h"

inline uint8_t send_to_usb(char *buf) {
    char dest[strlen(buf) + 3];
    strcpy(dest, buf);
    strcat(dest, "\r\n");
    return CDC_Transmit_FS((uint8_t *) dest, strlen(dest));
}

#endif

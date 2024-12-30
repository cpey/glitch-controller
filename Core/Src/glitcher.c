#include "glitcher.h"
#include "serial.h"

extern inline void pg_set_value(uint8_t);
extern inline void pg_set_value_fast(uint8_t);
extern inline void pg_sig_set_high();
extern inline void pg_sig_set_low();


void generate_glitch() {
#ifdef PG_DAC_BYPASS
    pg_sig_set_low();
    pg_sig_set_high();
    send_to_usb("glitched");
#else
    pg_set_value_fast(0x00);
    pg_set_value_fast(PG_VOLTAGE_FS);
#endif
}

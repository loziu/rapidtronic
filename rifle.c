#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "light_ws2812.h"
#include "ws2812_config.h"

struct cRGB led;
volatile char display[4] = {0};
volatile uint8_t adc_ready = 0;
volatile uint8_t status = 0;

#define UART_BAUDRATE   9600
#define UART_TX_PORT    PORTB
// UART to Sparkfun Serial7SegmentDisplay (out)
// https://github.com/sparkfun/Serial7SegmentDisplay/wiki
#define UART_TX_PIN     PB1
#define UART_TX_DDR     DDRB

#define TINYPORT        UART_TX_PORT
#define DDRPORT         UART_TX_DDR
// Line follower digital in
#define IR_PIN          PB3
// Pin to magazine switch (in + pullup)
#define MAG_PIN         PB0
// Pin to voltage divider (adc)
#define BATT_PIN        PB2

#define tx_init()   do { \
        UART_TX_PORT |= (1 << UART_TX_PIN); \
        UART_TX_DDR |= (1 << UART_TX_PIN); \
        } while(0)

#define ir_init() do { \
        DDRPORT &= ~(1 << IR_PIN); \
        TINYPORT &= ~(1 << IR_PIN); \
        }while(0)

#define rgb_init() do { \
        DDRPORT |= (1 << ws2812_pin); \
        }while(0)

#define mag_init() do { \
        DDRPORT &= ~(1 << MAG_PIN); \
        TINYPORT |= (1 << MAG_PIN); \
        }while(0)

#define batt_init() do {\
        sei(); \
        ADMUX = (0 << ADLAR) | \
                (0 << REFS1) | \
                (0 << REFS0) | \
                (0 << MUX3)  | \
                (0 << MUX2)  | \
                (0 << MUX1)  | \
                (1 << MUX0); \
        ADCSRA = (1<<ADPS2) | \
                 (1<<ADPS1) | \
                 (1<<ADPS0) | \
                 (1<<ADIE)  | \
                 (1<<ADATE) | \
                 (1<<ADEN); \
        ADCSRB = (0 << ADTS2) | \
                 (0 << ADTS1) | \
                 (0 << ADTS0); \
        ADCSRA |= (1<<ADSC); \
        }while(0)

#define fill_led(rr,gg,bb) do { \
        led.r = rr;\
        led.g = gg;\
        led.b = bb;\
        }while(0)

#define clear_serial_display() do { \
        putbyte(0x76);\
        }while(0)

#define move_serial_cursor(position) do { \
        putbyte(0x79);\
        putbyte(position);\
        }while(0)

#define write_serial_msg(a,b,c,d) do { \
        putbyte(a);\
        putbyte(b);\
        putbyte(c);\
        putbyte(d);\
        }while(0)

#define segment_definition(top, rx_top, rx_bottom, bottom, sx_bottom, sx_top, center) do { \
        (top << 0 | rx_top << 1 | rx_bottom << 2 | bottom << 3 | sx_bottom << 4 | sx_top << 5 | center << 6);\
} while(0)

#define write_serial_segments(a,b,c,d) do { \
        putbyte(0x7B);\
        putbyte(a);\
        putbyte(0x7C);\
        putbyte(b);\
        putbyte(0x7D);\
        putbyte(c);\
        putbyte(0x7E);\
        putbyte(d);\
        }while(0)

#define write_serial_brightness(level) do { \
        putbyte(0x7A);\
        putbyte(level);\
        }while(0)

#define write_serial_dots(d1, d2, d3, d4, colon, apostrophe) do {\
        putbyte(0x77); \
        putbyte(d1 << 0 | d2 << 1 | d3 << 2 | d4 << 3 | colon << 4 | apostrophe << 5);\
        }while(0)

#define update_led()                ws2812_setleds(&led,1)
#define is_magazine_inserted()     (PINB & (1 << MAG_PIN))
#define is_ammo_low()              (PINB & (1 << IR_PIN))

/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42, by Joerg Wunsch):
 * <dinuxbg .at. gmail.com> wrote this file.  As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy me a beer in return.
 *                                                             Dimitar Dimitrov
 * ----------------------------------------------------------------------------
 */
void putbyte(uint8_t c) {
#define UART_TX_NUM_DELAY_CYCLES    ((F_CPU/UART_BAUDRATE-16)/4+1)
#define UART_TX_NUM_ADD_NOP     ((F_CPU/UART_BAUDRATE-16)%4)
    uint8_t sreg;
    uint16_t tmp;
    uint8_t numiter = 10;

    sreg = SREG;
    cli();

    asm volatile (
        /* put the START bit */
        "in %A0, %3"        "\n\t"  /* 1 */
        "cbr %A0, %4"       "\n\t"  /* 1 */
        "out %3, %A0"       "\n\t"  /* 1 */
        /* compensate for the delay induced by the loop for the
         * other bits */
        "nop"           "\n\t"  /* 1 */
        "nop"           "\n\t"  /* 1 */
        "nop"           "\n\t"  /* 1 */
        "nop"           "\n\t"  /* 1 */
        "nop"           "\n\t"  /* 1 */

        /* delay */
       "1:" "ldi %A0, lo8(%5)"  "\n\t"  /* 1 */
        "ldi %B0, hi8(%5)"  "\n\t"  /* 1 */
       "2:" "sbiw %A0, 1"       "\n\t"  /* 2 */
        "brne 2b"       "\n\t"  /* 1 if EQ, 2 if NEQ */
#if UART_TX_NUM_ADD_NOP > 0
        "nop"           "\n\t"  /* 1 */
  #if UART_TX_NUM_ADD_NOP > 1
        "nop"           "\n\t"  /* 1 */
    #if UART_TX_NUM_ADD_NOP > 2
        "nop"           "\n\t"  /* 1 */
    #endif
  #endif
#endif
        /* put data or stop bit */
        "in %A0, %3"        "\n\t"  /* 1 */
        "sbrc %1, 0"        "\n\t"  /* 1 if false,2 otherwise */
        "sbr %A0, %4"       "\n\t"  /* 1 */
        "sbrs %1, 0"        "\n\t"  /* 1 if false,2 otherwise */
        "cbr %A0, %4"       "\n\t"  /* 1 */
        "out %3, %A0"       "\n\t"  /* 1 */

        /* shift data, putting a stop bit at the empty location */
        "sec"           "\n\t"  /* 1 */
        "ror %1"        "\n\t"  /* 1 */

        /* loop 10 times */
        "dec %2"        "\n\t"  /* 1 */
        "brne 1b"       "\n\t"  /* 1 if EQ, 2 if NEQ */
        : "=&w" (tmp),          /* scratch register */
          "=r" (c),         /* we modify the data byte */
          "=r" (numiter)        /* we modify number of iter.*/
        : "I" (_SFR_IO_ADDR(UART_TX_PORT)),
          "M" (1<<UART_TX_PIN),
          "i" (UART_TX_NUM_DELAY_CYCLES),
          "1" (c),          /* data */
          "2" (numiter)
    );
    SREG = sreg;
}

#undef UART_TX_NUM_DELAY_CYCLES
#undef UART_TX_NUM_ADD_NOP
/*
 * ***************************************************************************/

int main(void) {
    uint8_t dot = 0x01;

    tx_init();
    ir_init();
    rgb_init();
    mag_init();

    _delay_ms(500);
    write_serial_brightness(0xFF);
    // write hello world
    clear_serial_display();
    write_serial_msg('b','o','o','t');
    _delay_ms(1000);
    // write revision rE:x.x
    write_serial_msg('r','e','0','2');
    write_serial_dots(0,0,1,0,1,0);
    _delay_ms(1000);
    write_serial_dots(0,0,0,0,0,0);

    batt_init();

    for(;;) {
        // red warning on low ammo
        if(is_magazine_inserted() && is_ammo_low()) {
            fill_led(100, 0, 0);
        // yellow warning on absent magazine
        } else  if(!is_magazine_inserted()) {
            fill_led(255, 255, 0);
        // otherwise switch off
        } else {
            fill_led(0, 0, 0);
        }

        // if we get a valid voltage
        if(adc_ready) {
            adc_ready = 0;
            // print numbers
            move_serial_cursor(0);
            write_serial_msg(display[0], display[1], display[2], display[3]);
            // print decimal dot
            write_serial_dots(0,1,0,0,0,0);
            // assume LiPo 3S power supply: violet alert if Vcc < 11V
            if(display[0] == '0' || display[1] == '0') {
                fill_led(208, 32, 144);
            }
        // otherwise print moving dots & message
        } else {
            dot = (dot << 1 | dot >> 3) & 0x0F;
            move_serial_cursor(0);
            write_serial_msg('c','o','o','l');
            putbyte(0x77);
            putbyte(dot);
        }

        // send light data
        update_led();
        _delay_ms(1);
    }
    return 0;
}

// on ADC interrupt
ISR(ADC_vect)
{
#define ASCII_OFFSET 48
    // fetch 10 bits
    uint16_t v = ADCL;
    v = (ADCH << 8) + v;

    // if the switch is connected to a battery
    if(v > 1) {
        // adjust using conversion formula and voltage divider ratio
        // R1 = 37.5 KOhm (according to multimeter)
        // R2 = 9.06 KOhm (according to multimeter)
        // Vcc ~= 5V
        //
        v = v * 2 + (v*5/10);
        // split number in digits
        uint8_t msd  = v / 1000;
        v -= msd * 1000;
        uint8_t rd = v /100;
        v -= rd * 100;
        uint8_t ld = v / 10;
        v -= ld *10;
        uint8_t lsd = v;
        // convert digits to ASCII
        display[0] = msd + ASCII_OFFSET;
        display[1] = rd + ASCII_OFFSET;
        display[2] = ld + ASCII_OFFSET;
        display[3] = lsd + ASCII_OFFSET;
        // signal main loop
        adc_ready = 1;
    }
    else {
        adc_ready = 0;
    }
}

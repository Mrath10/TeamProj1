/*
 * LCD.c
 *
 * Created: 7/09/2021 2:57:07 PM
 * Author : mayvin
 */

#include "LCD_LIB/lcd.h"
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#define F_CPU 8000000UL // 16MHz
#include "LCD_LIB/i2cmaster.h"
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define BAUD 9600
#define ADS1115_ADDR 0x48
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01

/*DECLARATIONS*/
void uart_init(uint8_t baud);
void uart_transmit(unsigned char data);
void ALRT_init(void);
unsigned char uart_receive(void);
void uart_newline(void);
void change_mode(int mode);
void change_brightness(int level);

void ads1115_write(uint8_t addr, uint8_t pointerReg, uint16_t configReg);
uint16_t ads1115_read(uint8_t addr, uint8_t pointerReg);
uint16_t ads1115_read_SE(uint8_t addr, uint16_t configReg);
int16_t ads1115_read_DIFF_A2_A3(uint8_t addr, uint16_t configReg);

uint8_t EEPROM_read(unsigned int uiAddress);
void EEPROM_write(unsigned int uiAdress, uint8_t ucData);

// uint8_t eepromThresh;
uint8_t contThresh;
volatile double voltMaxDC = 0, voltMinDC = 0, voltMaxAC = 0, voltMinAC = 0;
volatile double resOffset = 0;
uint16_t ads1115ConfigVoltageInt = 0b11 << 0 | 0b111 << 5 | 0b0 << 8 | 0b010 << 9 | 0b011 << 12 | 0b1 << 15;

int current_level = 4;
int current_mode = 1;
int hold = 0;
int mode = 1;
int calc = 1;
double dataVoltage;
double final;
double resVoltage;

/*Custom chars*/
void customChar()
{
    lcd_command(0x40);
    lcd_data(0x1F);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x1F);

    lcd_command(0x48);
    lcd_data(0x1F);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x1B);
    lcd_data(0x1F);
    lcd_data(0x1F);

    lcd_command(0x50);
    lcd_data(0x1F);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x11);
    lcd_data(0x1B);
    lcd_data(0x1F);
    lcd_data(0x1F);
    lcd_data(0x1F);

    lcd_command(0x58);
    lcd_data(0x1F);
    lcd_data(0x11);
    lcd_data(0x1B);
    lcd_data(0x1F);
    lcd_data(0x1B);
    lcd_data(0x1F);
    lcd_data(0x1F);
    lcd_data(0x1F);

    lcd_command(0x60);
    lcd_data(0x00);
    lcd_data(0x0E);
    lcd_data(0x11);
    lcd_data(0x00);
    lcd_data(0x1F);
    lcd_data(0x0E);
    lcd_data(0x04);
    lcd_data(0x04);

    lcd_command(0x68);
    lcd_data(0x1B);
    lcd_data(0x1B);
    lcd_data(0x1B);
    lcd_data(0x1B);
    lcd_data(0x1B);
    lcd_data(0x1B);
    lcd_data(0x1B);
    lcd_data(0x1B);
}

/* BRIGHTNESS STUFF*/
void change_brightness(int level)
{
    customChar();
    lcd_gotoxy(15, 0);
    if (level == 0) {
        lcd_putc(0);
    } else if (level == 1) {
        lcd_putc(1);
    } else if (level == 2) {
        lcd_putc(2);
    } else if (level == 3) {
        lcd_putc(3);
    } else if (level == 4) {
        lcd_putc(255);
    }
}

void change_mode(int mode)
{
    if (mode == 1) {
        lcd_clrscr();
        lcd_gotoxy(0, 0);
        lcd_puts("Mode: V(DC)");
        calc = 1;
    } else if (mode == 2) {
        lcd_clrscr();
        lcd_gotoxy(0, 0);
        lcd_puts("MODE:V(AC)");
        calc = 2;
    } else if (mode == 3) {
        lcd_clrscr();
        lcd_gotoxy(0, 0);
        lcd_puts("MODE:Resistance");
        calc = 3;
    } else if (mode == 4) {
        lcd_clrscr();
        lcd_gotoxy(0, 0);
        lcd_puts("MODE:Continuity");
        calc = 4;
    }
}

/*INTERRUPT HANDLER*/
ISR(PCINT1_vect)
{
    /*check what button was pressed*/
    /* if the pin is low decrease the brightness */
    if ((PINC & (1 << PINC0)) == 0 && hold == 0) {
        _delay_ms(250);
        if (current_level - 1 <= 0 && OCR0A + 63 > 251) {
            current_level = 0;
            OCR0A = 255;
        } else {
            current_level--;
            OCR0A = OCR0A + 63;
        }
        change_brightness(current_level);
    }

    /* if the pin is high increase the brightness */
    if ((PINC & (1 << PINC1)) == 0 && hold == 0) {
        _delay_ms(250);
        if (current_level + 1 >= 4 && OCR0A - 63 <= 3) {
            current_level = 4;
            OCR0A = 0;
        } else {
            current_level++;
            OCR0A = OCR0A - 63;
        }
        change_brightness(current_level);
    }

    /* if pin changes change mode*/
    if ((PINC & (1 << PINC2)) == 0 && hold == 0) {
        _delay_ms(250);
        if (current_mode + 1 <= 4) {
            current_mode++;
        } else {
            current_mode = 1;
        }
        change_mode(current_mode);
        change_brightness(current_level);
    }

    if ((PINC & (1 << PINC3)) == 0) {
        _delay_ms(250);
        if (hold == 0) {
            hold = 1;
        } else {
            hold = 0;
        }
    }
}

int main(void)
{
    customChar();
    _delay_ms(500);
    lcd_init(LCD_DISP_ON);
    /* set bits 6 as outputs*/
    DDRD = (1 << PORTD6);
    DDRD &= ~(1 << PORTD7);
    /* set port B as output*/
    DDRC &= 0xF9;

    /* relevant bitshift to activate pullup resistor*/
    PORTC = (1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3);

    /*clear the LCD screen*/
    lcd_clrscr();

    /*create the bar and relevant messages*/
    // int current_level = 4;
    int current_mode = 1;

    /*Set Up Interrupts*/
    PCICR = (1 << PCIE1);

    // Trigger interrupts on changes to the following pins (C0, C1, C2, C3)
    PCMSK1 = (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);

    /* set output compare values*/
    OCR0A = 0;
    change_mode(current_mode);
    change_brightness(current_level);

    /* set up timer/counter 0 for past PWM, set on compare match */
    TCCR0A = (1 << COM0A0) | (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS00);
    /*UART I2C INIT*/
    // uint8_t UBBRValue = FCPU/(16*BAUD)-1;

    // uint8_t UBBRValue = 49;
    uart_init(12);
    i2c_init();
    ALRT_init();

    uint16_t ads1115ConfigVoltage = 0b11 << 0 | 0b111 << 5 | 0b0 << 8 | 0b010 << 9 | 0b011 << 12 | 0b1 << 15;
    uint16_t ads1115ConfigRes = 0b11 << 0 | 0b111 << 5 | 0b0 << 8 | 0b001 << 9 | 0b100 << 12 | 0b1 << 15;
    uint16_t ads1115ConfigRes1k = 0b11 << 0 | 0b111 << 5 | 0b0 << 8 | 0b101 << 9 | 0b100 << 12 | 0b1 << 15;

    int16_t dataBinary;
    double sum = 0;
    char buff[16];
    char buff2[16];

    char voltageString[8];
    char thresh[8];

    /*
    contThresh = 20;
    sprintf(cont, "%.1f", (double)contThresh/10);
    */
    contThresh = eeprom_read_byte((const uint8_t*)20);
    if (contThresh > 200 || contThresh < 0) {
        eeprom_update_byte((uint8_t*)20, 20);
    }

    sei();

    // resistance parameters

    /* Replace with your application code */
    while (1) {
        if (hold == 1) {
            // print hold icon
            lcd_gotoxy(19, 0);
            lcd_putc(5);
            uart_transmit('H');
            uart_newline();
            _delay_ms(400);
        } else {
            lcd_gotoxy(19, 0);
            lcd_putc(' ');

            lcd_gotoxy(18, 0);
            lcd_putc(' ');

            if ((PIND) & (1 << PIND7)) {
                lcd_gotoxy(18, 0);
                lcd_putc(4);
            }

            if (calc == 1) {
                /**************************************************************/
                /*					DC VOLTAGE								  */

                MCUCR &= ~(1 << PUD);
                dataBinary = ads1115_read_DIFF_A2_A3(ADS1115_ADDR, ads1115ConfigVoltage);
                dataVoltage = (dataBinary + 137.4546) / 1175.4182;
                if (dataVoltage > voltMaxDC) {
                    voltMaxDC = dataVoltage;
                }
                if (dataVoltage < voltMinDC) {
                    voltMinDC = dataVoltage;
                }

                sprintf(voltageString, "%.3f", dataVoltage);
                sprintf(buff2, "%.3f", voltMinDC);
                sprintf(buff, "%.3f", voltMaxDC);
                uart_transmit('D');
                uart_transmit(':');
                for (int i = 0; i < sizeof(voltageString); i++) {
                    uart_transmit(voltageString[i]);
                }
                uart_transmit(':');
                for (int i = 0; i < sizeof(buff2); i++) {
                    uart_transmit(buff2[i]);
                }
                uart_transmit(':');
                for (int i = 0; i < sizeof(buff); i++) {
                    uart_transmit(buff[i]);
                }
                uart_transmit(':');
                uart_transmit(current_level + '0');

                lcd_gotoxy(0, 2);
                lcd_puts("Min:");
                lcd_puts(buff2);
                lcd_gotoxy(11, 2);
                lcd_puts("Max:");
                lcd_puts(buff);

                // put reading
                lcd_gotoxy(0, 1);
                lcd_puts(voltageString);
                lcd_gotoxy(9, 1);
                lcd_puts("V");
                memset(voltageString, 0, sizeof(voltageString));
                memset(buff2, 0, sizeof(buff2));
                memset(buff, 0, sizeof(buff));
                uart_newline();
                _delay_ms(400);
            } else if (calc == 2) {
                /*************************************************************/
                /*					AC VOLTAGE								 */
                MCUCR &= ~(1 << PUD);
                sum = 0;
                for (int i = 0; i < 100; i++) {
                    dataBinary = ads1115_read_DIFF_A2_A3(ADS1115_ADDR, ads1115ConfigVoltage);
                    // dataVoltage = ((double)dataBinary-32.0)/1006.4;
                    dataVoltage = (dataBinary + 137.4546) / 1175.4182;
                    sum += (dataVoltage * dataVoltage);
                }
                sum = sum / 100;
                final = sqrt(sum);
                if (final > voltMaxAC) {
                    voltMaxAC = final;
                }
                if (final < voltMinAC) {
                    voltMinAC = final;
                }
                // transmit voltage string
                sprintf(voltageString, "%.3f", final);
                sprintf(buff, "%.3f", voltMaxAC);
                sprintf(buff2, "%.3f", voltMinAC);
                // transmit packet
                uart_transmit('A');
                uart_transmit(':');
                for (int i = 0; i < sizeof(voltageString); i++) {
                    uart_transmit(voltageString[i]);
                }
                uart_transmit(':');
                for (int i = 0; i < sizeof(buff2); i++) {
                    uart_transmit(buff2[i]);
                }
                uart_transmit(':');
                for (int i = 0; i < sizeof(buff); i++) {
                    uart_transmit(buff[i]);
                }
                uart_transmit(':');
                uart_transmit(current_level + '0');

                uart_newline();
                lcd_gotoxy(0, 1);
                lcd_puts(voltageString);
                lcd_gotoxy(6, 1);
                lcd_puts("VRMS");
                lcd_gotoxy(0, 2);
                lcd_puts("Min:");
                lcd_puts(buff2);
                lcd_gotoxy(11, 2);
                lcd_puts("Max:");
                lcd_puts(buff);
                memset(voltageString, 0, sizeof(voltageString));
                memset(buff2, 0, sizeof(buff2));
                memset(buff, 0, sizeof(buff));
            } else if (calc == 4 || calc == 3) {
                /*************************************************************/
                /*					  CONTINUITY & resistance							*/
                if (calc == 3) {
                    uart_transmit('R');
                    uart_transmit(':');
                } else if (calc == 4) {
                    uart_transmit('C');
                    uart_transmit(':');
                }

                if (calc == 3) {
                    lcd_gotoxy(0, 2);
                    lcd_puts("                    ");
                }
                DDRD |= (1 << DDD3); // 100k resistor OUPUT
                PORTD |= (1 << PIND3); // 100k resistor HIGH
                DDRD &= ~(1 << PIND4); // 1k resistor INPUT - high Zin
                MCUCR |= (1 << PUD); // hopefully hi zin??

                for (int i = 0; i < 5; i++) {
                    dataBinary = ads1115_read_SE(ADS1115_ADDR, ads1115ConfigRes);
                }
                // dataVoltage = (dataBinary+39.9944)/8096.948;
                resVoltage = (dataBinary + 97.4) / 8120.866;

                if (resVoltage > 0.14792) {
                    resVoltage = ((-100280 * resVoltage) / (resVoltage - 3.23)) - resOffset;
                    if (resVoltage > 2000000)
                        if (resVoltage > 970000) {
                            resVoltage = 1000000;
                        }
                    if (calc == 4) {
                        lcd_gotoxy(0, 2);
                        lcd_puts("           ");
                        lcd_gotoxy(0, 2);
                        lcd_puts("OPEN");
                    }
                    if (log10(resVoltage) < 6 && log10(resVoltage) > 5) {
                        sprintf(voltageString, "%.3f", resVoltage);
                        uart_transmit(voltageString[0]);
                        uart_transmit(voltageString[1]);
                        uart_transmit(voltageString[2]);
                        uart_transmit('k');
                        // lcd
                        lcd_gotoxy(0, 1);
                        lcd_puts("          ");
                        lcd_gotoxy(0, 1);
                        lcd_putc(voltageString[0]);
                        lcd_putc(voltageString[1]);
                        lcd_putc(voltageString[2]);
                        lcd_putc('k');
                        lcd_putc(244);
                        uart_transmit('k');
                    } else if (log10(resVoltage) < 4 && log10(resVoltage) > 3) {
                        sprintf(voltageString, "%.3g", resVoltage);
                        uart_transmit(voltageString[0]);
                        uart_transmit(voltageString[1]);
                        uart_transmit(voltageString[2]);
                        uart_transmit(voltageString[3]);
                        uart_transmit('k');
                        // lcd
                        lcd_gotoxy(0, 1);
                        lcd_puts("          ");
                        lcd_gotoxy(0, 1);
                        lcd_putc(voltageString[0]);
                        lcd_putc(voltageString[1]);
                        lcd_putc(voltageString[2]);
                        lcd_putc(voltageString[3]);
                        lcd_putc('k');
                        lcd_putc(244);
                    } else if (log10(resVoltage) < 5 && log10(resVoltage) > 4) {
                        sprintf(voltageString, "%.3f", resVoltage);
                        uart_transmit(voltageString[0]);
                        uart_transmit(voltageString[1]);
                        uart_transmit('.');
                        uart_transmit(voltageString[2]);
                        uart_transmit('k');
                        // lcd
                        lcd_gotoxy(0, 1);
                        lcd_puts("          ");
                        lcd_gotoxy(0, 1);
                        lcd_putc(voltageString[0]);
                        lcd_putc(voltageString[1]);
                        lcd_putc('.');
                        lcd_putc(voltageString[2]);
                        lcd_putc('k');
                        lcd_putc(244);
                    } else if (log10(resVoltage) >= 6) {
                        sprintf(voltageString, "%.3f", resVoltage);
                        uart_transmit(voltageString[0]);
                        uart_transmit('.');
                        uart_transmit(voltageString[1]);
                        uart_transmit(voltageString[2]);
                        uart_transmit('M');
                        // lcd
                        lcd_gotoxy(0, 1);
                        lcd_puts("          ");
                        lcd_gotoxy(0, 1);
                        lcd_putc(voltageString[0]);
                        lcd_putc('.');
                        lcd_putc(voltageString[1]);
                        lcd_putc(voltageString[2]);
                        lcd_putc('M');
                        lcd_putc(244);
                    }

                } else {
                    // 4k and under
                    for (int i = 0; i < 5; i++) {
                        dataBinary = ads1115_read_SE(ADS1115_ADDR, ads1115ConfigRes1k);
                    }
                    if (dataBinary < 81) {
                        // 15 ohms and less - change refernece resistor by ddrd
                        // change pin modes
                        DDRD &= ~(1 << DDD3); // 100k resistor INPUT - high ZIN

                        DDRD |= (1 << PIND4); // 1k resistor OUTPUT
                        PORTD |= (1 << PIND4); // 1k resistor HIGH
                        MCUCR |= (1 << PUD); // hopefully hi zin??
                        for (int i = 0; i < 5; i++) {
                            dataBinary = ads1115_read_SE(ADS1115_ADDR, ads1115ConfigRes1k);
                        }

                        resVoltage = (dataBinary - 63.17906) / 128272.823;
                        resVoltage = (-996 * resVoltage) / (resVoltage - 3.25) - resOffset;

                        if (resVoltage < (double)contThresh / 10 && calc == 4) {
                            lcd_gotoxy(0, 2);
                            lcd_puts("           ");
                            lcd_gotoxy(0, 2);
                            lcd_puts("SHORT");
                        } else if (calc == 4) {
                            lcd_gotoxy(0, 2);
                            lcd_puts("           ");

                            lcd_gotoxy(0, 2);
                            lcd_puts("OPEN");
                        }
                        sprintf(voltageString, "%.3g", resVoltage);
                        for (int i = 0; i < sizeof(voltageString); i++) {
                            uart_transmit(voltageString[i]);
                        }
                        lcd_gotoxy(0, 1);
                        lcd_puts("          ");
                        lcd_gotoxy(0, 1);
                        lcd_puts(voltageString);
                        lcd_putc(244);

                    } else {
                        // above 15 ohms
                        resVoltage = (dataBinary + 2.85064) / 128227.4618;
                        resVoltage = ((-100280 * resVoltage) / (resVoltage - 3.25)) - resOffset;
                        if (log10(resVoltage) >= 3) {
                            sprintf(voltageString, "%.3g", resVoltage);
                            uart_transmit(voltageString[0]);
                            uart_transmit(voltageString[1]);
                            uart_transmit(voltageString[2]);
                            uart_transmit(voltageString[3]);
                            uart_transmit('k');
                            // lcd
                            lcd_gotoxy(0, 1);
                            lcd_puts("          ");
                            lcd_gotoxy(0, 1);
                            lcd_putc(voltageString[0]);
                            lcd_putc(voltageString[1]);
                            lcd_putc(voltageString[2]);
                            lcd_putc(voltageString[3]);
                            lcd_putc('k');
                            lcd_putc(244);

                        } else {
                            sprintf(voltageString, "%.3f", resVoltage);
                            uart_transmit(voltageString[0]);
                            uart_transmit(voltageString[1]);
                            uart_transmit(voltageString[2]);
                            // lcd
                            lcd_gotoxy(0, 1);
                            lcd_puts("          ");
                            lcd_gotoxy(0, 1);
                            lcd_putc(voltageString[0]);
                            lcd_putc(voltageString[1]);
                            lcd_putc(voltageString[2]);
                            lcd_putc(244);
                        }
                        if (calc == 4) {
                            lcd_gotoxy(0, 2);
                            lcd_puts("               ");
                            lcd_gotoxy(0, 2);
                            lcd_puts("OPEN");
                        }
                    }
                }
                if (calc == 4) {
                    sprintf(thresh, "%.1f", (double)(eeprom_read_byte((const uint8_t*)20)) / 10);
                    lcd_gotoxy(8, 2);
                    lcd_puts("        ");
                    lcd_gotoxy(8, 2);
                    lcd_puts(thresh);
                    lcd_putc(244);
                }
                // transmit rest of packet
                uart_transmit(':');
                uart_transmit(current_level + '0');

                memset(voltageString, 0, sizeof(voltageString));
                memset(thresh, 0, sizeof(thresh));
                uart_newline();
                _delay_ms(200);
            }
        }
    }
}

void uart_newline(void)
{
    uart_transmit('\n');
    uart_transmit('\r');
    return;
}
/*******************************************************************/
/*                      UART FUNCTIONS                             */
void uart_init(uint8_t ubrr)
{
    /*initialise uart on ATMega328P*/
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    // UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    // UCSR0C = (1<<USBS0)|(3<<UCSZ00);
    UCSR0B = (1 << TXEN0);
    UCSR0B |= (1 << RXCIE0) | (1 << RXEN0);
    UCSR0A |= (1 << U2X0);

    UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}
void uart_transmit(unsigned char data)
{
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    /* Put data into buffer, sends the data */
    UDR0 = data;
}
unsigned char uart_receive(void)
{
    /* Wait for data to be received */
    while (!(UCSR0A & (1 << RXC0)))
        ;

    /* Get and return received data from buffer */
    return UDR0;
}
ISR(USART_RX_vect)
{
    unsigned char dummy;
    dummy = uart_receive();
    if (dummy == 'x') {
        voltMaxAC = final;
        voltMinAC = final;
        voltMaxDC = dataVoltage;
        voltMinDC = dataVoltage;
        // transmit final and datavoltage? or do if calc == 1 if calc == 2?

    } else if (dummy == 'R') {
        change_mode(3);
    } else if (dummy == 'C') {
        change_mode(4);
    } else if (dummy == 'A') {
        change_mode(2);
    } else if (dummy == 'D') {
        change_mode(1);
    } else if (dummy == '+') {
        // increase contThresh
        contThresh = eeprom_read_byte((const uint8_t*)20);
        contThresh++;
        if (contThresh > 200) {
            contThresh = 200;
        }
        eeprom_update_byte((uint8_t*)20, contThresh);
    } else if (dummy == '-') {
        // decrease contThresh
        contThresh = eeprom_read_byte((const uint8_t*)20);
        contThresh--;
        if (contThresh > 200 || contThresh < 0) {
            contThresh = 20;
        }
        eeprom_update_byte((uint8_t*)20, contThresh);
    } else if (dummy == '1') {
        // set backlight to 1
        if (dummy)
            change_brightness(0);
        current_level = 0;
        OCR0A = 255;
    } else if (dummy == '2') {
        change_brightness(1);
        current_level = 1;
        OCR0A = 189;
    } else if (dummy == '3') {
        change_brightness(2);
        current_level = 2;
        OCR0A = 126;
    } else if (dummy == '4') {
        change_brightness(3);
        current_level = 3;
        OCR0A = 63;
    } else if (dummy == '5') {
        change_brightness(4);
        current_level = 4;
        OCR0A = 0;
    } else if (dummy == 'H') {
        hold = 1;
    } else if (dummy == 'N') {
        hold = 0;
    } else if (dummy == 'm') {
        unsigned char x;
        lcd_gotoxy(0, 3);
        uint8_t i = 0;
        while ((x = uart_receive()) != '\n') {
            lcd_putc(x);
            i++;
        }
        while (i < 16) {
            lcd_putc(' ');
            i++;
        }
    }
}

/*******************************************************************/
/*                    ADS1115 FUNCTIONS                            */
void ads1115_write(uint8_t addr, uint8_t pointerReg, uint16_t configReg)
{
    /*
     * write bytes to the config register for reading after
     */
    // send start condition and wait
    i2c_start_wait((addr << 1) + I2C_WRITE);
    // write to pointer reg - config register setting
    i2c_write(pointerReg);
    // write data for config reg
    i2c_write(configReg >> 8);
    i2c_write((configReg && 0xFF));
    // send stop condition
    i2c_stop();

    return;
}
uint16_t ads1115_read(uint8_t addr, uint8_t pointerReg)
{
    /*
     * read from conversion register from ads1115
     */
    // send start condition and wait
    i2c_start_wait((addr << 1) + I2C_WRITE);
    // write to the point reg - conversion register setting
    i2c_write(pointerReg);
    // send stop
    i2c_stop();
    // read 16 bit adc conversion
    i2c_rep_start((addr << 1) + I2C_READ);
    uint8_t MSB = i2c_readAck();
    uint8_t LSB = i2c_readNak();
    // send stop
    i2c_stop();

    uint16_t data = (MSB << 8 | LSB);
    return data;
}
uint16_t ads1115_read_SE(uint8_t addr, uint16_t configReg)
{
    /*
     * Read from channel 0 on ads1115 for a given config reg
     */
    ads1115_write(addr, ADS1115_REG_CONFIG, configReg);
    _delay_ms(8);
    return ads1115_read(addr, ADS1115_REG_CONVERSION);
}
int16_t ads1115_read_DIFF_A2_A3(uint8_t addr, uint16_t configReg)
{
    /*
     * Read from channel 2 and 3 in diff mode
     */
    ads1115_write(addr, ADS1115_REG_CONFIG, configReg);
    //_delay_ms(8);
    return (int16_t)ads1115_read(addr, ADS1115_REG_CONVERSION);
}
void ALRT_init(void)
{
    DDRD &= 0xF3;
    DDRD |= 0xF0;

    PORTD = 0x0C;
    EICRA = (0 << ISC00) | (1 << ISC01);
    EIMSK = (1 << INT0);
}

ISR(INT0_vect)
{
    if (calc == 1 || calc == 2) {
        voltMaxAC = final;
        voltMinAC = final;
        voltMaxDC = dataVoltage;
        voltMinDC = dataVoltage;
    } else if (calc == 3) {
        resOffset = resVoltage;
    }
}

void EEPROM_write(unsigned int uiAdress, uint8_t ucData)
{
    // wait for completion of previous write
    while (EECR & (1 << EEPE))
        ;
    EEAR = uiAdress;
    EEDR = ucData;
    // write '1' to EEMPE
    EECR |= (1 << EEMPE);
    // start EEPROM write
    EECR |= (1 << EEPE);
}

uint8_t EEPROM_read(unsigned int uiAddress)
{
    // wait for completion of previous write
    while (EECR & (1 << EEPE))
        ;

    EEAR = uiAddress;
    // start EEPROM read
    EECR |= (1 << EERE);

    return EEDR;
}



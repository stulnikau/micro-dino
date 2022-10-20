#include <stdint.h>
#include <stdio.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdarg.h>
#include <LiquidCrystal.h>

#define EVER ;;

#define EMPTY_ROW 0b00000000
#define no_obstacle runner_area[1] == 32 || runner_area[1] == 0

char runner_area[16] = {32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32}; // 32 = space

uint8_t bmp3[10] = {EMPTY_ROW,
                   0b00111100,
                   0b00000010,
                   0b00000010,
                   0b00111100,
                   0b00000010,
                   0b00000010,
                   0b00000010,
                   0b00111100,
                   EMPTY_ROW};

uint8_t bmp2[10] = {EMPTY_ROW,
                   0b00011100,
                   0b00100010,
                   0b00000010,
                   0b00000100,
                   0b00001000,
                   0b00010000,
                   0b00100000,
                   0b00111110,
                   EMPTY_ROW};

uint8_t bmp1[10] = {EMPTY_ROW,
                   0b00000010,
                   0b00000110,
                   0b00001010,
                   0b00010010,
                   0b00000010,
                   0b00000010,
                   0b00000010,
                   0b00000010,
                   EMPTY_ROW};

uint8_t runner[8] = {
                0b00000,
                0b00111,
                0b00111,
                0b10110,
                0b11111,
                0b01010,
                0b01001,
                0b00000
                };

uint8_t obstacle[8] = {
                0b00100,
                0b10100,
                0b11101,
                0b00101,
                0b10111,
                0b11100,
                0b00100,
                0b00000
                };

// void device_setup(void);
void uart_init(void);
void uart_putbyte(unsigned char data);
int uart_getbyte(unsigned char *buffer);
void uart_printf(const char* format_text, ...);
void uart_receive_chars(char* buff, int buff_len);
void game_loop(void);
void exit_screen(void);
void button_press_left(void);
void button_press_right(void);

unsigned long prev_ms = 0; // ms
unsigned long jump_time = 0; // ms
int jump_dur = 500; // ms

unsigned long global_clock = 0; // ms since startup

int scroll_speed = 300; // step size in milliseconds
char jump = 32;
int score = 0;
int stop_updates_to_score = 0;

uint8_t pwm_comp = (uint8_t) (0.36 * 255); // DC% = sn/2 + 25. n10585222 => sn = 22. DC% = 22/2 + 25 = 36%

uint8_t en = 8, rs = 2;
LiquidCrystal lcd(rs, en, 4, 5, 6, 7);

int top_score = 0;
int num_rounds = 1;
unsigned char inp;

void device_setup(void) {
    //  ******************************************
    //     DIGITAL I/O: Switches
    //  ******************************************
    //
    //  Enable the I/O pins labelled A0, A1, A2 for digital input
    //  from buttons LEFT, SELECT and RIGHT.
    #define LEFT 2
    #define SELECT 1
    #define RIGHT 0
    
    DDRC &= ~(1 << RIGHT);
    DDRC &= ~(1 << SELECT);
    DDRC &= ~(1 << LEFT);


    DDRD |= (1 << PD3); // For software PWM

    //  ******************************************
    //     DIGITAL I/O: Interrupt-based debouncing
    //  ******************************************
    //
    //  Debounce switches LEFT, SELECT and RIGHT using interrupts.
    //  Use Timer2 because Timer0 is reserved for the LCD.
    //
    //  Initialise Timer2 in normal mode so that it overflows 
    //  with a period of approximately 0.000128 seconds (prescaler = 8)
    TCCR2A = 0;
    TCCR2B = (1 << CS21);

    //  Enable timer overflow interrupt for Timer 2.
    TIMSK2 = 1;

    //  Set interrupts.
    //  sei();

    //  ******************************************
    //     ANALOG INPUT: Potentiometer
    //  ******************************************
    //
	// ADC Enable and pre-scaler of 128
    // ADEN  = 1
    // ADPS2 = 1, ADPS1 = 1, ADPS0 = 1
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // select channel and ref input voltage
    // channel 5, PC5 (A5 on the uno)
    // MUX0=1, MUX1=0, MUX2=1, MUX3=0
    // REFS0=1
    // REFS1=0
    ADMUX = (1 << REFS0);
    ADMUX |= 0b00000101;

    // //  ******************************************
    // //     LED Matrix Timer1
    // //  ******************************************
    // //
    // //  Set up a very fast Timer1 in normal mode with no prescaler to achieve
    // //  an overflow period of 0.000016s
    // TCCR1A = 0;
    // TCCR1B = (1 << CS10);

    // //  Enable timer overflow interrupt for Timer1
    // TIMSK1 = 1;

    //  Set interrupts.
    sei();
}

void matrix_setup(void) {
    DDRB |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 4); // Set led matrix output ports
    DDRC |= (1 << 3) | (1 << 4); // Clock and reset pins

    // Reset the decade counter by signalling to the reset input for a short while.
    PORTC |= (1 << 3);
    PORTB &= ~(1 << 3);

}

//  Define an interrupt service routine to process timer overflow
//  interrupts for Timer2. Every time the interrupt service
//  routine is called, each button's switch counter should:
//      Left-shift switch counter one place.
//      Bitwise AND the switch counter with a mask in which the 
//          5 bits on the right are 1 and the others are 0.
//      Use bitwise OR to add the current open/closed value of the 
//          corresponding I/O pin (A0, A1, A2) to the history.
//      If the switch counter is equal to the bit mask, then the switch has been 
//          observed 5 times in a row to be closed. Assign 1 to 
//          pressed, indicating that the switch is now officially "closed".
//      If the switch counter is equal to 0, then the switch has been observed 
//          to be open at least 5 in a row, so store 0 in pressed, 
//          indicating that the switch should now be considered to be officially "open".

volatile uint8_t switch_counter_left = 0;
volatile uint8_t switch_counter_select = 0;
volatile uint8_t switch_counter_right = 0;

volatile uint8_t pressed_left = 0;
volatile uint8_t pressed_select = 0;
volatile uint8_t pressed_right = 0;

volatile uint8_t prevState_left = 0;
volatile uint8_t prevState_select = 0;
volatile uint8_t prevState_right = 0;

volatile uint16_t prevState_ADC = 1000;

volatile uint8_t ISRcounter = 0;
unsigned long cycle_count = 0; // Total number of overflow interrupts since startup

#define FULLY_PRESSED 0b00011111

ISR(TIMER2_OVF_vect) {
    switch_counter_left <<= 1;
    switch_counter_select <<= 1;
    switch_counter_right <<= 1;

    switch_counter_left &= FULLY_PRESSED;
    switch_counter_select &= FULLY_PRESSED;
    switch_counter_right &= FULLY_PRESSED;

    switch_counter_left |= ((PINC >> LEFT) & 1);
    switch_counter_select |= ((PINC >> SELECT) & 1);
    switch_counter_right |= ((PINC >> RIGHT) & 1);

    //  Left switch checks
    if (switch_counter_left == FULLY_PRESSED) {
        pressed_left = 1;
    }
    else if (switch_counter_left == 0) {
        pressed_left = 0;
    }
    if ((pressed_left == 1) & (prevState_left == 0)) {
        button_press_left();
    }
    prevState_left = pressed_left;

    //  Select switch checks
    if (switch_counter_select == FULLY_PRESSED) {
        pressed_select = 1;
    }
    else if (switch_counter_select == 0) {
        pressed_select = 0;
    }

    prevState_select = pressed_select;

    //  Right switch checks
    if (switch_counter_right == FULLY_PRESSED) {
        pressed_right = 1;
    }
    else if (switch_counter_right == 0) {
        pressed_right = 0;
    }
    if (pressed_right > prevState_right) {
        button_press_right();
    }
    prevState_right = pressed_right;

    //  Reading off ADC
    //  Start single conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1 << ADSC);

	//  Wait for ADSC bit to clear, signalling conversion complete.
	while (ADCSRA & (1 << ADSC)) {}

	// Result now available in ADC
    // ADC returns value between 0 and 1000
	uint16_t adc_val = ADC;

    if (adc_val != prevState_ADC) {
        if (adc_val <= 250) {
            lcd.setCursor(0,0);
            lcd.print("Slow     ");
            scroll_speed = 300;
            jump_dur = 500;
        }
        else if (adc_val <= 500) {
            lcd.setCursor(0,0);
            lcd.print("Medium   ");
            scroll_speed = 200;
            jump_dur = 380;
        }
        else if (adc_val <= 750) {
            lcd.setCursor(0,0);
            lcd.print("Fast     ");
            scroll_speed = 100;
            jump_dur = 180;
        }
        else {
            lcd.setCursor(0,0);
            lcd.print("Very fast");
            scroll_speed = 40;
            jump_dur = 90;
        }
        prevState_ADC = adc_val;
    }

    // Software PWM
    if(ISRcounter < pwm_comp) {
		PORTD |= (1 << 3); // Set port B3 high
	}
    else {
		PORTD &= ~(1 << 3); // Set low
	}
    ISRcounter++;

    //  Clock
    cycle_count++;
}

// volatile uint8_t ISRTimer1Counter;

// ISR(TIMER1_OVF_vect) {
//     ISRTimer1Counter++;
// }

// uint8_t wait_for_counter;

// int delay_16microseconds(void) {
//     wait_for_counter = ISRTimer1Counter + 1;
//     while (ISRTimer1Counter < wait_for_counter) {} // wait for ISRTimer1Counter to update
//     return 1;
// }

//  Control buttons
void button_press_left(void) {
    exit_screen();
    _delay_ms(1000);
    num_rounds = 0;
}

void button_press_right(void) {
    num_rounds++;
}

void uart_init(void) {
    UBRR0 = F_CPU / 16 / 9600 - 1;
    UCSR0A = 0;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (3 << UCSZ00);
}

void uart_putbyte(unsigned char data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));

    // Send data by assigning into UDR0
    UDR0 = data;
}

//  Formatted output to serial
char serial_buffer[100];
void uart_printf(const char* format_text, ...) {
    va_list format_vars; // List of arguments
    va_start(format_vars, format_text); // Initialise format_vars to retrieve all arguments after format_text
    vsnprintf(serial_buffer, sizeof(serial_buffer), format_text, format_vars); // Store a string in the buffer, formatted as if it was in printf. In other words, printf to buffer.
    for (int i = 0; serial_buffer[i]; i++) {
        uart_putbyte(serial_buffer[i]);
    }
    va_end(format_vars); // Clear memory reserved for the argument list
}

int uart_getbyte(unsigned char *buffer) {
    // If receive buffer contains data...
    if (UCSR0A & (1 << RXC0)) {
        // Copy received byte from UDR0 into memory location (*buffer)
        *buffer = UDR0;
        // 
        return 1;
    }
    else {
        return 0;
    }
}

void uart_receive_chars(char* buff, int buff_len) {
    int i = 0;
    unsigned char ch;
    for(EVER) {
        while(!uart_getbyte(&ch)) {}
        if (ch == 0) {
            break;
        }
        if (i < (buff_len - 1)) {
            buff[i] = ch;
            i++;
        }
    }
    buff[i] = 0;
}

void serial_greeting(void) {
    //  ******************************************
    //     Serial greeting
    //  ******************************************

    char str_top_score[5]; // for some reason uart_printf does not support %d formatting so I have to use a string to display it...
    itoa(top_score, str_top_score, 10);

    uart_printf("Welcome to MicroDino!\n");
    uart_printf("The current top score is %s\n", str_top_score);
    uart_printf("Please select an option (a-d):\n");
    uart_printf("a) Play 1 round.\n");
    uart_printf("b) Play 2 rounds.\n");
    uart_printf("c) Play 3 rounds.\n");
    uart_printf("d) End game.\n");
    uart_printf("Best of luck!\n");

    while(!(UCSR0A & (1 << RXC0))) {_delay_ms(100);} // Wait for user input
    uart_getbyte(&inp);

    uart_printf("Selected option: %c\n", inp);

    if (inp == 'a') {
        num_rounds = 1;
    }
    else if (inp == 'b') {
        num_rounds = 2;
    }
    else if (inp == 'c') {
        num_rounds = 3;
    }
    else if (inp == 'd') {
        num_rounds = 0;
    }
    else {uart_printf("Invalid selection.\n");}

}

void matrix_display_bmp3(void) {
    for (int frame = 0; frame < 60; frame++) {
        for (int row = 0; row < 10; row++) {
            PORTB |= bmp3[row];
            _delay_ms(1);
            PORTB &= ~(0b00111110); // Clear row
            PORTC |= (1 << 4); // Set clock to high and back to low to move to the next row.
            PORTC &= ~(1 << 4);
        }
    }
}

void matrix_display_bmp2(void) {
    for (int frame = 0; frame < 60; frame++) {
        for (int row = 0; row < 10; row++) {
            PORTB |= bmp2[row];
            _delay_ms(1);
            PORTB &= ~(0b00111110); // Clear row
            PORTC |= (1 << 4); // Set clock to high and back to low to move to the next row.
            PORTC &= ~(1 << 4);
        }
    }
}

void matrix_display_bmp1(void) {
    for (int frame = 0; frame < 60; frame++) {
        for (int row = 0; row < 10; row++) {
            PORTB |= bmp1[row];
            _delay_ms(1);
            PORTB &= ~(0b00111110); // Clear row
            PORTC |= (1 << 4); // Set clock to high and back to low to move to the next row.
            PORTC &= ~(1 << 4);
        }
    }
}

void lcd_greeting(void) {
    lcd.createChar(0, runner);
    lcd.createChar(1, obstacle);

    lcd.begin(16,2);

    lcd.setCursor(5,0);
    lcd.print("Welcome to");
    _delay_ms(1000);
    for (int i = 0; i < 5; i++) {
        lcd.scrollDisplayLeft();
      	_delay_ms(300);
    }
    lcd.setCursor(5,1);
    lcd.print("MicroDino!");
    _delay_ms(1500);
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("Follow serial to");
    lcd.setCursor(0,1);
    lcd.print("play");

}

void exit_screen(void) {
    lcd.clear();
    lcd.print("See you soon!");
}

void update_lcd() {
    for (int i = 0; i <= 15; i++) {
        lcd.setCursor(i, 1);
        lcd.write(runner_area[i]);
    }
    lcd.setCursor(1, 0);
    lcd.write(jump);
}

void draw_bounds() {
    runner_area[0] = 32;
    runner_area[15] = 32;
}

char cur_score[5];
void print_score() {
    lcd.setCursor(11, 0);
    itoa(score, cur_score, 10);
    lcd.print(cur_score);
}

void game_over() {
    lcd.setCursor(4, 1);
    lcd.print("Game over!");
    if (score > top_score) {
        top_score = score;
        char new_top_score[5]; // for some reason uart_printf does not support %d formatting so I have to use a string to display it...
        itoa(top_score, new_top_score, 10);
        uart_printf("The new top score is %s. Good job!\n", new_top_score);
    }
    score = 0;
    _delay_ms(2500);
    num_rounds--;
}

unsigned long get_ms() {
    return (unsigned long) ((cycle_count * 256.0 + TCNT2) * 8 / 16000.0);
}

void game_loop(void) {
    while (num_rounds > 0) {
        lcd.clear();
        lcd.print("Counting down...");
        // _delay_ms(1000);
        matrix_display_bmp3();
        // _delay_ms(1000);
        matrix_display_bmp2();
        // lcd.clear();
        // _delay_ms(1000);
        matrix_display_bmp1();
        lcd.print("Go!");
        _delay_ms(300);
        lcd.clear();

        for (EVER) {
            unsigned long cur_ms_cp = get_ms();
            if (cur_ms_cp - prev_ms >= (unsigned long) scroll_speed) {
                prev_ms = cur_ms_cp;
                if (rand() % 10 > 8) {
                    runner_area[15] = 1;
                } else {
                    runner_area[15] = 32;
                }
                for (int i = 0; i <= 15; i++) {
                    runner_area[i] = runner_area[i + 1];
                }
                if (stop_updates_to_score == 0) {
                    score++;
                }
            }
            draw_bounds();

            if (pressed_select == 1) {
                if ((runner_area[1] != 32) && (runner_area[1] != 1)) {
                runner_area[1] = 32;
                }
                jump = 0;
                stop_updates_to_score = 1;
                jump_time = get_ms();
            }
            if (get_ms() - jump_time >= (unsigned long) jump_dur) {
                if (no_obstacle) {
                runner_area[1] = 0;
                jump = 32;
                stop_updates_to_score = 0;
                } else {
                game_over();
                break;
                }
            }
            update_lcd();
            print_score();
        }
    }
    exit_screen();
}

int main() {
    //  ******************************************
    //     Initialisation sequence
    //  ******************************************
    uart_init(); // UART setup
    device_setup(); // Data direction registers and interrupts
    lcd_greeting();
    serial_greeting();
    game_loop();
    return 0;
}



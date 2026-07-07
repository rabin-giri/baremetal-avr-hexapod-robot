#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h> // Included for abs() function used in tilt()

// --- UART & Clock Configuration ---
#define USART_BAUDRATE 9600
#define F_CPU 8000000
#define BAUD_PRESCALE ((( F_CPU / ( USART_BAUDRATE * 16UL))) - 1)

// --- PWM & Timer Configuration ---
#define TOP 20000          // 20ms period for 50Hz servo refresh rate
#define ON 1
#define OFF 0

// --- Servo Pulse Limits (in microseconds) ---
#define wlmax 1650         // Peak pulse width during walking (down & forward)
#define wlmin 1349         // Least pulse width during walking (up & backward)
#define wrmin 1650         // Peak pulse width during walking (up & backward)
#define wrmax 1349         // Least pulse width during walking (down & forward)
#define wlmax_ver 1700     // Vertical max
#define wlmin_ver 1299     // Vertical min
#define wrmin_ver 1700
#define wrmax_ver 1299

// --- System & Serial Command Macros ---
#define CYCLE_STEPS 4      // Steps per gait cycle
#define CENTER_POS 127     // 8-bit neutral center position for tracking

#define CMD_TRACK_FACE 232 // Trigger for facial tracking mode
#define CMD_FORWARD    'F'
#define CMD_BACKWARD   'B'
#define CMD_LEFT       'L'
#define CMD_RIGHT      'R'
#define CMD_LEAN_FWD   'M'
#define CMD_LEAN_BWD   'N'
#define CMD_TWIST_CW   'O'
#define CMD_TWIST_CCW  'P'
#define CMD_STAND_UP   'U'
#define CMD_STAND_DOWN 'V'
#define CMD_RESET_POS  '1'
#define CMD_TOGGLE_MTR '2'

// --- Global State & Flags ---
struct fla_g {
    char timer_flag;
    char cycle_complete_flag;
    char timer_OC_complete;
};

volatile struct fla_g flag;

// --- Custom Software PWM Arrays ---
volatile unsigned int servo_pos[14];     // Desired pulse widths (in us) for all 14 servos
volatile unsigned char port_valA[8];     // Pre-calculated port states for multiplexing
volatile unsigned char port_valB[8];
volatile unsigned char port_valC[8];
volatile unsigned int counter[8];        // Sorted timeline events for timer interrupts
volatile char count_down;                // Tracks current event in the PWM cycle
volatile char count;

// --- Gait Sequence Variables ---
volatile char total_step;
volatile char step;
volatile char cycle_count;
volatile char locator = 0;
volatile char x = CYCLE_STEPS;

// --- Tracking State Variables ---
volatile char prevx = CENTER_POS;
volatile char prevy = CENTER_POS;

// --- Function Prototypes ---
void left_forward(void);
void left_backward(void);
void right_forward(void);
void right_backward(void);
void arrange_servo(void);
void init_timer1(void);
void clear_counter(void);
void clear_servo_pos(void);
void forward(void);
void turn_left(void);
void turn_right(void);
void backward(void);
void init_uart(void);
void refresh_servo(void);
void track_face(unsigned char, unsigned char, unsigned char);
void tilt(unsigned char x, unsigned char y);
void stand(unsigned char rise);
void lean(unsigned char forw);
void twist(unsigned char right);
int LinearInterPos(unsigned char x, unsigned char flag);
int LinearInterNeg(unsigned char x, unsigned char flag);
int motor_Off_ON(void);
void USART_Transmit(unsigned char data);

// -------------------------------------------------------------------------------- //
// --- Timer Interrupt Service Routines (The Core PWM Engine) ---
// -------------------------------------------------------------------------------- //

// Triggers every 20ms. Sets all active servo pins HIGH to start the pulse.
ISR(TIMER1_COMPA_vect) {
    if (flag.timer_flag == ON) {
        PORTA = 0xFF;
        PORTB = 0xFF;
        PORTC = 0xFF;
        OCR1B = counter[0]; // Queue the shortest pulse width
        count_down = 0;
    }
}

// Triggers at specific pulse width durations. Pulls specific pins LOW.
ISR(TIMER1_COMPB_vect) {
    if (flag.timer_flag == ON) {
        OCR1B = counter[count_down + 1]; // Load the next shortest pulse width
        
        // Turn off only the pins whose pulse width has elapsed
        PORTA = port_valA[count_down];
        PORTB = port_valB[count_down];
        PORTC = port_valC[count_down++];
        
        flag.timer_OC_complete = ON;
    }
}

// -------------------------------------------------------------------------------- //
// --- Main Application Loop ---
// -------------------------------------------------------------------------------- //

int main() {
    char received = CENTER_POS;
    char receivedx;
    char receivedy;
    
    MCUCSR = (1 << JTD); // Disable JTAG to free up port pins for servos
    DDRA = 0xFF;         // Set Port A, B, and C as outputs for servos
    DDRB = 0xFF;
    DDRC = 0xFF;

    count_down = 0;
    total_step = 4;
    cycle_count = 0;
    step = 0;

    flag.timer_flag = ON;
    
    // Initialization Sequence
    clear_counter();
    init_timer1();
    init_uart();
    refresh_servo();
    arrange_servo();

    while (1) {
        // Only process serial commands after safely passing the active PWM generation phase
        if ((TCNT1 > 2000) && (flag.timer_OC_complete == ON)) {
            
            // Check if UART data is available
            if (!((UCSRA & (1 << RXC)) == 0)) {
                received = UDR;

                // --- Command Routing ---
                if (received == CMD_TRACK_FACE) {
                    if (!((UCSRA & (1 << RXC)) == 0))
                        receivedx = UDR;
                    if (!((UCSRA & (1 << RXC)) == 0))
                        receivedy = UDR;
                    
                    track_face(receivedx, receivedy, CENTER_POS);
                    arrange_servo();
                } 
                else if (received == CMD_FORWARD) {
                    USART_Transmit('f');
                    forward();
                } 
                else if (received == CMD_BACKWARD) {
                    USART_Transmit('b');
                    backward();
                } 
                else if (received == CMD_LEFT) {
                    USART_Transmit('l');
                    turn_left();
                } 
                else if (received == CMD_RIGHT) {
                    USART_Transmit('r');
                    turn_right();
                    arrange_servo();
                } 
                else if (received == CMD_LEAN_FWD) {
                    USART_Transmit('m');
                    lean(1);
                    arrange_servo();
                } 
                else if (received == CMD_LEAN_BWD) {
                    USART_Transmit('n');
                    lean(0);
                    arrange_servo();
                } 
                else if (received == CMD_TWIST_CW) {
                    USART_Transmit('o');
                    twist(1);
                    arrange_servo();
                } 
                else if (received == CMD_TWIST_CCW) {
                    USART_Transmit('p');
                    twist(0);
                    arrange_servo();
                } 
                else if (received == CMD_STAND_UP) {
                    USART_Transmit('u');
                    stand(1);
                    arrange_servo();
                } 
                else if (received == CMD_STAND_DOWN) {
                    USART_Transmit('v');
                    stand(0);
                    arrange_servo();
                } 
                else if (received == CMD_RESET_POS) {
                    USART_Transmit('1');
                    refresh_servo();
                    arrange_servo();
                } 
                else if (received == CMD_TOGGLE_MTR) {
                    USART_Transmit('2');
                    motor_Off_ON();
                }
            }
            flag.timer_OC_complete = OFF;
        }
    }
}

// -------------------------------------------------------------------------------- //
// --- Motion & Kinematics Control ---
// -------------------------------------------------------------------------------- //

// Sets all servos to 1500us (neutral 90-degree position)
void refresh_servo(void) {
    char i;
    for (i = 0; i < 14; i++) {
        servo_pos[i] = 1500;
    }
}

void forward(void) {
    left_forward();
    right_forward();

    if ((locator++) == (4 * x)) locator = 0;
    arrange_servo();
}

void backward(void) {
    left_backward();
    right_backward();

    if ((locator++) == (4 * x)) locator = 0;
    arrange_servo();
}

void turn_left(void) {
    left_backward();
    right_forward();

    if ((locator++) == (4 * x)) locator = 0;
    arrange_servo();
}

void turn_right(void) {
    left_forward();
    right_backward();

    if ((locator++) == (4 * x)) locator = 0;
    arrange_servo();
}

// -------------------------------------------------------------------------------- //
// --- Hardware Initialization ---
// -------------------------------------------------------------------------------- //

void init_timer1(void) {
    TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC Mode, Prescaler 8
    TIMSK |= (1 << OCIE1A) | (1 << OCIE1B);
    OCR1A = TOP;                          // Sets the 20ms period
    TCNT1 = TOP - 100;
    sei();                                // Enable global interrupts
}

void init_uart(void) {
    UCSRB |= (1 << RXEN) | (1 << TXEN);
    UCSRC |= (1 << UCSZ1) | (1 << UCSZ0) | (1 << URSEL);
    UBRRH = (BAUD_PRESCALE >> 8);
    UBRRL = BAUD_PRESCALE;
    sei();
}

// -------------------------------------------------------------------------------- //
// --- Gait Sequences (Tripod Gait) ---
// -------------------------------------------------------------------------------- //

void left_forward(void) {
    if (locator < x) {
        servo_pos[0] = servo_pos[2] = wlmax;
        servo_pos[6] = servo_pos[8] = wlmin_ver;
        servo_pos[1] = wlmin;
        servo_pos[7] = wlmax_ver;
    } else if (locator < (2 * x)) {
        servo_pos[0] = servo_pos[2] = wlmax;
        servo_pos[6] = servo_pos[8] = wlmax_ver;
        servo_pos[1] = wlmin;
        servo_pos[7] = wlmin_ver;
    } else if (locator < (3 * x)) {
        servo_pos[0] = servo_pos[2] = wlmin;
        servo_pos[6] = servo_pos[8] = wlmax_ver;
        servo_pos[1] = wlmax;
        servo_pos[7] = wlmin_ver;
    } else if (locator < (4 * x)) {
        servo_pos[0] = servo_pos[2] = wlmin;
        servo_pos[6] = servo_pos[8] = wlmin_ver;
        servo_pos[1] = wlmax;
        servo_pos[7] = wlmax_ver;
    }
}

void left_backward(void) {
    if (locator < x) {
        servo_pos[0] = servo_pos[2] = wlmin;
        servo_pos[6] = servo_pos[8] = wlmin_ver;
        servo_pos[1] = wlmax;
        servo_pos[7] = wlmax_ver;
    } else if (locator < (2 * x)) {
        servo_pos[0] = servo_pos[2] = wlmin;
        servo_pos[6] = servo_pos[8] = wlmax_ver;
        servo_pos[1] = wlmax;
        servo_pos[7] = wlmin_ver;
    } else if (locator < (3 * x)) {
        servo_pos[0] = servo_pos[2] = wlmax;
        servo_pos[6] = servo_pos[8] = wlmax_ver;
        servo_pos[1] = wlmin;
        servo_pos[7] = wlmin_ver;
    } else if (locator < (4 * x)) {
        servo_pos[0] = servo_pos[2] = wlmax;
        servo_pos[6] = servo_pos[8] = wlmin_ver;
        servo_pos[1] = wlmin;
        servo_pos[7] = wlmax_ver;
    }
}

void right_forward(void) {
    if (locator < x) {
        servo_pos[3] = servo_pos[5] = wrmin;
        servo_pos[9] = servo_pos[11] = wrmax_ver;
        servo_pos[4] = wrmax;
        servo_pos[10] = wrmin_ver;
    } else if (locator < (2 * x)) {
        servo_pos[3] = servo_pos[5] = wrmin;
        servo_pos[9] = servo_pos[11] = wrmin_ver;
        servo_pos[4] = wrmax;
        servo_pos[10] = wrmax_ver;
    } else if (locator < (3 * x)) {
        servo_pos[3] = servo_pos[5] = wrmax;
        servo_pos[9] = servo_pos[11] = wrmin_ver;
        servo_pos[4] = wrmin;
        servo_pos[10] = wrmax_ver;
    } else if (locator < (4 * x)) {
        servo_pos[3] = servo_pos[5] = wrmax;
        servo_pos[9] = servo_pos[11] = wrmax_ver;
        servo_pos[4] = wrmin;
        servo_pos[10] = wrmin_ver;
    }
}

void right_backward(void) {
    if (locator < x) {
        servo_pos[3] = servo_pos[5] = wrmax;
        servo_pos[9] = servo_pos[11] = wrmax_ver;
        servo_pos[4] = wrmin;
        servo_pos[10] = wrmin_ver;
    } else if (locator < (2 * x)) {
        servo_pos[3] = servo_pos[5] = wrmax;
        servo_pos[9] = servo_pos[11] = wrmin_ver;
        servo_pos[4] = wrmin;
        servo_pos[10] = wrmax_ver;
    } else if (locator < (3 * x)) {
        servo_pos[3] = servo_pos[5] = wrmin;
        servo_pos[9] = servo_pos[11] = wrmin_ver;
        servo_pos[4] = wrmax;
        servo_pos[10] = wrmax_ver;
    } else if (locator < (4 * x)) {
        servo_pos[3] = servo_pos[5] = wrmin;
        servo_pos[9] = servo_pos[11] = wrmax_ver;
        servo_pos[4] = wrmax;
        servo_pos[10] = wrmin_ver;
    }
}

// -------------------------------------------------------------------------------- //
// --- PWM Multiplexing Core Algorithm ---
// -------------------------------------------------------------------------------- //

/* 
 * arrange_servo() sorts the current desired pulse widths for all 14 servos.
 * It builds a timeline array (counter) and prepares port bitmasks (port_valA/B/C) 
 * so the TIMER1_COMPB ISR can turn off specific pins exactly when their pulse width expires.
 */
void arrange_servo(void) {
    char i, j;
    unsigned int temp[14];
    unsigned char temp_portA = 0xFF;
    unsigned char temp_portB = 0xFF;
    unsigned char temp_portC = 0xFF;
    count = 0;
    
    // Copy current target positions
    for (i = 0; i < 14; i++) {
        temp[i] = servo_pos[i];
    }
    
    // Bubble sort the pulse widths (shortest to longest)
    for (i = 0; i < 13; i++) {
        for (j = 0; j < 13; j++) {
            if (temp[j] > temp[j + 1]) {
                unsigned int Temp = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = Temp;
            }
        }
    }
    
    // Remove duplicate timings to create distinct event triggers
    for (i = 0; i < 13; i++) {
        if (temp[i] == temp[i + 1]) continue;
        else counter[count++] = temp[i];
    }
    counter[count++] = temp[13];
    
    // Pre-calculate which pins go LOW at each event trigger
    for (i = 0; i < count; i++) {
        temp_portA &= ~((servo_pos[0] == counter[i]) | ((servo_pos[1] == counter[i]) << 1) | 
                        ((servo_pos[2] == counter[i]) << 2) | ((servo_pos[6] == counter[i]) << 3) |
                        ((servo_pos[12] == counter[i]) << 6));
        
        temp_portB &= ~((servo_pos[3] == counter[i]) | ((servo_pos[4] == counter[i]) << 1) | 
                        ((servo_pos[5] == counter[i]) << 2) | ((servo_pos[9] == counter[i]) << 3) | 
                        ((servo_pos[13] == counter[i]) << 7)); 
        
        temp_portC &= ~((servo_pos[7] == counter[i]) | ((servo_pos[8] == counter[i]) << 1) | 
                        ((servo_pos[10] == counter[i]) << 6) | ((servo_pos[11] == counter[i]) << 7));
        
        port_valA[i] = temp_portA;
        port_valB[i] = temp_portB;
        port_valC[i] = temp_portC;
    }
}

void clear_servo_pos(void) {
    char i;
    for (i = 0; i < 14; i++) {
        servo_pos[i] = TOP + 1;
    }
}

void clear_counter(void) {
    char i;
    for (i = 0; i < 8; i++) {
        counter[i] = 1500;
    }
}

// -------------------------------------------------------------------------------- //
// --- Pan/Tilt Face Tracking & Body Inverse Kinematics approximations ---
// -------------------------------------------------------------------------------- //

void track_face(unsigned char x, unsigned char y, unsigned char z) {
    float lx = 75, ly = -40, hx = 180, hy = 40;
    
    // Y-axis tracking (Tilt)
    if (((y - prevy) > 3 || (y - prevy) < -3) && ((y - prevy) < 50 || (y - prevy) > -50)) {
        if (y > 75 && y < 180) {
            servo_pos[13] -= (y - lx) * (hy - ly) / (hx - lx) + ly;
        } else if (y < 75) {
            servo_pos[13] += hy;
        } else if (y > 180) {
            servo_pos[13] -= hy;
        }
        
        prevy = y;
        
        // Clamp values safely
        if (servo_pos[13] > 1900) servo_pos[13] = 1900;
        if (servo_pos[13] < 1100) servo_pos[13] = 1100;
    }
    
    // X-axis tracking (Pan)
    if (((x - prevx) > 10 || (x - prevx) < -10) && ((x - prevx) < 50 || (x - prevx) > -50)) {
        if (x > 75 && x < 180) {
            servo_pos[12] -= (x - lx) * (hy - ly) / (hx - lx) + ly;
        } else if (x < 75) {
            servo_pos[12] += hy;
        } else if (x > 180) {
            servo_pos[12] -= hy;
        }
        
        prevx = x;
        
        // Clamp values safely
        if (servo_pos[12] > 1850) servo_pos[12] = 1850;
        if (servo_pos[12] < 1150) servo_pos[12] = 1150;
    }
}

void tilt(unsigned char x, unsigned char y) {
    static int tilt_prevx = CENTER_POS, tilt_prevy = CENTER_POS;
    int mod[6];
    int x_shift, y_shift, x_mod, y_mod;
    
    x_shift = x - CENTER_POS;
    y_shift = y - CENTER_POS;
    
    x_mod = abs(x_shift);
    y_mod = abs(y_shift);

    // Apply simple low-pass noise filter
    if (abs(x_shift - tilt_prevx) < 3) x_shift = tilt_prevx;
    if (abs(y_shift - tilt_prevy) < 3) y_shift = tilt_prevy;
    
    // Calculate kinematic offsets
    mod[0] =  x_shift * x_mod + y_shift * y_mod;
    mod[1] =  x_shift * x_mod;
    mod[2] =  x_shift * x_mod - y_shift * y_mod;
    mod[3] = -x_shift * x_mod - y_shift * y_mod;
    mod[4] = -x_shift * x_mod;
    mod[5] = -x_shift * x_mod + y_shift * y_mod;
    
    // Apply interpolation
    servo_pos[6]  = LinearInterNeg(mod[0] * (sqrt(abs(mod[0]))) / mod[0], 0);
    servo_pos[7]  = LinearInterNeg(mod[1] * (sqrt(abs(mod[1]))) / mod[1], 0);
    servo_pos[8]  = LinearInterNeg(mod[2] * (sqrt(abs(mod[2]))) / mod[2], 0);
    servo_pos[9]  = LinearInterNeg(mod[3] * (sqrt(abs(mod[3]))) / mod[3], 1);
    servo_pos[10] = LinearInterNeg(mod[4] * (sqrt(abs(mod[4]))) / mod[4], 1);
    servo_pos[11] = LinearInterNeg(mod[5] * (sqrt(abs(mod[5]))) / mod[5], 1);
    
    tilt_prevx = x_shift;
    tilt_prevy = y_shift;
}

void stand(unsigned char rise) {
    unsigned char a = (rise == 1) ? 255 : 0;
    
    servo_pos[6]  = LinearInterPos(a, 0);
    servo_pos[7]  = LinearInterPos(a, 0);
    servo_pos[8]  = LinearInterPos(a, 0);
    servo_pos[9]  = LinearInterPos(a, 1);
    servo_pos[10] = LinearInterPos(a, 1);
    servo_pos[11] = LinearInterPos(a, 1);
}

void lean(unsigned char forw) {
    unsigned char a = (forw == 1) ? 255 : 0;
    
    servo_pos[0] = LinearInterPos(a, 0);
    servo_pos[1] = LinearInterPos(a, 0);
    servo_pos[2] = LinearInterPos(a, 0);
    servo_pos[3] = LinearInterPos(a, 1);
    servo_pos[4] = LinearInterPos(a, 1);
    servo_pos[5] = LinearInterPos(a, 1);
}

void twist(unsigned char right) {
    unsigned char a = (right == 1) ? 255 : 0;
    
    servo_pos[0] = LinearInterPos(a, 0);
    servo_pos[1] = LinearInterPos(a, 0);
    servo_pos[2] = LinearInterPos(a, 0);
    servo_pos[3] = LinearInterPos(a, 0);
    servo_pos[4] = LinearInterPos(a, 0);
    servo_pos[5] = LinearInterPos(a, 0);
}

// -------------------------------------------------------------------------------- //
// --- Utility Functions ---
// -------------------------------------------------------------------------------- //

int LinearInterPos(unsigned char x, unsigned char flag) {    
    int y;
    float lx = 0, ly = 1200, hx = 255, hy = 1800;
    
    if (flag == 0) {
        y = (x - lx) * (hy - ly) / (hx - lx) + ly;
    } else if (flag == 1) {
        y = (ly - hy) * (x - lx) / (hx - lx) + hy;
    }
    return y;
}

int LinearInterNeg(unsigned char x, unsigned char flag) {    
    int y;
    float lx = -127, ly = 1200, hx = 128, hy = 1800;
    
    if (flag == 0) {
        y = (x - lx) * (hy - ly) / (hx - lx) + ly;
    } else if (flag == 1) {
        y = (ly - hy) * (x - lx) / (hx - lx) + hy;
    }
    return y;
}

int motor_Off_ON(void) {
    char received;
    flag.timer_flag = OFF; // Halt PWM generation
    
    while (1) {
        if (!((UCSRA & (1 << RXC)) == 0)) {
            received = UDR;
            if (received == CMD_TOGGLE_MTR) {
                flag.timer_flag = ON; // Resume PWM generation
                return 0;
            }                  
        }              
    }
}

void USART_Transmit(unsigned char data) {
    /* 
     * Note: This is a blocking transmit. This is acceptable for low-frequency telemetry 
     * here, but in highly time-critical environments, blocking a processor can cause 
     * servo jitter if the transmit hangs during software PWM generation. 
     */
    while (!(UCSRA & (1 << UDRE))); 
    UDR = data;
}

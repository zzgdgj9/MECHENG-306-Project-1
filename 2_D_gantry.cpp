#include <avr/io.h>
#include <Arduino.h>
#include <avr/interrupt.h>

#define E1 5 // PE3
#define M1 4 // PE2
#define E2 6 // PE4
#define M2 7 // PE5
#define ENCODER_RESOLUTION 48
#define GEAR_RATIO 172
#define DEBOUNCE_COUNT 2

// =============================================================================
// Section: Initialisation
// =============================================================================

/* Initialise the variable to store the power send to the motors and the encoder readings. */
volatile uint8_t left_motor_power = 0;
volatile unsigned int left_encoder = 0;
volatile uint8_t right_motor_power = 0;
volatile unsigned int right_encoder = 0;

/* Build two data type, which use for finite state machine, and store the G-Code command.
   The state of the finite state machine is idle. */
enum MachineState {IDLE, PARSING, MOVING, HOMING, ERROR};
MachineState state = IDLE;
struct GCode {int g; float x; float y; float f;};
GCode command = {0, 0, 0, 0};

/* Initilise the variable to count the time that the timer 3 overflow as a clock.
   The array represent the time that the button interrupt last triggered, each element corresponds to
   top, button, left and right. homing_step shows the current step when the machine do homing. */
volatile uint16_t clock = 0;
volatile uint16_t last_clock[4] = {0};
volatile int homing_step = 1;

/* Help functions are initialised here. */
void idleSystem(void);
void systemParsing(void);
void systemHoming(void);
void systemMoving(void);
bool switchDebounce(int button_number);
void moveInDistance(float x, float y);
void performHoming(void);

// =============================================================================
// Section: Setup and Main Loop
// =============================================================================

void setup() {
    cli();

    // Set motor control pin as output
    // DDRE |= (1 << M1) | (1 << M2) | (1 << E1) | (1 << E2);
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);

    /* Set pin 45(D2) & 46(D3) as input, and enable INT2 & 3, which are associated to top and bottom button respectively.
       Set rising edge to trigger interrupt. */
    DDRD &= ~((1 << PD2) | (1 << PD3));
    EIMSK |= (1 << INT2) | (1 << INT3);
    EICRA |= (1 << ISC21) | (1 << ISC20) | (1 << ISC31) | (1 << ISC30);

    /* Set pin 6(E4) & 7(E5) as input, and enable INT4 & 5, which are associated to left and right button respectively.
       Set rising egde to trigger interrupt. */
    DDRE &= ~((1 << PE4) | (1 << PE5));
    EIMSK |= (1 << INT4) | (1 << INT5);
    EICRB |= (1 << ISC41) | (1 << ISC40) | (1 << ISC51) | (1 << ISC50);

    /* For left motor encoder, set pin 24(B5) & 25(B6) as input, enable PCINT0.
       For right motor encoder, set pin 88(K1) & 89(K0) as input, enable PCINT2 */
    PCICR |= (1 << PCIE0) | (1 << PCIE2);
    DDRB &= ~((1 << PB5) | (1 << PB6));
    PCMSK0 |= (1 << PCINT5) | (1 << PCINT6);
    DDRK &= ~((1 << PK1) | (1 << PK0));
    PCMSK2 |= (1 << PCINT16) | (1 << PCINT17);

    /* Set up timer 2 for clock using. Use overflow interrupt, prescaler = 1024. */
    TCCR2A = 0;
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    TIMSK2 |= (1 << TOIE2);

    Serial.begin(9600);

    sei();
}

void loop() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        systemHoming();
    }
    if (state == HOMING) {
      EICRA &= ~(1 << ISC31);
      EICRB &= ~(1 << ISC41);
      performHoming();
    }
}

// =============================================================================
// Section: HardWare Interrupt Configuration
// =============================================================================

// External interrupt 2 is triggered on the rising edge when the top button is pressed.
ISR(INT2_vect) {
    if (switchDebounce(0)) {
        idleSystem();
        Serial.println("Top button pressed.");
    }
    
}

// External interrupt 2 is triggered on the rising edge when the bottom button is pressed.
ISR(INT3_vect) {
    if (switchDebounce(1)) {
        if (state == HOMING) homing_step++;;
        idleSystem();
        Serial.println("Bottom button pressed.");
    }
}

// External interrupt 2 is triggered on the rising edge when the left button is pressed.
ISR(INT4_vect) {
    if (switchDebounce(2)) {
        if (state == HOMING) homing_step++;
        idleSystem();
        Serial.println("Left button pressed.");
    }
    
}

// External interrupt 2 is triggered on the rising edge when the right button is pressed.
ISR(INT5_vect) {
    if (switchDebounce(3)) {
        idleSystem();
        Serial.println("Right button pressed.");
    }
}

ISR(TIMER2_OVF_vect) {
    clock++;
}

ISR(PCINT0_vect) {
    /* When this interrupt triggred by any logic change in corresponding pin, increment the left encoder value */
    left_encoder++;
    if (left_encoder >= (ENCODER_RESOLUTION * GEAR_RATIO)) {
        left_motor_power = 0;   
    }
}

ISR(PCINT2_vect) {
    /* When this interrupt triggred by any logic change in corresponding pin, increment the right encoder value */
    right_encoder++;
    if (right_encoder >= (ENCODER_RESOLUTION * GEAR_RATIO)) {
        right_motor_power = 0;   
    }
}

// =============================================================================
// Section: Function Implementation
// =============================================================================

void idleSystem(void) {
    // state = IDLE;
    left_motor_power = 0;
    right_motor_power = 0;
}

void systemParsing(void) {
    state = PARSING;
}

void systemHoming(void) {
    state = HOMING;
}

void systemMoving(void) {
    state = MOVING;
}

bool switchDebounce(int button_number) {
    if ((clock - last_clock[button_number]) >= DEBOUNCE_COUNT) {
        last_clock[button_number] = clock;
        return true;
    } 
    return false;
}

void moveInDistance(float x, float y) {

}

void performHoming(void) {
    if (homing_step == 1) {
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
        right_motor_power = 150;
        left_motor_power = 150;
    } else if (homing_step ==2) {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, HIGH);
        right_motor_power = 100;
        left_motor_power = 100;
    } else if (homing_step == 3) {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        right_motor_power = 160;
        left_motor_power = 150;
    } else if (homing_step == 4) {
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
        right_motor_power = 100;
        left_motor_power = 100;
    } else {
        right_motor_power = 0;
        left_motor_power = 0;
    }
    analogWrite(E1, right_motor_power);
    analogWrite(E2, left_motor_power);
}
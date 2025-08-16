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
// Section: Initialise Global Variable
// =============================================================================

/* Build a struct for motors, where store the information power, encoder reading value,
   direction (true is clockwise and false is anti-clockwise), and speed. */
struct Motor {
    uint8_t power;
    unsigned int encoder;
    bool direction;
    float speed;
};

Motor left_motor = {0, 0, 0, 0};
Motor right_motor = {0, 0, 0, 0};
// volatile uint8_t left_motor.power = 0;
// volatile unsigned int left_motor.encoder = 0;
// volatile uint8_t right_motor.power = 0;
// volatile unsigned int right_motor.encoder = 0;

/* Build two data type, which use for finite state machine, and store the G-Code command.
   The state of the finite state machine is idle. */
enum MachineState {IDLE, PARSING, MOVING, HOMING, ERROR};
MachineState state = IDLE;
struct GCode {int g; float x; float y; float f;};
GCode command = {0, 0, 0, 0};
String input_g_code = "";

/* Initilise the variable to count the time that the timer 3 overflow as a clock.
   The array represent the time that the button interrupt last triggered, each element corresponds to
   top, button, left and right. homing_step shows the current step when the machine do homing. */
volatile uint16_t clock = 0;
volatile uint16_t last_clock[4] = {0};
volatile int homing_step = 1;

/* Initilise the variable the store the time for an error occur, and the time to start clear the error*/
volatile uint16_t error_occur_time = 0;
volatile uint16_t error_clear_time = 0;

// =============================================================================
// Section: Function Prototypes
// =============================================================================

void idleSystem(void);
void systemParsing(void);
void systemHoming(void);
void systemMoving(void);
void systemError(void);
bool switchDebounce(int button_number);
void stopMotor(void);
void moveInDistance(float x, float y);
void processGCode(String line);
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
    switch (state) {
        case IDLE:
            EICRA |= (1 << ISC31);
            EICRB |= (1 << ISC41);
            stopMotor();
            if (Serial.available() > 0) systemParsing();
            break;
        case PARSING:
            if (Serial.available() > 0) {
                char c = Serial.read();
                if (c != '\n' && c != '\r') {
                    input_g_code += c;
                } else {
                    processGCode(input_g_code);
                    input_g_code = "";
                }
            } else {
                idleSystem();
            }
            
            if (command.g == 28) systemHoming();
            if (command.g == 1) systemMoving();
            break;
        case HOMING:
            // Reset the logical to any logical change at button generates an interrupt request.
            EICRA &= ~(1 << ISC31);
            EICRB &= ~(1 << ISC41);
            performHoming();
            break;
        case MOVING:
            digitalWrite(M1, LOW);
            digitalWrite(M2, HIGH);
            analogWrite(E1, 100);
            analogWrite(E2, 100);
            break;
        case ERROR:
        /* When error occur, stop all the motor and reset everything. Give the user 10 seconds to check the error.
           If the error persists after 10 seconds, check which button is triggering every seconds.
           Move the motor in the opposite direction to the triggering button to leave the error state. 
           Once get into safe position, return home position. */
            if ((clock - error_occur_time > 610) && (clock - error_clear_time > 61)) {
                error_clear_time = clock;
                if (PIND & (1 << PD2)) {
                    digitalWrite(M1, HIGH);
                    digitalWrite(M2, LOW);
                } else if (PIND & (1 << PD3)) {
                    digitalWrite(M1, LOW);
                    digitalWrite(M2, HIGH);
                }else if (PINE & (1 << PE4)) {
                    digitalWrite(M1, HIGH);
                    digitalWrite(M2, HIGH);
                } else if (PINE & (1 << PE5)) {
                    digitalWrite(M1, LOW);
                    digitalWrite(M2, LOW);
                } else {
                    Serial.println("Error condition cleared. System returns to home position now.");
                    systemHoming();
                    break;
                }
                analogWrite(E1, 100);
                analogWrite(E2, 100);
            } else if (clock - error_occur_time <= 610) {
                stopMotor();
                homing_step = 1;
            }
            break;
    }
}

// =============================================================================
// Section: HardWare Interrupt Configuration
// =============================================================================

// External interrupt 2 is triggered on the rising edge when the top button is pressed.
ISR(INT2_vect) {
    if (switchDebounce(0)) {
        systemError();
    }
}

// External interrupt 2 is triggered on the rising edge when the bottom button is pressed.
ISR(INT3_vect) {
    if (switchDebounce(1)) {
        if (state == HOMING) {
            homing_step++;
            return;
        }
        systemError();
    }
}

// External interrupt 2 is triggered on the rising edge when the left button is pressed.
ISR(INT4_vect) {
    if (switchDebounce(2)) {
        if (state == HOMING) {
            homing_step++;
            return;
        }
        systemError();
    }
}

// External interrupt 2 is triggered on the rising edge when the right button is pressed.
ISR(INT5_vect) {
    if (switchDebounce(3)) {
        systemError();
    }
}

ISR(TIMER2_OVF_vect) {
    clock++;
}

ISR(PCINT0_vect) {
    /* When this interrupt triggred by any logic change in corresponding pin, increment the left encoder value */
    left_motor.encoder++;
    if (left_motor.encoder >= (ENCODER_RESOLUTION * GEAR_RATIO)) {
        left_motor.power = 0;   
    }
}

ISR(PCINT2_vect) {
    /* When this interrupt triggred by any logic change in corresponding pin, increment the right encoder value */
    right_motor.encoder++;
    if (right_motor.encoder >= (ENCODER_RESOLUTION * GEAR_RATIO)) {
        right_motor.power = 0;   
    }
}

// =============================================================================
// Section: Function Implementation
// =============================================================================

void idleSystem(void) {
    state = IDLE;
    // left_motor.power = 0;
    // right_motor.power = 0;
    // analogWrite(E1, right_motor.power);
    // analogWrite(E2, left_motor.power);
}

void systemParsing(void) {
    state = PARSING;
}

void systemHoming(void) {
    state = HOMING;
    command.g = 0;
}

void systemMoving(void) {
    state = MOVING;
    command.g = 0;
}

void systemError(void) {
    state = ERROR;
    error_occur_time = clock;
    Serial.print("Error detected — please check the issue and take action. ");
    Serial.println("If the error persists after 10 seconds, the machine will automatically move to a safe position.");
}

bool switchDebounce(int button_number) {
    if ((clock - last_clock[button_number]) >= DEBOUNCE_COUNT) {
        last_clock[button_number] = clock;
        return true;
    } 
    return false;
}

void stopMotor(void) {
    left_motor.power = 0;
    right_motor.power = 0;
    analogWrite(E1, right_motor.power);
    analogWrite(E2, left_motor.power);
}

void moveInDistance(float x, float y) {

}

void processGCode(String line) {
    // Parse character-by-character
    int i = 0;
    while (i < line.length()) {
        char letter = line[i++];
        if (letter < 65 || letter >122 || (letter > 90 && letter < 97)) { continue; }

        // Gather number after the letter
        String numberStr = "";
        while (i < line.length() && (isDigit(line[i]) || line[i] == '.' || line[i] == ' ')) {
            numberStr += line[i++];
        }
        float value = numberStr.toFloat();

        switch (letter) {
            case 'G': case 'g': command.g = (int)value; break;
            case 'X': case 'x': command.x = value; break;
            case 'Y': case 'y': command.y = value; break;
            case 'F': case 'f': command.f = value; break;
        }
    }
}

void performHoming(void) {
    /* There are 8 steps in the homing procedure. Fast move to left to touch the left button then fast leave.
       Slow down to touch the left botton again and slowly leave. Once finish, do the same for going down. 
       Once the homing step is not within the range, that means either the homing finish or something going wrong.
       Send the machine to idle state and reset the homing_step.*/
    if (homing_step == 1) {
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
        right_motor.power = 200;
        left_motor.power = 200;
    } else if (homing_step == 2) {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, HIGH);
        right_motor.power = 200;
        left_motor.power = 200;
    } else if (homing_step == 3) {
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
        right_motor.power = 100;
        left_motor.power = 100;
    } else if (homing_step == 4) {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, HIGH);
        right_motor.power = 60;
        left_motor.power = 60;
    } else if (homing_step == 5) {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        right_motor.power = 210;
        left_motor.power = 200;
    } else if (homing_step == 6) {
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
        right_motor.power = 200;
        left_motor.power = 200;
    } else if (homing_step == 7) {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        right_motor.power = 110;
        left_motor.power = 100;
    } else if (homing_step == 8) {
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
        right_motor.power = 60;
        left_motor.power = 60;
    } else {
        idleSystem();
        homing_step = 1;
    }

    analogWrite(E1, right_motor.power);
    analogWrite(E2, left_motor.power);
}

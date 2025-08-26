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
#define MOTOR_PERIMETER 45.5
#define X_LIMIT 216
#define Y_LIMIT 135

// =============================================================================
// Section: Initialise Global Variable
// =============================================================================

/* Build a struct for motors, where store the information power, encoder reading value,
   direction (true is clockwise and false is anti-clockwise), and speed. */
struct Motor {
    uint8_t power;
    long encoder;
    long previous_encoder;
    double last_stop;
    double position;
    double previous_position;
    bool direction;
    float speed;
};

Motor left_motor = {0, 0, 0, 0, 0, 0};
Motor right_motor = {0, 0, 0, 0, 0, 0};

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
volatile uint8_t homing_step = 1;

// =============================================================================
// Section: Function Prototypes
// =============================================================================

void idleSystem(void);
void systemParsing(void);
void systemHoming(void);
void systemMoving(void);
void systemError(void);
bool switchDebounce(int button_number);
uint8_t sendPower(int power);
void stopMotor(void);
bool motorFullyStopped(void);
void resetOrigin(void);
void updateLastStop(void);
void moveInDirection(char direction, uint8_t power);
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
            stopMotor();
            updateLastStop();
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
            performHoming();
            break;
        case MOVING:
            moveInDirection('U', 100);
            break;
        case ERROR:
        /* When error occur, stop all the motor and reset everything. Wait the M999 command,
           then return to idle state. */
            stopMotor();
            homing_step = 1;
            if (Serial.available()) {
              char c = Serial.read();
              if (c != '\n' && c != '\r') {
                    input_g_code += c;
                } else {
                    if (input_g_code == "M999") idleSystem();
                    input_g_code = "";
                }
            break;
        }
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
    /* Everytime the timer 2 overflow interrupt triggered, increment the clock to change the system time.
       Then calculate and update the motor speed. */
    clock++;
    left_motor.speed = (left_motor.position - left_motor.previous_position) / 0.016384;
    right_motor.speed = (right_motor.position - right_motor.previous_position) / 0.016384;
    left_motor.previous_position = left_motor.position;
    right_motor.previous_position = right_motor.position;
}

ISR(PCINT0_vect) {
    /* When this interrupt triggred by any logic change in corresponding pin, change the left encoder value.
       If motor rotate CCW, increment the value. If motor rotate CW, decrement the value. */
    if (digitalRead(M2)) left_motor.position += 0.00551478;
    if (!digitalRead(M2)) left_motor.position -= 0.00551478;
}

ISR(PCINT2_vect) {
    /* When this interrupt triggred by any logic change in corresponding pin, change the right encoder value.
       If motor rotate CCW, increment the value. If motor rotate CW, decrement the value. */
    if (digitalRead(M1)) right_motor.position += 0.00551478;
    if (!digitalRead(M1)) right_motor.position -= 0.00551478;
}

// =============================================================================
// Section: Function Implementation
// =============================================================================

void idleSystem(void) {
    state = IDLE;
    EICRA |= (1 << ISC31);
    EICRB |= (1 << ISC41);
    stopMotor();
}

void systemParsing(void) {
    state = PARSING;
}

void systemHoming(void) {
    /* Change the state of the machine to HOMING, clear the G command and the encoder value. */
    state = HOMING;
    EICRA &= ~(1 << ISC31);
    EICRB &= ~(1 << ISC41);
    command.g = 0;
    resetOrigin();
}

void systemMoving(void) {
    state = MOVING;
    command.g = 0;
}

void systemError(void) {
    state = ERROR;
    Serial.println("Error detected — please solve the issue and input M999. ");
}

bool switchDebounce(int button_number) {
    if ((clock - last_clock[button_number]) >= DEBOUNCE_COUNT) {
        last_clock[button_number] = clock;
        return true;
    } 
    return false;
}

uint8_t sendPower(int power) {
    if (power < 0) {
        return 0;
    } else if (power > 255) {
        return 255;
    } else {
        return power;
    }
}

void stopMotor(void) {
    left_motor.power = sendPower(0);
    right_motor.power = sendPower(0);
    analogWrite(E1, right_motor.power);
    analogWrite(E2, left_motor.power);
}

bool motorFullyStopped(void) {
    return ((left_motor.speed == 0) && (right_motor.speed == 0));
}

void resetOrigin(void) {
    while (!motorFullyStopped()) { asm("nop"); }
    left_motor.position = 0;
    right_motor.position = 0;
}

void updateLastStop(void) {
    while (!motorFullyStopped()) { asm("nop"); }
    left_motor.last_stop = left_motor.position;
    right_motor.last_stop = right_motor.position;
}

void moveInDirection(char direction, uint8_t power) {
    /* Calculate the difference between the left and right encoder value, calculate kp base on the power,
       then change the left and right motor power in order to get rid off this difference. */
    double left_difference = abs(left_motor.position - left_motor.last_stop);
    double right_difference = abs(right_motor.position - right_motor.last_stop);
    double error = left_difference - right_difference;
    float k_p = 1000 + power * 6;
    left_motor.power = sendPower(power - k_p * error);
    right_motor.power = sendPower(power + k_p * error);

    /* Depends on the input, set up the direction to move. */
    if (direction == 'U') {
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
    } else if (direction == 'D') {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
    } else if (direction == 'L') {
        digitalWrite(M1, LOW);
        digitalWrite(M2, LOW);
    } else if (direction == 'R') {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, HIGH);
    }
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
       Send the machine to idle state and reset the homing_step. Clear the encoder value to set the origin. */
    if (homing_step == 1) {
        moveInDirection('L', 200);
    } else if (homing_step == 3) {
        moveInDirection('R', 200);
    } else if (homing_step == 5) {
        moveInDirection('L', 100);
    } else if (homing_step == 7) {
        moveInDirection('R', 60);
    } else if (homing_step == 9) {
        moveInDirection('D', 200);
    } else if (homing_step == 11) {
        moveInDirection('U', 200);
    } else if (homing_step == 13) {
        moveInDirection('D', 100);
    } else if (homing_step == 15) {
        moveInDirection('U', 60);
    } else if (homing_step == 16) {
        idleSystem();
        homing_step = 1;
        resetOrigin();
    } else {
        stopMotor();
        updateLastStop();
        homing_step++;
    }
}

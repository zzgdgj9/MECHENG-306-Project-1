#include <avr/io.h>
#include <Arduino.h>
#include <avr/interrupt.h>
#include <math.h>

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
#define ACCELATION 0.1

// =============================================================================
// Section: Initialise Global Variable
// =============================================================================

/* Build a struct for motors, where store the information of a motor including :
   The power send to the motor
   The position of the motor, which is represent by the position of a point on the belt (i.e. delta A).
   The previous position of the motor, which is represent by the previous position of a point on the belt (i.e. delta A-1).
   The position of the motor when it last time stopped, which is represent by the position of a point on the belt when the motor last time stopped.
   The speed of the motor, which is represent by the speed of a point on the belt.
   The unit of all the distance measurement is mm, and the unit of all the speed measurement is mm/s. */
struct Motor {
    uint8_t power;
    double position;
    double previous_position;
    double last_stop;
    float speed;
};

Motor left_motor = {0, 0, 0, 0, 0};
Motor right_motor = {0, 0, 0, 0, 0};
double belt_speed[2] = {0};

/* Build two data type, which use for finite state machine, and store the G-Code command.
   The state of the finite state machine is idle. */
enum MachineState {IDLE, PARSING, MOVING, HOMING, ERROR};
MachineState state = IDLE;
struct GCode {int g; double x; double y; double f;};
GCode command = {0, 0, 0, 0};
String input_g_code = "";

/* Initilise the variable to count the time that the timer 3 overflow as a clock.
   The array represent the time that the button interrupt last triggered, each element corresponds to
   top, button, left and right. homing_step shows the current step when the machine do homing. */
volatile uint16_t clock = 0;
volatile uint16_t last_clock[4] = {0};
volatile uint16_t speed_clock = 0;
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
void processGCode(String line);
void moveInDirection(char direction, uint8_t power);
void moveController(void);
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
            if (clock - speed_clock > 0) {
                speed_clock = clock;
                moveController();
            }

            (belt_speed[0] >= 0) ? digitalWrite(M2, HIGH) : digitalWrite(M2, LOW);
            (belt_speed[1] >= 0) ? digitalWrite(M1, HIGH) : digitalWrite(M1, LOW);
            analogWrite(E1, right_motor.power);
            analogWrite(E2, left_motor.power);

            break;
        case ERROR:
        /* When error occur, stop all the motor and reset everything. Wait the M999 command,
           then return to idle state. */
            stopMotor();
            updateLastStop();
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
    /* When this interrupt triggred by any logic change in corresponding pin, update the left motor position.
       If motor rotate CCW, add 0.00551478 to the position, the number is the distance that a point on belt travelled (mm/encoder pulse). 
       If motor rotate CW, minus 0.00551478 to the position. */
    if (digitalRead(M2)) left_motor.position += 0.00551478;
    if (!digitalRead(M2)) left_motor.position -= 0.00551478;
}

ISR(PCINT2_vect) {
    /* When this interrupt triggred by any logic change in corresponding pin, update the right motor position.
       If motor rotate CCW, add 0.00551478 to the position, the number is the distance that a point on belt travelled (mm/encoder pulse). 
       If motor rotate CW, minus 0.00551478 to the position. */
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

void processGCode(String line) {
    // Parse character-by-character
    int i = 0;
    while (i < line.length()) {
        char letter = line[i++];
        if (letter < 65 || letter >122 || (letter > 90 && letter < 97)) { continue; }

        // Gather number after the letter
        String numberStr = "";
        while (i < line.length() && (isDigit(line[i]) || line[i] == '.' || line[i] == ' ' || line[i] == '-')) {
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

void moveController(void) {
    double total_distance = sqrt(command.x * command.x + command.y * command.y);
    double x_ratio = command.x / total_distance;
    double y_ratio = command.y / total_distance;
    double max_x_speed = command.f * x_ratio / 60;
    double max_y_speed = command.f * y_ratio / 60;
    double max_a_speed = max_x_speed + max_y_speed;
    double max_b_speed = max_x_speed - max_y_speed;
    double a_ratio = max_a_speed / (command.f / 60);
    double b_ratio = max_b_speed / (command.f / 60);

    double delta_a = left_motor.position - left_motor.last_stop;
    double delta_b = right_motor.position - right_motor.last_stop;
    double sync_error = delta_a * b_ratio - delta_b * a_ratio;
    double sync_kp = 0.2;
    belt_speed[0] -= sync_error * sync_kp;
    belt_speed[1] += sync_error * sync_kp;

    double delta_x = (delta_a + delta_b) / 2;
    double delta_y = (delta_a - delta_b) / 2;
    double travelled_distance = sqrt(delta_x * delta_x + delta_y * delta_y);
    double remain_distance = total_distance - travelled_distance;   


    static double integral_remain_distance = 0;
    double distance_kp = 3;
    double distance_ki = 0.002;

    if ((remain_distance / (command.f / 60)) < 1) {
        integral_remain_distance += remain_distance;
        if (integral_remain_distance > 100) integral_remain_distance = 100;
        if (integral_remain_distance < -100) integral_remain_distance = -100;
        belt_speed[0] = (remain_distance * distance_kp + integral_remain_distance * distance_ki) * a_ratio;
        belt_speed[1] = (remain_distance * distance_kp + integral_remain_distance * distance_ki) * b_ratio;
    } else {
        belt_speed[0] += ACCELATION * a_ratio;
        belt_speed[1] += ACCELATION * b_ratio;
    }

    (abs(belt_speed[0]) > abs(max_a_speed)) ? belt_speed[0] = max_a_speed : belt_speed[0] = belt_speed[0];
    (abs(belt_speed[1]) > abs(max_b_speed)) ? belt_speed[1] = max_b_speed : belt_speed[1] = belt_speed[1];

    double speed_kp = 2;
    double left_speed_error = abs(belt_speed[0]) - abs(left_motor.speed);
    double right_speed_error = abs(belt_speed[1]) - abs(right_motor.speed);

    left_motor.power = sendPower(left_motor.power + speed_kp * left_speed_error);
    right_motor.power = sendPower(right_motor.power + speed_kp * right_speed_error);

    double motor_speed = sqrt(left_motor.speed * left_motor.speed + right_motor.speed * right_motor.speed);
    if (abs(remain_distance) < 0.1 && abs(motor_speed) < 0.01) {
        idleSystem();
        belt_speed[0] = 0;
        belt_speed[1] = 0;
        Serial.println("move finish");
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
        moveInDirection('R', 50);
    } else if (homing_step == 9) {
        moveInDirection('D', 200);
    } else if (homing_step == 11) {
        moveInDirection('U', 200);
    } else if (homing_step == 13) {
        moveInDirection('D', 100);
    } else if (homing_step == 15) {
        moveInDirection('U', 50);
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

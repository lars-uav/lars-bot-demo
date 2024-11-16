/* LARS UAV Ground Bot Movement Code
   By: Tanay Srinivasa (U20220086)
   Date Modified: 9 Nov 2024 9:58 PM
*/

/* Pin Definitions for Right Motor */
#define RPWM_R 5
#define LPWM_R 6
#define L_EN_R 7
#define R_EN_R 8

/* Pin Definitions for Left Motor */
#define RPWM_L 10
#define LPWM_L 9
#define L_EN_L 11
#define R_EN_L 12

void setup() {
        // Initialize all pins as output
        pinMode(RPWM_R, OUTPUT);
        pinMode(LPWM_R, OUTPUT);
        pinMode(L_EN_R, OUTPUT);
        pinMode(R_EN_R, OUTPUT);

        pinMode(RPWM_L, OUTPUT);
        pinMode(LPWM_L, OUTPUT);
        pinMode(L_EN_L, OUTPUT);
        pinMode(R_EN_L, OUTPUT);

        // Start with motors disabled
        stop();

        Serial.begin(9600);
}

// Functions for Motor Control
void backward(int speed) {
        // Enable both motors
        digitalWrite(R_EN_R, HIGH);
        digitalWrite(L_EN_R, HIGH);
        digitalWrite(R_EN_L, HIGH);
        digitalWrite(L_EN_L, HIGH);

        // Set motors to move backward
        analogWrite(RPWM_R, speed);
        analogWrite(LPWM_R, 0);
        analogWrite(RPWM_L, speed);
        analogWrite(LPWM_L, 0);
}

void forward(int speed) {
        // Enable both motors
        digitalWrite(R_EN_R, HIGH);
        digitalWrite(L_EN_R, HIGH);
        digitalWrite(R_EN_L, HIGH);
        digitalWrite(L_EN_L, HIGH);

        // Set motors to move forward
        analogWrite(RPWM_R, 0);
        analogWrite(LPWM_R, speed);
        analogWrite(RPWM_L, 0);
        analogWrite(LPWM_L, speed);
}

void right(int speed) {
        // Enable both motors
        digitalWrite(R_EN_R, HIGH);
        digitalWrite(L_EN_R, HIGH);
        digitalWrite(R_EN_L, HIGH);
        digitalWrite(L_EN_L, HIGH);

        // Set motors to turn right
        analogWrite(RPWM_R, speed);
        analogWrite(LPWM_R, 0);
        analogWrite(RPWM_L, 0);
        analogWrite(LPWM_L, speed);
}

void left(int speed) {
        // Enable both motors
        digitalWrite(R_EN_R, HIGH);
        digitalWrite(L_EN_R, HIGH);
        digitalWrite(R_EN_L, HIGH);
        digitalWrite(L_EN_L, HIGH);

        // Set motors to turn left
        analogWrite(RPWM_R, 0);
        analogWrite(LPWM_R, speed);
        analogWrite(RPWM_L, speed);
        analogWrite(LPWM_L, 0);
}

void stop() {
        // Disable both motors
        digitalWrite(R_EN_R, LOW);
        digitalWrite(L_EN_R, LOW);
        digitalWrite(R_EN_L, LOW);
        digitalWrite(L_EN_L, LOW);

        // Set all PWM to 0
        analogWrite(RPWM_R, 0);
        analogWrite(LPWM_R, 0);
        analogWrite(RPWM_L, 0);
        analogWrite(LPWM_L, 0);
}

void loop() {
        if (Serial.available()){
                char c = Serial.read();
                        switch(c){
                        case 'F':
                        forward(120);
                        break;
                        case 'B':
                        backward(120);
                        break;
                        case 'L':
                        left(120);
                        break;
                        case 'R':
                        right(120);
                        break;
                        case 'S':
                        stop();
                        break;
                }
        }
	Serial.flush();
}

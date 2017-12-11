/*******************************************************************************
 * led_control.ino - Arduino side LED control for ferret robot
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

// PWM duty cycle for LED control recieved from serial port
int dutycycle;

void setup() {
    // Setup serial port
    Serial.begin(115200);
    // Set control pins as outputs
    pinMode(12,OUTPUT);
    pinMode(10,OUTPUT);
    // Initialize the lights as off
    analogWrite(10,255);
    analogWrite(11,255);
}

void loop() {
    while( Serial.available() > 0 ) {

        // Look for valid integer
        dutycycle = Serial.parseInt();

        // Look for newline
        if( Serial.read() == '\n' ) {
            // Write PWM value to LED control pints
            analogWrite(10,dutycycle);
            analogWrite(11,dutycycle);
        }
    }
}

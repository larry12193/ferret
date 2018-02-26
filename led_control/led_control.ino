/*******************************************************************************
 * led_control.ino - Arduino side LED control for ferret robot
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

// PWM duty cycle for LED control recieved from serial port
int dutyCycle;

void setup() {
    // Setup serial port
    Serial.begin(115200);
    // Set control pins as outputs
    pinMode(11,OUTPUT);
    pinMode(10,OUTPUT);
    // Initialize the lights as off
    analogWrite(10,255);
    analogWrite(11,255);
}

void loop() {
    // Check if there is serial data
    if( Serial.available() > 0 ) {
        // Pull integer from buffer
        dutyCycle = Serial.parseInt();
        // Clear the serial buffer
        Serial.read();
        Serial.read();
        // Write pwm to pins
        analogWrite(10,255-dutyCycle);
        analogWrite(11,255-dutyCycle);
    }
}

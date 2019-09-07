// Motor constants
byte MOTOR_L_A = 8;
byte MOTOR_L_B = 7;
byte MOTOR_L_PWM = 6;

byte MOTOR_R_A = 11;
byte MOTOR_R_B = 10;
byte MOTOR_R_PWM = 9;

byte motor, count, pwm, read_char;

unsigned long time;

void setup() {
    Serial.begin(115200);
    pinMode(MOTOR_L_A, OUTPUT);
    pinMode(MOTOR_L_B, OUTPUT);
    pinMode(MOTOR_R_A, OUTPUT);
    pinMode(MOTOR_R_B, OUTPUT);
    digitalWrite(MOTOR_L_A, LOW);
    digitalWrite(MOTOR_L_B, HIGH);
    digitalWrite(MOTOR_R_A, LOW);
    digitalWrite(MOTOR_R_B, HIGH);

    count = 0;
    pwm = 0;

    time = millis();
}

void loop() {
    
    if(Serial.available()){
        read_char = Serial.read();

        if(count){
            if(read_char > '9' || read_char < '0'){
                count = 0;
                pwm = 0;
            }
            else {
                pwm = pwm + count*(read_char - '0');
                count = count/10;

                if(!count){
                    if(motor == 'd'){
                        analogWrite(MOTOR_R_PWM, pwm);
                    }
                    else if(motor == 'e'){
                        analogWrite(MOTOR_L_PWM, pwm);
                    }
                    pwm = 0;
                }
            }
        }

        // Not inside an 'else' for resetting count even if lost some bytes 
        if(read_char == 'd' || read_char == 'e'){
            motor = read_char;
            count = 100;
        }
   
        time = millis();
    }
    else if( (millis()-time) > 3*1e3 ){
        // 3 seconds not receiving anything, turn off motors
        
        analogWrite(MOTOR_R_PWM, 0);
        analogWrite(MOTOR_L_PWM, 0);
    }
}
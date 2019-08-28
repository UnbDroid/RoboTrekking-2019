#define MOTOR_D 6
#define MOTOR_E 9

byte motor, count, pwm, read_char;

unsigned long time;

void setup() {
    Serial.begin(115200);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);

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
                        analogWrite(MOTOR_D, pwm);
                    }
                    else if(motor == 'e'){
                        analogWrite(MOTOR_E, pwm);
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
        
        analogWrite(MOTOR_D, 0);
        analogWrite(MOTOR_E, 0);
    }
}
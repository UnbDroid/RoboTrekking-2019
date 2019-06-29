#define MOTOR_D 6
#define MOTOR_E 9

byte motor_d_pwm = 0;
byte motor_e_pwm = 0;

byte str[4];

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

    time = millis();
}

void loop() {
    
    if(Serial.available() == 8){
        Serial.readBytes(str, 8);

        // Sanity check
        if(str[0] == 'd' && str[4] == 'e'){
            motor_d_pwm = (str[1]-'0')*100 + (str[2]-'0')*10 + (str[3]-'0');
            motor_e_pwm = (str[5]-'0')*100 + (str[6]-'0')*10 + (str[7]-'0');
        }

        analogWrite(MOTOR_D, motor_d_pwm);
        analogWrite(MOTOR_E, motor_e_pwm);
        
        time = millis();
    }
    else if( (millis()-time) > 3*1e3 ){
        // 3 seconds not receiving anything, turn off motors
        
        analogWrite(MOTOR_D, 0);
        analogWrite(MOTOR_E, 0);
    }
}
#define MOTOR_D 6
#define MOTOR_E 9

byte motor_d_pwm = 0;
byte motor_e_pwm = 0;

byte str[4];

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
}

void loop() {
  if(Serial.available() == 4){
    Serial.readBytes(str, 4);

    // Sanity check
    if(str[0] == 'd' && str[2] == 'e'){
      motor_d_pwm = str[1];
      motor_e_pwm = str[3];
    }

    analogWrite(MOTOR_D, motor_d_pwm);
    analogWrite(MOTOR_E, motor_e_pwm);
  }
}
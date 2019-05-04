/*Accept command from BBB via Serial
 *Turns on/off on-board LED
 *by OP from teachmemicro.com */

#define PKG_SIZE 4

char inChar[PKG_SIZE];
void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}
 
void loop() // run over and over
{
  if (Serial.available() == PKG_SIZE){
    Serial.readBytes(inChar, PKG_SIZE);
    for(int i = 0; i < PKG_SIZE; ++i){
      if(inChar[i] == 'z'){
        inChar[i] = 'a';
      }else{
        inChar[i] += 1;
      }
    }
   Serial.write(inChar, PKG_SIZE);
   digitalWrite(LED_BUILTIN, HIGH);
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
  }
}

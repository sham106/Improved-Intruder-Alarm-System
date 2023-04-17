#include <SoftwareSerial.h>
int pirSensor = 4;
int redLed = A5;
int buzzer = A4;
String PHONE = "+254748804536";
SoftwareSerial mySerial(9, 10);
String smsStatus, senderNumber, receivedDate, msg;
boolean isReply = false;
boolean fullstate = true;
boolean emptystate = true;
void setup() {
  Serial.begin(115200);
   mySerial.begin(9600);
  pinMode(pirSensor, INPUT);
  pinMode(redLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
   pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Ready");
  
  while (!mySerial.available()) {
    mySerial.println("AT");
    delay(1000);
    Serial.println("Connecting...");
    digitalWrite(LED_BUILTIN, HIGH);
  }
  Serial.println("Connected!");
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(buzzer, HIGH);
  delay(200);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  mySerial.println("AT+CMGF=1");  //Set SMS to Text Mode
  delay(1000);
  mySerial.println("AT+CNMI=1,2,0,0,0");  //Procedure to handle newly arrived messages(command name in text: new message indications to TE)
  delay(1000);
  mySerial.println("AT+CMGL=\"REC UNREAD\""); // Read Unread Messages
  delay(2000);
}

void loop() {
  int sensorValue = digitalRead(pirSensor);

  if (sensorValue == 1) {
    Serial.println("motion");
digitalWrite(redLed, HIGH);
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);
  delay(300);  
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);

        if (fullstate == true) {
      Reply("\nSMART INTRUDER DETECTION SYSTEM \n \n Dear Management,\nIntruder detected at kabarak University SN:RT15245GH. \nAn Image has been sent to your email.\nTake Action Immediately");
      fullstate = false;
      emptystate = true;
      delay(5000);
    }
  }
  else{
      digitalWrite(redLed, LOW);
   Serial.println("no motion");
    if (emptystate == true) {
      //Reply("Dear Management,Property is now safe.SN:RT15245GH");
      emptystate = false;
      fullstate = true;

      delay(5000);
    }
    delay(1000);
}
}
void Reply(String text)
{
  mySerial.print("AT+CMGF=1\r");
  delay(1000);
  mySerial.print("AT+CMGS=\"" + PHONE + "\"\r");
  delay(1000);
  mySerial.print(text);
  delay(100);
  mySerial.write(0x1A); //ascii code for ctrl-26 //mySerial.println((char)26); //ascii code for ctrl-26
  delay(1000);
  Serial.println("SMS Sent Successfully.");
}

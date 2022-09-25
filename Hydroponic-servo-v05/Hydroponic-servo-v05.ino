/* Hydroponic drain & Flood by #ApaITU
 *  https://www.youtube.com/channel/UChBQyf27VmOEk8uWdQYuX-Q
 *  
 *  September 2022
 *  
 *  MicroController   : Arduino Nano
 *  Servo             : SG90 on pin 10
 *  Mosfet DC-Pump    : IRF520 Module
 *  StepDown          : 12v to 5V
 *  TDS Probe         : DIY Pin A0(ecPin),A1(ecGnd), A3(ecPlus)
 *  Temperatur Sensor : DS18B20 RED VCC,  YELLOW Data, Black Gnd, Data Pullup 4k7
 *  
 *  Drain & Flood
 *  1) Servo Close the drain
 *  2) pump run for 2 minute
 *  3) at the minute 8, servo open the drain
 *  4) at minute 15 reset cycle
 *  
 *  
 * TDS :
 * 
 * Electrical Conductivy ~  ec[mS/cm2] = 1/ρ 
 * 1S = 1/Ohm >>  
 * conductivy is reciproc to resistivity, higher conductivity lower the Resistance
 * Higher the Resistance lower the conductivity
 * 
 * 
 * from wire conductivy we have 
 * Resistance R = ρ* (l/A)
 * l :  Length  of the wire [m]
 * A :  Cross Sectional Area [m2]
 * ρ :  resistivity (property of material) [Ohm.m]
 * 
 * we know that ρ ~ R * l, we could measure the cross sectional area on solid wire
 * how about salt diluted solution? we could not measure the area since it fluid
 * the EC measure the electrical conductivity of solution between to probe in distance of 1cm
 * we make it simple, since ρ ~ R we bring const K to make the statement related
 *      ρ   = R.K
 *      ec  = 1/ρ
 *          = 1/R.K
 *          
 * we could measure the R by drawning probe into the solution, we use simple voltage divider Formula
 *      Vout  = Vin*(R2/(R1+R2))
 *      Vec   = Vcc*(Rec/R1+Rec)
 *      Vec   : Voltage we read from the probe on ecPin
 *      Vcc   : 5V from Arduino Nano A3(ecPlus)
 *      R1    : 1K resistor
 *      Ra    : internal  resistance of arduino pin, we need add into account Ra around 25 Ohm
 *      R1 = R1+Ra = 1025K
 *      
 * we formulate the statement :
 *      Vec.R1 + Vec.Rec  = Vcc*Rec
 *            Rec(Vcc-Vec)= Vec.R1
 *                    Rec = R1.(Vec/(Vcc-Vec))
 * we know
 *      R1  = 1k
 *      Vcc = 5V
 *      Vec = 5*(analogRead(A0)/1023.0)
 * 
 * next step ec=1/(Rec.K*1000)        1000 because ec [mS/cm]  1mS = 1/1000*R
 *  since we use DIY style probe, we need to measure the K value
 *  there is around 1.6 we could calibrate later
 *  
 *  now we get the EC value from the probe, actually electrical conductivity deviate by temperatur, around 2% per C
 *  we could look at the graph, the 2% deviation only valid in very small interval around 25C, there for we  need to calculate
 *  EC on 25C or
 *  EC25 =  EC/(1+Tc(T-25.0)
 *  
 *  Finaly PPM (Partical per Million)
 *  PPM = EC25*(ppmConversion*1000)
 *  
 *  ppmConversion : 3 standard 0.5, 0.6, 0.7 
 *  PPM (Truncheon), PPM (Eutech), PPM (Hanna)
 *  
 *  we use 1EC = 0.5/1000 PPM or 500PPM
 *  
 *  PPM = EC25.500
 *  
 *  
 * Since we are not using AC voltage we could not draw currennt to the solution all the time,
 * the measurement process should not longer then 6 seconds, otherwise the solution will be electrolys
 * 
 * Version
 * V01    : initial state
 * V02    : ds18b20
 * V02b   : floodNdrain()
 * 
 *  
*/

#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2
Servo dServo; 
LiquidCrystal_I2C lcd(0x23,16,2);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tSensor(&oneWire);
DeviceAddress insideThermometer; 

int pos;
uint32_t prevMillis;

byte ss;
byte mm;
int counter;
const int startC=15;
const int slowPumpC=60;
const int stopPumpC=120;
const int servoReleaseC=240;
const int resetC=300;
bool dFlag;

float tempC;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(4,0);
  lcd.print("ApaITU");
  lcd.setCursor(2,1);
  lcd.print("Hydroponic");
  pinMode(5,OUTPUT);
  digitalWrite(5,0);
  dServo.attach(10);
  delay(1000);
  dServo.write(135);
  delay(1000); 
  dServo.write(45);
  delay(1000);
  dServo.write(95);
  delay(3000);
  tSensor.begin();
  tSensor.requestTemperatures();
  tempC = tSensor.getTempCByIndex(0);
  lcd.clear();
}

void loop() {
  tickSecond();
  serialLoop();
  drainNflood();
}

String twoDigit(int i){
  String s;
  if(i<10){
    s = "0"+String(i);
  } else {
    s = String(i);
  }
  return s;
}

void tickSecond(){
  if(millis()-prevMillis>1000){
    prevMillis=millis();
    counter++;
    int mm  = counter/60;
    int ss  = counter%60;
  if(counter%3==0){
    tSensor.requestTemperatures();
    tempC = tSensor.getTempCByIndex(0);
  }
    lcd.setCursor(0,0);
    lcd.print("T" + String(tempC) + "C");
    lcd.setCursor(9,0);
    lcd.print("[" + twoDigit(mm) + ":" + twoDigit(ss) +"]" );
    Serial.println( "[" + twoDigit(mm)+ ":" + twoDigit(ss) + "] : [" + String(counter) + "]");
    Serial.println("Temp : ["  + String(tempC) + "C]");
    dFlag=true;
  }
}

void timeStamp(){
  ss = counter%60;
  mm = counter/60;
}
void drainNflood(){
  if(dFlag){
    switch (counter){
      case startC :
        dServo.write(45);
        digitalWrite(5,HIGH);
        Serial.println("[startC]: [" + String(startC) + "] dServo.write(55) UP");
        Serial.println("[startC]: [" + String(startC) + "] pump pwm 255");
        lcd.setCursor(0,1);
        lcd.print("[S0] : P&S High"); 
        break;
      case slowPumpC :
        analogWrite(5,245);
        Serial.println("[slowPumpC]: " + String(slowPumpC) + " pump pwm 200");
        lcd.setCursor(0,1);
        lcd.print("[S1] : Pump SlowDown"); 
        break;
      case stopPumpC :
        digitalWrite(5,LOW);
        Serial.println("[stopPumpC]: " + String(stopPumpC) + " pump digitalWrite LOW");
        lcd.setCursor(0,1);
        lcd.print("[S2] : Pump Stop"); 
        break;
      case servoReleaseC:
        dServo.write(150);
        Serial.println("[servoReleaseC]: " + String(servoReleaseC) + " dServo.write(135) DOWN");
        lcd.setCursor(0,1);
        lcd.print("[S3] : Servo Down"); 
        break;
      case resetC :
        counter=14;
        lcd.setCursor(0,1);
        lcd.print("[S4] : Reset"); 
      Serial.println("[resetC]: " + String(resetC) + "dServo.write(30) DOWN");
        break;
    }
    dFlag=false;

  }
}

void serialLoop(){
  if(Serial.available()){
    int i = Serial. parseInt();
    if(i!=0){
      //myservo.write(i);
      Serial.println("Serial-input :" + String(i));
      if(i>100){
        dServo.write(i-100);
        Serial.println("dServo.write :" + String(i-100));
      }
      if(i<100){
        Serial.println("Speed :" + String(i));
        int pwm = map(i,0,100,0,255);
        Serial.println("pwm : " + String(pwm));
        analogWrite(5,pwm);
      }
    
    }
  }
}

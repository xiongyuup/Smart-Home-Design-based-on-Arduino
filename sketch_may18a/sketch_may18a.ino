#include <Blinker.h>
#include <Wire.h>
#include <DHT.h>
#include <Stepper.h>
#include "LiquidCrystal_I2C.h"
#include <Servo.h>   
Servo myservo;  
#define BLINKER_BLE
#define STEPS 100

Stepper stepper(STEPS, 10, 12, 11, 13);

LiquidCrystal_I2C lcd(0x27, 16, 2); 

BlinkerButton Button1("kd");     //开灯
BlinkerButton Button2("gd");     //关灯
BlinkerButton Button3("open");   //开窗
BlinkerButton Button4("close");  //关窗
BlinkerButton Button5("op");   //开风扇
BlinkerButton Button6("cl");  //关风扇
BlinkerNumber HUMI("humi");      //湿度
BlinkerNumber TEMP("temp");      //温度

/*************温湿度检测************/
#define DHTPIN 4
#define DHTTYPE DHT11   
DHT dht(DHTPIN, DHTTYPE);
float humi_read, temp_read;
/*************温湿度检测************/


/*************光敏传感器检测************/
int  ADPIN  = A0  ;
int  LEDPIN = 9 ;        //光敏控制的灯引脚
int  value = 0 ;
float voltage = 0.0 ;
float ligh_read;
int turn_light=0;
String comdata = "";
/*************光敏传感器检测************/

/*********气体传感器引脚************/ 
int Buzzer=5; 
int SensorPin=8;         //定义传感器引脚 
int val=0;                //定义数字变量val 
/*********气体传感器引脚************/


/*********继电器引脚************/
int Relay=7;               //定义继电器引脚--气体
int Relay1=6;            //定义继电器1引脚--窗户
/*********继电器引脚************/


/*********开关灯************/
void button1_callback(const String & state)
{
    BLINKER_LOG("get button state: ", state);
    if(state=="on"){
      turn_light=1;
    }
}

void button2_callback(const String & state)
{
    BLINKER_LOG("get button state: ", state);
    if(state=="off"){
      turn_light=0;
    analogWrite(LEDPIN,0);
    Serial.println("关灯");}
}
/*********开关灯************/

/*************窗户转动************/
void button3_callback(const String & state)
{
    BLINKER_LOG("get button state: ", state);
    if(state=="on"){
      stepper.step(3000);
    }
}
void button4_callback(const String & state)
{
    BLINKER_LOG("get button state: ", state);
    if(state=="off"){
       stepper.step(-3000);
   }
}
/*************窗户转动************/

/*************窗户转动************/
void button5_callback(const String & state)
{
    BLINKER_LOG("get button state: ", state);
    if(state=="on"){
       digitalWrite(Relay1,LOW);
    }
}
void button6_callback(const String & state)
{
    BLINKER_LOG("get button state: ", state);
    if(state=="off"){
       digitalWrite(Relay1,HIGH);
   }
}
/*************窗户转动************/

void dataRead(const String & data)
{
    BLINKER_LOG("Blinker readString: ", data);
    Blinker.vibrate();
    uint32_t BlinkerTime = millis();
    Blinker.print("millis", BlinkerTime);
}


void heartbeat()
{
    HUMI.print(humi_read);
    TEMP.print(temp_read);
}


void setup()
{
    Serial.begin(115200);
    BLINKER_DEBUG.stream(Serial);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    Blinker.begin();
    Blinker.attachData(dataRead);
    Blinker.attachHeartbeat(heartbeat);
    Button1.attach(button1_callback);
    Button2.attach(button2_callback);    
    Button3.attach(button3_callback);    
    Button4.attach(button4_callback);
    Button5.attach(button5_callback);    
    Button6.attach(button6_callback); 
    dht.begin();
    stepper.setSpeed(180);

    pinMode(DHTPIN, OUTPUT);
    lcd.init();
    lcd.backlight();
    
    pinMode(LEDPIN,OUTPUT);
    pinMode(ADPIN,INPUT);
    Serial.begin(9600);          
           
    pinMode(5,OUTPUT);
    pinMode(SensorPin,INPUT);          
    pinMode(Relay,OUTPUT);  
    pinMode(Relay1,OUTPUT);
    digitalWrite(Relay1,HIGH);
    digitalWrite(Relay,HIGH);
}


/*********主函数************/
void loop()
{
  
 
  /*********启动手机blinker************/
    Blinker.run();
  /*********启动手机blinker************/  


  /*********温湿度获取************/
        float h = dht.readHumidity();
        float t = dht.readTemperature();        

        if (isnan(h) || isnan(t)) {
            BLINKER_LOG("Failed to read from DHT sensor!");
            return;
        }

        humi_read = h;
        temp_read = t;

        BLINKER_LOG("Humidity: ", h, " %");
        BLINKER_LOG("Temperature: ", t, " *C");
        
    lcd.setCursor(0, 0);
    lcd.print("Tep: ");
    lcd.print((float)dht.readTemperature(), 2);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print((float)dht.readHumidity(), 2);
    lcd.print("%");
    delay(200);
  /*********温湿度获取************/


  /*********可燃气体监测************/
    val=digitalRead(SensorPin);  
    Serial.print("val=");
    Serial.println(val);
    if(val==0)
       { 
         digitalWrite(Buzzer,LOW);
         digitalWrite(Relay,HIGH);
       } 
    else 
       {    
        digitalWrite(Buzzer,HIGH); 
        digitalWrite(Relay,LOW);
       } 
  /*********可燃气体监测************/


  /*********光敏电阻和灯光的控制************/
     if(turn_light==1){
        value =  analogRead(ADPIN);    
        voltage = ( ( float )value )/1023 ;
        value = (int)(voltage * 256) ;   
        Serial.print("value=");           
        Serial.println(value);   
        analogWrite(LEDPIN,value);
        ligh_read=value;
    Serial.println("开灯");
   }
  /*********光敏电阻和灯光的控制************/
}
/*********主函数************/

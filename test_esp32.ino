#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "DHT.h"

/*****Adafruit*****/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "hienhien612"
#define AIO_KEY         "aio_nKFK93kF3fmnQ8OnCf8mivQMBCZP"
/*****Wifi********/
#define WIFI_SSID "realme C2"
#define WIFI_PASS "22222223"
/*****Pin number********/
#define GAS_PIN   39
#define HC_TRIG   16
#define HC_ECHO   27
#define DHT_PIN   26
#define DHT_TYPE  DHT11
#define FAN_PIN   4
#define LED_PIN   14
/*****Parameter********/
// set the LCD number of columns and rows
int lcd_col = 16;
int lcd_row = 2;
LiquidCrystal_I2C lcd(0x27, lcd_col, lcd_row);  
//Use for store states
int auto_mode = 0;
int led_state = 0;
/*****SET UP********/
//DHT11
DHT dht(DHT_PIN, DHT_TYPE);
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
/******Feeds *******/
// Setup a feed for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
/**Publish**/
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DADN_TEMP_1");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DADN_HUMI_1");
Adafruit_MQTT_Publish gas_value = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DADN_GAS");
Adafruit_MQTT_Publish led_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DADN_LED_1");
Adafruit_MQTT_Publish fan_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DADN_FAN_1");
Adafruit_MQTT_Publish auto_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DADN_AUTO");
/**Subscribe**/
Adafruit_MQTT_Subscribe led_control = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/DADN_LED_1");
Adafruit_MQTT_Subscribe fan_control = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/DADN_FAN_1");
Adafruit_MQTT_Subscribe auto_control = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/DADN_AUTO");
/*****PROTOTYPE*****/
int HC_distance();
void setup_wifi();
void MQTT_connect();
void data_subscribe();
void temp_humi();
void gas_warn();
void auto_check();

void setup() {
  // initialize pin
  pinMode(GAS_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  
  Serial.begin(115200);
  dht.begin();
  delay(10);
  
  Serial.println(F("Adafruit MQTT demo"));
  setup_wifi();

  //Setup MQTT subscribe
  mqtt.subscribe(&led_control);
  mqtt.subscribe(&fan_control);
  mqtt.subscribe(&auto_control);
  //Connect server
  MQTT_connect();
  //Reset button
  led_pub.publish(0);
  delay(200);
  fan_pub.publish(0);
  delay(200);
  auto_pub.publish(0);
  delay(200);
}

void loop() {
  // Ensure the connection to the MQTT server is alive
  MQTT_connect();

  //Subsribe
  auto_check();
  data_subscribe();
  
  //Read and publish temp and humid
  temp_humi();
  gas_warn();

  delay(2000);
}


/*****FUNCTION********/
//Calculate the distance
int HC_distance() {
  digitalWrite(HC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG, LOW);
  //Echo receive signal
  int time_receive = pulseIn(HC_ECHO, HIGH);
  int distance = time_receive * 0.034/2;
  return distance;
}

//Connect Wifi
void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//Function to connect and reconnect as necessary to the MQTT server.
//Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  //Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

//Read data from server
void data_subscribe() {
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &led_control) {
      led_state = atoi((char *)led_control.lastread);
      if (atoi((char *)led_control.lastread) == 0) {
        digitalWrite(LED_PIN, LOW); 
      }
      if (atoi((char *)led_control.lastread) == 1) {
        digitalWrite(LED_PIN, HIGH); 
      }
      Serial.print(F("LED: "));
      Serial.println((char *)led_control.lastread);
    } 
    if (subscription == &fan_control) {
      uint16_t fan_value = atoi((char *)fan_control.lastread);
      analogWrite(FAN_PIN, fan_value);
      Serial.print(F("FAN: "));
      Serial.println((char *)fan_control.lastread);
    }
    if (subscription == &auto_control) {
      auto_mode = atoi((char *)auto_control.lastread);
      auto_check();
      Serial.print(F("AUTO_MODE: "));
      Serial.println((char *)auto_control.lastread);
    }
  }
}

//Process data sensor
void temp_humi(){
  // Read humidity
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();

  //publish temperature and humidity
  Serial.print(F("\nTemperature: "));
  Serial.print(t);
  Serial.print(F("\nHumidity: "));
  Serial.println(h);

  //print LCD
  lcd.clear();
  // set cursor to first column, first row
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.setCursor(6, 0);
  lcd.print(t);
  // set cursor to first column, second row
  lcd.setCursor(0, 1);
  // print message
  lcd.print("Humi:");
  lcd.setCursor(6, 1);
  lcd.print(h);

  //publish to server
  temperature.publish(t);
  humidity.publish(h);
}

void gas_warn() {
  int sensorValue = analogRead(GAS_PIN);
  //display LCD
  if(sensorValue >= 600) {
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("WARNING!!!");
    lcd.setCursor(2, 1);
    lcd.print("HIGH GAS!!!");
    Serial.println("GAS WARNING!!!!");
  } else {
    lcd.setCursor(13, 0);
    lcd.print("GAS:");
    lcd.setCursor(13, 1);
    lcd.print(sensorValue);
  }
  //print to screen
  Serial.print("GAS: ");
  Serial.println(sensorValue);
  //publish to server
  delay(500);
  gas_value.publish(sensorValue);
}

//Check for AUTO mode
void auto_check() {
  if (auto_mode == 1) {
    int dis = HC_distance();
    Serial.print("Distance: ");
    Serial.println(dis);
    if(dis <= 10) {
      if (led_state != 1) {
        digitalWrite(LED_PIN, HIGH);
        analogWrite(FAN_PIN, 100);
        led_pub.publish(1);
        fan_pub.publish(100);
      }
    } else {
      if (led_state != 0) {
        digitalWrite(LED_PIN, LOW);
        analogWrite(FAN_PIN, 0);
        led_pub.publish(0);
        fan_pub.publish(0);
      }
    }
  } 
}

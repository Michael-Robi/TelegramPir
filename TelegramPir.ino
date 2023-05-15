//Definir Red
// const char* ssid = "FAMILIA_BAQUERO-2.4";
// const char* password = "75532091";
// const char* ssid = "alfonso";
// const char* password = "12345678";
// const char* ssid = "STAR. 2G";
// const char* password = "CHILACOA#504";

#include "esp_wifi.h"
#include <Arduino.h>
#include <WiFiMulti.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <FirebaseESP32.h>
#include <ESP32Servo.h>

const uint32_t TiempoEsperaWifi = 5000;
WiFiMulti wifiMulti;

IPAddress local_IP(192, 168, 0, 16);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 0, 1);

// Firebase.
#define FIREBASE_HOST "esp32proyect-da533-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "IWlJuYX04NliWvJF88oRWZs2MXPcQzABLZcJS7eo"

// Instancia firebase
FirebaseData firebaseData;

// Configuracion Boot Telegram
String BOTtoken = "6062930272:AAFnxf6BYuT5ewJXeMDJeIGvAWIrwUfy9eQ";
String CHAT_ID = "5329377159";
WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

//Configuracion Camara
bool sendPhoto = false;
#define FLASH_LED_PIN 4
bool flashState = LOW;

//Configuracion recepcion de mensajes a 1 segundo
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

int inicio = 1;

//Componente electronicos
//Leds

// #define DGREEN 15

//Instancia servomotor
//Servo servo;
// Servo servo1;
int pinServo=15;
int pinServo1=13;

//Estado dispositivos
int estadoServoMotor12 = 0;
int estadoServoMotor2 = 0;
int estadoLed13 = 0;

//Si se mueven 5 veces el servomotor se muestan las raciones del dia
static int contadorRaciones2 = 0;
static int contadorMoverServomotor2 = 0;

//Fecha Sistema
static String fechaSistema = "History ";

const int PIRsensor = 12;
const int led = 16;
int PIRstate = LOW; //Sin deteccion de movimiento
int val = 0;

// tiempo de calibracion sensor
const int calibrationTime = 15; // 15 segundos

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void configInitCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
}

//Init Config
//Configuraciones
void setup() {
  //Configuracion general ESP32
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

   // Configuracion Monitor Serial en 115200
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Bienvenido a Esp32Cam");

//Configuracion Servomotor
  // servo.attach(pinServo, 50, 2500); // 15
  //servo1.attach(pinServo1, 50, 2500); // 13
  pinMode(pinServo,OUTPUT);
  pinMode(pinServo1,OUTPUT);
  ledcSetup(5,50,16); // 50 hz PWM, 16-bit resolution, range from 3250 to 6500.
  ledcAttachPin(pinServo,5); //15
  ledcSetup(3,50,16); // 50 hz PWM, 16-bit resolution, range from 3250 to 6500.
  ledcAttachPin(pinServo1,3); //13

  //Configuracion Leds
  // pinMode(DGREEN, OUTPUT); // Configurar el pin GPIO para el LED como salida
  // digitalWrite(DGREEN, HIGH); //15 enciende el Led
  // delay(2000); // espera 2 segundos
  // digitalWrite(DGREEN, LOW); // apaga el LED

  // Configuracion Flash
  pinMode(FLASH_LED_PIN, OUTPUT);
  
   //Configuracion sensor y led
  pinMode(PIRsensor, INPUT);
  pinMode(led, OUTPUT);

  // Give some time for the PIR sensor to warm up
  Serial.println("Esperando a que el sensor se caliente en el primer arranque");
  delay(calibrationTime * 1000); // Time converted back to miliseconds

  // Blink LED 3 times to indicate PIR sensor warmed up
  // digitalWrite(led, HIGH);
  // delay(500);
  // digitalWrite(led, LOW);
  // delay(500);
  // digitalWrite(led, HIGH);
  // delay(500);
  // for (int i = 0; i < 5; i++) {
    digitalWrite(FLASH_LED_PIN, 10);  // flash led
    delay(50);
    digitalWrite(FLASH_LED_PIN, 0); 
    delay(50);
  // }

  // Configuracion Camara
  configInitCamera();
  wifiMulti.addAP("alfonso", "12345678");
  wifiMulti.addAP("STAR. 2G", "CHILACOA#504");
  wifiMulti.addAP("FAMILIA_BAQUERO-2.4", "75532091");  

  // Establecimiento de la IP estática.
  // if (!WiFi.config(local_IP, gateway, subnet)) {
  //   Serial.println("Fallo en la configuración de la IP Estatica");
  // }
  
  // Configuracion Conexiones
  WiFi.mode(WIFI_STA);
  //WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // adjuntar certificado api.telegram.org
  // while (WiFi.status() != WL_CONNECTED) {
  //   Serial.print(".");
  //   delay(500);
  // }
  
  while (wifiMulti.run(TiempoEsperaWifi) != WL_CONNECTED) {
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi Conectado a la red:");
  // Serial.println(ssid);
  Serial.print("IP Estatica: ");
  Serial.println(WiFi.localIP());

  if(inicio == 1){
    Serial.println("Sistema PETSBOT preparado");
    //Enviamos un mensaje a telegram para informar que el sistema está listo
    bot.sendMessage(CHAT_ID, "Sistema Petsboot preparado, Escribe Ayuda para ver las opciones:", "");
    inicio = 0;
  }

  //Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.setString(firebaseData,fechaSistema+"/Esp32Servo", WiFi.localIP().toString());
}

//Compilador
void loop() {
  ActualizarWifi();

  if (WiFi.status() == WL_CONNECTED) {
    String WiFiAddr = WiFi.localIP().toString();    
    Serial.println("Codigo con Wifi: "+WiFiAddr);
  } else {
    Serial.println("Codigo sin Wifi");
  }

  //Comentado Viernes  
  val = digitalRead(PIRsensor);
  Serial.println("Estado Sensor"+String(val));    

  if (val == HIGH) {//Si esta encendido
    // digitalWrite(led, HIGH);//led encendido
    digitalWrite(FLASH_LED_PIN, HIGH);  // flash led
    delay(50);
    digitalWrite(FLASH_LED_PIN, LOW); 
    if (PIRstate == LOW) {
      // we have just turned on because movement is detected
      // Serial.println("Motion detected!");
      // delay(500);
      Serial.println("Envio de foto telegram");
      sendPhotoTelegram();
      PIRstate = HIGH;
    }
  }
  
  //Metodo envio de fotos  
  //if    
  else if (sendPhoto) {
    Serial.println("Foto enviada al boot");
    sendPhotoTelegram();
    sendPhoto = false;
  }
  //Receptor de Mensajes por telegram  
  else if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      Serial.println("Respuesta Mensaje");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
  else {
    // digitalWrite(led, LOW);
    digitalWrite(FLASH_LED_PIN, LOW); // flash led
    if (PIRstate == HIGH) {
      Serial.println("Movimiento terminado");
      PIRstate = LOW;
    }
  }
}

//Metodo Para recepciona mensajes
void handleNewMessages(int numNewMessages) {
  Serial.print("Estado: ");
  Serial.println(numNewMessages);
  //"encender","apagar"
  String comandos[9]={"mover","encender","apagar","estado","racion","camara","camara1","camara2","camara3"};

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    // if (chat_id != CHAT_ID) {
    //   bot.sendMessage(chat_id, "Unauthorized user", "");
    //   continue;
    // }

    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;
    
    if (text.equalsIgnoreCase("flash")) {
      flashState = !flashState;
      digitalWrite(FLASH_LED_PIN, flashState);
      delay(500);  // Espera 5 segundos
      digitalWrite(FLASH_LED_PIN, LOW);  // Apaga el flash LED
      //Enviar Foto
      sendPhoto = true; 
      Serial.println("Flash Activo");
    }
    else if (text.equalsIgnoreCase("foto")) {
      sendPhoto = true;
      Serial.println("Envio de foto");
    }
        else if (text.equalsIgnoreCase(comandos[0])) {
      contadorRaciones2++;
      contadorMoverServomotor2++;
      Firebase.setInt(firebaseData,fechaSistema+"/Raciones del dia", contadorRaciones2);
      if(contadorMoverServomotor2==3){
        Firebase.setString(firebaseData,fechaSistema+"/Alimentador Total", "Completado");          
        bot.sendMessage(chat_id, "Completaste las raciones del dia");
        contadorMoverServomotor2=0;   
        }        
      bot.sendMessage(chat_id, "Alimentador Encendido");
      moverServomotor();
    }
    // else if (text.equalsIgnoreCase(comandos[1])) {
    //   estadoLed13 = 1;
    //   Serial.println("Luz 1 encendida");
    //   digitalWrite(DGREEN, HIGH);
    //   bot.sendMessage(chat_id, "Luz 1 Encendida");
    //   Firebase.setString(firebaseData,fechaSistema+"/Estado Dispositivos/Luz1", "ON");
    // }
    // else if (text.equalsIgnoreCase(comandos[2])) {
    //   estadoLed13 = 0;
    //   Serial.println("Luz 1 apagada");
    //   digitalWrite(DGREEN, LOW);       
    //   bot.sendMessage(chat_id, "Luz 1 Apagada");
    //   Firebase.setString(firebaseData,fechaSistema+"/Estado Dispositivos/Luz1", "OFF");
    // }
    else if (text.equalsIgnoreCase(comandos[3])) {
      if(estadoServoMotor12)
      {
        bot.sendMessage(chat_id, "Alimentador Encendido");
        Firebase.setString(firebaseData,fechaSistema+"/Estado Dispositivos/Alimentador", "ON");
      }
      else{
        bot.sendMessage(chat_id, "Alimentador Apagado");
        Firebase.setString(firebaseData,fechaSistema+"/Estado Dispositivos/Alimentador", "OFF");
      }
      if(estadoServoMotor2)
      {
        bot.sendMessage(chat_id, "Camara giro Encendido");
        Firebase.setString(firebaseData,fechaSistema+"/Estado Dispositivos/Camara Giro", "ON");
      }
      else{
        bot.sendMessage(chat_id, "Camara giro Apagado");
        Firebase.setString(firebaseData,fechaSistema+"/Estado Dispositivos/Camara Giro", "OFF");
      }
      // if (estadoLed13)
      // {                    
      //   bot.sendMessage(chat_id, "Luz 1 Encendida");
      //   Firebase.setString(firebaseData,fechaSistema+"/Estado Dispositivos/Luz1", "ON");
      // }
      // else
      // {
      //   bot.sendMessage(chat_id, "Luz 1 Apagada");
      //   Firebase.setString(firebaseData,fechaSistema+"/Estado Dispositivos/Luz1", "OFF");
      // }
    }
    else if (text.equalsIgnoreCase(comandos[4])) {
      bot.sendMessage(chat_id, "Racion del dia: "+String(contadorRaciones2));
    }
    else if (text.equalsIgnoreCase(comandos[5])) {
      estadoServoMotor2 = 1;
      int giro = 75;
      Serial.println("Camara Giro "+String(giro));
      // servo1.write(giro);
      ServoPWM(giro);
      bot.sendMessage(chat_id, "Camara Giro: "+String(giro));
      Firebase.setInt(firebaseData,fechaSistema+"/Camara Giro", giro);
    }
    else if (text.equalsIgnoreCase(comandos[6])) {
      estadoServoMotor2 = 1;
      int giro = 100;
      Serial.println("Camara2 Giro "+String(giro));
      // servo1.write(giro);
      ServoPWM(giro);
      bot.sendMessage(chat_id, "Camara Giro: "+String(giro));
      Firebase.setInt(firebaseData,fechaSistema+"/Camara Giro", giro);
    }
    else if (text.equalsIgnoreCase(comandos[7])) {
      estadoServoMotor2 = 1;
      int giro = 135;
      Serial.println("Camara2 Giro "+String(giro));
      // servo1.write(giro);
      ServoPWM(giro);
      bot.sendMessage(chat_id, "Camara Giro: "+String(giro));
      Firebase.setInt(firebaseData,fechaSistema+"/Camara Giro", giro);
    }
    else if (text.equalsIgnoreCase(comandos[8])) {
      estadoServoMotor2 = 1;
      int giro = 160;
      Serial.println("Camara Giro "+String(giro));
      // servo1.write(giro);
      ServoPWM(giro);
      bot.sendMessage(chat_id, "Camara giro: "+String(giro));
      Firebase.setInt(firebaseData,fechaSistema+"/Camara Giro", giro);      
    }
    else {
      String menu = comandos[0]+" - "+ comandos[1]+" - "+ comandos[2]+" - "+ comandos[3]+" - "+ comandos[4]+" - "+ comandos[5]; 
      
      String ayuda = "Bienvenido , " + from_name + "\n";
      ayuda += "Estas son tus opciones:\n";
      ayuda += comandos[0]+": Para alimentar mascota. \n";
      ayuda += comandos[4]+": Muestra las porciones diarias. \n";
      ayuda += comandos[5]+"1-3: Para girar en una posicion. \n";
      // ayuda += comandos[1]+": Enciende la Luz 1. \n";        
      // ayuda += comandos[2]+": Apaga la luz 1. \n";
      ayuda += comandos[3]+": Muestra los componentes en uso. \n";     
      ayuda += "foto : tomar foto\n";
      ayuda += "flash : tomar foto con flash \n";
      bot.sendMessage(CHAT_ID, ayuda, "");

      Firebase.setString(firebaseData,fechaSistema+"/Boot/Ayuda", menu);
    }
  }
}

//Metodo ActualizarWifi
void ActualizarWifi() {
  if (wifiMulti.run(TiempoEsperaWifi) != WL_CONNECTED) {
    Serial.println("No conectado a Wifi!");
  }
}

//MetodoServomotor
void moverServomotor(){
  Firebase.setString(firebaseData,fechaSistema+"/Alimentador", "Activo");
  
  // servo.write(0);
  Servo2PWM(180);
  estadoServoMotor12 = 1;
  // String giro1 = String(servo.read());
  // Serial.println("Alimentador Giro1 "+giro1);
  delay(1000);

  Servo2PWM(75);
  // String giro2 = String(servo.read());
  // Serial.println("Alimentador Giro2 "+giro2);
  //delay(30000);
}

//Servo PWM
void ServoPWM(float pos)
{
    uint32_t equivalenciaGiro = (((pos/180.0)
              *2000)/20000.0*65536.0) + 1634;
         // convert 0-180 degrees to 0-65536

    ledcWrite(3,equivalenciaGiro);
        // set channel to pos
}

void Servo2PWM(float pos)
{
    uint32_t equivalenciaGiro = (((pos/180.0)
              *2000)/20000.0*65536.0) + 1634;
         // convert 0-180 degrees to 0-65536

    ledcWrite(5,equivalenciaGiro);
        // set channel to pos
}

//Metodo enviar Foto Telegram
String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera Fallo");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  Serial.println("Conectado a " + String(myDomain));


  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Conexion Exitosa");

    String head = "--c010blind3ngineer\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--c010blind3ngineer\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--c010blind3ngineer--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=c010blind3ngineer");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);

    esp_camera_fb_return(fb);

    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + waitTime) > millis()) {
      Serial.print(".");
      delay(100);
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state == true) getBody += String(c);
        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        }
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connected to api.telegram.org failed.";
    Serial.println("Coneccion a api.telegram.org fallo.");
  }
  return getBody;
}
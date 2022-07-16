#include <ESP8266WiFi.h>
#include <espnow.h>
#include<Wire.h>
#include <Ticker.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Ticker timer;

volatile int interrupts;

Adafruit_MPU6050 mpu;
//------------------------------------------------------------------------------------
String msg = "test";
int acelX, acelY, acelZ;
int temperatura, giroX, giroY, giroZ;

char tacelX[7];
char tacelY[7];
char tacelZ[7];
char tgiroX[7];
char tgiroY[7];
char tgiroZ[7];
unsigned long tempo2;

//Endereco I2C do MPU6050
const int MPU = 0x68; //pino aberto 0X68

// REPLACE WITH RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0xDC, 0x4F, 0x22, 0x61, 0x19, 0xDE}; //DC:4F:22:61:19:DE ESP8266
uint8_t broadcastAddress1[] = {0x7C, 0x9E, 0xBD, 0x61, 0x5F, 0xAC}; //7C:9E:BD:61:5F:AC  ESP32
uint8_t broadcastAddress2[] = {0x7C, 0x9E, 0xBD, 0x4E, 0x9E ,0xB4};



// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char aX[7];
  char aY[7];
  char aZ[7];
  char gX[7];
  char gY[7];
  char gZ[7];

  //int b;
  //float c;
  //String d;
  // bool e;
} struct_message;

// Create a struct_message called myData
struct_message myData;

//unsigned long lastTime = 0;
//unsigned long timerDelay = 0;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  //Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.print("Dado Enviado:");
    Serial.println(msg);

  }
  else {
    Serial.println("Conecção Perdida, Tentando reconectar...");
    //Chame esta função para emparelhar um dispositivo.
    esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    delay(250);
  }
}

// ISR to Fire when Timer is triggered
void ICACHE_RAM_ATTR onTime() {
  //  interrupts++;
  //  tempo2 = millis();
  //
  //  Serial.println(tempo2);
  //  Serial.print("Total Ticks:");
  //  Serial.println(interrupts);
  // Re-Arm the timer as using TIM_SINGLE
  acel();
  sprintf(tacelX, "%05d", acelX);
  strcpy(myData.aX, tacelX);

  sprintf(tacelY, "%05d",acelY);
  strcpy(myData.aY, tacelY);

  sprintf(tacelZ, "%05d", acelZ);
  strcpy(myData.aZ, tacelZ);

  sprintf(tgiroX, "%05d", giroX);
  strcpy(myData.gX, tgiroX);

  sprintf(tgiroY, "%05d",giroY);
  strcpy(myData.gY, tgiroY);

  sprintf(tgiroZ, "%05d", giroZ);
  strcpy(myData.gZ, tgiroZ);
  //sinal();
  //esp_now_send(broadcastAddress1, (uint8_t *) &myData, sizeof(myData));
  esp_now_send(0, (uint8_t *) &myData, sizeof(myData));
  //timer1_write(20000);
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  //pinMode(D4, OUTPUT);
  //digitalWrite(D4, HIGH);
  timer1_attachInterrupt(onTime); // Add ISR Function
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  /* Dividers:
    TIM_DIV1 = 0,   //80MHz (80 ticks/us - 104857.588 us max)
    TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
    TIM_DIV256 = 3  //312.5Khz (1 tick = 3.2us - 26843542.4 us max)
    Reloads:
    TIM_SINGLE  0 //on interrupt routine you need to write a new value to start the timer again
    TIM_LOOP  1 //on interrupt the counter will start with the same value again
  */

  // Intervalo de 8ms ou 125Hz
  timer1_write(40000); // 0,000002*40000 = 8ms


  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //====================================================================================
  Wire.begin();                 //inicia I2C
  Wire.beginTransmission(MPU);  //Inicia transmissão para o endereço do MPU
  Wire.write(0x6B);

  //Inicializa o MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);
  //------------------------------------------------------------------------------------
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); // 2_G 4_G 8_G 16_G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);  // 250_DEG 500_DEG 1000_DEG 2000_DEG 
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); //5_Hz 10_Hz 21_Hz 44_Hz 94_Hz 184_Hz 260_Hz

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  /*Registre uma função de retorno de chamada que é acionada ao enviar dados. Quando uma mensagem é enviada,
    uma função é chamada – esta função retorna se a entrega foi bem sucedida ou não.*/
  esp_now_register_send_cb(OnDataSent);

  // Chame esta função para emparelhar um dispositivo.
  esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(broadcastAddress2, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {

  // Set values to send
  //  acel();
  //  //strcpy(myData.a, Buf);
  //  myData.aX = acelX;
  //  myData.aY = acelY;
  //  myData.aZ = acelZ;
  //  myData.gX = giroX;
  //  myData.gY = giroY;
  //  myData.gZ = giroZ;
  //myData.c = 1.2;
  //myData.d = "msg";
  //myData.e = false;

  // Envie dados com ESP-NOW.
  // esp_now_send(broadcastAddress1, (uint8_t *) &myData, sizeof(myData));
  //Serial.println(msg);

}

void acel() { //função que captura os dados do acelerometro e os transmite para variaveis

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  acelX = 100*a.acceleration.x;
  acelY = 100*a.acceleration.y;
  acelZ = 100*a.acceleration.z;
  giroX = 100*g.gyro.x;
  giroY = 100*g.gyro.y;
  giroZ = 100*g.gyro.z;
  
  
}


void sinal() {
  if (digitalRead(D4) == HIGH) {
    //GPIO.out_w1ts = ((uint32_t)1 << 22);
    digitalWrite(D4, LOW);
  } else {
    //GPIO.out_w1ts = ((uint32_t)0 << 22);
    digitalWrite(D4, HIGH);
  }
}

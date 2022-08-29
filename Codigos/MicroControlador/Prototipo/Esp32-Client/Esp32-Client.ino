#include <esp_now.h>
#include <WiFi.h>
#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>

#include "acel_model.h"
//volatile int interrupts;
//int totalInterrupts;

#define N_INPUTS 5
#define N_OUTPUTS 2
#define TENSOR_ARENA_SIZE 2*1024

Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//Variáveis Globais
volatile int interruptCounter;
int  OUT_PIN =  4 ;

#define LED 2
#define BUT 4
#define MOT 22
//-------------------------------------------------------------------------
//valor da variável do botão
int bt;
int val = 0; // sinal de validação
//-------------------------------------------------------------------------

// Structure example to receive data
// Must match the sender structure
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

float fila[3] = {0, 0, 0};

void sinal() {
  if (digitalRead(15) == HIGH) {
    //GPIO.out_w1ts = ((uint32_t)1 << 22);
    digitalWrite(15, LOW);
  } else {
    //GPIO.out_w1ts = ((uint32_t)0 << 22);
    digitalWrite(15, HIGH);
  }
}

void Button() {

  // faz a leitura do pino D1 (no nosso caso, o botão está ligado nesse pino)
  byte valor1 = digitalRead(BUT);

  // checa se o botão está pressionado
  if (valor1 == LOW) {
    //Serial.println("PRESSIONOU");
    bt = 1;
  }
  else {
    bt = 0;

  }

}

void func_fila(float x) {
  //confere();
  for (int i = 2; i >= 0; i--) {
    fila[i] = fila[i - 1];
  }
  fila[0] = x;
  //confere();
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  //if (interruptCounter > 0) {

  //portENTER_CRITICAL(&timerMux);
  //interruptCounter--;
  //portEXIT_CRITICAL(&timerMux);
  memcpy(&myData, incomingData, sizeof(myData));
  int taX = atoi(myData.aX);
  int taY = atoi(myData.aY);
  int taZ = atoi(myData.aZ);
  int tgX = atoi(myData.gX);
  int tgY = atoi(myData.gY);
  int tgZ = atoi(myData.gZ);

  //sinal();
//  char dados[50];
  Button();
//  snprintf(dados, 50, "%05d,%05d,%05d,%05d,%05d,%05d,%d", taX, taY, taZ, tgX, tgY, tgZ, bt);
//  Serial.println(dados);
  
  float mag = pow((taX/100.0),2) + pow((taY/100.0),2) + pow((taZ/100.0),2); 
  func_fila(mag);
  
  //Serial.println();
  
  //float x = float(taY);
  //float input[1] = { x };
  float predicted= tf.predict(fila);
  float output[2] = { 0 };
  tf.predict(fila, output);
  if (output[1] > 0.45){
     digitalWrite(LED, HIGH);
     digitalWrite(MOT, HIGH);
     val = 1;
  }else{
     digitalWrite(LED, LOW);
     digitalWrite(MOT, LOW);
     val = 0;
  }
//  Serial.print("Saida:");
//  Serial.print(output[0]);
//  Serial.print("----");
//  Serial.print(output[1]);
//  Serial.print("----");
//  Serial.print("Botão:");
//  Serial.print(bt);
//  Serial.print("----");
//  Serial.print("Validação:");
//  Serial.print(val);
//  Serial.print("----");
//  Serial.print("mag:");
//  Serial.println(mag);
  char dados[57];
  snprintf(dados, 57, "%05d,%05d,%05d,%05d,%05d,%05d,%d,%d", taX, taY, taZ, tgX, tgY, tgZ, bt,val);
  Serial.println(dados);

}
void IRAM_ATTR onTime() {
  //esp_now_register_recv_cb(OnDataRecv);
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void startTimer() {
  // inicialização do timer. Parametros:
  /* 0 - seleção do timer a ser usado, de 0 a 3.
    80 - prescaler. O clock principal do ESP32 é 80MHz. Dividimos por 80 para ter 1us por tick.
    true - true para contador progressivo, false para regressivo
  */
  timer = timerBegin(0, 80, true);

  /*conecta à interrupção do timer
    - timer é a instância do hw_timer
    - endereço da função a ser chamada pelo timer
    - edge=true gera uma interrupção
  */
  timerAttachInterrupt(timer, &onTime, true);

  /* - o timer instanciado no inicio
     - o valor em us para 1s
     - auto-reload. true para repetir o alarme
  */
  timerAlarmWrite(timer, 5000, true);

  //ativa o alarme
  timerAlarmEnable(timer);
}

void setup() {

  Serial.begin(115200);
  //Serial.println("1");

  if (fila == NULL)
  {
    Serial.println("Erro ao criar a fila");
  }
  //startTimer();
  //Serial.println("2");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Sinal digital transistor (Saída do ESP)
  //pinMode(pinTransistor, OUTPUT);
  //Sinal do botao D1
  pinMode(BUT, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  // inicia como motor desligado
  pinMode(MOT, OUTPUT);
  //digitalWrite(MOT, HIGH);
  // Init ESP-NOW
  //Serial.println("3");
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //Serial.println("4");
  tf.begin(acel_model);
  //Serial.println("5");
  // check if model loaded fine
  if (!tf.isOk()) {
    Serial.print("ERROR: ");
    Serial.println(tf.getErrorMessage());
    //Serial.println("6");
    while (true) delay(1000);
    //Serial.println("7");
  }
  //Serial.println("8");

  esp_now_register_recv_cb(OnDataRecv);  //Registre uma função de retorno de chamada que é acionada ao receber dados. Quando os dados são recebidos via ESP-NOW, uma função é chamada.
}






void loop() {

  //  //startTimer();
  //  if (interruptCounter > 0) {
  //
  //    portENTER_CRITICAL(&timerMux);
  //    interruptCounter--;
  //    portEXIT_CRITICAL(&timerMux);
  //
  //    esp_now_register_recv_cb(OnDataRecv);
  //
  //  }
}

#include <Arduino.h>

#define PINO_DIM    26
#define PINO_ZC     27
#define maxBrightness 800 // brilho maximo em us
#define minBrightness 7500 // brilho minimo em us
#define TRIGGER_TRIAC_INTERVAL 20 // tempo quem que o triac fica acionado
#define IDLE -1

#define pino_botao_up 4  // pino quem que o botao de aumentar o brilho esta conectado
#define pino_botao_down 2 // pino que o botao de diminuir o brilho esta conectado

//variaveis globais
int brilho = 0;
int brilho_convertido = 0;

unsigned long ultimo_millis1 = 0; 
unsigned long ultimo_millis2 = 0; 
unsigned long debounce_delay = 100;

hw_timer_t * timerToPinHigh;
hw_timer_t * timerToPinLow;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile bool isPinHighEnabled = false;
volatile long currentBrightness = minBrightness;

void IRAM_ATTR ISR_turnPinLow(){ // desliga o pino dim
  portENTER_CRITICAL_ISR(&mux); // desativa interrupçoes
    digitalWrite(PINO_DIM, LOW);
    isPinHighEnabled = false;
  portEXIT_CRITICAL_ISR(&mux); // ativa as interrupçoes novamente
}

void IRAM_ATTR setTimerPinLow(){ // executa as configuracoes de pwm e aplica os valores da luminosidade ao dimmer no tempo em que ra ficar em low
  timerToPinLow = timerBegin(1, 80, true);
  timerAttachInterrupt(timerToPinLow, &ISR_turnPinLow, true);
  timerAlarmWrite(timerToPinLow, TRIGGER_TRIAC_INTERVAL, false);
  timerAlarmEnable(timerToPinLow);
}

void IRAM_ATTR ISR_turnPinHigh(){ // liga o pino dim
  portENTER_CRITICAL_ISR(&mux);  // desativa interrupçoes
    digitalWrite(PINO_DIM, HIGH); 
    setTimerPinLow();
  portEXIT_CRITICAL_ISR(&mux); // ativa as interrupçoes novamente
}

void IRAM_ATTR setTimerPinHigh(long brightness){ // executa as configuracoes de pwm e aplica os valores da luminosidade ao dimmer no tempo que ira ficar em high
  isPinHighEnabled = true;
  timerToPinHigh = timerBegin(1, 80, true);
  timerAttachInterrupt(timerToPinHigh, &ISR_turnPinHigh, true);
  timerAlarmWrite(timerToPinHigh, brightness, false);
  timerAlarmEnable(timerToPinHigh);
}

void IRAM_ATTR ISR_zeroCross()  {// funçao que é chamada ao dimmer registrar passagem por 0
  if(currentBrightness == IDLE) return;
  portENTER_CRITICAL_ISR(&mux); // desativa interrupçoes
    if(!isPinHighEnabled){
       setTimerPinHigh(currentBrightness); // define o brilho
    }
  portEXIT_CRITICAL_ISR(&mux); // ativa as interrupçoes novamente
}

void turnLightOn(){ // liga o dimmer no brilho maximo
  portENTER_CRITICAL(&mux);// desativa interrupçoes
    currentBrightness = maxBrightness;
    digitalWrite(PINO_DIM, HIGH);
  portEXIT_CRITICAL(&mux);// ativa as interrupçoes novamente
}

void turnLightOff(){// deliga o dimmer
  portENTER_CRITICAL(&mux); // desativa interrupçoes
    currentBrightness = IDLE;
    digitalWrite(PINO_DIM, LOW);
  portEXIT_CRITICAL(&mux); // ativa as interrupçoes novamente
}

void setup() {
  Serial.begin(115200);//inicia a serial

  currentBrightness = IDLE;

  pinMode(PINO_ZC,  INPUT_PULLUP);
  pinMode(PINO_DIM, OUTPUT);
  digitalWrite(PINO_DIM, LOW);
  attachInterrupt(digitalPinToInterrupt(PINO_ZC), ISR_zeroCross, RISING);

  Serial.println("Controlando dimmer com esp32");

   pinMode(pino_botao_up, INPUT);
   pinMode(pino_botao_down, INPUT);
}

void loop() {
 Serial.println(brilho); // mostra a quantidade de brilho atual

 bool leitura_up = digitalRead(pino_botao_up);

  if ((millis() - ultimo_millis1) > debounce_delay) { // se ja passou determinado tempo que o botao foi precionado
    ultimo_millis1 = millis();
      if (leitura_up == HIGH) { // e o botao estiver precionado
        brilho++; // aumente o brilho
        brilho = constrain(brilho, 0, 100); // limita a variavel
        brilho_convertido = map(brilho, 100, 0, maxBrightness, minBrightness); //converte a luminosidade em microsegundos
         portENTER_CRITICAL(&mux); //desliga as interrupçoes
            currentBrightness = brilho_convertido; // altera o brilho
         portEXIT_CRITICAL(&mux);// liga as interrupçoes
    }
  }

  bool leitura_down = digitalRead(pino_botao_down);
  if ((millis() - ultimo_millis2) > debounce_delay) { // se ja passou determinado tempo que o botao foi precionado
    ultimo_millis2 = millis();
      if (leitura_down == HIGH) {// e o botao estiver precionado
        brilho--;// diminui o brilho
        brilho = constrain(brilho, 0, 100);// limita a variavel
          brilho_convertido = map(brilho, 100, 0, maxBrightness, minBrightness);//converte a luminosidade em microsegundos
         portENTER_CRITICAL(&mux); //desliga as interrupçoes
            currentBrightness = brilho_convertido; // altera o brilho
         portEXIT_CRITICAL(&mux);// liga as interrupçoes
    }
  }
}

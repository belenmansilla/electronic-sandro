// ==========================================
// PULSADOR ACTIVO -> RECIBE SEÑAL DE CONTROL PARA ADELANTE, ATRAS, IZQUIERDA Y DERECHA
// ==========================================

#include <IRremote.hpp>

// ==========================================
// PINES
// ==========================================
#define PIN_IR 4      
#define PIN_BOTON 23  // KEY2

// Semáforo
#define LED1 19 // Rojo: Esperando (Desarmado)
#define LED2 18 // Verde: Activo (Armado)
#define LED3 21 // Azul: Motor en movimiento

// Motores
#define PWMA 32 
#define AIN1 26
#define AIN2 25
#define PWMB 13 
#define BIN1 27
#define BIN2 14 

// ==========================================
// CODIGOS IR (mismos botones que ya documentaste)
// ==========================================
#define CODIGO_ADELANTE  0xE619FF00
#define CODIGO_ATRAS     0xE916FF00
#define CODIGO_IZQUIERDA 0xF807FF00
#define CODIGO_DERECHA   0xEA15FF00
#define CODIGO_PLAY      0xF609FF00   // ahora funciona como freno/STOP

// ==========================================
// VARIABLES
// ==========================================
bool sandroArmado = false;      
bool motorEnMovimiento = false;   

int velocidadLenta = 60;   // avanzar / retroceder
int velocidadGiro  = 60;   // izquierda / derecha (podés separarla si querés otra velocidad)

unsigned long tiempoAnterior = 0;
bool estadoLED3 = false;

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  detenerMotores();

  pinMode(PIN_BOTON, INPUT);
  
  IrReceiver.begin(PIN_IR, DISABLE_LED_FEEDBACK);
}

void loop() {
  // ==============================================================
  // LECTURA DEL PULSADOR (ON / OFF)
  // ==============================================================
  if (digitalRead(PIN_BOTON) == HIGH) {
    delay(50); // Antirrebote
    if (digitalRead(PIN_BOTON) == HIGH) {
      
      sandroArmado = !sandroArmado; 
      
      if (sandroArmado) {
        digitalWrite(LED1, LOW);   
        digitalWrite(LED2, HIGH);  
      } else {
        detenerMotores();
        motorEnMovimiento = false;
        digitalWrite(LED1, HIGH);  
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
      }
      
      while(digitalRead(PIN_BOTON) == HIGH); 
    }
  }

  // ==============================================================
  // ESTADO: ARMADO Y ESCUCHANDO IR
  // ==============================================================
  if (sandroArmado) {
    
    if (IrReceiver.decode()) {
      uint32_t codigo_recibido = IrReceiver.decodedIRData.decodedRawData;
      
      if (codigo_recibido == CODIGO_ADELANTE) {
        avanzar(velocidadLenta);
        motorEnMovimiento = true;
      }
      else if (codigo_recibido == CODIGO_ATRAS) {
        retroceder(velocidadLenta);
        motorEnMovimiento = true;
      }
      else if (codigo_recibido == CODIGO_IZQUIERDA) {
        girarIzquierda(velocidadGiro);
        motorEnMovimiento = true;
      }
      else if (codigo_recibido == CODIGO_DERECHA) {
        girarDerecha(velocidadGiro);
        motorEnMovimiento = true;
      }
      else if (codigo_recibido == CODIGO_PLAY) {
        detenerMotores();
        motorEnMovimiento = false;
        digitalWrite(LED3, LOW);
      }
      
      delay(200);
      IrReceiver.resume();
    }

    // EFECTO VISUAL: PARPADEO LED3
    if (motorEnMovimiento) {
      if (millis() - tiempoAnterior >= 150) { 
        tiempoAnterior = millis();
        estadoLED3 = !estadoLED3; 
        digitalWrite(LED3, estadoLED3);
      }
    }
  }
}

// ==========================================
// FUNCIONES DE MOVIMIENTO
// ==========================================
void avanzar(int velocidad) {
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);
  
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void retroceder(int velocidad) {
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);
  
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void girarIzquierda(int velocidad) {
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);

  // Motor derecho (A) hacia adelante + izquierdo (B) hacia atrás -> pivota a la izquierda
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void girarDerecha(int velocidad) {
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);

  // Motor derecho (A) hacia atrás + izquierdo (B) hacia adelante -> pivota a la derecha
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void detenerMotores() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
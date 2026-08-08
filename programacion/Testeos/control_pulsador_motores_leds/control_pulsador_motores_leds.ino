// ==========================================
// PULSADOR ACTIVO -> RECIBE SEÑAL DE CONTROL PARA QUE VAYA PARA ADELANTE Y PARA ATRÁS 
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
#define PWMA 33 
#define AIN1 26
#define AIN2 25
#define PWMB 13 
#define BIN1 27
#define BIN2 14 

// ==========================================
// VARIABLES
// ==========================================
bool sandroArmado = false;      
bool moviendoAdelante = true;     
bool motorEnMovimiento = false;   

// ¡Velocidad al mínimo! (Ajustar si el motor zumba pero no gira)
int velocidadLenta = 35;          

unsigned long tiempoAnterior = 0;
bool estadoLED3 = false;

void setup() {
  // LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // ESTADO INICIAL: Sandro Desarmado
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  // Motores apagados
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  detenerMotores();

  // BOTÓN
  pinMode(PIN_BOTON, INPUT);
  
  // IR
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
      
      if (codigo_recibido == 0x9D620707) {
        motorEnMovimiento = true; 
        
        if (moviendoAdelante) {
          avanzar(velocidadLenta);
        } else {
          retroceder(velocidadLenta);
        }
        moviendoAdelante = !moviendoAdelante; 
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
// FUNCIONES DE MOVIMIENTO (CORREGIDAS POR SOFTWARE)
// ==========================================
void avanzar(int velocidad) {
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);
  
  // Motor A normal
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  
  // Motor B invertido por software para que acompañe al A
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void retroceder(int velocidad) {
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);
  
  // Motor A invertido
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  
  // Motor B normal (lo invertimos respecto a avanzar)
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void detenerMotores() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
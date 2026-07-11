#include <IRremote.hpp>
#include "BluetoothSerial.h"

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
// VARIABLES Y OBJETOS
// ==========================================
BluetoothSerial SerialBT;

bool sandroArmado = false;      
bool motorEnMovimiento = false;   

// Velocidad base para pruebas
int velocidadLenta = 50; 

// Control de parpadeo del LED
unsigned long tiempoAnterior = 0;
bool estadoLED3 = false;

void setup() {
  // Inicializamos Bluetooth
  SerialBT.begin("Sandro_BT");
  
  // LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // ESTADO INICIAL
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  // Motores
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  detenerMotores();

  // BOTÓN e IR
  pinMode(PIN_BOTON, INPUT);
  IrReceiver.begin(PIN_IR, DISABLE_LED_FEEDBACK);
}

void loop() {
  // ==============================================================
  // 1. LECTURA DEL PULSADOR (ARMADO / DESARMADO)
  // ==============================================================
  if (digitalRead(PIN_BOTON) == HIGH) {
    delay(50); 
    if (digitalRead(PIN_BOTON) == HIGH) {
      
      sandroArmado = !sandroArmado; 
      
      if (sandroArmado) {
        digitalWrite(LED1, LOW);   
        digitalWrite(LED2, HIGH);  
        SerialBT.println("\n[SISTEMA] SANDRO ARMADO - Esperando comandos IR...");
      } else {
        detenerMotores();
        motorEnMovimiento = false;
        digitalWrite(LED1, HIGH);  
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        SerialBT.println("\n[SISTEMA] SANDRO DESARMADO - Motores bloqueados.");
      }
      
      while(digitalRead(PIN_BOTON) == HIGH); 
    }
  }

  // ==============================================================
  // 2. ESTADO: ARMADO Y ESCUCHANDO IR
  // ==============================================================
  if (sandroArmado) {
    
    if (IrReceiver.decode()) {
      uint32_t codigo_recibido = IrReceiver.decodedIRData.decodedRawData;
      
      if (codigo_recibido != 0) {
        
        switch(codigo_recibido) {
          case 0xE619FF00: // Vol+
            SerialBT.println("-> Comando: AVANZAR CONTINUO");
            motorEnMovimiento = true;
            avanzar(velocidadLenta);
            break;

          case 0xE916FF00: // Vol-
            SerialBT.println("-> Comando: RETROCEDER CONTINUO");
            motorEnMovimiento = true;
            retroceder(velocidadLenta);
            break;

          case 0xEA15FF00: // CH+
            SerialBT.println("-> Comando: GIRO DERECHA (90 grados) Y AVANZA");
            motorEnMovimiento = true;
            girarDerecha(velocidadLenta);
            delay(300); // El tiempo que tarda en hacer los 90 grados
            avanzar(velocidadLenta); // Retoma el avance
            SerialBT.println("   [Retomando avance automatico]");
            break;

          case 0xF807FF00: // CH-
            SerialBT.println("-> Comando: GIRO IZQUIERDA (90 grados) Y AVANZA");
            motorEnMovimiento = true;
            girarIzquierda(velocidadLenta);
            delay(300); // El tiempo que tarda en hacer los 90 grados
            avanzar(velocidadLenta); // Retoma el avance
            SerialBT.println("   [Retomando avance automatico]");
            break;

          case 0xF609FF00: // Play
            SerialBT.println("-> Comando: STOP / DETENER");
            detenerMotores();
            motorEnMovimiento = false;
            break;
            
          default:
            SerialBT.print("Codigo no asignado: 0x");
            SerialBT.println(codigo_recibido, HEX);
            break;
        }
      }
      
      delay(150);
      IrReceiver.resume();
    }

    // ==============================================================
    // 3. EFECTO VISUAL: PARPADEO LED3 (Baliza de movimiento)
    // ==============================================================
    if (motorEnMovimiento) {
      if (millis() - tiempoAnterior >= 150) { 
        tiempoAnterior = millis();
        estadoLED3 = !estadoLED3; 
        digitalWrite(LED3, estadoLED3);
      }
    } else {
      digitalWrite(LED3, LOW); // Aseguramos que se apague si no se mueve
    }
  }
}

// ==========================================
// FUNCIONES DE MOVIMIENTO
// ==========================================
void avanzar(int velocidad) {
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
}

void retroceder(int velocidad) {
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
}

void girarDerecha(int velocidad) {
  // Motor Izquierdo (A) avanza, Motor Derecho (B) retrocede
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); // B retrocede
}

void girarIzquierda(int velocidad) {
  // Motor Izquierdo (A) retrocede, Motor Derecho (B) avanza
  analogWrite(PWMA, velocidad);
  analogWrite(PWMB, velocidad);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); // B avanza
}

void detenerMotores() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
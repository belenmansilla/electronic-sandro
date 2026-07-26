#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <IRremote.hpp>

// --- PINES DE SENSORES IR ---
#define PIN_EMISORES 22   
#define SENSOR_IR_1 34    // S1 (Izquierda)
#define SENSOR_IR_2 39    // S2 (Centro - Pin VN) 
#define SENSOR_IR_3 36    // S3 (Derecha - Pin VP) 

// --- PINES DE MOTORES (TB6612FNG) ---
#define PWMA 33
#define AIN1 26
#define AIN2 25
#define PWMB 13
#define BIN1 27
#define BIN2 14

// --- PINES DE LEDS INDICADORES ---
#define LED_IZQ 19    // L1 (Indica sensor izquierdo)
#define LED_CENTRO 18 // L2 (Indica sensor centro)
#define LED_DER 21    // L3 (Indica sensor derecho)

// --- PULSADOR DE INICIO Y RECEPTOR ÁRBITRO ---
#define PIN_PULSADOR 23
#define PIN_RECEPTOR_IR 4

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool emisoresEncendidos = true;

// --- VARIABLES DE ESTADO Y CONTROL ---
bool robotArmado = false; // Manejado por pulsador (LEDs encendidos si es false)
bool pausaIR = false;     // Manejado por control (LEDs apagados si es true)
bool ultimoEstadoPulsador = LOW;
unsigned long tiempoAnterior = 0;
unsigned long ultimoDebounce = 0;

// --- MEMORIA DE MOVIMIENTO ---
enum AccionRobot { DETENIDO, ADELANTE, IZQUIERDA, DERECHA };
AccionRobot accionActual = DETENIDO;

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { 
      deviceConnected = false; 
      pServer->startAdvertising(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = String(pCharacteristic->getValue().c_str());
      if (rxValue.length() > 0) {
        char comando = rxValue[0];
        if (comando == '1') {
          emisoresEncendidos = true;
          digitalWrite(PIN_EMISORES, HIGH);
        } else if (comando == '0') {
          emisoresEncendidos = false;
          digitalWrite(PIN_EMISORES, LOW);
        }
      }
    }
};

// --- FUNCIÓN PARA ENVIAR MENSAJES AL CELULAR ---
void enviarMensajeBLE(String mensaje) {
  Serial.println(mensaje); 
  if (deviceConnected) {
    String mensajeFormateado = mensaje + "\n";
    pTxCharacteristic->setValue(mensajeFormateado.c_str());
    pTxCharacteristic->notify();
  }
}

void moverMotores(int velA, int dirA, int velB, int dirB) {
  if (dirA > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (dirA < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, abs(velA));

  if (dirB > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (dirB < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, abs(velB));
}

void apagarLeds() {
  digitalWrite(LED_IZQ, LOW);
  digitalWrite(LED_CENTRO, LOW);
  digitalWrite(LED_DER, LOW);
}

void encenderTodosLeds() {
  digitalWrite(LED_IZQ, HIGH);
  digitalWrite(LED_CENTRO, HIGH);
  digitalWrite(LED_DER, HIGH);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_EMISORES, OUTPUT);
  pinMode(SENSOR_IR_1, INPUT);
  pinMode(SENSOR_IR_2, INPUT);
  pinMode(SENSOR_IR_3, INPUT);
  digitalWrite(PIN_EMISORES, HIGH);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(LED_IZQ, OUTPUT);
  pinMode(LED_CENTRO, OUTPUT);
  pinMode(LED_DER, OUTPUT);
  
  pinMode(PIN_PULSADOR, INPUT);

  IrReceiver.begin(PIN_RECEPTOR_IR, DISABLE_LED_FEEDBACK);

  encenderTodosLeds(); // Estado inicial: Desarmado

  BLEDevice::init("Sandro_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_RX,
                                        BLECharacteristic::PROPERTY_WRITE
                                      );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println(">>> Sandro v2.0 encendido. <<<");
}

void loop() {
  // ==============================================================
  // 1. LECTURA DEL PULSADOR FÍSICO (ARMA / DESARMA EL ROBOT)
  // ==============================================================
  bool estadoPulsadorActual = digitalRead(PIN_PULSADOR);
  
  if (estadoPulsadorActual == HIGH && ultimoEstadoPulsador == LOW) {
    if (millis() - ultimoDebounce > 300) { 
      robotArmado = !robotArmado;
      ultimoDebounce = millis();
      
      if (robotArmado) {
        enviarMensajeBLE(">>> [PULSADOR] ROBOT ARMADO <<<");
        pausaIR = false;         
        accionActual = DETENIDO; 
      } else {
        enviarMensajeBLE(">>> [PULSADOR] ROBOT DESARMADO (ESPERA) <<<");
      }
    }
  }
  ultimoEstadoPulsador = estadoPulsadorActual;

  // ==============================================================
  // 2. VERIFICAR CONTROL REMOTO (PAUSA / REANUDAR)
  // ==============================================================
  if (IrReceiver.decode()) {
    uint32_t codigo_recibido = IrReceiver.decodedIRData.decodedRawData;
    
    if (robotArmado && codigo_recibido == 0xBA45FF00) {
      pausaIR = !pausaIR; 
      
      if (pausaIR) {
        enviarMensajeBLE(">>> [IR] ROBOT EN PAUSA <<<");
        accionActual = DETENIDO; 
      } else {
        enviarMensajeBLE(">>> [IR] ROBOT REANUDADO <<<");
      }
    }
    IrReceiver.resume();
  }

  // ==============================================================
  // 3. MÁQUINA DE ESTADOS Y CONTROL DE MOTORES
  // ==============================================================
  
  if (!robotArmado) {
    // --- ESTADO 1: DESARMADO ---
    encenderTodosLeds();      
    moverMotores(0, 0, 0, 0); 
  } 
  else if (pausaIR) {
    // --- ESTADO 2: PAUSADO POR CONTROL REMOTO ---
    apagarLeds();             
    moverMotores(0, 0, 0, 0); 
  } 
  else {
    // --- ESTADO 3: COMBATE Y MEMORIA DE MOVIMIENTO ---
    unsigned long tiempoActual = millis();
    
    if (tiempoActual - tiempoAnterior >= 35) { 
      tiempoAnterior = tiempoActual;

      digitalWrite(PIN_EMISORES, LOW);
      delay(2); 
      int s1_off = analogRead(SENSOR_IR_1);
      int s2_off = analogRead(SENSOR_IR_2);
      int s3_off = analogRead(SENSOR_IR_3);

      int señal_S1 = 0, señal_S2 = 0, señal_S3 = 0;

      if (emisoresEncendidos) {
        digitalWrite(PIN_EMISORES, HIGH);
        delay(2); 
        int s1_on = analogRead(SENSOR_IR_1);
        int s2_on = analogRead(SENSOR_IR_2);
        int s3_on = analogRead(SENSOR_IR_3);

        señal_S1 = max(0, s1_on - s1_off);
        señal_S2 = max(0, s2_on - s2_off);
        señal_S3 = max(0, s3_on - s3_off);
        
        int umbral_ruido = 100;
        if (señal_S1 < umbral_ruido) señal_S1 = 0;
        if (señal_S2 < umbral_ruido) señal_S2 = 0;
        if (señal_S3 < umbral_ruido) señal_S3 = 0;
      } else {
        digitalWrite(PIN_EMISORES, LOW);
      }

      // LÓGICA DE MEMORIA (Se actualiza si un sensor detecta señal válida)
      if (señal_S2 > 100) {
        accionActual = ADELANTE;
      } 
      else if (señal_S1 > 100) {
        accionActual = IZQUIERDA;
      } 
      else if (señal_S3 > 100) {
        accionActual = DERECHA;
      }

      // EJECUCIÓN DEL MOVIMIENTO GUARDADO (Avance curvo continuo)
      apagarLeds(); 

      if (accionActual == ADELANTE) {
        digitalWrite(LED_CENTRO, HIGH);
        moverMotores(50, -1, 50, -1); // Avanza recto hacia el frente
      } 
      else if (accionActual == IZQUIERDA) {
        digitalWrite(LED_IZQ, HIGH);
        // CURVA A LA IZQUIERDA: 
        // Motor A (Derecho) va rápido (50)
        // Motor B (Izquierdo) va lento pero avanzando (25)
        moverMotores(50, -1, 25, -1); 
      } 
      else if (accionActual == DERECHA) {
        digitalWrite(LED_DER, HIGH);
        // CURVA A LA DERECHA:
        // Motor A (Derecho) va lento pero avanzando (25)
        // Motor B (Izquierdo) va rápido (50)
        moverMotores(25, -1, 50, -1); 
      } 
      else {
        moverMotores(0, 0, 0, 0); // Solo se detiene si nunca vio nada desde que se armó
      }

      // REPORTE DE SENSORES BLE
      if (deviceConnected) {
        String datosSensores = "I:" + String(señal_S1) + " C:" + String(señal_S2) + " D:" + String(señal_S3);
        enviarMensajeBLE(datosSensores);
      }
    }
  }
  
  delay(10);
}
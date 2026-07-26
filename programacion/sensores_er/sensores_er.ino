#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Definición de pines según el esquemático de Sandro
#define PIN_EMISORES 22   // Q5 - Driver de emisores
#define SENSOR_IR_1 34    // S1 (Oponente 1)
#define SENSOR_IR_2 39    // S2 (Pin VN - Oponente 2)
#define SENSOR_IR_3 36    // S3 (Pin VP - Oponente 3)

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool emisoresEncendidos = true;
unsigned long tiempoAnterior = 0;

// UUIDs estándar de "Nordic UART" (Los que usan las apps de terminal BLE)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
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

void setup() {
  Serial.begin(115200);
  pinMode(PIN_EMISORES, OUTPUT);
  pinMode(SENSOR_IR_1, INPUT);
  pinMode(SENSOR_IR_2, INPUT);
  pinMode(SENSOR_IR_3, INPUT);

  // Encendemos los emisores por defecto
  digitalWrite(PIN_EMISORES, HIGH);

  // --- CONFIGURACIÓN BLE ---
  BLEDevice::init("Sandro_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Característica para ENVIAR datos al celular (TX)
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Característica para RECIBIR comandos del celular (RX)
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID_RX,
                                            BLECharacteristic::PROPERTY_WRITE
                                          );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println(">>> Sandro_BLE está listo para conectarse al iPhone! <<<");
}

void loop() {
  if (deviceConnected) {
    unsigned long tiempoActual = millis();
    
    // Actualizamos cada 2000 ms (2 segundos) sin congelar la placa
    if (tiempoActual - tiempoAnterior >= 2000) { 
      tiempoAnterior = tiempoActual;

      // 1. APAGAR Y DAR ESPACIO AL SENSOR PARA DESCARGARSE
      digitalWrite(PIN_EMISORES, LOW);
      delay(50); 
      
      int s1_off = analogRead(SENSOR_IR_1);
      int s2_off = analogRead(SENSOR_IR_2);
      int s3_off = analogRead(SENSOR_IR_3);

      int señal_S1 = 0;
      int señal_S2 = 0;
      int señal_S3 = 0;

      // 2. LECTURA CON REBOTE
      if (emisoresEncendidos) {
        digitalWrite(PIN_EMISORES, HIGH);
        delay(5); 
        int s1_on = analogRead(SENSOR_IR_1);
        int s2_on = analogRead(SENSOR_IR_2);
        int s3_on = analogRead(SENSOR_IR_3);

        // 3. LA MATEMÁTICA: Aislar solo nuestra propia luz
        señal_S1 = max(0, s1_on - s1_off);
        señal_S2 = max(0, s2_on - s2_off);
        señal_S3 = max(0, s3_on - s3_off);
        
        // --- FILTRO DE RUIDO (UMBRAL) ---
        int umbral_ruido = 100;
        if (señal_S1 < umbral_ruido) señal_S1 = 0;
        if (señal_S2 < umbral_ruido) señal_S2 = 0;
        if (señal_S3 < umbral_ruido) señal_S3 = 0;

        digitalWrite(PIN_EMISORES, HIGH); 
      } else {
        digitalWrite(PIN_EMISORES, LOW);
      }

      // 4. ENVIAR DATOS AL CELULAR
      String estado = emisoresEncendidos ? "ON" : "OFF";
      
      String mensaje = String("--- SEÑAL LIMPIA ---\n") +
                       "EMISORES: " + estado + "\n" +
                       "I: " + String(señal_S1) + "\n" +
                       "C: " + String(señal_S2) + "\n" +
                       "D: " + String(señal_S3) + "\n----------------\n";

      pTxCharacteristic->setValue(mensaje.c_str());
      pTxCharacteristic->notify();
    }
  }
  
  delay(10); 
}
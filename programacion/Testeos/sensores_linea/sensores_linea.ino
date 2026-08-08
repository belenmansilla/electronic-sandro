#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- PINES DE SENSORES DE LÍNEA ---
#define PIN_LINEA_1 16 // SL1 conectado a RX2
#define PIN_LINEA_2 17 // SL2 conectado a TX2

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
unsigned long tiempoAnterior = 0;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println(">>> ¡Celular conectado! <<<");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println(">>> Celular desconectado. <<<");
      pServer->startAdvertising(); 
    }
};

// Se necesita esta clase aunque esté vacía para que la app no tire error
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      // No hacemos nada, solo escuchamos por formalidad
    }
};

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LINEA_1, INPUT);
  pinMode(PIN_LINEA_2, INPUT);

  BLEDevice::init("Sandro_BLE_Lineas");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Canal TX (Robot -> Celular)
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Canal RX (Celular -> Robot) - Agregado para engañar a la app
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_RX,
                                        BLECharacteristic::PROPERTY_WRITE
                                      );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  
  Serial.println(">>> Monitor de Línea BLE iniciado. Esperando... <<<");
}

void loop() {
  unsigned long tiempoActual = millis();
  
  if (tiempoActual - tiempoAnterior >= 100) { 
    tiempoAnterior = tiempoActual;

    int lecturaLinea1 = digitalRead(PIN_LINEA_1);
    int lecturaLinea2 = digitalRead(PIN_LINEA_2);

    String texto = "L1:" + String(lecturaLinea1) + " L2:" + String(lecturaLinea2);
    
    Serial.println(texto);

    if (deviceConnected) {
      String mensajeBLE = texto + "\n";
      pTxCharacteristic->setValue(mensajeBLE.c_str());
      pTxCharacteristic->notify();
    }
  }
  
  delay(10); 
}
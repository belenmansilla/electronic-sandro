#include <Arduino.h>
#include <IRremote.hpp>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ==========================================
// 1. CONFIGURACIÓN DE HARDWARE (Sandro v1.5)
// ==========================================

// --- PINES DE MOTORES (Driver TB6612FNG) ---
#define PWMA 32           // PWM Motor Derecho[cite: 1]
#define AIN1 26           // Dirección 1 Motor Derecho[cite: 1]
#define AIN2 25           // Dirección 2 Motor Derecho[cite: 1]

#define PWMB 13           // PWM Motor Izquierdo[cite: 1]
#define BIN1 27           // Dirección 1 Motor Izquierdo[cite: 1]
#define BIN2 14           // Dirección 2 Motor Izquierdo[cite: 1]

// --- PINES DE SENSORES ---
#define SENSOR_IZQ 36       // S1 (Oponente Izquierda)[cite: 1]
#define SENSOR_CEN 39       // S2 / VN (Oponente Centro)[cite: 1]
#define SENSOR_DER 34       // S3 / VP (Oponente Derecha)[cite: 1]
#define PIN_EMISORES 22     // PWM Emisores IR (Q5)[cite: 1]

#define PIN_LINEA_IZQ 16    // SL1 (RX2)[cite: 1]
#define PIN_LINEA_DER 17    // SL2 (TX2)[cite: 1]

// --- INTERFAZ ---
#define PIN_RECEPTOR_IR 4   // VS1838B (Árbitro)[cite: 1]
#define LED_IZQ 18          // L1[cite: 1]
#define LED_CEN 19          // L2[cite: 1]
#define LED_DER 21          // L3[cite: 1]

// Variables globales para la Máquina de Estados
int estadoRobot = 0;           // Arranca en 0 (Esperando selección)
int estrategiaSeleccionada = 0; // Guardará el número de la estrategia

// EL SEGURO DE LOS MOTORES: 
// 0 = Totalmente inmovilizado (Seguro para probar sensores).
// 150 = Límite seguro para probar empuj.
int LIMITE_PWM = 40;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristicTX = NULL; // Le cambiamos el nombre a TX
bool deviceConnected = false;

// Variable para el cronómetro del Bluetooth
unsigned long tiempoAnteriorBLE = 0;

// Identificadores exactos que exige Bluefruit Connect
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // Canal de recepción
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // Canal de transmisión

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      pServer->startAdvertising(); 
    }
};
// ==========================================
// 2. INICIALIZACIÓN
// ==========================================
void setup() {
    Serial.begin(115200);

// --- CONFIGURACIÓN BLE UART ---
    BLEDevice::init("Evil_Sandro");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // 1. Creamos el canal TX (Para enviar datos al iPhone)
    pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pCharacteristicTX->addDescriptor(new BLE2902());

    // 2. Creamos el canal RX (Para que la app Bluefruit no tire error)
    BLECharacteristic *pCharacteristicRX = pService->createCharacteristic(
                                             CHARACTERISTIC_UUID_RX,
                                             BLECharacteristic::PROPERTY_WRITE
                                           );
    
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    // 1. Configurar entradas de sensores
    pinMode(SENSOR_IZQ, INPUT);
    pinMode(SENSOR_CEN, INPUT);
    pinMode(SENSOR_DER, INPUT);
    pinMode(PIN_LINEA_IZQ, INPUT);
    pinMode(PIN_LINEA_DER, INPUT);
    pinMode(PIN_RECEPTOR_IR, INPUT);

    // 2. Configurar salidas de motores
    pinMode(PWMA, OUTPUT); 
    pinMode(AIN1, OUTPUT); 
    pinMode(AIN2, OUTPUT);
    
    pinMode(PWMB, OUTPUT); 
    pinMode(BIN1, OUTPUT); 
    pinMode(BIN2, OUTPUT);

    // 3. Configurar salidas de interfaz y emisores
    pinMode(PIN_EMISORES, OUTPUT);
    pinMode(LED_IZQ, OUTPUT);
    pinMode(LED_CEN, OUTPUT);
    pinMode(LED_DER, OUTPUT);

    // 4. Estado seguro inicial (Todo apagado)
    digitalWrite(PIN_EMISORES, LOW);
    digitalWrite(LED_IZQ, LOW);
    digitalWrite(LED_CEN, LOW);
    digitalWrite(LED_DER, LOW);
    
    // Acá llamaremos a moverMotores(0, 0) cuando la creemos
    // Inicializamos el receptor IR en el pin D4
    IrReceiver.begin(PIN_RECEPTOR_IR, DISABLE_LED_FEEDBACK);


    Serial.println("Sandro v1.5 Inicializado. Esperando órdenes...");
}

void loop() {
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoAnteriorBLE >= 1000) {
        tiempoAnteriorBLE = tiempoActual; 
        
        int linIzq = digitalRead(PIN_LINEA_IZQ);
        int linDer = digitalRead(PIN_LINEA_DER);
        int opIzq = analogRead(SENSOR_IZQ);
        int opCen = analogRead(SENSOR_CEN);
        int opDer = analogRead(SENSOR_DER);

        String telemetria = "E:" + String(estadoRobot) + 
                            " L[" + String(linIzq) + "|" + String(linDer) + "] " +
                            "Op[I:" + String(opIzq) + " C:" + String(opCen) + " D:" + String(opDer) + "]";
        enviarMensajeBLE(telemetria);
    }

    switch (estadoRobot) {
        
        // ==================================================
        // ESTADO 0: ESPERANDO QUE ELIJAS LA ESTRATEGIA (1 al 9)
        // ==================================================
        case 0: 
            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;


                if (codigo == 0xF30CFF00) { // BOTON 1
                    estrategiaSeleccionada = 1;
                    estadoRobot = 1; 
                    digitalWrite(LED_IZQ, HIGH); 
                    enviarMensajeBLE("Estrategia 1 (Busqueda Lenta). Esperando PLAY...");
                }
                else if (codigo == 0xE718FF00) { // BOTON 2
                    estrategiaSeleccionada = 2;
                    estadoRobot = 1; 
                    digitalWrite(LED_CEN, HIGH);
                    enviarMensajeBLE("Estrategia 2 (Patrulla Borde). Esperando PLAY...");
                }                
                else if (codigo == 0xBC43FF00) { // BOTON "E/R" 
                    estadoRobot = 3; 
                    digitalWrite(LED_DER, HIGH); 
                    enviarMensajeBLE("Telemetría Exclusiva (Motores bloqueados)");
                }
                IrReceiver.resume(); // Prepara el receptor para el próximo botón
            }
            break;

        // ==================================================
        // ESTADO 1: ESPERANDO EL BOTÓN "PLAY" PARA ARRANCAR
        // ==================================================
        case 1: 
            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
                enviarMensajeBLE(">>> Botón IR detectado: " + String(codigo, HEX));    
                if (codigo == 0xF609FF00) { // Si apretaste "PLAY"
                    // Apagamos el LED de confirmación
                    digitalWrite(LED_IZQ, LOW); 
                    digitalWrite(LED_CEN, LOW);
                    enviarMensajeBLE("¡INICIANDO EN 5 SEGUNDOS!");
                    
                    // Parpadeo reglamentario de 5 segundos
                    for(int i = 0; i < 10; i++) {
                        digitalWrite(LED_CEN, HIGH);
                        delay(250);
                        digitalWrite(LED_CEN, LOW);
                        delay(250);
                    }
                    
                    estadoRobot = 2; // Entramos en modo combate
                }
                
                IrReceiver.resume();
            }
            break;

        // ==================================================
        // ESTADO 2: MODO COMBATE
        // ==================================================
        case 2: 
            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
                
                if (codigo == 0xF609FF00 || codigo == 0xF30CFF00 || codigo == 0xBC43FF00) { 
                    moverMotores(0, 0);
                    estadoRobot = 0;
                    digitalWrite(LED_IZQ, LOW); digitalWrite(LED_CEN, LOW); digitalWrite(LED_DER, LOW);
                    enviarMensajeBLE("¡ROBOT DETENIDO!");
                }
                IrReceiver.resume();
            }
            
            // Lógica de Combate
            if (estadoRobot == 2) {
                if (estrategiaSeleccionada == 1) rutinaBusquedaDeAPoco();
                else if (estrategiaSeleccionada == 2) rutinaPatrullaYAtaca();
            }
            break;
            
        // ==================================================
        // ESTADO 3: TELEMETRÍA POR BLUETOOTH 
        // ==================================================
        case 3:
        // El estado 3 ahora queda libre de interrupciones.
            // Solo escucha si querés salir de la telemetría.
            moverMotores(0, 0);
            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
                if (codigo == 0xF30CFF00 || codigo == 0xF609FF00) { // Botón "1" o "PLAY" para salir
                    estadoRobot = 0; digitalWrite(LED_DER, LOW); 
                    enviarMensajeBLE("Saliendo de telemetría.");
                }
                IrReceiver.resume();
            }
            break;
    }
}

void rutinaBusquedaDeAPoco() {
    int pos = buscarOponente(); 

    if (pos == 0) {
        // No ve a nadie -> Gira lento, avanza un poco
        moverMotores(120, -120); delay(100);             
        moverMotores(80, 80);    delay(50);             
        moverMotores(0, 0);      delay(20);
    } 
    else if (pos == 2) { // CENTRO -> Ataque
        moverMotores(255, 255);
    }
    else if (pos == 1) { // IZQUIERDA -> Gira Izquierda
        moverMotores(-120, 120); 
    }
    else if (pos == 3) { // DERECHA -> Gira Derecha
        moverMotores(120, -120);
    }
}

void rutinaPatrullaYAtaca() {
    // 1. PRIORIDAD ABSOLUTA: No caerse de la mesa (Línea Negra = 1)
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    
    if (lineaIzq == 1 || lineaDer == 1) {
        moverMotores(-100, -100); delay(200); // Retrocede
        moverMotores(100, -100);  delay(300); // Gira
        return; // Corta la función acá para no ejecutar lo de abajo
    }

    // 2. Si el piso es seguro, buscamos oponente
    int pos = buscarOponente(); 

    if (pos == 2) { // Lo ve de frente
        moverMotores(255, 255);
    }
    else if (pos == 1) { // Lo ve a la Izquierda
        moverMotores(-120, 120); 
    }
    else if (pos == 3) { // Lo ve a la Derecha
        moverMotores(120, -120);
    }
    else { // No ve a nadie, avanza suave patrullando
        moverMotores(60, 60);
    }
}

int buscarOponente() {
    int i = analogRead(SENSOR_IZQ);
    int c = analogRead(SENSOR_CEN);
    int d = analogRead(SENSOR_DER);
    int umbral = 2000;

    // Si todos están por encima del umbral, no hay nadie cerca
    if (i > umbral && c > umbral && d > umbral) {
        return 0; 
    }
    
    // Si llegamos acá, al menos uno cruzó el umbral. 
    // ¿Cuál tiene el valor MAS BAJO (El oponente está más cerca de él)?
    if (c <= i && c <= d) return 2; // El Centro es el más bajo
    if (i <= c && i <= d) return 1; // La Izquierda es el más bajo
    if (d <= c && d <= i) return 3; // La Derecha es el más bajo
    
    return 0; 
}

void moverMotores(int velIzq, int velDer) {
    if (velIzq > LIMITE_PWM) velIzq = LIMITE_PWM;
    if (velIzq < -LIMITE_PWM) velIzq = -LIMITE_PWM;
    if (velDer > LIMITE_PWM) velDer = LIMITE_PWM;
    if (velDer < -LIMITE_PWM) velDer = -LIMITE_PWM;

    // --- Motor Izquierdo (B) ---
    if (velIzq >= 0) {
        digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, velIzq);
    } else {
        digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, -velIzq); 
    }
    // --- Motor Derecho (A) ---
    if (velDer >= 0) {
        digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, velDer);
    } else {
        digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, -velDer);
    }
}

void enviarMensajeBLE(String mensaje) {
    if (deviceConnected) {
        mensaje += "\n"; 
        pCharacteristicTX->setValue(mensaje.c_str());
        pCharacteristicTX->notify();
    }
}
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
int LIMITE_PWM = 255;

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
        tiempoAnteriorBLE = tiempoActual; // Reseteamos el cronómetro
        
        if (estadoRobot == 3) {
            // Si estamos en telemetría, leemos y mandamos los sensores acá
            int linIzq = digitalRead(PIN_LINEA_IZQ);
            int linDer = digitalRead(PIN_LINEA_DER);
            int opIzq = analogRead(SENSOR_IZQ);
            int opCen = analogRead(SENSOR_CEN);
            int opDer = analogRead(SENSOR_DER);

            String telemetria = "Lin[I:" + String(linIzq) + " D:" + String(linDer) + "] " +
                                "Op[I:" + String(opIzq) + " C:" + String(opCen) + " D:" + String(opDer) + "]";
            enviarMensajeBLE(telemetria);
        } else {
            // Si estamos en otro estado, mandamos el ping de menú
            enviarMensajeBLE("Sandro -> Estado: " + String(estadoRobot) + " | Estrat: " + String(estrategiaSeleccionada));
        }
    }

    switch (estadoRobot) {
        
        // ==================================================
        // ESTADO 0: ESPERANDO QUE ELIJAS LA ESTRATEGIA (1 al 9)
        // ==================================================
        case 0: 
            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
                enviarMensajeBLE(">>> Botón IR detectado: " + String(codigo, HEX));

                if (codigo == 0xF30CFF00) { // Botón "1"
                    estrategiaSeleccionada = 1;
                    estadoRobot = 1; 
                    digitalWrite(LED_IZQ, HIGH); 
                    enviarMensajeBLE("¡Estrategia 1 Cargada! Esperando PLAY...");
                }
                else if (codigo == 0xE718FF00) { // BOTON 2
                    estrategiaSeleccionada = 2;
                    estadoRobot = 1; 
                    digitalWrite(LED_CEN, HIGH);
                    enviarMensajeBLE("¡Estrategia 2 (Borde) Cargada! Esperando PLAY...");
                }                
                else if (codigo == 0xBC43FF00) { // BOTON "E/R" 
                    estadoRobot = 3; 
                    digitalWrite(LED_DER, HIGH); 
                    enviarMensajeBLE("Cambiando a Telemetría de Sensores...");
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
        // ESTADO 2: MODO COMBATE (Acá ya no importa el control)
        // ==================================================
        case 2: 
                if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
                
                if (codigo == 0xBC43FF00) { // Botón E/R (Ir a Telemetría)
                    moverMotores(0, 0);     // Freno de emergencia
                    estadoRobot = 3; 
                    digitalWrite(LED_DER, HIGH); 
                    enviarMensajeBLE("Combate abortado. Entrando a Telemetría.");
                }
                else if (codigo == 0xF30CFF00) { // Botón 1 (Volver al menú principal)
                    moverMotores(0, 0);
                    estadoRobot = 0;
                    digitalWrite(LED_IZQ, LOW); 
                    digitalWrite(LED_CEN, LOW);
                    enviarMensajeBLE("Combate abortado. Volviendo al menú.");
                }
                IrReceiver.resume();
            }
            
            // Evaluamos las estrategias directamente
            if (estrategiaSeleccionada == 1) {
                rutinaBusquedaDeAPoco();
            }
            else if (estrategiaSeleccionada == 2) {
                rutinaPruebaBorde();
            }
            
            // } // Cierre del else comentado
            break;
            
        // ==================================================
        // ESTADO 3: TELEMETRÍA POR BLUETOOTH (CALIBRACIÓN)
        // ==================================================
        case 3:
        // El estado 3 ahora queda libre de interrupciones.
            // Solo escucha si querés salir de la telemetría.
            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
                if (codigo == 0xF30CFF00) { // Botón "1" para volver al Menú
                    estadoRobot = 0; 
                    digitalWrite(LED_DER, LOW); 
                    enviarMensajeBLE("Saliendo de telemetría. Volviendo al menú.");
                }
                IrReceiver.resume();
            }
            break;
    }
}

void rutinaBusquedaDeAPoco() {
    int posicionOponente = buscarOponente(); // Le preguntamos a los sensores IR

    if (posicionOponente == 0) {
        // NO VEMOS A NADIE -> Buscamos "de a poco"
        // (Estos números de velocidad y delay los vas a tener que calibrar en la pista)
        
        moverMotores(150, -150); // Gira a la derecha un poco
        delay(100);             // Mantiene el giro por 100ms
        
        moverMotores(100, 100);  // Avanza un poquito hacia adelante
        delay(50);              // Mantiene el avance por 50ms
        
        moverMotores(0, 0);      // Frena un instante para estabilizar la lectura IR
        delay(20);
    } 
    else if (posicionOponente == 2) {
        // LO VEMOS EN EL CENTRO -> ¡Ataque a fondo!
        moverMotores(255, 255);
    }
    else if (posicionOponente == 1) {
        // LO VEMOS A LA IZQUIERDA -> Tracking (corrección suave)
        moverMotores(100, 200); 
    }
    else if (posicionOponente == 3) {
        // LO VEMOS A LA DERECHA -> Tracking (corrección suave)
        moverMotores(200, 100);
    }
}

int buscarOponente() {
    // 1. Leemos el voltaje de los 3 sensores analógicos.
    // En el ESP32, analogRead devuelve un número de 0 a 4095.
    int lecturaIzq = analogRead(SENSOR_IZQ);
    int lecturaCen = analogRead(SENSOR_CEN);
    int lecturaDer = analogRead(SENSOR_DER);

    // 2. Definimos nuestro "umbral". 
    // Como los fototransistores bajan su voltaje cuando detectan el reflejo
    // del infrarrojo, un valor MENOR a 2000 significa que hay un oponente cerca.
    int umbral = 2000;

    // 3. Evaluamos por prioridad (El centro es el más importante para atacar)
    if (lecturaCen < umbral) {
        return 2; // ¡Lo vemos de frente! Devolvemos un 2 y la función termina acá.
    }
    else if (lecturaIzq < umbral) {
        return 1; // ¡Lo vemos a la izquierda! Devolvemos un 1.
    }
    else if (lecturaDer < umbral) {
        return 3; // ¡Lo vemos a la derecha! Devolvemos un 3.
    }

    // 4. Si pasamos todos los 'if' y ninguno vio nada...
    return 0; // Devolvemos un 0 (No hay nadie).
}

bool leerSensoresLinea() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);

    // Si cualquiera de los dos sensores lee 0 (blanco), devolvemos verdadero (hay peligro)
    if (lineaIzq == 0 || lineaDer == 0) {
        return true; 
    } else {
        return false;
    }
}

void rutinaEscape() {
    // 1. Frenamos y retrocedemos rápido a máxima potencia
    moverMotores(-255, -255);
    delay(250); // Mantenemos el retroceso por un cuarto de segundo
    
    // 2. Giramos en el lugar para volver a apuntar al centro del dohyo
    // Motor Izq adelante, Motor Der atrás
    moverMotores(200, -200);
    delay(150); 
    
}

void moverMotores(int velIzq, int velDer) {
    // 1. Recortamos la velocidad si supera el LIMITE_PWM permitido
    if (velIzq > LIMITE_PWM) velIzq = LIMITE_PWM;
    if (velIzq < -LIMITE_PWM) velIzq = -LIMITE_PWM;
    
    if (velDer > LIMITE_PWM) velDer = LIMITE_PWM;
    if (velDer < -LIMITE_PWM) velDer = -LIMITE_PWM;

    // --- CONTROL MOTOR IZQUIERDO (Motor B) ---
    if (velIzq >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, velIzq);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        analogWrite(PWMB, -velIzq); 
    }

    // --- CONTROL MOTOR DERECHO (Motor A) ---
    if (velDer >= 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, velDer);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, -velDer);
    }
}

void rutinaPruebaBorde() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);

    // Lógica invertida para la mesa casera: 0 es seguro (blanco), 1 es peligro (negro)
    if (lineaIzq == 1 || lineaDer == 1) {
        // ¡Tocó la cinta negra! 
        
        // 1. Retrocedemos un poquito para alejarnos del borde y no caernos al girar
        moverMotores(-70, -70);
        delay(200); // 200 milisegundos yendo para atrás
        
        // 2. Giramos a la derecha en su propio eje (Motor Izq adelante, Motor Der atrás)
        moverMotores(70, -70);
        delay(300); // Mantiene el giro por 300 milisegundos
        
        // 3. Frenamos un instante cortito para estabilizar la inercia
        moverMotores(0, 0);
        delay(50);
        
        // Al terminar esto, la función termina y el código vuelve arriba.
        // Como ya no está sobre la línea negra, va a entrar al "else" y seguir avanzando.
    } 
    else {
        // Sigue en lo blanco de la mesa, avanza lento y parejo
        moverMotores(70, 70);
    }
}

void enviarMensajeBLE(String mensaje) {
    if (deviceConnected) {
        mensaje += "\n"; // Salto de línea automático
        pCharacteristicTX->setValue(mensaje.c_str());
        pCharacteristicTX->notify();
    }
}
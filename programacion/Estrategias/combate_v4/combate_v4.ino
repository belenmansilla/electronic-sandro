// ==========================================
// 0. LIBRERIAS
// ==========================================
#include <Arduino.h>
#include <IRremote.hpp>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ==========================================
// 1. CONFIGURACIÓN DE HARDWARE
// ==========================================

// --- PINES DE MOTORES ---
#define PWMA 32           // PWM Motor Derecho
#define AIN1 26           // Dirección 1 Motor Derecho
#define AIN2 25           // Dirección 2 Motor Derecho

#define PWMB 13           // PWM Motor Izquierdo
#define BIN1 27           // Dirección 1 Motor Izquierdo
#define BIN2 14           // Dirección 2 Motor Izquierdo

// --- PINES DE SENSORES ---
#define SENSOR_IZQ 34       // S1 (Oponente Izquierda)
#define SENSOR_CEN 39       // S2 / VN (Oponente Centro)
#define SENSOR_DER 36       // S3 / VP (Oponente Derecha)
#define PIN_EMISORES 22     // PWM Emisores IR (Q5)

#define PIN_LINEA_IZQ 16    // SL1 (RX2)
#define PIN_LINEA_DER 17    // SL2 (TX2)

// --- INTERFAZ ---
#define PIN_RECEPTOR_IR 4   // VS1838B
#define LED_IZQ 18          // L1
#define LED_CEN 19          // L2
#define LED_DER 21          // L3
#define PIN_PULSADOR 23

#define LECTURA_MAXIMA_PLAUSIBLE 400

// Sensores de Linea Negro = 1, Blanco = 0
int SENSOR_LINEA = 0 ;

// Variables globales para la Máquina de Estados
int estadoRobot = 0;          
int estrategiaSeleccionada = 0;

// Motores Limite
int LIMITE_PWM = 200;

// Motores 
int VEL_BUSQUEDA_E1 = 70;
int VEL_BUSQUEDA_E2 = 100;
int VEL_BUSQUEDA_E3 = 100;

// Motores Giro
int VEL_GIRO = 80;

// Motores Ataque
int VEL_ATAQUE_E1 = 100; 
int VEL_ATAQUE_E2 = 150;     
int VEL_ATAQUE_E3 = 180; 

// Motores Seguimiento
int VEL_SEGUIMIENTO_E4 = 80;   // Tenia 50
int VEL_SEGUIMIENTO_E5 = 50; 

// Motores barrido  cuando no ve a nadie
#define TIEMPO_BARRIDO_E4_MS 800
#define TIEMPO_BARRIDO_E5_MS 800
int VEL_CORRECCION_BARRIDO_E4 = 15;   
int VEL_CORRECCION_BARRIDO_E5 = 15; 
unsigned long tiempoBarrido = 0;
int direccionBarrido = 1;          // 1 = barre hacia un lado, -1 = hacia el otro


float señalIzqSuave = 0;
float señalCenSuave = 0;
float señalDerSuave = 0;

#define DURACION_GIRO_MS 150   // cuánto dura cada pulso de giro (E1/E2/E3)
#define DURACION_PAUSA_MS 500  // cuánto se queda quieto después de girar, para estabilizarse

#define LECTURAS_PARA_CONFIRMAR 2          //  lecturas iguales seguidas antes de girar (filtra ruido/parpadeos)
#define DURACION_GIRO_SEGUIMIENTO_MS 120   // duración del pulso de giro decisivo. ----- CALIBRACION -----
                                            
#define DURACION_AVANCE_SEGUIMIENTO_MS 150 // cuánto avanza derecho después de girar, antes de volver a mirar 


int umbralIzq = 100;
int umbralCen = 100;
int umbralDer = 100;

// Muestras promediadas por cada lectura ON/OFF. Más muestras = menos
// ruido = umbral más bajo posible
#define MUESTRAS_SENSOR 4

// Guardamos la última señal calculada de cada sensor para poder
// mandarla por telemetría sin tener que volver a leer.
int señalIzqActual = 0;
int señalCenActual = 0;
int señalDerActual = 0;

// Cada estrategia tiene su propia "fase": 0=buscando, 1=girando, 2=pausa, 3=empujando
int faseE1 = 0; unsigned long tiempoFaseE1 = 0; int direccionE1 = 0;
int faseE2 = 0; unsigned long tiempoFaseE2 = 0; int direccionE2 = 0;
int faseE3 = 0; unsigned long tiempoFaseE3 = 0; int direccionE3 = 0;

// fase: 0=leer/decidir, 1=girando (decisivo), 2=avanzando derecho tras el giro
int faseE4 = 0;
unsigned long tiempoFaseE4 = 0;
int direccionE4 = 0;            // 1=izquierda, 3=derecha (última dirección confirmada)
int ultimaLecturaE4 = 0;
int lecturasConsecutivasE4 = 0; // debounce: cuántas veces seguidas vino la misma lectura no-cero

int faseE5 = 0;
unsigned long tiempoFaseE5 = 0;
int direccionE5 = 0;
int ultimaLecturaE5 = 0;
int lecturasConsecutivasE5 = 0;

int faseE6 = 0;
unsigned long tiempoFaseE6 = 0;


// --- Estrategia 7: Flanqueo por la derecha (embestida por atrás) ---
int faseE7 = 0; // 0=buscar, 1=arco de flanqueo, 2=verificar, 3=embestida
unsigned long tiempoFaseE7 = 0;
int ultimaLecturaE7 = 0;
int lecturasConsecutivasE7 = 0;

#define DURACION_FLANQUEO_MS 500      // cuánto dura el arco. ----- CALIBRACION -----
int VEL_FLANQUEO_EXT = 150;           // rueda exterior del arco (más rápida)
int VEL_FLANQUEO_INT = 110;            // rueda interior del arco (más lenta -> hace doblar a la derecha)
int VEL_EMBESTIDA_E7_MAX = 200;
#define TIEMPO_RAMPA_EMBESTIDA_E7_MS 400

#define DURACION_AVANCE_CUADRADO_MS 800  // cuánto dura cada lado del cuadrado. ----- CALIBRACION -----
#define DURACION_GIRO_90_MS 300          // cuánto tarda en girar realmente 90° con VEL_GIRO. ----- CALIBRACION -----


#define VEL_EMBESTIDA_MAX 180         // a dónde llega la rampa (que no supere LIMITE_PWM)
#define TIEMPO_RAMPA_EMBESTIDA_MS 400 // ms que tarda en pasar de VEL_SEGUIMIENTO a VEL_EMBESTIDA_MAX ----- CALIBRACION -----

// Bluetooth configurado para el celu desde la app 
BLEServer* pServer = NULL;  //activo el servidor
BLECharacteristic* pCharacteristicTX = NULL; //aca le digo que voy a estar guardando una configuracion gigante de bluetooth.
bool deviceConnected = false;
unsigned long tiempoAnteriorBLE = 0;

// para que la app Bluefruit Connect
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


// publicarse cuando desconecto del bluetooth
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

    // --- ble uart conexion ---
    BLEDevice::init("Evil_Sandro");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pCharacteristicTX->addDescriptor(new BLE2902());

    pService->createCharacteristic(
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
    pinMode(PIN_PULSADOR, INPUT_PULLUP);
    pinMode(PIN_EMISORES, OUTPUT);
    pinMode(LED_IZQ, OUTPUT);
    pinMode(LED_CEN, OUTPUT);
    pinMode(LED_DER, OUTPUT);

    // 4. Estado seguro inicial de LEDs
    digitalWrite(LED_IZQ, LOW);
    digitalWrite(LED_CEN, LOW);
    digitalWrite(LED_DER, LOW);

    // Estado seguro inicial: emisor apagado.
    digitalWrite(PIN_EMISORES, LOW);
    
    IrReceiver.begin(PIN_RECEPTOR_IR, DISABLE_LED_FEEDBACK);

    // Medimos el piso de ruido real de cada sensor (mesa vacía)
    calibrarUmbrales();

    Serial.println("Sandro v1.6 Inicializado. Esperando órdenes...");
}

void loop() {
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoAnteriorBLE >= 1000) {
        tiempoAnteriorBLE = tiempoActual; 
        
        int linIzq = digitalRead(PIN_LINEA_IZQ);
        int linDer = digitalRead(PIN_LINEA_DER);
        buscarOponente(); 
        String telemetria =  "E:" + String(estadoRobot) +
                    " Estr:" + String(estrategiaSeleccionada) +
                    " L[" + String(linIzq) + "|" + String(linDer) + "] " +
                    "Señal[I:" + String(señalIzqActual) + " C:" + String(señalCenActual) + " D:" + String(señalDerActual) + "]";
        enviarMensajeBLE(telemetria);
    }

    switch (estadoRobot) {
        
        // ==================================================
        // ESTADO 0: ESPERANDO QUE ESTRATEGIAS DEL 1 AL 9
        // ==================================================
        case 0: 
            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;

                if (codigo == 0xF30CFF00) { // BOTON 1
                    estrategiaSeleccionada = 1;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1; // estado de busqueda hasta el "play"
                    enviarMensajeBLE("Estrategia 1: Busqueda Lenta. Esperando PLAY...");
                }
                else if (codigo == 0xE718FF00) { // BOTON 2
                    estrategiaSeleccionada = 2;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1; 
                    enviarMensajeBLE("Estrategia 2: Busqueda Media. Esperando PLAY...");
                }                
                else if (codigo == 0xA15EFF00) { // BOTON 3 
                    estrategiaSeleccionada = 3;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1; 
                    enviarMensajeBLE("Ataque 3: Progresivo y embestida. Esperando PLAY...");
                }
                else if (codigo == 0xF708FF00) { // BOTON 4 - Seguimiento
                    estrategiaSeleccionada = 4;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1; 
                    enviarMensajeBLE("Estrategia 4: Embestida Media. Esperando PLAY...");
                }
                else if (codigo == 0xE31CFF00) { // BOTON 5 - Embestida Agresiva
                    estrategiaSeleccionada = 5;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1;
                    enviarMensajeBLE("Estrategia 5: Embestida Alta. Esperando PLAY...");
                }
                else if (codigo == 0xA55AFF00) { // BOTON 6 - Patrulla en cuadrado
                    estrategiaSeleccionada = 6;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1;
                    enviarMensajeBLE("Estrategia 6: Patrulla Cuadrado. Esperando PLAY...");
                }
                else if (codigo == 0xBD42FF00) { // BOTON 7- Flanqueo por la derecha
                estrategiaSeleccionada = 7;
                mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                estadoRobot = 1; // pasa por la cuenta regresiva de 5s igual que las demás
                enviarMensajeBLE("Estrategia 7: Flanqueo derecha. Esperando PLAY...");
            }
                else if (codigo == 0xBC43FF00) { // BOTON "E/R" 
                    estadoRobot = 3; 
                    digitalWrite(LED_DER, HIGH);
                    calibrarUmbrales();  
                    enviarMensajeBLE("Telemetría Exclusiva (Motores bloqueados)");
                }
                IrReceiver.resume();
            }
            break;

        // ==================================================
        // ESTADO 1: ARRANQUE NORMAL
        // ==================================================
        case 1: 
            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
                enviarMensajeBLE(">>> Botón IR detectado: " + String(codigo, HEX));    
                if (codigo == 0xF609FF00) { // Si apretaste "PLAY"
                    digitalWrite(LED_IZQ, LOW); 
                    digitalWrite(LED_CEN, LOW);
                    enviarMensajeBLE("¡INICIANDO EN 5 SEGUNDOS!"); // por regla de LNR
                    
                    for(int i = 0; i < 5; i++) {
                        digitalWrite(LED_IZQ, HIGH);
                        digitalWrite(LED_CEN, HIGH);
                        digitalWrite(LED_DER, HIGH);
                        delay(500);
                        digitalWrite(LED_IZQ, LOW);
                        digitalWrite(LED_CEN, LOW);
                        digitalWrite(LED_DER, LOW);
                        delay(500);
                    }
                    // reset de las máquinas de estado de todas las estrategias
                    faseE1 = 0; faseE2 = 0; faseE3 = 0;
                    faseE4 = 0; lecturasConsecutivasE4 = 0; ultimaLecturaE4 = 0;
                    faseE5 = 0; lecturasConsecutivasE5 = 0; ultimaLecturaE5 = 0;
                    faseE6 = 0; tiempoFaseE6 = millis();
                    estadoRobot = 2; // Entramos en modo combate
                }
                
                IrReceiver.resume();
            }
            break;

        // ==================================================
        // ESTADO 2: MODO COMBATE POR ESTRATEGIA 
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
                else if (estrategiaSeleccionada == 3) rutinaAtaqueProgresivo();
                else if (estrategiaSeleccionada == 4) rutinaSeguimiento();
                else if (estrategiaSeleccionada == 5) rutinaEmbestidaAgresiva();
                else if (estrategiaSeleccionada == 6) rutinaCuadrado();
                else if (estrategiaSeleccionada == 7) rutinaFlanqueoDerecha(); 
            }
            break;
            
        // ==================================================
        // ESTADO 3: TELEMETRÍA POR BLUETOOTH 
        // ==================================================
        case 3:
            moverMotores(0, 0); // Motores siempre bloqueados en este estado

            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;

                if (codigo == 0xF30CFF00) { // BOTON 1
                    estrategiaSeleccionada = 1;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1; // estado de busqueda hasta el "play"
                    enviarMensajeBLE("Estrategia 1: Busqueda Lenta. Esperando PLAY...");
                }
                else if (codigo == 0xE718FF00) { // BOTON 2
                    estrategiaSeleccionada = 2;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1; 
                    enviarMensajeBLE("Estrategia 2: Busqueda Media. Esperando PLAY...");
                }                
                else if (codigo == 0xA15EFF00) { // BOTON 3 
                    estrategiaSeleccionada = 3;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1; 
                    enviarMensajeBLE("Ataque 3: progresivo y agresivo. Esperando PLAY...");
                }
                else if (codigo == 0xF708FF00) { // BOTON 4 - Seguimiento
                    estrategiaSeleccionada = 4;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1; 
                    enviarMensajeBLE("Estrategia 4: Embestida Media. Esperando PLAY...");
                }
                else if (codigo == 0xE31CFF00) { // BOTON 5 - Embestida Agresiva
                    estrategiaSeleccionada = 5;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1;
                    enviarMensajeBLE("Estrategia 5: Embestida Agresiva. Esperando PLAY...");
                }
                else if (codigo == 0xA55AFF00) { // BOTON 6 - Patrulla en cuadrado
                    estrategiaSeleccionada = 6;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1;
                    enviarMensajeBLE("Estrategia 6: Patrulla Cuadrado. Esperando PLAY...");
                }
                else if (codigo == 0xBD42FF00) { // BOTON 7 - Flanqueo por la derecha
                    estrategiaSeleccionada = 7;
                    mostrarEstrategiaEnLEDs(estrategiaSeleccionada);
                    estadoRobot = 1; // pasa por la cuenta regresiva de 5s igual que las demás
                    enviarMensajeBLE("Estrategia 7: Flanqueo derecha. Esperando PLAY...");
                }
                else if (codigo == 0xBC43FF00) {
                    estadoRobot = 0;
                    mostrarEstrategiaEnLEDs(0);
                    enviarMensajeBLE("Saliendo de telemetría.");
                }

                IrReceiver.resume();
            }
            break;
    }
}

// ==================================================
// ESTRATEGIA 1: Busqueda VEL_BUSQUEDA_E1 
// ==================================================
void rutinaBusquedaDeAPoco() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == SENSOR_LINEA || lineaDer == SENSOR_LINEA) {
        moverMotores(-100, -100); delay(200);
        moverMotores(100, -100);  delay(300);
        faseE1 = 0;
        return;
    }

    unsigned long ahora = millis();

    if (faseE1 == 0) {
        int pos = buscarOponente();

        if (pos == 2) {
            faseE1 = 3;
        }
        else if (pos == 1 || pos == 3) {
            direccionE1 = pos;
            faseE1 = 1;
            tiempoFaseE1 = ahora;
        }
        else {
            moverMotores(VEL_BUSQUEDA_E1, VEL_BUSQUEDA_E1);
        }
    }
    else if (faseE1 == 1) {
        if (direccionE1 == 1) moverMotores(VEL_GIRO, -VEL_GIRO);
        else                  moverMotores(-VEL_GIRO, VEL_GIRO);

        if (ahora - tiempoFaseE1 >= DURACION_GIRO_MS) {
            moverMotores(0, 0);
            faseE1 = 2;
            tiempoFaseE1 = ahora;
        }
    }
    else if (faseE1 == 2) {
        moverMotores(0, 0);
        if (ahora - tiempoFaseE1 >= DURACION_PAUSA_MS) {
            faseE1 = 0;
        }
    }
    else if (faseE1 == 3) {
        int pos = buscarOponente();
        if (pos == 2) {
            moverMotores(VEL_ATAQUE_E1, VEL_ATAQUE_E1);
        } else {
            faseE1 = 0;
        }
    }
}

// ==================================================
// ESTRATEGIA 2: VEL_BUSQUEDA_E2
// ==================================================
void rutinaPatrullaYAtaca() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == SENSOR_LINEA || lineaDer == SENSOR_LINEA) {
        moverMotores(-100, -100); delay(200);
        moverMotores(100, -100);  delay(300);
        faseE2 = 0;
        return;
    }

    unsigned long ahora = millis();

    if (faseE2 == 0) {
        int pos = buscarOponente();

        if (pos == 2) {
            faseE2 = 3;
        }
        else if (pos == 1 || pos == 3) {
            direccionE2 = pos;
            faseE2 = 1;
            tiempoFaseE2 = ahora;
        }
        else {
            moverMotores(VEL_BUSQUEDA_E2, VEL_BUSQUEDA_E2);
        }
    }
    else if (faseE2 == 1) {
        if (direccionE2 == 1) moverMotores(VEL_GIRO, -VEL_GIRO);
        else                  moverMotores(-VEL_GIRO, VEL_GIRO);

        if (ahora - tiempoFaseE2 >= DURACION_GIRO_MS) {
            moverMotores(0, 0);
            faseE2 = 2;
            tiempoFaseE2 = ahora;
        }
    }
    else if (faseE2 == 2) {
        moverMotores(0, 0);
        if (ahora - tiempoFaseE2 >= DURACION_PAUSA_MS) {
            faseE2 = 0;
        }
    }
    else if (faseE2 == 3) {
        int pos = buscarOponente();
        if (pos == 2) {
            moverMotores(VEL_ATAQUE_E2, VEL_ATAQUE_E2);
        } else {
            faseE2 = 0;
        }
    }
}

// ==================================================
// ESTRATEGIA 3: Busqueda Progresiva
// ==================================================
void rutinaAtaqueProgresivo() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == SENSOR_LINEA || lineaDer == SENSOR_LINEA) {
        moverMotores(-100, -100); delay(200);
        moverMotores(100, -100);  delay(300);
        faseE3 = 0;
        return;
    }

    unsigned long ahora = millis();

    if (faseE3 == 0) {
        int pos = buscarOponente();

        if (pos == 2) {
            faseE3 = 3;
        }
        else if (pos == 1 || pos == 3) {
            direccionE3 = pos;
            faseE3 = 1;
            tiempoFaseE3 = ahora;
        }
        else {
            moverMotores(VEL_BUSQUEDA_E3, VEL_BUSQUEDA_E3);
        }
    }
    else if (faseE3 == 1) {
        if (direccionE3 == 1) moverMotores(VEL_GIRO, -VEL_GIRO);
        else                  moverMotores(-VEL_GIRO, VEL_GIRO);

        if (ahora - tiempoFaseE3 >= DURACION_GIRO_MS) {
            moverMotores(0, 0);
            faseE3 = 2;
            tiempoFaseE3 = ahora;
        }
    }
    else if (faseE3 == 2) {
        moverMotores(0, 0);
        if (ahora - tiempoFaseE3 >= DURACION_PAUSA_MS) {
            faseE3 = 0;
        }
    }
    else if (faseE3 == 3) {
        int pos = buscarOponente();
        if (pos == 2) {
            moverMotores(VEL_ATAQUE_E3, VEL_ATAQUE_E3);
        } else {
            faseE3 = 0;
        }
    }
}

// ==================================================
// ESTRATEGIA 4: Seguimiento con giro decisivo
// ==================================================
void rutinaSeguimiento() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == SENSOR_LINEA || lineaDer == SENSOR_LINEA) {
        moverMotores(-100, -100); delay(200);
        moverMotores(100, -100);  delay(300);
        faseE4 = 0;
        lecturasConsecutivasE4 = 0;
        return;
    }

    unsigned long ahora = millis();

    if (faseE4 == 0) { // LEER Y DECIDIR
        int pos = buscarOponenteSuave();

        // Debounce: solo confiamos en una lectura no-cero si se repite
        // dos veces seguidas. Si cambia o desaparece, arranca de nuevo.
        if (pos != 0 && pos == ultimaLecturaE4) {
            lecturasConsecutivasE4++;
        } else {
            lecturasConsecutivasE4 = (pos != 0) ? 1 : 0;
        }
        ultimaLecturaE4 = pos;

        if (pos == 0) {
            // Nadie detectado: barrido suave buscando, igual que antes
            if (ahora - tiempoBarrido > TIEMPO_BARRIDO_E4_MS) {
                direccionBarrido = -direccionBarrido;
                tiempoBarrido = ahora;
            }
            if (direccionBarrido == 1) {
                moverMotores(VEL_SEGUIMIENTO_E4, VEL_SEGUIMIENTO_E4 - VEL_CORRECCION_BARRIDO_E4);
            } else {
                moverMotores(VEL_SEGUIMIENTO_E4 - VEL_CORRECCION_BARRIDO_E4, VEL_SEGUIMIENTO_E4);
            }
            return;
        }

        if (lecturasConsecutivasE4 < LECTURAS_PARA_CONFIRMAR) {
            // Todavía no confirmamos: no tomamos ninguna decisión nueva
            // (se mantiene el último comando de motores del ciclo anterior)
            return;
        }

        // Lectura confirmada -> decidimos
        lecturasConsecutivasE4 = 0;
        if (pos == 2) {
            moverMotores(VEL_SEGUIMIENTO_E4, VEL_SEGUIMIENTO_E4); // de frente, directo
        } else {
            direccionE4 = pos; // 1 = izquierda, 3 = derecha
            faseE4 = 1;
            tiempoFaseE4 = ahora;
        }
    }
    else if (faseE4 == 1) { // GIRO DECISIVO EN EL LUGAR
        if (direccionE4 == 1) moverMotores(VEL_GIRO, -VEL_GIRO);
        else                  moverMotores(-VEL_GIRO, VEL_GIRO);

        if (ahora - tiempoFaseE4 >= DURACION_GIRO_SEGUIMIENTO_MS) {
            faseE4 = 2;
            tiempoFaseE4 = ahora;
        }
    }
    else if (faseE4 == 2) { // AVANZAR DERECHO TRAS EL GIRO
        moverMotores(VEL_SEGUIMIENTO_E4, VEL_SEGUIMIENTO_E4);
        if (ahora - tiempoFaseE4 >= DURACION_AVANCE_SEGUIMIENTO_MS) {
            faseE4 = 0;
        }
    }
}


// ==================================================
// ESTRATEGIA 5: Seguimiento con giro decisivo
// ==================================================
void rutinaEmbestidaAgresiva() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == SENSOR_LINEA || lineaDer == SENSOR_LINEA) {
        moverMotores(-100, -100); delay(200);
        moverMotores(100, -100);  delay(300);
        faseE5 = 0;
        lecturasConsecutivasE5 = 0;
        return;
    }

    unsigned long ahora = millis();

    if (faseE5 == 0) { // LEER Y DECIDIR (igual a E4)
        int pos = buscarOponenteSuave();

        if (pos != 0 && pos == ultimaLecturaE5) lecturasConsecutivasE5++;
        else lecturasConsecutivasE5 = (pos != 0) ? 1 : 0;
        ultimaLecturaE5 = pos;

        if (pos == 0) {
            if (ahora - tiempoBarrido > TIEMPO_BARRIDO_E5_MS) {
                direccionBarrido = -direccionBarrido;
                tiempoBarrido = ahora;
            }
            if (direccionBarrido == 1) moverMotores(VEL_SEGUIMIENTO_E5, VEL_SEGUIMIENTO_E5 - VEL_CORRECCION_BARRIDO_E5);
            else moverMotores(VEL_SEGUIMIENTO_E5 - VEL_CORRECCION_BARRIDO_E5, VEL_SEGUIMIENTO_E5);
            return;
        }

        if (lecturasConsecutivasE5 < LECTURAS_PARA_CONFIRMAR) return;

        lecturasConsecutivasE5 = 0;
        if (pos == 2) {
            faseE5 = 3;              // <-- acá está la diferencia con la 4: entra a EMBESTIDA
            tiempoFaseE5 = ahora;
        } else {
            direccionE5 = pos;
            faseE5 = 1;
            tiempoFaseE5 = ahora;
        }
    }
    else if (faseE5 == 1) { // GIRO (igual a E4)
        if (direccionE5 == 1) moverMotores(VEL_GIRO, -VEL_GIRO);
        else moverMotores(-VEL_GIRO, VEL_GIRO);
        if (ahora - tiempoFaseE5 >= DURACION_GIRO_SEGUIMIENTO_MS) {
            faseE5 = 2;
            tiempoFaseE5 = ahora;
        }
    }
    else if (faseE5 == 2) { // AVANZAR TRAS GIRO (igual a E4)
        moverMotores(VEL_SEGUIMIENTO_E5, VEL_SEGUIMIENTO_E5);
        if (ahora - tiempoFaseE5 >= DURACION_AVANCE_SEGUIMIENTO_MS) faseE5 = 0;
    }
    else if (faseE5 == 3) { // EMBESTIDA: rampa de velocidad mientras siga de frente
        int pos = buscarOponenteSuave();
        if (pos == 2) {
            unsigned long transcurrido = ahora - tiempoFaseE5;
            if (transcurrido > (unsigned long)TIEMPO_RAMPA_EMBESTIDA_MS) transcurrido = TIEMPO_RAMPA_EMBESTIDA_MS;
            int velocidadActual = VEL_SEGUIMIENTO_E5 +
                (long)(VEL_EMBESTIDA_MAX - VEL_SEGUIMIENTO_E5) * transcurrido / TIEMPO_RAMPA_EMBESTIDA_MS;
            moverMotores(velocidadActual, velocidadActual);
        } else {
            faseE5 = 0;
            lecturasConsecutivasE5 = 0;
        }
    }
}

// ==================================================
// ESTRATEGIA 6: Baile del cuadrado
// ==================================================

void rutinaCuadrado() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == SENSOR_LINEA || lineaDer == SENSOR_LINEA) {
        moverMotores(-100, -100); delay(200);
        moverMotores(100, -100);  delay(300);
        faseE6 = 0;
        return;
    }

    unsigned long ahora = millis();

    if (faseE6 == 0) { // AVANZAR UN LADO DEL CUADRADO
        int pos = buscarOponente();
        if (pos == 2) {
            faseE6 = 2; // lo tiene de frente: interrumpe el recorrido y empuja
            return;
        }
        moverMotores(VEL_BUSQUEDA_E1, VEL_BUSQUEDA_E1);
        if (ahora - tiempoFaseE6 >= DURACION_AVANCE_CUADRADO_MS) {
            faseE6 = 1;
            tiempoFaseE6 = ahora;
        }
    }
    else if (faseE6 == 1) { // GIRO DE 90° (siempre para el mismo lado)
        moverMotores(VEL_GIRO, -VEL_GIRO);
        if (ahora - tiempoFaseE6 >= DURACION_GIRO_90_MS) {
            faseE6 = 0;
            tiempoFaseE6 = ahora;
        }
    }
    else if (faseE6 == 2) { // EMPUJE, igual que la fase 3 de E1
        int pos = buscarOponente();
        if (pos == 2) {
            moverMotores(VEL_ATAQUE_E1, VEL_ATAQUE_E1);
        } else {
            faseE6 = 0;
            tiempoFaseE6 = ahora; // retoma el recorrido, tramo recto desde cero
        }
    }
}

// ==================================================
// ESTRATEGIA 7: Flanqueo por la derecha (embestida por atrás)
// ==================================================
void rutinaFlanqueoDerecha() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == SENSOR_LINEA || lineaDer == SENSOR_LINEA) {
        moverMotores(-100, -100); delay(200);
        moverMotores(100, -100);  delay(300);
        faseE7 = 0;
        lecturasConsecutivasE7 = 0;
        return;
    }

    unsigned long ahora = millis();

    if (faseE7 == 0) { // LEER Y DECIDIR
        int pos = buscarOponenteSuave();

        if (pos != 0 && pos == ultimaLecturaE7) lecturasConsecutivasE7++;
        else lecturasConsecutivasE7 = (pos != 0) ? 1 : 0;
        ultimaLecturaE7 = pos;

        if (pos == 0) { // barrido buscando, igual que E4/E5
            if (ahora - tiempoBarrido > TIEMPO_BARRIDO_E4_MS) {
                direccionBarrido = -direccionBarrido;
                tiempoBarrido = ahora;
            }
            if (direccionBarrido == 1) moverMotores(VEL_SEGUIMIENTO_E4, VEL_SEGUIMIENTO_E4 - VEL_CORRECCION_BARRIDO_E4);
            else moverMotores(VEL_SEGUIMIENTO_E4 - VEL_CORRECCION_BARRIDO_E4, VEL_SEGUIMIENTO_E4);
            return;
        }

        if (lecturasConsecutivasE7 < LECTURAS_PARA_CONFIRMAR) return;
        lecturasConsecutivasE7 = 0;

        if (pos == 3) {
            // Detectado a la derecha -> arranca el arco de flanqueo
            faseE7 = 1;
            tiempoFaseE7 = ahora;
        }
        else if (pos == 2) {
            // De frente -> directo a embestir
            faseE7 = 3;
            tiempoFaseE7 = ahora;
        }
        else { // pos == 1, izquierda: giro normal (no requiere flanqueo)
            moverMotores(-VEL_GIRO, VEL_GIRO);
        }
    }
    else if (faseE7 == 1) { // ARCO: avanza curveando hacia la derecha
        moverMotores(VEL_FLANQUEO_INT, VEL_FLANQUEO_EXT);
        if (ahora - tiempoFaseE7 >= DURACION_FLANQUEO_MS) {
            faseE7 = 2;
            tiempoFaseE7 = ahora;
        }
    }
    else if (faseE7 == 2) { // VERIFICAR SI YA QUEDÓ DETRÁS DEL RIVAL
        int pos = buscarOponente();
        if (pos == 2) {
            faseE7 = 3;             // quedó de frente -> ahí está su espalda
            tiempoFaseE7 = ahora;
        } else if (pos == 1 || pos == 3) {
            faseE7 = 1;              // se corrió, repite el arco
            tiempoFaseE7 = ahora;
        } else {
            faseE7 = 0;              // lo perdió, vuelve a buscar
        }
    }
    else if (faseE7 == 3) { // EMBESTIDA con rampa de velocidad
        int pos = buscarOponenteSuave();
        if (pos == 2) {
            unsigned long transcurrido = ahora - tiempoFaseE7;
            if (transcurrido > (unsigned long)TIEMPO_RAMPA_EMBESTIDA_E7_MS) transcurrido = TIEMPO_RAMPA_EMBESTIDA_E7_MS;
            int velocidadActual = VEL_SEGUIMIENTO_E4 +
                (long)(VEL_EMBESTIDA_E7_MAX - VEL_SEGUIMIENTO_E4) * transcurrido / TIEMPO_RAMPA_EMBESTIDA_E7_MS;
            moverMotores(velocidadActual, velocidadActual);
        } else {
            faseE7 = 0;
            lecturasConsecutivasE7 = 0;
        }
    }
}

// ==================================================
// 3: Resto funciones
// ==================================================

// lee n veces y me da el promedio para bajar le ruido
int leerPromedio(int pin) {
    long suma = 0;
    for (int k = 0; k < MUESTRAS_SENSOR; k++) {
        suma += analogRead(pin);
    }
    return suma / MUESTRAS_SENSOR;
}

// Hace una lectura ON/OFF completa (promediada) y devuelve la señal
// resultante de cada sensor, sin aplicar todavía ningún umbral.
void medirSenalesCrudas(int &i, int &c, int &d) {
    digitalWrite(PIN_EMISORES, LOW);
    delay(4); // asentamiento del fototransistor
    int s1_off = leerPromedio(SENSOR_IZQ);
    int s2_off = leerPromedio(SENSOR_CEN);
    int s3_off = leerPromedio(SENSOR_DER);
    
    digitalWrite(PIN_EMISORES, HIGH);
    delay(4);
    int s1_on = leerPromedio(SENSOR_IZQ);
    int s2_on = leerPromedio(SENSOR_CEN);
    int s3_on = leerPromedio(SENSOR_DER);

    i = max(0, s1_on - s1_off);
    c = max(0, s2_on - s2_off);
    d = max(0, s3_on - s3_off);

    // NUEVO: cualquier lectura por encima de este techo es ruido
    // eléctrico, no una señal real — la descartamos.
    if (i > LECTURA_MAXIMA_PLAUSIBLE) i = 0;
    if (c > LECTURA_MAXIMA_PLAUSIBLE) c = 0;
    if (d > LECTURA_MAXIMA_PLAUSIBLE) d = 0;
}

//repite 20 veces el sañeles crudas
void calibrarUmbrales() {
    int maxI = 0, maxC = 0, maxD = 0;
    for (int ronda = 0; ronda < 20; ronda++) {
        int i, c, d;
        medirSenalesCrudas(i, c, d);
        if (i > maxI) maxI = i;
        if (c > maxC) maxC = c;
        if (d > maxD) maxD = d;
    }
    // Margen de seguridad sobre el pico de ruido medido.
    umbralIzq = maxI + 25;
    umbralCen = maxC + 25;
    umbralDer = maxD + 25;

    enviarMensajeBLE("Umbrales calibrados I:" + String(umbralIzq) +
                      " C:" + String(umbralCen) + " D:" + String(umbralDer));
}

// compara las señales crudas y compara con su propio umbral, luego da la direcion si lo supera
int buscarOponente() {
    int i, c, d;
    medirSenalesCrudas(i, c, d);

    señalIzqActual = i;
    señalCenActual = c;
    señalDerActual = d;

    bool detectaI = i >= umbralIzq;
    bool detectaC = c >= umbralCen;
    bool detectaD = d >= umbralDer;

    if (!detectaI && !detectaC && !detectaD) {
        return 0; // nadie detectado
    }

    int margenI = detectaI ? (i - umbralIzq) : -1;
    int margenC = detectaC ? (c - umbralCen) : -1;
    int margenD = detectaD ? (d - umbralDer) : -1;

    if (margenC >= margenI && margenC >= margenD) return 2; // Centro
    if (margenI >= margenC && margenI >= margenD) return 1; // Izquierda
    if (margenD >= margenC && margenD >= margenI) return 3; // Derecha

    return 0;
}

// Casi idéntica a buscarOponente(), pero mezcla la lectura nueva con el 40% del historial antes de comparar contra el umbral (promedio exponencial), para que un parpadeo puntual no borre al rival de la lectura de golpe. 
int buscarOponenteSuave() {
    int i, c, d;
    medirSenalesCrudas(i, c, d);

    // Promedio exponencial: 60% la lectura nueva, 40% el historial.
    señalIzqSuave = (i * 0.6) + (señalIzqSuave * 0.4);
    señalCenSuave = (c * 0.6) + (señalCenSuave * 0.4);
    señalDerSuave = (d * 0.6) + (señalDerSuave * 0.4);

    // Igual que buscarOponente(), pero usando los valores suavizados.
    bool detectaI = señalIzqSuave >= umbralIzq;
    bool detectaC = señalCenSuave >= umbralCen;
    bool detectaD = señalDerSuave >= umbralDer;

    if (!detectaI && !detectaC && !detectaD) {
        return 0;
    }

    float margenI = detectaI ? (señalIzqSuave - umbralIzq) : -1;
    float margenC = detectaC ? (señalCenSuave - umbralCen) : -1;
    float margenD = detectaD ? (señalDerSuave - umbralDer) : -1;

    if (margenC >= margenI && margenC >= margenD) return 2;
    if (margenI >= margenC && margenI >= margenD) return 1;
    if (margenD >= margenC && margenD >= margenI) return 3;

    return 0;
}

void moverMotores(int velIzq, int velDer) {
    // los motores están cableados invertidos 
    velIzq = -velIzq;
    velDer = -velDer;

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
            digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, velDer);
        } else {
            digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, -velDer);
}
}

void enviarMensajeBLE(String mensaje) {
    if (deviceConnected) {
        mensaje += "\n"; 
        pCharacteristicTX->setValue(mensaje.c_str());
        pCharacteristicTX->notify();
    }
}

void mostrarEstrategiaEnLEDs(int numero) {
    digitalWrite(LED_IZQ, (numero & 1) ? HIGH : LOW);
    digitalWrite(LED_CEN, (numero & 2) ? HIGH : LOW);
    digitalWrite(LED_DER, (numero & 4) ? HIGH : LOW);
}
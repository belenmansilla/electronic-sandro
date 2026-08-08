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
#define PIN_RECEPTOR_IR 4   // VS1838B (Árbitro)
#define LED_IZQ 18          // L1
#define LED_CEN 19          // L2
#define LED_DER 21          // L3

#define LECTURA_MAXIMA_PLAUSIBLE 400


// Variables globales para la Máquina de Estados
int estadoRobot = 0;           // Arranca en 0 (Esperando selección)
int estrategiaSeleccionada = 0; // Guardará el número de la estrategia

// Motores Limite
int LIMITE_PWM = 100;

// Motores estrategias
int VEL_BUSQUEDA = 70;
int VEL_GIRO = 80;
int VEL_ATAQUE = 90;      // fuerza de empuje para Estrategia 1 y 2
int VEL_ATAQUE_E3 = 100;  // fuerza de empuje para Estrategia 3 (la más fuerte,tope = LIMITE_PWM)
int VEL_SEGUIMIENTO = 50;   // velocidad base de avance, tranquila
int VEL_CORRECCION = 25;    // cuánto se frena la rueda interna al curvar (no se invierte, solo se frena)

int VEL_CORRECCION_BARRIDO = 15;   // corrección suave mientras "barre" buscando, más chica que VEL_CORRECCION
unsigned long tiempoBarrido = 0;
int direccionBarrido = 1;          // 1 = barre hacia un lado, -1 = hacia el otro

float señalIzqSuave = 0;
float señalCenSuave = 0;
float señalDerSuave = 0;

#define DURACION_GIRO_MS 150   // cuánto dura cada pulso de giro
#define DURACION_PAUSA_MS 500  // cuánto se queda quieto después de girar, para estabilizarse


// Detección de oponente por sustracción ON/OFF del emisor. Cada
// llamada apaga el emisor, mide el ruido ambiental, prende el emisor,
// mide de nuevo, y usa la diferencia. Esto cancela la luz ambiental
// en cada ciclo (no hace falta calibrar una vez al arrancar).
//
// Contra rivales negro mate la señal útil es chica, así que en vez de
// un umbral fijo para los 3 sensores, cada uno tiene su propio piso
// de ruido medido en el arranque (con la mesa vacía) y el umbral se
// calcula apenas por encima de eso. Así se puede bajar mucho la
// sensibilidad sin que el ruido eléctrico de cada canal dispare
// falsos positivos.
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

// Bluetooth configurado para el celu desde la app 
BLEServer* pServer = NULL;  //activo el servidor
BLECharacteristic* pCharacteristicTX = NULL; //aca le digo que voy a estar guardando una configuracion gigante de bluetooth.
bool deviceConnected = false;

unsigned long tiempoAnteriorBLE = 0;

// Timers no bloqueantes para la rutina de búsqueda (Estrategia 1)
unsigned long tiempoBusqueda = 0;
int faseBusqueda = 0; // 0=girar, 1=avanzar, 2=pausa

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

    // --- CONFIGURACIÓN BLE UART ---
    BLEDevice::init("Evil_Sandro");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pCharacteristicTX->addDescriptor(new BLE2902());

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

    // 4. Estado seguro inicial de LEDs
    digitalWrite(LED_IZQ, LOW);
    digitalWrite(LED_CEN, LOW);
    digitalWrite(LED_DER, LOW);

    // Estado seguro inicial: emisor apagado. buscarOponente() lo
    // prende/apaga solo en cada lectura (técnica ON/OFF).
    digitalWrite(PIN_EMISORES, LOW);
    
    IrReceiver.begin(PIN_RECEPTOR_IR, DISABLE_LED_FEEDBACK);

    // Medimos el piso de ruido real de cada sensor (mesa vacía) y
    // fijamos el umbral apenas por encima. IMPORTANTE: al prender el
    // robot no debe haber nada cerca de los sensores.
    calibrarUmbrales();

    Serial.println("Sandro v1.5 Inicializado. Esperando órdenes...");
}

void loop() {
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoAnteriorBLE >= 1000) {
        tiempoAnteriorBLE = tiempoActual; 
        
        int linIzq = digitalRead(PIN_LINEA_IZQ);
        int linDer = digitalRead(PIN_LINEA_DER);
        buscarOponente(); // refresca señalIzqActual/CenActual/DerActual para telemetría
        String telemetria = "E:" + String(estadoRobot) + 
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
                    estadoRobot = 1; // estado de busqueda hasta el "play"
                    digitalWrite(LED_IZQ, HIGH); 
                    enviarMensajeBLE("Estrategia 1 (Busqueda Lenta). Esperando PLAY...");
                }
                else if (codigo == 0xE718FF00) { // BOTON 2
                    estrategiaSeleccionada = 2;
                    estadoRobot = 1; 
                    digitalWrite(LED_CEN, HIGH);
                    enviarMensajeBLE("Estrategia 2 (Patrulla Borde). Esperando PLAY...");
                }                
                else if (codigo == 0xA15EFF00) { // BOTON 3 
                    estrategiaSeleccionada = 3;
                    estadoRobot = 1; 
                    digitalWrite(LED_DER, HIGH);
                    digitalWrite(LED_IZQ, HIGH); 
                    enviarMensajeBLE("Ataque progresivo. Esperando PLAY...");
                }
                else if (codigo == 0xF708FF00) { // BOTON 4 - Seguimiento
                    estrategiaSeleccionada = 4;
                    estadoRobot = 1; 
                    digitalWrite(LED_IZQ, HIGH);
                    digitalWrite(LED_CEN, HIGH);
                    digitalWrite(LED_DER, HIGH);
                    enviarMensajeBLE("Estrategia 4 (Seguimiento). Esperando PLAY...");
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
                    
                    for(int i = 0; i < 10; i++) {
                        digitalWrite(LED_CEN, HIGH);
                        delay(250);
                        digitalWrite(LED_CEN, LOW);
                        delay(250);
                    }
                    
                    faseBusqueda = 0;
                    tiempoBusqueda = millis();
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
            }
            break;
            
        // ==================================================
        // ESTADO 3: TELEMETRÍA POR BLUETOOTH 
        // ==================================================
        case 3:
            moverMotores(0, 0); // Motores siempre bloqueados en este estado

            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;

                if (codigo == 0xF30CFF00) { // BOTON 1 -> selecciona Estrategia 1
                    estrategiaSeleccionada = 1;
                    estadoRobot = 1;
                    digitalWrite(LED_DER, LOW);
                    digitalWrite(LED_IZQ, HIGH);
                    enviarMensajeBLE("Estrategia 1 (Busqueda Lenta). Esperando PLAY...");
                }
                else if (codigo == 0xE718FF00) { // BOTON 2 -> selecciona Estrategia 2
                    estrategiaSeleccionada = 2;
                    estadoRobot = 1;
                    digitalWrite(LED_DER, LOW);
                    digitalWrite(LED_CEN, HIGH);
                    enviarMensajeBLE("Estrategia 2 (Patrulla Borde). Esperando PLAY...");
                }
                else if (codigo == 0xA15EFF00) { // BOTON 3 -> selecciona Estrategia 3
                    estrategiaSeleccionada = 3;
                    estadoRobot = 1;
                    digitalWrite(LED_DER, LOW);
                    digitalWrite(LED_IZQ, HIGH);
                    digitalWrite(LED_CEN, HIGH);
                    enviarMensajeBLE("Ataque progresivo. Esperando PLAY...");
                }
                else if (codigo == 0xF708FF00) { // BOTON 4 -> selecciona Estrategia 4
                    estrategiaSeleccionada = 4;
                    estadoRobot = 1;
                    digitalWrite(LED_DER, LOW);
                    digitalWrite(LED_IZQ, HIGH);
                    digitalWrite(LED_CEN, HIGH);
                    enviarMensajeBLE("Estrategia 4 (Seguimiento). Esperando PLAY...");
                }
                else if (codigo == 0xBC43FF00) { // BOTON E/R de nuevo -> vuelve a espera neutral, sin elegir nada
                    estadoRobot = 0;
                    digitalWrite(LED_DER, LOW);
                    enviarMensajeBLE("Saliendo de telemetría.");
                }

                IrReceiver.resume();
            }
            break;
    }
}

// ==================================================
// ESTRATEGIA 1: Búsqueda lenta, no bloqueante
// ==================================================
void rutinaBusquedaDeAPoco() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == 1 || lineaDer == 1) {
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
            moverMotores(VEL_BUSQUEDA, VEL_BUSQUEDA);
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
            moverMotores(VEL_ATAQUE, VEL_ATAQUE);
        } else {
            faseE1 = 0;
        }
    }
}

// ==================================================
// ESTRATEGIA 2: Patrulla el dohyo, esquiva el borde,
// e interrumpe la patrulla si detecta oponente
// ==================================================
void rutinaPatrullaYAtaca() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == 1 || lineaDer == 1) {
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
            moverMotores(VEL_BUSQUEDA, VEL_BUSQUEDA);
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
            moverMotores(VEL_ATAQUE, VEL_ATAQUE);
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
    if (lineaIzq == 1 || lineaDer == 1) {
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
            moverMotores(VEL_BUSQUEDA, VEL_BUSQUEDA);
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
            moverMotores(VEL_ATAQUE_E3, VEL_ATAQUE_E3); // única diferencia real: empuja más fuerte
        } else {
            faseE3 = 0;
        }
    }
}

// ==================================================
// ESTRATEGIA 4: Seguimiento continuo (para blanco en movimiento)
// ==================================================
// ==================================================
// ESTRATEGIA 4: Seguimiento continuo (para blanco en movimiento)
// ==================================================
void rutinaSeguimiento() {
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    if (lineaIzq == 1 || lineaDer == 1) {
        moverMotores(-100, -100); delay(200);
        moverMotores(100, -100);  delay(300);
        return;
    }

    int pos = buscarOponenteSuave(); // <-- usa la versión suavizada

    if (pos == 1) { // Lo tiene a la izquierda -> curva suave a la izquierda
        moverMotores(VEL_SEGUIMIENTO - VEL_CORRECCION, VEL_SEGUIMIENTO);
    }
    else if (pos == 3) { // Lo tiene a la derecha -> curva suave a la derecha
        moverMotores(VEL_SEGUIMIENTO, VEL_SEGUIMIENTO - VEL_CORRECCION);
    }
    else if (pos == 2) { // De frente -> derecho
        moverMotores(VEL_SEGUIMIENTO, VEL_SEGUIMIENTO);
    }
    else {
        // NUEVO: no ve a nadie -> en vez de ir derecho a ciegas, barre
        // suavemente de un lado al otro mientras avanza, para que el
        // rival tenga más chances de entrar en el cono del sensor.
        unsigned long ahora = millis();
        if (ahora - tiempoBarrido > 800) {
            direccionBarrido = -direccionBarrido; // cambia de lado cada 800ms
            tiempoBarrido = ahora;
        }

        if (direccionBarrido == 1) {
            moverMotores(VEL_SEGUIMIENTO, VEL_SEGUIMIENTO - VEL_CORRECCION_BARRIDO);
        } else {
            moverMotores(VEL_SEGUIMIENTO - VEL_CORRECCION_BARRIDO, VEL_SEGUIMIENTO);
        }
    }
}

// ==================================================
// Lectura de sensores de oponente
// ==================================================
// Lee un sensor N veces y promedia, para bajar el ruido del ADC.
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

// Mide el piso de ruido real de cada sensor (con la mesa vacía) y fija
// el umbral de detección apenas por encima
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

// Detección con suavizado temporal — promedia la lectura nueva con el
// historial reciente, así un parpadeo puntual del sensor (algo común
// cuando el rival está muy cerca) no hace que "desaparezca" de golpe.
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
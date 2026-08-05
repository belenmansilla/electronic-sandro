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

// Variables globales para la Máquina de Estados
int estadoRobot = 0;           // Arranca en 0 (Esperando selección)
int estrategiaSeleccionada = 0; // Guardará el número de la estrategia

// Motores Limite
int LIMITE_PWM = 150;

// Motores estrategias
int VEL_BUSQUEDA_E3 = 50;
int VEL_ATAQUE_E3 = 90;
int VEL_GIRO_E3 = 40;

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

// Cuántas lecturas seguidas por encima del umbral hacen falta para
// aceptar una detección como real (filtra picos aislados de ruido).
#define CONFIRMACIONES_REQUERIDAS 1
int confirmacionesIzq = 0, confirmacionesCen = 0, confirmacionesDer = 0;

// "Memoria" de la última dirección detectada: si el rival cae
// momentáneamente en el hueco ciego entre dos sensores, mantenemos
// la última dirección conocida en vez de perderlo de vista al toque.
#define MEMORIA_DETECCION_MS 250
int ultimaPosDetectada = 0;
unsigned long tiempoUltimaDeteccion = 0;

// Guardamos la última señal calculada de cada sensor para poder
// mandarla por telemetría sin tener que volver a leer.
int señalIzqActual = 0;
int señalCenActual = 0;
int señalDerActual = 0;


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
                else if (codigo == 0xBC43FF00) { // BOTON "E/R" 
                    estadoRobot = 3; 
                    digitalWrite(LED_DER, HIGH); 
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
            }
            break;
            
        // ==================================================
        // ESTADO 3: TELEMETRÍA POR BLUETOOTH 
        // ==================================================
        case 3:
            moverMotores(0, 0);
            if (IrReceiver.decode()) {
                uint32_t codigo = IrReceiver.decodedIRData.decodedRawData;
                if (codigo == 0xF30CFF00 || codigo == 0xF609FF00) {
                    estadoRobot = 0; digitalWrite(LED_DER, LOW); 
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
    int pos = buscarOponente(); 

    if (pos == 0) {
        // No ve a nadie -> patrón de búsqueda, pero sin "delay()"
        // para poder seguir revisando sensores y control remoto.
        unsigned long ahora = millis();
        switch (faseBusqueda) {
            case 0: // girar un poco
                moverMotores(120, -120);
                if (ahora - tiempoBusqueda >= 100) { faseBusqueda = 1; tiempoBusqueda = ahora; }
                break;
            case 1: // avanzar un poco
                moverMotores(80, 80);
                if (ahora - tiempoBusqueda >= 50) { faseBusqueda = 2; tiempoBusqueda = ahora; }
                break;
            case 2: // pausa breve
                moverMotores(0, 0);
                if (ahora - tiempoBusqueda >= 20) { faseBusqueda = 0; tiempoBusqueda = ahora; }
                break;
        }
    } 
    else if (pos == 2) { // CENTRO -> Ataque de frente
        moverMotores(255, 255);
    }
    else if (pos == 1) { // IZQUIERDA -> Gira Izquierda hasta centrarlo
        moverMotores(-120, 120); 
    }
    else if (pos == 3) { // DERECHA -> Gira Derecha hasta centrarlo
        moverMotores(120, -120);
    }
}

// ==================================================
// ESTRATEGIA 2: Patrulla el dohyo, esquiva el borde,
// e interrumpe la patrulla si detecta oponente
// ==================================================
void rutinaPatrullaYAtaca() {
    // 1. PRIORIDAD ABSOLUTA: No caerse de la mesa (Línea Negra = 1)
    int lineaIzq = digitalRead(PIN_LINEA_IZQ);
    int lineaDer = digitalRead(PIN_LINEA_DER);
    
    if (lineaIzq == 1 || lineaDer == 1) {
        moverMotores(-100, -100); delay(200); // Retrocede
        moverMotores(100, -100);  delay(300); // Gira
        return; // No sigue con la lógica de ataque este ciclo
    }

    // 2. Si el piso es seguro, buscamos oponente (esto interrumpe la patrulla)
    int pos = buscarOponente(); 

    if (pos == 2) { // Lo ve de frente -> ataca
        moverMotores(255, 255);
    }
    else if (pos == 1) { // Lo ve a la Izquierda -> gira hasta centrarlo
        moverMotores(-120, 120); 
    }
    else if (pos == 3) { // Lo ve a la Derecha -> gira hasta centrarlo
        moverMotores(120, -120);
    }
    else { // No ve a nadie, sigue patrullando derecho
        moverMotores(60, 60);
    }
}

// ==================================================
// ESTRATEGIA 3: Busqueda Progresiva
// ==================================================
void rutinaAtaqueProgresivo() {
    int pos = buscarOponente(); 

    if (pos == 0) {
        // No ve a nadie -> patrón de búsqueda, pero sin "delay()"
        // para poder seguir revisando sensores y control remoto.
        unsigned long ahora = millis();
        switch (faseBusqueda) {
            case 0: // girar un poco
                moverMotores(VEL_BUSQUEDA_E3, -VEL_BUSQUEDA_E3);
                if (ahora - tiempoBusqueda >= 100) { faseBusqueda = 1; tiempoBusqueda = ahora; }
                break;
            case 1: // avanzar un poco
                moverMotores(VEL_BUSQUEDA_E3, VEL_BUSQUEDA_E3);
                if (ahora - tiempoBusqueda >= 50) { faseBusqueda = 2; tiempoBusqueda = ahora; }
                break;
            case 2: // pausa breve
                moverMotores(0, 0);
                if (ahora - tiempoBusqueda >= 20) { faseBusqueda = 0; tiempoBusqueda = ahora; }
                break;
        }
    } 
    else if (pos == 2) { // CENTRO -> Ataque de frente
        moverMotores(VEL_ATAQUE_E3, VEL_ATAQUE_E3);
    }
    else if (pos == 1) { // IZQUIERDA -> Gira Izquierda hasta centrarlo
        moverMotores(-VEL_GIRO_E3, VEL_GIRO_E3); 
    }
    else if (pos == 3) { // DERECHA -> Gira Derecha hasta centrarlo
        moverMotores(VEL_GIRO_E3, -VEL_GIRO_E3);
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
    medirSenalesCrudas(i, c, d); // esta funcion me da los datos limpios

    señalIzqActual = i;
    señalCenActual = c;
    señalDerActual = d;

    bool detectaI = i >= umbralIzq;
    bool detectaC = c >= umbralCen;
    bool detectaD = d >= umbralDer;

    // Exigimos varias lecturas seguidas por encima del umbral antes
    // de confiar en la detección (filtra picos aislados de ruido,
    // algo más probable ahora que los umbrales son bajos).
    confirmacionesIzq = detectaI ? confirmacionesIzq + 1 : 0;
    confirmacionesCen = detectaC ? confirmacionesCen + 1 : 0;
    confirmacionesDer = detectaD ? confirmacionesDer + 1 : 0;

    bool confirmaI = confirmacionesIzq >= CONFIRMACIONES_REQUERIDAS;
    bool confirmaC = confirmacionesCen >= CONFIRMACIONES_REQUERIDAS;
    bool confirmaD = confirmacionesDer >= CONFIRMACIONES_REQUERIDAS;

    if (!confirmaI && !confirmaC && !confirmaD) {
        // Nadie confirmado en ESTE ciclo. Pero si hace poquito sí
        // tuvimos al rival detectado, probablemente sigue ahí y solo
        // cayó en el hueco ciego entre dos haces de sensores. En ese
        // caso mantenemos la última dirección un instante en vez de
        // resetear a modo búsqueda de golpe.
        if (millis() - tiempoUltimaDeteccion < MEMORIA_DETECCION_MS) {
            return ultimaPosDetectada;
        }
        return 0; // Ya pasó el margen de gracia: asumimos que no hay nadie
    }

    // Entre los confirmados, gana el que tiene mayor señal relativa
    // [Condición a evaluar] ? [Valor si es VERDADERO] : [Valor si es FALSO];
    int margenI = confirmaI ? (i - umbralIzq) : -1;
    int margenC = confirmaC ? (c - umbralCen) : -1;
    int margenD = confirmaD ? (d - umbralDer) : -1;

    int pos = 0;
    if (margenC >= margenI && margenC >= margenD) pos = 2; // Centro
    else if (margenI >= margenC && margenI >= margenD) pos = 1; // Izquierda
    else if (margenD >= margenC && margenD >= margenI) pos = 3; // Derecha

    ultimaPosDetectada = pos;
    tiempoUltimaDeteccion = millis();
    return pos; 
}

void moverMotores(int velIzq, int velDer) {
    // FIX: los motores están cableados invertidos (lo que el código
    // manda como "adelante" gira físicamente hacia atrás). En vez de
    // recablear, compensamos acá invirtiendo el signo una sola vez.
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
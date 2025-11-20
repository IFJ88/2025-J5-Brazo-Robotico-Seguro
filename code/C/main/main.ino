// --- NUEVAS INCLUSIONES PARA WEB SERVER Y JSON ---
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h> // ¡Asegúrate de instalar esta librería!

// --- CONFIGURACIÓN DE RED ---
const char* ssid = "Personal-58C-2.4GHz";
const char* password = "9114DFC58C";
const IPAddress localIP(192, 168, 1, 184); // IP Fija sugerida para el ESP32
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);

// Objeto del Servidor en el puerto 80
WebServer server(80);

#include <ESP32Servo.h>
#include <cmath> 
#include <string.h> 
#include <stdlib.h> // Necesario para 'free' en el setup

// --- INCLUSIÓN DE MÓDULOS DE CINEMÁTICA ---
#include "numpy.h"         
#include "algebra_lineal.h" 
#include "configuracion.h" 
#include "cinematica.h"    

// ================== PINES Y CONFIGURACIÓN HARDWARE ==================

// ----------- Pines de Control -----------
const int BTN_NEXT = 13;   
const int BTN_PREV = 14;   
const int POT_PIN  = 34;   

// ----------- Configuración de Servos (5 Eslabones) -----------
const int SERVO_COUNT = 5;
const int servoPins[SERVO_COUNT]  = {26, 19, 25, 18, 23};   
const int servoMinUs[SERVO_COUNT] = {400, 400, 500, 400, 400};
const int servoMaxUs[SERVO_COUNT] = {2600,2600,2600,2200,2600};

// ================== CONSTANTES DE CONTROL ==================
const float ALPHA = 0.25f;       // suavizado exponencial ADC
const int   DEADZONE_US = 150;    
const int   DEBOUNCE_MS = 50;    

// ---- Tabla de calibración (us -> grados) ----
struct CalibPoint {
    int   us;
    float deg;
};

CalibPoint LUT[] = {
    {1000,   0.0},
    {1250,  45.0},
    {1500,  90.0},
    {1750, 135.0},
    {2000, 180.0}
};
const int LUT_SIZE = sizeof(LUT)/sizeof(LUT[0]);

// ================== ESTADO GLOBAL DEL ROBOT ==================
Servo servos[SERVO_COUNT];
float filteredAdc = 0.0f;
int currentPulse[SERVO_COUNT]; 
double angulos_grados[SERVO_COUNT] = {0.0}; 

// Índice del servo actualmente controlado por el potenciómetro
int activeServo = 0; 

// Variables de estado para el Debounce
int lastRawNext = HIGH;
int lastStableNext = HIGH;
unsigned long lastDebounceNext = 0;
int lastRawPrev = HIGH;
int lastStablePrev = HIGH;
unsigned long lastDebouncePrev = 0;


// ================== UTILITARIOS ==================

float angleFromPulseUS(int us) {
    if (us <= LUT[0].us)   return LUT[0].deg;
    if (us >= LUT[LUT_SIZE-1].us) return LUT[LUT_SIZE-1].deg;
    for (int i = 0; i < LUT_SIZE-1; i++) {
        int   x0 = LUT[i].us;
        int   x1 = LUT[i+1].us;
        float y0 = LUT[i].deg;
        float y1 = LUT[i+1].deg;
        if (us >= x0 && us <= x1) {
            float t = float(us - x0) / float(x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }
    return NAN;
}

// Función de Antirrebote (Debounce)
bool fallingEdgeDebounced(int pin, int &lastRaw, int &lastStable, unsigned long &tLast) {
    int raw = digitalRead(pin);
    unsigned long now = millis();

    if (raw != lastRaw) { lastRaw = raw; tLast = now; }
    if ((now - tLast) > DEBOUNCE_MS) {
        if (raw != lastStable) {
            lastStable = raw;
            if (lastStable == LOW) return true; // flanco de bajada (pulso a GND)
        }
    }
    return false;
}

// ================== FUNCIÓN DE AYUDA DE KINEMÁTICA ==================

// NOTA: Esta función DEBE liberar la memoria de los resultados devueltos después de usarlos.
// En el handler JSON, construiremos el JSON y luego liberaremos la memoria.
bool obtener_pose_global(EsferasGlobales esferas_resultado, ColisionSet* centros_resultado) {
    
    // Ejecutar Sistema de Colisión con la pose global actual
    return calcular_configuracion_modular(
        angulos_grados, // Usa la pose actual validada
        esferas_local, 
        RADIO_ESFERA, 
        esferas_resultado, 
        centros_resultado
    );
}

// ================== HANDLER WEB ==================

int get_sphere_row_count(int eslabon_idx) {
    if (eslabon_idx == 0) return 1;
    if (eslabon_idx == 1) return 2;
    if (eslabon_idx == 2) return 3;
    if (eslabon_idx == 3) return 1;
    if (eslabon_idx == 4) return 2;
    return 0; // Error
}

void handleDataRequest() {
    // 1. Ejecutar Cinemática para obtener la pose actual
    EsferasGlobales esferas_globales_temp;
    ColisionSet centros_colisionantes;
    
    // Inicializar punteros a NULL (CRÍTICO)
    centros_colisionantes.coords = NULL;
    centros_colisionantes.count = 0;
    for(int j = 0; j < SERVO_COUNT; j++) esferas_globales_temp[j] = NULL;
    
    bool colision_detectada = obtener_pose_global(esferas_globales_temp, &centros_colisionantes);

    // 2. Crear Documento JSON (Tamaño estimado)
    // El tamaño depende del número de esferas: 5 eslabones * (2 o 3 esferas por eslabón) * 3 coordenadas
    const size_t CAPACITY = JSON_OBJECT_SIZE(4) + // 4 campos principales (angles, collision, spheres, centers)
                            JSON_ARRAY_SIZE(SERVO_COUNT) + // Array de ángulos (5 doubles)
                            SERVO_COUNT * JSON_ARRAY_SIZE(5) + // 5 eslabones
                            300; // Buffer de seguridad
                            
    DynamicJsonDocument doc(CAPACITY);
    
    // --- 3. Llenar el JSON ---
    
    // a) Ángulos
    JsonArray anglesArray = doc.createNestedArray("angles");
    for (int i = 0; i < SERVO_COUNT; i++) {
        anglesArray.add(angulos_grados[i]);
    }
    
    // b) Estado de Colisión
    doc["collision_detected"] = colision_detectada;
    doc["active_servo_index"] = activeServo;

    // c) Centros Colisionantes (si los hay)
    JsonArray collidingCentersArray = doc.createNestedArray("colliding_centers");
    if (centros_colisionantes.count > 0 && centros_colisionantes.coords != NULL) {
        // Los centros colisionantes son una lista aplanada de coordenadas (x1, y1, z1, x2, y2, z2...)
        // La lista centros_colisionantes.coords contiene (x, y, z) de cada centro
        for (int i = 0; i < centros_colisionantes.count; i++) {
            JsonArray center = collidingCentersArray.createNestedArray();
            center.add(centros_colisionantes.coords[i][0]);
            center.add(centros_colisionantes.coords[i][1]);
            center.add(centros_colisionantes.coords[i][2]);
        }
    }
    
    // d) Esferas Globales (La pose completa)
    doc["sphere_radius"] = RADIO_ESFERA;
    JsonArray spheresArray = doc.createNestedArray("spheres");
    
    for (int i = 0; i < SERVO_COUNT; i++) {
        JsonArray eslabon = spheresArray.createNestedArray();
        
        // --- Solución al Error 1: Usar helper para obtener rows_i ---
        int rows_i = get_sphere_row_count(i);
        
        if (esferas_globales_temp[i] != NULL) {
            
            // --- Solución al Error 2: Casting explícito a Vector3D* ---
            const Vector3D *esf_global_i = (const Vector3D *)esferas_globales_temp[i];
            
            for (size_t k = 0; k < rows_i; k++) {
                JsonArray sphereCenter = eslabon.createNestedArray();
                
                // Ahora la indexación es correcta: esf_global_i[k] es el Vector3D (double[3])
                sphereCenter.add(esf_global_i[k][0]); // x
                sphereCenter.add(esf_global_i[k][1]); // y
                sphereCenter.add(esf_global_i[k][2]); // z
            }
        }
    }

    // 4. Serializar JSON a String
    String response;
    serializeJson(doc, response);

    // 5. Enviar Respuesta HTTP
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", response);

    // 6. Liberar Memoria (CRÍTICO)
    if (centros_colisionantes.coords != NULL) free(centros_colisionantes.coords);
    for(int j = 0; j < SERVO_COUNT; j++) {
        if (esferas_globales_temp[j] != NULL) free((void*)esferas_globales_temp[j]);
    }
}


// ================== SETUP ==================

void setup() {
    Serial.begin(115200);
    delay(400);

    // --- NUEVO: Conexión Wi-Fi ---
    // if (!WiFi.config(localIP, gateway, subnet)) {
    //   Serial.println("Falló la configuración STA");
    // }
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi conectado.");
    Serial.print("IP del servidor: ");
    Serial.println(WiFi.localIP());

    // --- NUEVO: Inicialización del Web Server ---
    server.on("/", HTTP_GET, [](){
        server.send(200, "text/plain", "ESP32 Robot Arm Server. Endpoint /data para JSON.");
    });
    server.on("/data", HTTP_GET, handleDataRequest);
    server.begin();
    Serial.println("Servidor HTTP iniciado.");

    // Inicialización del hardware y cinemática
    analogReadResolution(12);    
    analogSetPinAttenuation(POT_PIN, ADC_11db); 
    pinMode(BTN_NEXT, INPUT_PULLUP);
    pinMode(BTN_PREV, INPUT_PULLUP);
    inicializar_matrices_offset(); 

    // Inicialización de Servos y ángulos
    for (int i = 0; i < SERVO_COUNT; i++) {
        servos[i].setPeriodHertz(50); 
        servos[i].attach(servoPins[i], servoMinUs[i], servoMaxUs[i]);
        currentPulse[i] = servoMinUs[i]; 
        //servos[i].writeMicroseconds(currentPulse[i]);
        //servos_idle(); --> funcion que lleva a los servos a la posicion idle
        angulos_grados[i] = angleFromPulseUS(currentPulse[i]);
    }
    
    // Inicialización del ADC para el filtro
    int raw = analogRead(POT_PIN);
    filteredAdc = (float)raw;

    Serial.println("--- Robot Modular: Control por Potenciómetro + Anticolisión ---");
    
    // ---------------------------------------------------------------------------------
    // --- LÓGICA DE INICIALIZACIÓN DE POSE BASE (Migrada de simulacion.py) ---
    // ---------------------------------------------------------------------------------
    
    // Definir N_eslabones (simula len(esferas_local))
    const int N_eslabones = SERVO_COUNT; 

    Serial.println("--- FASE 0: Calculando e Inicializando el Gráfico Base ---");
    
    // --> Pose de inicio (ángulos fijos).
    double angulos_base[SERVO_COUNT] = {
        ANGULO_FIJO_ESLABON_0, ANGULO_FIJO_ESLABON_1,
        ANGULO_FIJO_ESLABON_2, ANGULO_FIJO_ESLABON_3, ANGULO_FIJO_ESLABON_4
    };
    
    // --> Se lleva el brazo a la posición base, y se verifica si ya en la posicion base hay colisión.
    EsferasGlobales esferas_base;
    ColisionSet centros_colisionantes;
    
    // Inicializar punteros a NULL antes de la llamada (CRÍTICO para evitar el error 'free')
    centros_colisionantes.coords = NULL;
    centros_colisionantes.count = 0;
    for(int j = 0; j < N_eslabones; j++) esferas_base[j] = NULL;

    bool colision_base = calcular_configuracion_modular(
        angulos_base, 
        esferas_local, 
        RADIO_ESFERA, 
        esferas_base, 
        &centros_colisionantes
    );
    
    // Lógica del if colision_base: sys.exit()
    // if (colision_base) {
    //     Serial.println("¡Error Crítico! Colisión detectada en la posición base. Terminando simulación.");
    //     // Liberar memoria asignada antes de salir
    //     if (centros_colisionantes.coords != NULL) free(centros_colisionantes.coords);
    //     for(int j = 0; j < N_eslabones; j++) {
    //         if (esferas_base[j] != NULL) free((void*)esferas_base[j]);
    //     }
    //     // En un ESP32, exit() o un loop infinito es la forma de "salir" de un error crítico.
    //     while(1) { 
    //         delay(100); 
    //         Serial.println("TERMINADO POR ERROR CRÍTICO.");
    //     } 
    // } else {
        Serial.println("Base OK.");
    //}
    
    // Si la base es OK, actualizamos los ángulos globales con la posición base fija
    memcpy(angulos_grados, angulos_base, sizeof(double) * SERVO_COUNT);
    
    // Intentamos mover los servos a la posición base real (conversión de ángulo a pulso)
    for (int i = 0; i < SERVO_COUNT; i++) {
        // Asumiendo que la función inversa (angle -> pulse) se haría con interpolación inversa.
        // Por simplicidad, aquí usamos un placeholder.
        // int pulse_base = mapeo_angulo_a_pulso(angulos_base[i]); 
        // servos[i].writeMicroseconds(pulse_base);
        
        // **IMPORTANTE**: Liberar la memoria de esferas_base después del chequeo de colisión
        if (esferas_base[i] != NULL) free((void*)esferas_base[i]);
    }
    if (centros_colisionantes.coords != NULL) free(centros_colisionantes.coords);


    Serial.println("Usa BTN_NEXT/PREV para cambiar de eslabón.");
    Serial.printf("Servo activo inicial -> %d (GPIO%d)\n", activeServo, servoPins[activeServo]);
}

// ================== LOOP ==================

void loop() {
    
    // NUEVO: Manejar las peticiones HTTP (debe ser lo primero)
    server.handleClient();

    // 1. Leer Botones (Navegación de servo activo con Antirrebote)
    if (fallingEdgeDebounced(BTN_NEXT, lastRawNext, lastStableNext, lastDebounceNext)) {
        activeServo = (activeServo + 1) % SERVO_COUNT;
        Serial.printf("\n--- Servo activo -> Eslabón %d (GPIO%d) ---\n", activeServo, servoPins[activeServo]);
    }
    if (fallingEdgeDebounced(BTN_PREV, lastRawPrev, lastStablePrev, lastDebouncePrev)) {
        activeServo = (activeServo - 1 + SERVO_COUNT) % SERVO_COUNT;
        Serial.printf("\n--- Servo activo -> Eslabón %d (GPIO%d) ---\n", activeServo, servoPins[activeServo]);
    }

    // 2. Leer y Filtrar Potenciómetro
    int raw = analogRead(POT_PIN);
    filteredAdc = ALPHA * raw + (1.0f - ALPHA) * filteredAdc;

    // 3. Mapear ADC -> µs (Ancho de pulso deseado para el servo ACTUAL)
    int i = activeServo;
    int min_us = servoMinUs[i];
    int max_us = servoMaxUs[i];

    int desiredPulse = map((int)filteredAdc, 0, 4095, min_us, max_us);

    int check = abs(desiredPulse - currentPulse[i]);

    // 4. Chequear Zona Muerta
    if (check >= DEADZONE_US) {
        // 5. Mapear Pulso -> Ángulo Propuesto
        float angulo_propuesto = angleFromPulseUS(desiredPulse);

        if (i == 2 || i == 3){
            angulo_propuesto = -1*angulo_propuesto;
        }

        Serial.print("Eslabón "); Serial.print(i);
        Serial.print(": F_ADC="); Serial.print((int)filteredAdc);
        Serial.print(" | us="); Serial.print(desiredPulse);
        Serial.print(" | Angulo_Prop="); Serial.print(angulo_propuesto, 1);
        Serial.print(" deg | ");
        
        // 6. Crear Configuración Propuesta (Array de 5 ángulos)
        double angulos_propuestos[SERVO_COUNT];
        memcpy(angulos_propuestos, angulos_grados, sizeof(angulos_grados)); 
        angulos_propuestos[i] = angulo_propuesto; // Aplica el cambio a la junta 'i'

        // 7. Ejecutar Sistema de Colisión
        EsferasGlobales esferas_globales_result; 
        ColisionSet centros_colisionantes;
        
        // Inicializar punteros a NULL antes de la llamada (CRÍTICO)
        centros_colisionantes.coords = NULL;
        centros_colisionantes.count = 0;
        for(int j = 0; j < SERVO_COUNT; j++) esferas_globales_result[j] = NULL;
        
        bool colision_detectada = calcular_configuracion_modular(
            angulos_propuestos, 
            esferas_local, 
            RADIO_ESFERA, 
            esferas_globales_result, 
            &centros_colisionantes
        );

        // 8. ACTUALIZACIÓN DEL ESTADO SIEMPRE (SE ENVÍA AL FRONTEND)
        angulos_grados[i] = angulo_propuesto;  // SIEMPRE actualizar, haya o no colisión

        // 8. ACTUACIÓN (Si NO hay colisión, mover el servo)
        if (!colision_detectada) {
            // MOVER SERVO
            servos[i].writeMicroseconds(desiredPulse);
            currentPulse[i] = desiredPulse;
            // Lógica OK
            angulos_grados[i] = angulo_propuesto; // Actualiza el estado global
            Serial.print("Eslabón "); Serial.print(i);
            Serial.print(": OK. Ang: ");
            Serial.print(angulo_propuesto, 1);
            Serial.println(" deg");
            
        } else {
            // COLISIÓN DETECTADA: NO mover el servo, advertir.
            Serial.print("Eslabón "); Serial.print(i);
            Serial.print("!! COLISIÓN PREVENIDA: Angulo ");
            Serial.print(angulo_propuesto, 1);
            Serial.println("° resulta en choque.");
        }

        // 9. ACTUALIZACIÓN CRÍTICA DEL ESTADO:
        // El desiredPulse pasó la deadzone, por lo tanto, es el nuevo pulso *actual*
        // para el chequeo de la próxima Deadzone, sin importar si el servo se movió.
        currentPulse[i] = desiredPulse; // <--- MOVER AQUÍ

        // ... (Tu código de log y liberación de memoria) ...
        Serial.print("TERMINE CHEQUEO");

        // 9. Liberar memoria (Crítico en ESP32) 
        if (centros_colisionantes.coords != NULL) free(centros_colisionantes.coords);
        for(int j = 0; j < SERVO_COUNT; j++) {
            if (esferas_globales_result[j] != NULL) free((void*)esferas_globales_result[j]);
        }
    }
}
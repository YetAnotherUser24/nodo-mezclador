# Guía de Implementación: WiFi y OTA Robusto para ESP32

Esta guía documenta los patrones arquitectónicos implementados para lograr una conexión WiFi a prueba de fallos y una funcionalidad OTA (Over-The-Air) confiable en los nodos de control (ej. ESP32-S3 SuperMini). 

---

## 1. Problemas de la Implementación Clásica

### ¿Por qué fallaba el OTA (Error Uploading 29%)?
1. **Caídas de Voltaje (Brownouts):** El ESP32 consume picos enormes de corriente cuando transmite a máxima potencia (20dBm). En placas pequeñas (SuperMini) con reguladores LDO minúsculos, esto causa caídas de voltaje que reinician el módem de radio perdiendo paquetes.
2. **Contención del Bus (SPI/I2C):** Durante el proceso de escritura en memoria Flash (OTA), si una tarea en segundo plano (como escribir métricas en la MicroSD) interrumpe la CPU, el bus colapsa y la transferencia TCP hace _timeout_.

### ¿Por qué el WiFi era frágil?
1. **Loops Bloqueantes:** Usar `while(WiFi.status() != WL_CONNECTED)` bloquea toda la tarea (y el núcleo) durante segundos.
2. **Reinicio de la Máquina de Estados:** Llamar a `WiFi.begin()` repetidamente en un loop resetea el proceso de negociación DHCP, provocando un ciclo infinito si el router es lento en asignar la IP.

---

## 2. Implementación de OTA Robusto

### A. Reducción de Potencia de Transmisión
Se debe limitar la potencia del WiFi justo antes de llamar a `WiFi.begin()`. Esto reduce el consumo eléctrico a más de la mitad sin sacrificar alcance útil (~20 metros).

```cpp
WiFi.setTxPower(WIFI_POWER_8_5dBm); // Crucial para la estabilidad de placas SuperMini
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
```

### B. Congelamiento de Tareas Periféricas
Apenas el ESP32 recibe la señal de que va a recibir un firmware, debe soltar todos los buses de hardware (SPI, I2C, UART) y apagar motores por seguridad.

```cpp
ArduinoOTA.onStart([]() {
  s_isOtaUpdating = true;
  odriveSdLoggerPause(true); // DETIENE EL BUS SPI
  if (s_engine) {
    s_engine->triggerEmergencyStop(); // DETIENE MOTORES
  }
});

ArduinoOTA.onEnd([]() {
  s_isOtaUpdating = false;
  odriveSdLoggerPause(false); // LIBERA EL BUS SPI
});

ArduinoOTA.onError([](ota_error_t error) {
  s_isOtaUpdating = false;
  odriveSdLoggerPause(false); // RESTAURA ESTADO EN CASO DE ERROR
});
```

### C. Aumentar el Timeout (platformio.ini)
Agregar el parámetro `--timeout` en la configuración de upload para dar margen a la red si hay lag.

```ini
[env:esp32-s3-ota]
upload_protocol = espota
upload_port = aquacontrol.local
upload_flags =
    --port=3232
    --timeout=60
```

---

## 3. Implementación de WiFi Robusto (No Bloqueante)

Este patrón confía en la función interna `AutoReconnect` de Espressif y usa una máquina de estados asíncrona dentro del bucle de FreeRTOS. Adicionalmente, incluye un "Seguro de Vida" que reinicia la pila TCP/IP si se queda atascada por más de 30 segundos.

### Configuración (Fuera del `while` / en el `setup`)
```cpp
// 1. Forzar modo Estación
WiFi.mode(WIFI_STA);

// 2. Delegar la reconexión básica al hardware
WiFi.setAutoReconnect(true);

// 3. Estabilidad de hardware
WiFi.setTxPower(WIFI_POWER_8_5dBm); 

// 4. Iniciar (SOLO UNA VEZ)
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

unsigned long lastWifiRetry = millis();
bool wasConnected = false;
```

### Loop Asíncrono (Dentro del `while`)
```cpp
if (WiFi.status() != WL_CONNECTED) {
  if (wasConnected) {
    Serial.println("[Red] Desconectado. Auto-reconnect activo...");
    digitalWrite(LED_BUILTIN, HIGH); // Apagar LED
    wasConnected = false;
    lastWifiRetry = millis();
  }
  
  // SEGURO DE VIDA: Si el AutoReconnect interno falla por 30 segundos
  // Forzamos un reinicio de la pila WiFi
  if (millis() - lastWifiRetry > 30000) {
    Serial.println("[Red] Pila WiFi atascada. Reiniciando hardware de red...");
    WiFi.disconnect(true);           // Limpia buffers internos
    vTaskDelay(pdMS_TO_TICKS(100));  // Breve pausa
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiRetry = millis();
  }
} else {
  // Cuando se reconecta exitosamente
  if (!wasConnected) {
    Serial.printf("[Red] Conectado. IP: %s\n", WiFi.localIP().toString().c_str());
    digitalWrite(LED_BUILTIN, LOW); // Encender LED
    wasConnected = true;
  }
}
```

### Resumen de Ventajas:
* **Cero bloqueos:** La función `loop()` o la tarea de FreeRTOS sigue ejecutándose a máxima velocidad sin importar si hay internet o no.
* **Auto-Recuperación:** Sobrevive a cortes de luz del router principal o a interrupciones largas de cobertura.
* **Feedback Visual:** El LED indica en todo momento el estado del socket sin penalizar el rendimiento del procesador.

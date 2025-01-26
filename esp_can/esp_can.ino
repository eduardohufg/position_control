#include <driver/twai.h>

// Pines para CAN
#define CAN_TX_PIN GPIO_NUM_5 // Conversion explícita al tipo gpio_num_t
#define CAN_RX_PIN GPIO_NUM_4 

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Configurando TWAI (CAN)...");

    // Configurar los pines y la velocidad del bus CAN
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500 kbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Aceptar todos los mensajes

    // Instanciar el controlador TWAI
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI configurado correctamente.");
    } else {
        Serial.println("Error al configurar TWAI.");
        while (1);
    }

    // Iniciar el controlador TWAI
    if (twai_start() == ESP_OK) {
        Serial.println("TWAI iniciado.");
    } else {
        Serial.println("Error al iniciar TWAI.");
        while (1);
    }
}

void loop() {
    twai_message_t message;

    // Intentar recibir un mensaje (espera bloqueante de 100 ms)
    esp_err_t res = twai_receive(&message, pdMS_TO_TICKS(100));
    if (res == ESP_OK) {
        Serial.println("Mensaje recibido:");

        // Imprimir ID y tipo del mensaje
        Serial.printf("  ID: 0x%03X\n", message.identifier);
        Serial.printf("  Tipo: %s\n", message.extd ? "Extendido" : "Estándar");
        Serial.printf("  Longitud: %d bytes\n", message.data_length_code);

        // Imprimir los datos
        Serial.print("  Datos: ");
        for (int i = 0; i < message.data_length_code; i++) {
            Serial.printf("0x%02X ", message.data[i]);
        }
        Serial.println("\n");
    } else if (res == ESP_ERR_TIMEOUT) {
        // Si no hay mensajes disponibles
        Serial.println("Esperando mensajes...");
    } else {
        // Error inesperado
        Serial.println("Error al recibir mensaje.");
    }

    delay(500); // Reducir frecuencia de depuración
}

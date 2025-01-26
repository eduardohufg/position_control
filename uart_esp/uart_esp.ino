#include <HardwareSerial.h>

// Inicializamos el puerto serie 1 del ESP32
HardwareSerial MySerial(1);
bool init_state = false;

void setup() {
  // Inicializamos el monitor serie
  Serial.begin(115200);

  // Configuramos el UART1: RX en GPIO16 y TX en GPIO17
  // Puedes cambiar los pines según tu configuración
  MySerial.begin(1000000, SERIAL_8N1, 16, 17);

  // Mensaje de inicio
  Serial.println("ESP32 UART Receiver Initialized");
}

void loop() {
  targetpos_uart();
}

void targetpos_uart() {
  if (MySerial.available() > 0) {
    String velString = MySerial.readStringUntil('\n');

    if(velString == "init_uart"){
      init_state = true;
    }
    else if(velString == "close_uart"){
      init_state = false;
    }
    else if (init_state){
    int vel = velString.toInt();
    int targetPosition = vel;
  
    Serial.print("Target Position: ");
    Serial.println(targetPosition);
    }   
  }
}

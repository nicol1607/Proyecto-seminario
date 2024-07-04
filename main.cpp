#include <Arduino.h>
#include <StarterKitNB.h>

//Definitions
#define PIN_VBAT WB_A0
#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)      // Compensation factor for the VBAT divider, depend on the board (por default 1.73); 1905
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

// Objects
StarterKitNB sk;

// NB-IoT
String apn = "m2m.entel.cl";
String user = "entelpcs";
String password = "entelpcs";
String band = "B28 LTE";
String Network = "NB";

// MQTT
String clientID = "160702";
String userBroker = "Nicol1607";
String passwordBroker = "aio_OKrj684O2aHtL8NzZvCk1imXAu4l";
String broker = "io.adafruit.com";
int port = 1883;

// msg
String topic1 = "Nicol1607/feeds/movimiento";
float batery;             

// Movimiento
String  mov_x = "";

int contador = 0;

// funciones para el sensor

void setup() {

  sk.Setup();
  delay(500);

  // código para el sensor

  sk.UserAPN(apn, user, password);
	sk.Connect(apn, band, Network);

  
}

void loop() {

  if (!sk.ConnectionStatus()) // Si no hay conexion a NB
	{
		sk.Reconnect(apn, band, Network);  // Se intenta reconectar
		delay(2000);
  }

  // código para el sensor
  mov = 40;
 
  batery = (analogRead(PIN_VBAT) * REAL_VBAT_MV_PER_LSB /1000);

  contador = contador + 1;

  Serial.println(hum);
  Serial.println(temp);
  Serial.println(batery);

  if (!sk.LastMessageStatus)  // Para conectar al broker
  {
    sk.ConnectBroker(clientID, userBroker, passwordBroker, 0, broker, port);  
    delay(2000);
  }

  sk.SendMessage(String(contador), topic3, 0, 0, 0, 0, 10000);    // Se envia el mensaje
  delay(2000);
  


}

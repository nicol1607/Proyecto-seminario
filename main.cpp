#include <Arduino.h>
#include <StarterKitNB.h>
#include "SparkFunLIS3DH.h" //http://librarymanager/All#SparkFun-LIS3DH
#include <Wire.h>

//sensor 3-axis
LIS3DH SensorTwo(I2C_MODE, 0x18);


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
String topic2  = "Movimiento en x";
String topic3 = "Movimiento en y";

float batery;             

// Movimiento
String  mov_x = "";
String mov_y = "";



void lis3dh_read_data()
{
  // read the sensor value
  uint8_t cnt = 0;

  Serial.print(" X(g) = ");
  Serial.println(SensorTwo.readFloatAccelX(), 3);
  Serial.print(" Y(g) = ");
  Serial.println(SensorTwo.readFloatAccelY(), 3);
  Serial.print(" Z(g)= ");
  Serial.println(SensorTwo.readFloatAccelZ(), 3);
}



// funciones para el sensor

void setup() {

  sk.Setup();
  delay(500);

  // código para el sensor
  time_t timeout = millis();
	Serial.begin(115200);
	while (!Serial)
	{
		if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
	}

	if (SensorTwo.begin() != 0)
	{
		Serial.println("Problem starting the sensor at 0x18.");
	}
	else
	{
		Serial.println("Sensor at 0x18 started.");
		// Set low power mode
		uint8_t data_to_write = 0;
		SensorTwo.readRegister(&data_to_write, LIS3DH_CTRL_REG1);
		data_to_write |= 0x08;
		SensorTwo.writeRegister(LIS3DH_CTRL_REG1, data_to_write);
		delay(100);

		data_to_write = 0;
		SensorTwo.readRegister(&data_to_write, 0x1E);
		data_to_write |= 0x90;
		SensorTwo.writeRegister(0x1E, data_to_write);
		delay(100);
	}
	Serial.println("enter !");











  //final dejarlo asi

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
  lis3dh_read_data();
	delay(1000);
 
  batery = (analogRead(PIN_VBAT) * REAL_VBAT_MV_PER_LSB /1000);

  

  
  Serial.println(batery);

  if (!sk.LastMessageStatus)  // Para conectar al broker
  {
    sk.ConnectBroker(clientID, userBroker, passwordBroker, 0, broker, port);  
    delay(2000);
  }

  sk.SendMessage(String(topic2), topic3, 0, 0, 0, 0, 10000);    // Se envia el mensaje
  delay(2000);
  sk.SendMessage(String(topic3), topic3, 0, 0, 0, 0, 10000);    // Se envia el mensaje
  delay(2000);
  
  


}

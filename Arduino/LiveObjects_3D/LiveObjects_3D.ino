/*
* Software Name : iotContinuum
* SPDX-FileCopyrightText: Copyright (c) 2023 Orange SA
* SPDX-License-Identifier: MIT
*
* This software is distributed under the MIT License,
* the text of which is available at https://spdx.org/licenses/MIT.html
* or see the "LICENSE" file for more details.
*/

/*! @file LiveObjects.ino
Copyright (c) 2023 ORANGE

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,


distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION */

#include <Arduino.h>
#include <HL7812.h>
#include <LiveObjects.h>
#include <Wire.h>
#include "PCA9534.h"
#include <BME280I2C.h>
#include <BME280.h>
#include "LSM6DSOXSensor.h"
#include <PubSubClient.h>
#include <SoftwareSerial.h>

#define DELAY_INFO      60000

LiveObjects* mLiveObjects = NULL;
// HL7812Base* hl7812 = mLiveObjects->getHl7812();

PCA9534 gpio;
BME280I2C bme;
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);
  String temperature;
  String pression;
  String humidite;
  String position_gyro;
  bool   connected_var;

// ===== VARIABLES POUR ORIENTATION 3D =====
float pitch = 0;  // Inclinaison avant/arrière
float roll = 0;   // Inclinaison gauche/droite  
float yaw = 0;    // Rotation horizontale
unsigned long lastOrientationTime = 0;
unsigned long lastYawResetTime = 0;  // Pour reset automatique du yaw

/* This code example shows how to send data to LiveObjects servers using MQTT protocol with the HL7812.
IMPORTANT: LiveObjects credentials must be replaced in ConfigLiveObjects.h file first. */


void enable_battery_check() {

  pinMode(BAT_LEVEL, INPUT_ANALOG);
  analogReadResolution(12);

  delay(2000);
  Serial.println("Enable battery level monitoring");
}

void display_battery_level()
{

  float batLevel = (float)((analogRead(BAT_LEVEL) * 3.3) / 4096) * 2;
  Serial.print("Battery Level:"); Serial.print(batLevel, 2); Serial.println("V");
  delay(1000);
}


void setupSensor() { 


  /* Init Serial (Serial Monitor) */
  Serial.begin(115200);

  /* Init GPIOs */
  gpio.begin();

  /* Init I2C */
  Wire.begin();
  
  /* Init BME280 - temperature + humidity + pressure sensor */
  bme.begin();
  
  /* Init LSM6DSOX sensor - accelerometer & gyroscope */
  lsm6dsoxSensor.begin();

  /* Setup LED GPIOs in ouput mode */
  gpio.pinMode(RED_LED, OUTPUT);
  gpio.pinMode(GREEN_LED, OUTPUT);
  gpio.pinMode(BLUE_LED, OUTPUT);


  gpio.digitalWrite(RED_LED, HIGH); // LED rouge Off
  gpio.digitalWrite(GREEN_LED, HIGH); // LED verte Off
  gpio.digitalWrite(BLUE_LED, HIGH); // LED bleu Off

  delay(500);
 
  /* Enable accelerometer and gyroscope, and check success */
  if (lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK) {
    Serial.println("Success enabling accelero and gyro");
  } else {
    Serial.println("Error enabling accelero and gyro");
  }
 
  /* Read ID of device and check that it is correct */
  uint8_t id;
  lsm6dsoxSensor.ReadID(&id);
  if (id != LSM6DSOX_ID) {
    Serial.println("Wrong ID for LSM6DSOX sensor. Check that device is plugged");
  } else {
    Serial.println("Receviced correct ID for LSM6DSOX sensor");
  }

  // Initialiser le timer pour le calcul d'orientation
  lastOrientationTime = millis();
  lastYawResetTime = millis();  // Timer pour reset yaw
  
  Serial.println("Start Setup Sensor");
  Serial.println("✅ Orientation 3D activée");
  Serial.println("⚙️  Zone morte gyroscope: 5 dps");
  Serial.println("🔄 Reset automatique yaw: toutes les 30s");
}

// ===== FONCTION CALCUL ORIENTATION 3D =====
void calculateOrientation(int32_t acceleration[3], int32_t rotation[3]) {
  // Convertir accélération de mg vers g
  float accelX = acceleration[0] / 1000.0;
  float accelY = acceleration[1] / 1000.0;
  float accelZ = acceleration[2] / 1000.0;
  
  // Convertir rotation de mdps vers dps
  float gyroX = rotation[0] / 1000.0;
  float gyroY = rotation[1] / 1000.0;
  float gyroZ = rotation[2] / 1000.0;
  
  // ===== PITCH et ROLL depuis accéléromètre (STABLE) =====
  pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  roll = atan2(accelY, accelZ) * 180.0 / PI;
  
  // ===== YAW depuis gyroscope (intégration) =====
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastOrientationTime) / 1000.0; // secondes
  lastOrientationTime = currentTime;
  
  // ⭐ ZONE MORTE : ignorer les petites valeurs (seuil à 5 dps)
  const float GYRO_DEAD_ZONE = 5.0; // degrés par seconde
  
  if (abs(gyroZ) > GYRO_DEAD_ZONE) {
    yaw += gyroZ * deltaTime;
  }
  // Sinon, on considère que l'objet est immobile et on n'intègre pas
  
  // Normaliser yaw entre -180° et +180°
  while (yaw > 180.0) yaw -= 360.0;
  while (yaw < -180.0) yaw += 360.0;
  
  // ⭐ RESET AUTOMATIQUE du yaw toutes les 30 secondes pour éviter dérive
  const unsigned long YAW_RESET_INTERVAL = 30000; // 30 secondes
  if (currentTime - lastYawResetTime > YAW_RESET_INTERVAL) {
    yaw = 0;
    lastYawResetTime = currentTime;
    Serial.println("🔄 Yaw réinitialisé automatiquement");
  }
}

bool getSensorInfo()

{

  Serial.println(" declenchement fonction getSensorInfo()");
/* Read temperature, humidity & pressure values */
  float temp = bme.temp();
  float hum = bme.hum();
  float pres = bme.pres();
  /* Plot data (friendly format for the Serial Plotter) */

  temperature = String(temp);
  pression = String(pres);
  humidite = String(hum);
  
  // Serial.println("Temp:" + temperature + "°C, Humidity:" + humidite + "%, Pressure:" + pression) + "Pa");

  /* Read accelerometer */
  int32_t acceleration[3] = {0, 0, 0};
  uint8_t acceleroStatus;
  lsm6dsoxSensor.Get_X_DRDY_Status(&acceleroStatus);
  if (acceleroStatus == 1) { /* Status == 1 means a new data is available */
    lsm6dsoxSensor.Get_X_Axes(acceleration);
    /* Plot data (friendly format for the Serial Plotter) */
    Serial.println("Accel_X:" + String(acceleration[0]) + "mg, Accel_Y:" + String(acceleration[1]) + "mg, Accel_Z:" + String(acceleration[2]) + "mg");
  }

  /* Read gyroscope */
  int32_t rotation[3] = {0, 0, 0};
  uint8_t gyroStatus;
  lsm6dsoxSensor.Get_G_DRDY_Status(&gyroStatus);
  if (gyroStatus == 1) { // Status == 1 means a new data is available
    lsm6dsoxSensor.Get_G_Axes(rotation);
    /* Plot data in milli degrees per second (friendly format for the Serial Plotter) */
    Serial.println("Rot_X:" + String(rotation[0]) + "mdps, Rot_Y:" + String(rotation[1]) + "mdps, Rot_Z:" + String(rotation[2]) + "mdps");
    
    // ===== CALCULER L'ORIENTATION 3D =====
    calculateOrientation(acceleration, rotation);
    
    // ===== CONSTRUIRE LE JSON COMPLET POUR DASHBOARD 3D =====
    // Format: {gyro:{x,y,z}, accel:{x,y,z}, orientation:{pitch,roll,yaw}}
    
    // Convertir les valeurs
    float gyroX_dps = rotation[0] / 1000.0;
    float gyroY_dps = rotation[1] / 1000.0;
    float gyroZ_dps = rotation[2] / 1000.0;
    
    float accelX_g = acceleration[0] / 1000.0;
    float accelY_g = acceleration[1] / 1000.0;
    float accelZ_g = acceleration[2] / 1000.0;
    
    // Construire le JSON
    position_gyro = "{";
    position_gyro += "\"gyro\":{\"x\":" + String(gyroX_dps, 2) + ",\"y\":" + String(gyroY_dps, 2) + ",\"z\":" + String(gyroZ_dps, 2) + "},";
    position_gyro += "\"accel\":{\"x\":" + String(accelX_g, 3) + ",\"y\":" + String(accelY_g, 3) + ",\"z\":" + String(accelZ_g, 3) + "},";
    position_gyro += "\"orientation\":{\"pitch\":" + String(pitch, 1) + ",\"roll\":" + String(roll, 1) + ",\"yaw\":" + String(yaw, 1) + "}";
    position_gyro += "}";
    
    Serial.println("🧭 Orientation: Pitch=" + String(pitch, 1) + "° Roll=" + String(roll, 1) + "° Yaw=" + String(yaw, 1) + "°");
  }
  
	return true;
}


void hl7812Callback(void* object, uint16_t type , uint32_t value) {
   Serial.println("hl7812Callback callback"); 
}

void mqttMessageReceived(String &topic, String &payload) {
    Serial.println("incoming: " + topic + " - " + payload);    
}



bool publishDeviceInfo() {
    HL7812Base* hl7812 = mLiveObjects->getHl7812();

    String imsi = hl7812->getIMSI();
    Serial.print("ISMI="); Serial.println(imsi); 

    String rssi = hl7812->getRSSI();
    Serial.print("RSSI="); Serial.println(rssi); 

    String imei = hl7812->getIMEI();
    Serial.print("IMEI="); Serial.println(imei); 

    String apn = hl7812->getAPN();
    Serial.print("APN="); Serial.println(apn); 

    String deviceInfo = "{\"imsi\":\"" + imsi + "\",\"imei\":\"" + imei + "\",\"rssi\":" + rssi + ",\"apn\":" + apn + "}";
    
    return mLiveObjects->publish(String(STREAM_ID_CONFIG).c_str(), deviceInfo.c_str(), TAG_CONFIG);  
  return true;
}


void schedule() {




  
    uint32_t countBeforeReset = 0;
    uint32_t current = millis();

    while(true) {
        bool state = mLiveObjects->loop();   
        
        if(!state) {
            Serial.println("Module disconnected");
            Serial.println("Attempt to reconnect");

            mLiveObjects->disconnect();
            delay(3000);

            if(!mLiveObjects->connect()) {
                countBeforeReset++;
                if(countBeforeReset > 3) {
                    NVIC_SystemReset();
                }
            }

            delay(2000);
        }        
     
        if(millis() - current > DELAY_INFO) {
            Serial.println("Publish info device");
            bool toto = publishDeviceInfo();
            delay(250);            
            current = millis();           
        }
        
        delay(250);
    } 
}




bool publishSensorInfo() {

    gpio.digitalWrite(BLUE_LED, LOW); // LED On
    
    bool status = getSensorInfo();
    Serial.println("publishSensorInfo : Temp = " + temperature + " °C, Humidity = " + humidite + " %, Pressure = " + pression + " Pa");
    Serial.println("publishSensorInfo : Position 3D = " + position_gyro);
    
/* case liveObjects
    status = mLiveObjects->publish(String(STREAM_ID_CONFIG).c_str(), temperature.c_str(), TAG_TEMPERATURE);
    status = mLiveObjects->publish(String(STREAM_ID_CONFIG).c_str(), humidite.c_str(), TAG_HUMIDITE);
    status = mLiveObjects->publish(String(STREAM_ID_CONFIG).c_str(), pression.c_str(), TAG_PRESSION);
*/
// case mosquitto

    status = mLiveObjects->publish_mosquitto(String(STREAM_ID_DATA).c_str(), temperature.c_str(), TAG_TEMPERATURE, 1);
    status = mLiveObjects->publish_mosquitto(String(STREAM_ID_DATA).c_str(), humidite.c_str(), TAG_HUMIDITE, 3);
    status = mLiveObjects->publish_mosquitto(String(STREAM_ID_DATA).c_str(), pression.c_str(), TAG_PRESSION, 2);
    
    // ⭐ PUBLICATION POSITION 3D AVEC FORMAT COMPLET
    status = mLiveObjects->publish_mosquitto(String(STREAM_ID_DATA).c_str(), position_gyro.c_str(), TAG_POSITION, 4);
   
    if (status) {
      Serial.println("✅ Position 3D publiée avec succès");
    } else {
      Serial.println("❌ Erreur publication Position 3D");
    }
    
    delay(1000);
    gpio.digitalWrite(BLUE_LED, HIGH); // LED Off
    
    return status;
}

void receptionSMS() {
  
  bool smsrecu = false;
  HL7812Base* hl7812 = mLiveObjects->getHl7812();
  SoftwareSerial mySerial(2, 3); // RX, TX pour Arduino

  while (!smsrecu) {
   Serial.println("En attente de Message SMS"); 
  // Lire les messages entrants du module HL7812
  if (mySerial.available()) {
    String message = mySerial.readString();
    Serial.println("Message reçu : ");
    Serial.println(message);
    
    // Si un message de notification de réception de SMS est détecté
    if (message.indexOf("+CMTI:") != -1) {
      // Extraire l'indice du message
      int index = message.substring(message.indexOf(",") + 1).toInt();
      
      // Lire le SMS à partir de l'indice
      hl7812->ReadSMS(index);
      smsrecu = true;
    }
  }

  // Attendre avant de vérifier à nouveau
  delay(500);
  }

  Serial.println("Message affiché sur la CONSOLE");
}



void execute() {    

   
     
    mLiveObjects = new LiveObjects();
    Serial.println("Connect to LiveObjects");
    HL7812Base* hl7812 = mLiveObjects->getHl7812();
	  Serial.println("getHl7812 objects");

    //hl7812->initializeSMSMT();
    //receptionSMS();
    connected_var = mLiveObjects->begin(); 
    if (connected_var) {
    Serial.println("connected_var = TRUE : PDP Context ouvert");
    }
    else if (!connected_var) {
    Serial.println("connected_var = FALSE : PDP Context ferme");
    }
    if(!mLiveObjects->connect()) {
        Serial.println("Connection to LiveObjects failed");
        
        while(true) {
         gpio.digitalWrite(RED_LED, LOW); // LED On
         delay(500);
         gpio.digitalWrite(RED_LED, HIGH); // LED Off
         delay(500);
        }
    }     




    Serial.println("Connected to LiveObjects");

/*
	mLiveObjects->setReceivedMessageCallback(mqttMessageReceived);
    mLiveObjects->subscribeCommand();
*/    
    if(!publishDeviceInfo()) {
        Serial.println("publishDeviceInfo failed");
        return;
    }
    else {
        Serial.println("publishDeviceInfo success ");
    }
    
   // schedule(); 

	setupSensor();
  display_battery_level();
	bool Status = publishSensorInfo();
	//boucle envoi de donnée
	while(Status == true) { 
    display_battery_level();
    Status = publishSensorInfo();
    delay(1000);
    
		// delay(7200000);
	  
 	

//session recovery

if (Status == false) {
            Serial.println("on relance la session MQTT car Status = false");
            //connected_var = mLiveObjects->updatePDP();
            if(!mLiveObjects->reconnect()) {
                    Serial.println("on tente un reouverture de PDP");
                    connected_var = mLiveObjects->updatePDP();
                    Serial.println("on tente un reouverture de socket TCP");
   
                    if(!mLiveObjects->reconnect()) {
                    Serial.println("recovery en echec");
                          while(true)
                          {
                            gpio.digitalWrite(RED_LED, LOW); // LED On
                            delay(500);
                            gpio.digitalWrite(RED_LED, HIGH); // LED Off
                            delay(500);
                           } // end while
   
   
                  }
                  else
                  {
                    Status = true ;
                  }
            }
            else
            {
              Status = true ;
            }
}
 else
 {
  delay(DEVICE_DELAY);
 }
} // fin while 

/*boucle envoi de donnée
  while(Status == true) { 
    Status = publishSensorInfo();
   //  delay(7200000);
   delay(DEVICE_DELAY);
  }*/
 
}
  
	

void setup()
{  
/* while(!Serial){
  }
*/ 
  Serial.begin(115200);
  delay(2000);
  Serial.println("Start test Liveobjects");
  Serial.println("=== VERSION AVEC ORIENTATION 3D ===");

  enable_battery_check();

  execute();
}

void loop(){
}

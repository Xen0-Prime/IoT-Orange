/*
* Software Name : iotContinuum
* SPDX-FileCopyrightText: Copyright (c) 2023 Orange SA
* SPDX-License-Identifier: MIT
*
* This software is distributed under the MIT License,
* the text of which is available at https://spdx.org/licenses/MIT.html
* or see the "LICENSE" file for more details.
*/

#include "LiveObjects.h"

LiveObjects::LiveObjects() : mHl7812client(), mMqttClient(MQTT_BUFFER) {        
    mMqttClient.begin(mHl7812client);
    mMqttClient.setTimeout(500);
}

LiveObjects::~LiveObjects() {
}

// void LiveObjects::begin() {
bool LiveObjects::begin() {
    delay(1000);
   /* if(mHl7812client.initialize(LTE, "", true, 80000)) { */ 
  //  if(mHl7812client.initialize(RAT_TYPE, "orangeweb" , true, 80000)) {
    
    if(mHl7812client.initialize(ENABLE_PSM,ENABLE_EDRX,CMD_PSM,CMD_EDRX,RAT_TYPE, APN_NAME , true, 80000)) {
        Serial.println("HL7812 initialized");
        return true;
    }
    else {
        Serial.println("HL7812 not initialized");
        return false;
    }
}

bool LiveObjects::updatePDP() {
    delay(1000);
   /* if(mHl7812client.initialize(LTE, "", true, 80000)) { */ 
  //  if(mHl7812client.initialize(RAT_TYPE, "orangeweb" , true, 80000)) {
    
    if(mHl7812client.initialize_update(ENABLE_PSM,ENABLE_EDRX,CMD_PSM,CMD_EDRX,RAT_TYPE, APN_NAME , true, 80000)) {
        Serial.println("HL7812 initialized");
        return true;
    }
    else {
        Serial.println("HL7812 not initialized");
        return false;
    }
}

bool LiveObjects::connect() {
    mMqttClient.setHost(LIVEOBJECTS_HOST, LIVEOBJECTS_MQTT_PORT);
    
    Serial.println("mMqttClient.setHost : OK");
    return mMqttClient.connect(DEVICE_NAME, MQTT_USER_NAME, MQTT_PASSWORD);
}

bool LiveObjects::reconnect() {
    // mMqttClient.setHost(LIVEOBJECTS_HOST, LIVEOBJECTS_MQTT_PORT);
    
    Serial.println("debut de reconnexion");
    return mMqttClient.connect(DEVICE_NAME, MQTT_USER_NAME, MQTT_PASSWORD);
}


bool LiveObjects::publish(const char* streamId, const char* value, const char* tag) {
    String request = "{\"s\":\"" + String (streamId) + "\",\"v\":" + String(value) + ", \"tags\":[\"" + String(tag) + "\"]}";
    return mMqttClient.publish(LIVEOBJECTS_DATA_TOPIC, request);
}

bool LiveObjects::publish_mosquitto(const char* streamId, const char* value, const char* tag , int type_donnee) {
     bool Status = false; 
     String request = "{\"s\":\"" + String (streamId) + "\",\"v\":" + String(value) + ", \"tags\":[\"" + String(tag) + "\"]}";
     if ( type_donnee == 1 ) {
         return mMqttClient.publish(LIVEOBJECTS_DATA_TOPIC_TEMPERATURE, request); 
     }
     else if ( type_donnee == 2 ) {
         return mMqttClient.publish(LIVEOBJECTS_DATA_TOPIC_PRESSION, request); 
     }
     else if ( type_donnee == 3 ) {
         return mMqttClient.publish(LIVEOBJECTS_DATA_TOPIC_HUMIDITE, request); 
     }
     else if ( type_donnee == 4 ) {
         return mMqttClient.publish(LIVEOBJECTS_DATA_TOPIC_POSITION, request); 
     }
     return Status;
}

bool LiveObjects::subscribeCommand()
{
    return mMqttClient.subscribe(LIVEOBJECTS_CMD_TOPIC);
}

void LiveObjects::setReceivedMessageCallback(MQTTClientCallbackSimple callback)
{
    mMqttClient.onMessage(callback);
}

bool LiveObjects::disconnect() {
    return mMqttClient.disconnect();      
}

bool LiveObjects::loop() {
    return mMqttClient.loop();
}

HL7812Base* LiveObjects::getHl7812() {
    return &mHl7812client;
}

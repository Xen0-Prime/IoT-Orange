/*
* Software Name : iotContinuum
* SPDX-FileCopyrightText: Copyright (c) 2023 Orange SA
* SPDX-License-Identifier: MIT
*
* This software is distributed under the MIT License,
* the text of which is available at https://spdx.org/licenses/MIT.html
* or see the "LICENSE" file for more details.
*/

/*! @file ConfigLiveObjects.h
 Copyright (c) 2023 ORANGE

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. */

#pragma once

#define MQTT_BUFFER             512
#define LIVEOBJECTS_MQTT_PORT   1883
#define LIVEOBJECTS_HOST		    "test.mosquitto.org"
/*#define LIVEOBJECTS_HOST        "91.121.93.94"*/
#define DEVICE_NAME             "SampleOAGua"
#define STREAM_ID_CONFIG        "urn:lo:nsid:SampleConfig"
#define STREAM_ID_DATA          "urn:lo:nsid:SampleData"
#define STREAM_ID_EVENT         "urn:lo:nsid:SampleEvent"

#define MQTT_USER_NAME          ""
#define MQTT_PASSWORD           ""

#define RAT_TYPE                LTE   //value : GSM or LTE
#define APN_NAME                "orangeiot"       
/*#define DEVICE_DELAY            3000000*/
#define DEVICE_DELAY            10000
#define CMD_EDRX                 "AT+KEDRXCFG=2,4,1,3"
#define CMD_PSM                  "AT+CPSMS=1,,,\"00100100\",\"00001000\""
#define ENABLE_EDRX              "ON"    // possible value : ON or OFF
#define ENABLE_PSM               "ON"    // possible value : ON or OFF

#define LIVEOBJECTS_DATA_TOPIC              "dev/data"
#define LIVEOBJECTS_DATA_TOPIC_TEMPERATURE  "dev/data/guadeloupe/temperature"
#define LIVEOBJECTS_DATA_TOPIC_PRESSION     "dev/data/guadeloupe/pression"
#define LIVEOBJECTS_DATA_TOPIC_HUMIDITE     "dev/data/guadeloupe/humidite"
#define LIVEOBJECTS_DATA_TOPIC_POSITION     "dev/data/guadeloupe/position"

#define LIVEOBJECTS_CMD_TOPIC   "dev/cmd"

#define TAG_SENSORS             "sensor"
#define TAG_EVENT               "event"
#define TAG_CONFIG              "config"
#define TAG_TEMPERATURE         "temperature"
#define TAG_PRESSION            "pression"
#define TAG_HUMIDITE            "humidite"
#define TAG_POSITION            "position"

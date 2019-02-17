/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include "mbed.h"

// #ifdef DEVICE_SPI

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "trace_helper.h"
#include "lora_radio_helper.h"
#include "board.h"

using namespace events;

#define LORAWAN_APP_DATA_SIZE 40

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[LORAWAN_APP_DATA_SIZE];
uint8_t rx_buffer[LORAWAN_APP_DATA_SIZE];

int BuffPtr=0;


/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        5000

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3


/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS * EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it down the radio object.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;


const int buffer_size = 255;
// might need to increase buffer size for high baud rates
char gps_buffer[buffer_size+1];

volatile int rx_in=0;

/**
 * Entry point for application
 */
int main (void)
{
    // setup tracing
    setup_trace();

    printf("Starting\n");

    BoardInit();
    
    Gps.enable(1);
    wait(0.5);
    Gps.service();
    Gps.verbose= true ;

    // To test that data is received from GPS this you must make m_uart public
    Gps.m_uart.baud(9600);
    // GPS setup
// https://nmeachecksum.eqth.net/
const char PMTK_API_SET_SUPPORT_QZSS_NMEA[] = "$PMTK351,1*28\r\n";

Gps.m_uart.printf(PMTK_API_SET_SUPPORT_QZSS_NMEA);
 wait(0.5);
    NVIC_DisableIRQ(USART3_IRQn);

    printf("GPS Echo\n");    
    for (int j=0;j<20000;j++) {
        while ( Gps.m_uart.readable()) {
            gps_buffer[rx_in] = Gps.m_uart.getc();
            printf("%c",gps_buffer[rx_in]);
        }
    }

    NVIC_EnableIRQ(USART3_IRQn);
const char PMTK_API_SET_NMEA_OUTPUT[] = "$PMTK314,1,1,0,1,1,2,3,2,0,0,0,0,0,0,0,0,0,1,2*28\r\n";
Gps.m_uart.printf(PMTK_API_SET_NMEA_OUTPUT);
/*
 0 GLL GLL interval - Geographic position - latitude longitude
1 RMC RMC interval - Recommended minimum specific GNSS sentence
2 VTG VTG interval - Course over ground and ground speed
3 GGA GGA interval - GPS fix data
4 GSA GSA interval - GNSS DOPS and active satellites
5 GSV GSV interval - GNSS satellites in view
6 GRS GRS interval – GNSS range residuals
7 GST GST interval – GNSS pseudorange error statistics
17 ZDA ZDA interval - Time and date
18 MCHN PMTKCHN interval - GNSS channel status
*/
const char PMTK_API_SET_GNSS_SEARCH_MODE[] = "$PMTK353,1,1,1,1,0*2B\r\n";
/*
GPS,GLONASS,GALILEO,GALILEO_FULL
*/
Gps.m_uart.printf(PMTK_API_SET_GNSS_SEARCH_MODE);
// enable SBAS from QZSS?
const char PMTK_API_SET_DGPS_MODE[] = "$PMTK301,2*2E\r\n";
Gps.m_uart.printf(PMTK_API_SET_DGPS_MODE);

const char PMTK_API_SET_SBAS_ENABLED[] = "$PMTK313,1*2E\r\n";
Gps.m_uart.printf(PMTK_API_SET_SBAS_ENABLED);
 wait(0.5);
    NVIC_DisableIRQ(USART3_IRQn);

    printf("GPS Echo\n");    
    for (int j=0;j<20000;j++) {
        while ( Gps.m_uart.readable()) {
            gps_buffer[rx_in] = Gps.m_uart.getc();
            printf("%c",gps_buffer[rx_in]);
        }
    }

    NVIC_EnableIRQ(USART3_IRQn);
    Gps.service();

    printf(" LIS3DH dev id is %d \n", acc.read_id());   
    if (acc.read_id() == I_AM_LIS3DH){
            printf(" LIS3DH found\n");   
    } else 
    {
        printf(" LIS3DH not found!\n");           
    }

    // CHECK we got the pins right from mbed
    if (PA_7 != MBED_CONF_APP_LORA_SPI_MOSI)
       printf("MOSI %d\n",MBED_CONF_APP_LORA_SPI_MOSI);

    if (PA_6 != MBED_CONF_APP_LORA_SPI_MISO)
       printf("MISO %d\n",MBED_CONF_APP_LORA_SPI_MISO);

    if (PA_5 != MBED_CONF_APP_LORA_SPI_SCLK)
       printf("SCLK %d\n",MBED_CONF_APP_LORA_SPI_SCLK);

    if (PB_0 != MBED_CONF_APP_LORA_CS)
       printf("CS %d\n",MBED_CONF_APP_LORA_CS);

    if (PH_1 != MBED_CONF_APP_LORA_TCXO)
       printf("TXCO %d\n",MBED_CONF_APP_LORA_TCXO);


    // End check


    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");

        return -1;
    }

    printf("\r\n New Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
                                          != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }


    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");


    Gps.service( );


    retcode = lorawan.connect();

    if (retcode == LORAWAN_STATUS_OK ||
        retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

static int send_counter=0;

/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;

    Gps.service( );

    NVIC_DisableIRQ(USART3_IRQn);

    printf("GPS Echo\n");    
    for (int j=0;j<30000;j++) {
        while ( Gps.m_uart.readable()) {
            gps_buffer[rx_in] = Gps.m_uart.getc();
            printf("%c",gps_buffer[rx_in]);
        }
    }

    NVIC_EnableIRQ(USART3_IRQn);

    if (Gps.have_fix) 
    {
        BuffPtr = 0;

        uint16_t altitudeGps = atoi( Gps.NmeaGpsData.NmeaAltitude );
        double hdopGps;
        double latitude=Gps.Latitude;
        double longitude=Gps.Longitude;
		altitudeGps =  atoi( Gps.NmeaGpsData.NmeaAltitude ); // in m
		hdopGps =  atof( Gps.NmeaGpsData.NmeaHorizontalDilution);
		printf("[Debug]: latitude: %f, longitude: %f , altitudeGps: %d \n", latitude, longitude, altitudeGps);
		int32_t lat = ((latitude + 90) / 180.0) * 16777215;
		int32_t lon = ((longitude + 180) / 360.0) * 16777215;
		int16_t alt = altitudeGps;
		int8_t hdev =  hdopGps; 
		

        printf("Lat: %d Lon: %d, Alt: %d\r\n", (int)lat, (int)lon, alt);

        tx_buffer[BuffPtr++] = lat >> 16;
        tx_buffer[BuffPtr++] = lat >> 8;
        tx_buffer[BuffPtr++] = lat;

        tx_buffer[BuffPtr++] = lon >> 16;
        tx_buffer[BuffPtr++] = lon >> 8;
        tx_buffer[BuffPtr++] = lon;

        tx_buffer[BuffPtr++] = alt >> 8;
        tx_buffer[BuffPtr++] = alt;

        tx_buffer[BuffPtr++] = hdev;
        tx_buffer[BuffPtr++] = atoi( Gps.NmeaGpsData.NmeaFixQuality);
        tx_buffer[BuffPtr++] = atoi( Gps.NmeaGpsData.NmeaSatelliteTracked);
  
        packet_len=BuffPtr;
        
        retcode = lorawan.send(2, tx_buffer, packet_len,
                            MSG_CONFIRMED_FLAG);

        if (retcode < 0) {
            retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
                    : printf("\r\n send() - Error code %d \r\n", retcode);

            return;
        }
        


        printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
        memset(tx_buffer, 0, sizeof(tx_buffer));
    }
    else {
        printf("No GPS Fix yet\n");
        
    }
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    int16_t retcode;
    retcode = lorawan.receive(MBED_CONF_LORA_APP_PORT, rx_buffer,
                              sizeof(rx_buffer),
                              MSG_CONFIRMED_FLAG|MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" Data:");

    for (uint8_t i = 0; i < retcode; i++) {
        printf("%x", rx_buffer[i]);
    }

    printf("\r\n Data Length: %d\r\n", retcode);

    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    Gps.service( );

    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");

            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");

            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);

            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");

            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);    

            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

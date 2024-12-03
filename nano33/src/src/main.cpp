#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include <cassert>
#include <FastLED.h>
#include <queue.h>
#include <process_serial.cpp>
#define LED_NUM 13
#define LED_PIN 6
#define TIMEOUT 5000
#define DEBUG 0

CRGB leds[LED_NUM];
float x, y, z;
int degreesX = 0;
int degreesY = 0;
int plusThreshold = 30, minusThreshold = -30;
char ssid[] = "R9000P";
char pass[] = "12348765";
int status = WL_IDLE_STATUS;
int current_shape = -1;
int in_data = -1;
int counter = 0;
int LED_flow_counter = 0;
int LED_flow_enable = 0;
CRGB LED_flow_color = CRGB::White;
WiFiClient client_to_esp32;
const char server_esp32[] = "192.168.137.156";
const uint16_t port_esp32 = 1234;
WiFiClient client_to_py;
const char server_py[] = "192.168.137.218";
const uint16_t port_py = 1234;
int command = 0;
int trigger = 0;
CRGB command_color[3] = {CRGB::Red, CRGB::Green, CRGB::Blue};
float imu_x, imu_y, imu_z, gyro_x, gyro_y, gyro_z;
char IMU_data[50];
uint32_t IMU_index;

bool connectToServer(WiFiClient* client, const char * server, uint16_t port);
void configure_timer();
void show_wave();
void sendMessage(WiFiClient* client, const char* message);
String receiveMessage(WiFiClient* client, int in_timeout_ms);
void read_IMU();

void setup() {
    // test_case();
    if (DEBUG)
    {
        Serial.begin(9600);
        while (!Serial);
        Serial.println("Started");
    }

    // check if IMU is up
    if (!IMU.begin())
    {
        if (DEBUG)
            Serial.println("Failed to initialize IMU!");
        while (1);
    }

    if (DEBUG)
    {
        Serial.print("Accelerometer sample rate = ");
        Serial.print(IMU.accelerationSampleRate());
        Serial.println("Hz");
        Serial.print("Attempting to connect to network: ");
        Serial.println(ssid);
    }

    // try to connect to wifi
    while (status != WL_CONNECTED)
    {
        status = WiFi.begin(ssid, pass);
        delay(500);
        if (DEBUG)
            Serial.println(".");
    }
    if (DEBUG)
        Serial.println("Connected");

    // initialize ws2812b light strip using FastLED
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_NUM);
    FastLED.clear();
    FastLED.show();

    // Configure TC3 timer for 100 ms interrupts
    configure_timer();

    // connect to server
    while (!connectToServer(&client_to_esp32, server_esp32, port_esp32));

    // Send a message to the server
    // while(1)
    // {
    //     sendMessage(client_to_esp32, "1");
    //     delay(2000);
    //     sendMessage(client_to_esp32, "2");
    //     delay(2000);
    // }
    // Receive a response from the server
    // receiveMessage();

    // Disconnect from the server
    // client.stop();
    // Serial.println("Disconnected from server.");
}

void loop()
{
    // block here to receive commands
    int result = MainTask();
    if (DEBUG)
    {
        if (result != -1)
        {
            Serial.println(result);
        }
    }
    if (result != -1)
    {
        if (result < 3)
        {
            LED_flow_enable = 1;
            LED_flow_color = command_color[0];
            sendMessage(&client_to_esp32, "1");
        }
        else if (result < 6)
        {
            LED_flow_enable = 1;
            LED_flow_color = command_color[1];
            sendMessage(&client_to_esp32, "2");
        }
        else
        {
            LED_flow_enable = 1;
            LED_flow_color = command_color[2];
            sendMessage(&client_to_esp32, "3");
        }
    }
    // String recv_data = receiveMessage(client_to_py, -1);

    // trigger = 1;
    // command = recv_data[0] - 48;
    // if (DEBUG)
    //     Serial.println(command);

    // // if confirmed gesture, set trigger = 1 and color to corresponding color
    // if (trigger == 1)
    // {
    //     LED_flow_enable = 1;
    //     LED_flow_color = command_color[command];
    // }
}

void read_IMU()
{
    if (IMU.accelerationAvailable())
        IMU.readAcceleration(imu_x, imu_y, imu_z);
    if (IMU.gyroscopeAvailable())
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    // sprintf(IMU_data, "%.3f %.3f %.3f %.3f %.3f %.3f\n", imu_x, imu_y, imu_z, gyro_x, gyro_y, gyro_z);
    add_to_waitingList(gyro_x, gyro_y, gyro_z, imu_x, imu_y, imu_z, IMU_index);
    // add_to_waitingList(imu_x, imu_y, imu_z, gyro_x, gyro_y, gyro_z, IMU_index);
    ++ IMU_index;
}

void configure_timer()
{
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC2_TC3_Val;    // select peripheral channel
    GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_GEN_GCLK0;        // select source
    GCLK->CLKCTRL.bit.CLKEN = 1;            // enable TC3 generic clock

    // Synchronous bus clock
    PM->APBCSEL.bit.APBCDIV = 0;            // no prescaler
    PM->APBCMASK.bit.TC3_ = 1;              // enable TC3

    // Count Mode
    TC3->COUNT16.CTRLA.bit.MODE = 0x0;

    // Prescale 1024
    TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;;

    // TC3 Compare Mode
    TC3->COUNT16.CTRLA.bit.WAVEGEN = 0x1;

    // compare value for 10ms
    TC3->COUNT16.CC[0].reg = 4680;

    // TC3 enable compare match interrupt
    TC3->COUNT16.INTENSET.bit.MC0 = 0x1;

    // Enable TC3 interrupt
    NVIC_EnableIRQ(TC3_IRQn);

    // TC3 enbale
    TC3->COUNT16.CTRLA.bit.ENABLE = 1;

    // Wait until TC3 is enabled
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY == 1);

    // Set TC3 interrupt priority
    NVIC_SetPriority(TC3_IRQn, 3);

    // Enable global interrupt
    __enable_irq();
}

extern "C" void TC3_Handler()
{
    // Check if the match interrupt flag is set
    if (TC3->COUNT16.INTFLAG.bit.MC0)
    {
        // Clear the interrupt flag
        TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
        read_IMU();
        // sendMessage(IMU_data);
        show_wave();
    }
}

void show_wave()
{
    if (!LED_flow_enable)
        return;
    if (LED_flow_counter >= LED_NUM)
    {
        leds[LED_NUM - 1] = CRGB::Black;
        FastLED.show();
        LED_flow_enable = 0;
        LED_flow_counter = 0;
    }
    else
    {
        leds[max(LED_flow_counter - 1, 0)] = CRGB::Black;
        leds[LED_flow_counter] = LED_flow_color;
        FastLED.show();
        ++ LED_flow_counter;
    }
}

void rainbowCycle()
{
    for (int hue = 0; hue < 255; hue++)
    {
        for (int i = 0; i < LED_NUM; i++)
        {
            leds[i] = CHSV((hue + (i * 10)) % 255, 255, 255); 
        }
        FastLED.show();
        delay(20);
    }
}

bool connectToServer(WiFiClient* client, const char * server, const uint16_t port)
{
    if (DEBUG)
    {
        Serial.print("Connecting to server ");
        Serial.print(server);
        Serial.print(":");
        Serial.println(port);
    }

    if ((*client).connect(server, port))
    {
        if (DEBUG)
            Serial.println("Connected to server!");
        return true;
    }
    else
    {
        if (DEBUG)
            Serial.println("Connection to server failed.");
        return false;
    }
}

void sendMessage(WiFiClient* client, const char* message)
{
    if ((*client).connected())
    {
        if (DEBUG)
        {
            Serial.print("Sending: ");
            Serial.println(message);
        }
        (*client).println(message);
    }
    else
    {
        if (DEBUG)
            Serial.println("Client not connected. Cannot send message.");
    }
}

String receiveMessage(WiFiClient* client, int in_timeout_ms)
{
    if (DEBUG)
        Serial.println("Waiting for server response...");
    if (in_timeout_ms == -1)
    {
        while (1)
        {
            if ((*client).available())
            {
                String response = (*client).readString();
                if (DEBUG)
                {
                    Serial.print("Received message: ");
                    Serial.println(response);
                }
                return response;
            }
            else
            {
                return "";
            }
        }
    }
    unsigned long timeout = millis() + in_timeout_ms; // Wait for Timeout
    while (millis() < timeout)
    {
        if ((*client).available())
        {
            String response = (*client).readString();
            if (DEBUG)
            {
                Serial.print("Received message: ");
                Serial.println(response);
            }
            return response;
        }
        else
        {
            return "";
        }
    }
    return "";
}
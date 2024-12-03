#include <WiFi.h>
#include <SPI.h>
#include <Arduino.h>

#define VSPI_MISO 37
#define VSPI_MOSI 35
#define VSPI_SCLK 36
#define VSPI_SS   33
#define VSPI FSPI
#define DEBUG 0

static const int spiClk = 10000;
SPIClass *vspi = NULL;


const char* ssid = "R9000P";
const char* password = "12348765";

// Server settings
const uint16_t port = 1234;  // Port to listen on
WiFiServer server(port);
void connectToWiFi();
void spiCommand(SPIClass *spi, byte data);

void setup()
{
    if (DEBUG)
    {
        Serial.begin(9600);
        while (!Serial);
        Serial.println("Started");
    }
    // Connect to WiFi
    connectToWiFi();
    // Start the server
    server.begin();
    vspi = new SPIClass(VSPI);
    vspi->begin();
    pinMode(vspi->pinSS(), OUTPUT);
    if (DEBUG)
        Serial.println("finished");
}

void loop() {
    // Check for incoming client connections
    WiFiClient client = server.available();
    if (client)
    {
        // Wait for data from the client
        while (client.connected())
        {
            if (client.available())
            {
                uint8_t message = client.readString()[0] - 48;
                if (DEBUG) Serial.println(message);
                spiCommand(vspi, message);
            }
        }
        // Close the connection
        client.stop();
    }
}

// Function to connect to WiFi
void connectToWiFi()
{
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
    }
}

void spiCommand(SPIClass *spi, byte data)
{
    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(spi->pinSS(), LOW);
    spi->transfer(data);
    digitalWrite(spi->pinSS(), HIGH);
    spi->endTransaction();
}
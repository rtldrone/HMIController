#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

extern "C" {
#include <buffer.h>
}

//Communication constants
#define kHMI_REQ_SIZE 4 //how many bytes the HMI controller will send us (4 as of 12/18/2019)
#define kHMI_REP_SIZE 22 //how many bytes we send the HMI controller (22 as of 12/18/2019)

//Address of the ESP-32 on the created Wi-Fi network
#define SOFT_AP_IP 192, 168, 4, 1

//Subnet mask on the created Wi-Fi network
#define SOFT_AP_SUBNET_MASK 255, 255, 255, 0

//SSID of the created Wi-Fi network
#define SOFT_AP_SSID "rtldrone"

#define HMI_TIMEOUT_MS 2000 //How long to wait before stopping motion if the tablet disconnects


#define HMI_INCOMING_CMD_UPDATE 'U'
#define HMI_INCOMING_CMD_STOP 'X'
#define HMI_INCOMING_CMD_SET_SPEED 'V'

#define HMI_OUTGOING_RESPONSE_SIZE \
  sizeof(float) /* Battery voltage */ \
+ sizeof(uint8_t) /* Battery state*/ \
+ sizeof(float) /* Current draw */ \
+ sizeof(float) /* Speed */ \
+ sizeof(float) /* Target speed */ \
+ sizeof(uint32_t) /* Fault code */

//Buffers for reading and writing data to the HMI over I2C
uint8_t toVehicleController[kHMI_REQ_SIZE];
uint8_t fromVehicleController[kHMI_REP_SIZE];
uint8_t sendBuffer[HMI_OUTGOING_RESPONSE_SIZE];

//Runtime values
unsigned long lastValidRecvTime = 0UL;
volatile float activeSpeedTarget = 0.0f;

volatile float currentInputVoltage = 0.0f;
volatile uint8_t currentBatteryState = 0;
volatile float currentCurrentDrawAmps = 0.0f;
volatile float currentSpeedMph = 0.0f;
volatile float currentSpeedTargetMph = 0.0f;
volatile uint8_t currentEstopActive = 0;
volatile uint32_t currentFaultCode = 0;


struct HMIIncomingPacket {
    enum IncomingCommand {
        UNKNOWN,
        UPDATE,
        STOP,
        SET_SPEED
    };

    bool valid = false;
    IncomingCommand command = UNKNOWN;
    float speedSetpoint = 0.0f;
};

/**
 * Function called when the primary controller requests data
 */
void sendSerialResponse() {
    int i = 0;
    float currentSpeedCommand = activeSpeedTarget;

    buffer_append_float32_auto(toVehicleController, currentSpeedCommand, &i);

    Serial2.write(toVehicleController, kHMI_REQ_SIZE);
}

/**
 * Function called after we have received data from the primary controller to process the data
 */
void process() {
    if (currentEstopActive) {
        activeSpeedTarget = 0.0; //Stop sending a speed if the E-stop is active
    }

    unsigned long time = millis();
    unsigned long recvTime = lastValidRecvTime;

    //Need to check time > lastValidRecvTime since lastValidRecvTime and millis() are both volatile,
    //So lastValidRecvTime could actually be greater than the current time
    if (time > recvTime && time - recvTime > HMI_TIMEOUT_MS) {
        activeSpeedTarget = 0.0; //Stop sending a speed if the HMI times out
    }
}

/**
 * Function called when the primary controller sends us data
 */
void readSerialData() {
    Serial2.readBytes(fromVehicleController, kHMI_REP_SIZE);

    int i = 0;

    currentInputVoltage = buffer_get_float32_auto(fromVehicleController, &i);
    currentBatteryState = buffer_get_uint8(fromVehicleController, &i);
    currentCurrentDrawAmps = buffer_get_float32_auto(fromVehicleController, &i);
    currentSpeedMph = buffer_get_float32_auto(fromVehicleController, &i);
    currentSpeedTargetMph = buffer_get_float32_auto(fromVehicleController, &i);
    currentEstopActive = buffer_get_uint8(fromVehicleController, &i);
    currentFaultCode = buffer_get_uint32(fromVehicleController, &i);
}

AsyncWebServer webserver(80);
AsyncWebSocket websocket("/ws");

HMIIncomingPacket parseIncomingPacket(const uint8_t *data, size_t len) {
    HMIIncomingPacket packet{};
    if (len >= 1) {
        uint8_t cmd = data[0];
        if (cmd == HMI_INCOMING_CMD_UPDATE) {
            packet.command = HMIIncomingPacket::UPDATE;
            packet.valid = true;
        } else if (cmd == HMI_INCOMING_CMD_STOP) {
            packet.command = HMIIncomingPacket::STOP;
            packet.valid = true;
        } else if (cmd == HMI_INCOMING_CMD_SET_SPEED) {
            packet.command = HMIIncomingPacket::SET_SPEED;
            if (len == 5) { //1 byte command identifier + 4 bytes of floating point data
                packet.valid = true;
                memcpy(&packet.speedSetpoint, data + 1, sizeof(packet.speedSetpoint));
            } else {
                packet.valid = false;
            }
        } else {
            packet.valid = false;
            packet.command = HMIIncomingPacket::UNKNOWN;
        }
    } else {
        //A packet with zero length is always invalid
        packet.valid = false;
    }
    return packet;
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
//Note - All incoming data is LITTLE ENDIAN, all outgoing data is BIG ENDIAN
    //This is because the ESP-32 is configured for little endian, so memcpy on incoming data is little endian
    //But our buffer library is big endian
    unsigned long time = millis();

    if (type == WS_EVT_DATA) {
        HMIIncomingPacket packet = parseIncomingPacket(data, len);
        if (packet.valid) {
            lastValidRecvTime = time;
            if (packet.command == HMIIncomingPacket::UPDATE) {
                int32_t i = 0;

                buffer_append_float32_auto(sendBuffer, currentInputVoltage, &i); //Battery voltage (4 bytes)
                buffer_append_uint8(sendBuffer, currentBatteryState, &i); //Battery State (1 byte)
                buffer_append_float32_auto(sendBuffer, currentCurrentDrawAmps, &i); //Current draw (4 bytes)
                buffer_append_float32_auto(sendBuffer, currentSpeedMph, &i); //Speed (4 bytes)
                buffer_append_float32_auto(sendBuffer, currentSpeedTargetMph, &i); //Target speed (4 bytes)
                buffer_append_uint32(sendBuffer, currentFaultCode, &i); //Fault code (4 bytes)

                client->binary(sendBuffer, HMI_OUTGOING_RESPONSE_SIZE);
            } else if (packet.command == HMIIncomingPacket::STOP) {
                activeSpeedTarget = 0.0f;
            } else if (packet.command == HMIIncomingPacket::SET_SPEED) {
                activeSpeedTarget = packet.speedSetpoint;
            }
        }
    }
}

void setup() {
    SPIFFS.begin();
    WiFi.mode(WIFI_AP);
    auto ip = IPAddress(SOFT_AP_IP);
    auto subnet = IPAddress(SOFT_AP_SUBNET_MASK);
    WiFi.softAPConfig(ip, ip, subnet);
    WiFi.softAP(SOFT_AP_SSID);

    webserver.serveStatic("/", SPIFFS, "/www/").setDefaultFile("index.html");
    websocket.onEvent(onWsEvent);
    webserver.addHandler(&websocket);
    webserver.begin();

    Serial2.begin(921600, SERIAL_8N1, 36, 25);
}

void loop() {
    if (Serial2.available()) {
        readSerialData();
        process();
        sendSerialResponse();
    }
}
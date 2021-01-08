#include <sewparser.h>
#include "sewdistance.hpp"
#include "sewmotors.hpp"

#define SEW_PARSER_MAX_BUFFER_SIZE 120

#define MAX_DISTANCE 200
#define trigPinBack 4
#define echoPinBack 5
#define trigPinFront 7
#define echoPinFront 6

#define enableM1 8
#define forwardM1 9
#define reverseM1 10

#define enabledM2 13
#define forwardM2 12
#define reverseM2 11

uint8_t macMotorLeft[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
uint8_t macMotorRight[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8_t macDistanceFront[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
uint8_t macDistanceBack[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
uint8_t waitingMAC[] = {0x53, 0x45, 0x57};

SewDistance distanceFront(trigPinFront, echoPinFront, MAX_DISTANCE);
SewDistance distanceBack(trigPinBack, echoPinBack, MAX_DISTANCE);
void distanceFrontCallback(int distance) {
  FRAME frame;
  SewParser::encodeDistance(frame, macDistanceFront, distance);
  Serial1.write(frame.frame, frame.size);
}
void distanceBackCallback(int distance) {
  FRAME frame;
  SewParser::encodeDistance(frame, macDistanceBack, distance);
  Serial1.write(frame.frame, frame.size);
}

SewMotors leftMotor(enableM1, forwardM1, reverseM1);
SewMotors rightMotor(enabledM2, forwardM2, reverseM2);

SewParser sewParser;
uint8_t sewParserBuffer[SEW_PARSER_MAX_BUFFER_SIZE];
int handleFrames(FRAME frame, int status) {
    if(frame.type == DCMOTOR) {
        uint8_t data[3];
        decodeDCMotorPayload(frame, data);
        if(frame.mac[7] == 0x01) {
            leftMotor.runMotor(data[1], data[2]);
        }
        else {
            rightMotor.runMotor(data[1], data[2]);
        }
    }

    return 0;
}

void syncESP32() {
    uint8_t mac[6];
    bool macReceived = false;
    while(!macReceived) {
        if(Serial1.available()) {
            size_t macSize = Serial1.readBytes(mac, 6);
            if(macSize == 6) {
                macReceived = true;
                for(int i = 0; i < 6; i++) {
                    macMotorLeft[i] = mac[i];
                    macMotorRight[i] = mac[i];
                    macDistanceFront[i] = mac[i];
                    macDistanceBack[i] = mac[i];
                }
            }
        }
        else {
            Serial1.write(waitingMAC, 3);
            delay(500);
        }
    }
}

void setup() {
    Serial1.begin(115200);

    distanceFront.init();
    distanceFront.registerCallback(1000, distanceFrontCallback);

    distanceBack.init();
    distanceBack.registerCallback(1000, distanceBackCallback);

    leftMotor.init();
    rightMotor.init();

    // Sync with ESP32
    syncESP32();
}

void loop() {
    int byteReaded = 0;
    while(Serial1.available() > 0 && byteReaded < SEW_PARSER_MAX_BUFFER_SIZE) {
        sewParserBuffer[byteReaded] = Serial1.read();
        byteReaded++;
    }
    if(byteReaded > 0) {
        sewParser.decodeFrameWithCallback(sewParserBuffer, byteReaded, handleFrames);
    }
    // Manage distanceFront
    distanceFront.handleDistances();
    distanceBack.handleDistances();
    }

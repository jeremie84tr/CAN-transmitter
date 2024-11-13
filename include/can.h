//
// Created by secondaire on 12/11/2024.
//

#ifndef CANTRANSMITTER_CAN_H
#define CANTRANSMITTER_CAN_H

#include "Arduino.h"
#include "thread"

enum eCANStatus {
    UNKNOWN,
    READING,
    TRANSMITTED,
    ACKNOWLEDGED,
    IGNORED,
    ERROR
};

class CANFrame {
public:
    static const int CF_SOF = 1;          // le bit dominant (0)
    static const int CF_ARBITRAGE = 12;   // 11 identifiant + 1 dominant (Remote Transmission Request)
    static const int CF_COMANDE = 6;      // 1 poids fort dominant (0) (donnée, sinon standard) + 4 Data Lenght Code (0 à 8)
    static const int CF_DONNEE = 8;       // les données, de 0*8 a 8*8 bits (64)
    static const int CF_CRC = 16;         // 15 bits calculés + delimiteur récessif (1)
    static const int CF_ACK = 2;          // 1 bit recessif à rendre dominant par un tiers + 1 bit toujours récessif
    static const int CF_EOF = 7;          // 7 bits recessifs

    int arbitrationID;
    int control;
    bool* data;
    int crc;
    int acknowledgement;
    CANFrame(int identifier);
    int computeCrc();
};

class CAN {
    int rxGPIO;
    int txGPIO;
    std::thread* listeningThread;
    static void listeningThreadFunction(CAN* can, void (*callback)(CANFrame*));
public:
    eCANStatus status;
    CAN(int rxGPIO, int txGPIO);
    void listen(void (*callback)(CANFrame*));
    void stop();
};

// Enumération pour représenter les différents champs de la trame
enum eCANField {
    NONE,
    START_OF_FRAME,
    ARBITRATION_ID,
    CONTROL,
    DATA,
    CRC,
    ACKNOWLEDGEMENT,
    END_OF_FRAME
};

class DataCatcher {
    CAN* can;
    CANFrame* currentFrame;
    int bitCount;
    eCANField currentField;
    int currentBitPosition;
    void (*save)(CANFrame*);
public: 
    DataCatcher(CAN* can, void (*save)(CANFrame*));
    void onNext(bool item);
};


#endif //CANTRANSMITTER_CAN_H
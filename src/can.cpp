#include "can.h"

// ------------------------------ CANFrame ------------------------------

CANFrame::CANFrame() {
    
}

CANFrame::CANFrame(int identifier) {
    this->arbitrationID = identifier;
}

int CANFrame::computeCrc() {
    return 0;
}


// ------------------------------ ListeningThreadParameters ------------------------------

ListeningThreadParameters::ListeningThreadParameters(CAN* can, void (*callback)(CANFrame*)) {
    this->can = can;
    this->callback = callback;
}


// ------------------------------ CAN ------------------------------

CAN::CAN(int rxGPIO, int txGPIO, int transmissionSpeed) {
    this->rxGPIO = rxGPIO;
    pinMode(rxGPIO, INPUT);
    this->txGPIO = txGPIO;
    pinMode(txGPIO, OUTPUT);
    this->transmissionSpeed = transmissionSpeed;
    this->status = UNKNOWN;
    this->listeningTask = NULL;
}

void CAN::listeningThreadFunction(void* param) {
    ListeningThreadParameters* params = (ListeningThreadParameters*) param;
    DataCatcher* dataCatcher = new DataCatcher(params->can, params->callback);
    while(1) {
        int read = digitalRead(params->can->rxGPIO);
        if (read == HIGH) {
            Serial.print("0");
            dataCatcher->onNext(0);
        }
        if (read == LOW) {
            Serial.print("1");
            dataCatcher->onNext(1);
        }
        usleep(1000000 / params->can->transmissionSpeed);
    }
}

void CAN::listen(void (*callback)(CANFrame*)) {
    xTaskCreate(listeningThreadFunction, "linstening", 1024, new ListeningThreadParameters(this, callback), tskIDLE_PRIORITY, listeningTask);
}

void CAN::stop() {
    vTaskDelete(listeningTask);
}


// ------------------------------ DataCatcher ------------------------------

DataCatcher::DataCatcher(CAN* can, void (*save)(CANFrame*)) {
    this->can = can;
    this->bitCount = 0;
    this->currentField = NONE;
    this->save = save;
}

void DataCatcher::onNext(bool item) {
    // Une trame de données CAN commence par un bit dominant (false)
    if (!item && bitCount == 0) {
        // Réinitialisation des variables pour une nouvelle trame
        currentFrame = new CANFrame(0);
        currentField = START_OF_FRAME;
        can->status = READING;
    }
    // Incrément du nombre de bits dans la trame
    bitCount++;
    switch (currentField) {
        case NONE:
            bitCount = 0;
            break;
        case START_OF_FRAME:
            if (bitCount == 1) {
                // Début de trame détecté
                currentField = ARBITRATION_ID;
                // Initialisation de l'ID d'arbitrage
                currentFrame->arbitrationID = 0;
                currentBitPosition = 0;
            }
            break;
        case ARBITRATION_ID:
            // Lecture des 11 bits de l'ID d'arbitrage
            if (currentBitPosition < 11) {
                currentFrame->arbitrationID |= (item ? 1 : 0) << (10 - currentBitPosition);
            }
            currentBitPosition++;
            if (currentBitPosition == 12) {
                // Fin de lecture de l'ID d'arbitrage
                currentField = CONTROL;
                currentBitPosition = 0;
            }
            break;
        case CONTROL:
            // Lecture des 6 bits du champ de commande
            if (currentBitPosition > 1) {
                currentFrame->control |= (item ? 1 : 0) << (5 - currentBitPosition);
            }
            currentBitPosition++;
            if (currentBitPosition == 6) {
                // Fin de lecture du champ de commande
                // Détermination du type de trame (standard ou étendue)
                boolean isStandardFrame = (currentFrame->control & 0b100000) == 0; // Le bit de poids fort est dominant pour une trame standard
                currentFrame->data = new bool[(currentFrame->control & 0b1111) * 8] ; // Les 4 bits de poids faibles représentent la longueur des données
                // Passage au champ de données si c'est une trame standard, sinon, au CRC
                currentField = isStandardFrame ? DATA : CRC;
                currentBitPosition = 0;
            }
            break;
        case DATA:
            if (currentBitPosition < (currentFrame->control & 0b1111) * 8) {       // (control & 0b1111) * 8 = data.length
                currentFrame->data[currentBitPosition] = item;
            }
            currentBitPosition++;
            if (currentBitPosition == (currentFrame->control & 0b1111) * 8 ) {     // (control & 0b1111) * 8 = data.length
                currentField = CRC;
                currentBitPosition = 0;
            }
            break;
        case CRC:
            // Lecture des 15 bits du CRC
            if (currentBitPosition < 15) {
                currentFrame->crc |= (item ? 1 : 0) << (14 - currentBitPosition);
            }
            // je vérifie que le 16e bit est récessif
            if (currentBitPosition == 15 && !item) {
                can->status = ERROR;
                //System.out.println("COLLISION car récessif est dominant dans le CRC");
                currentField = NONE;
                bitCount = 0;
                break;
            }
            currentBitPosition++;
            if (currentBitPosition == 16) {
                currentField = ACKNOWLEDGEMENT;
                currentBitPosition = 0;
            }
            break;
        case ACKNOWLEDGEMENT:
            // Lecture des 2 bits d'ack
            if (currentBitPosition < 2) {
                currentFrame->acknowledgement |= (item ? 1 : 0) << (1 - currentBitPosition);
            }
            currentBitPosition++;
            if (currentBitPosition == 2) {
                currentField = END_OF_FRAME;
                currentBitPosition = 0;
            }
            break;
        default:
            can->status = UNKNOWN;
            save(currentFrame);
            currentField = NONE;
            bitCount = 0;
            break;
        // Ajoutez des cas pour les autres champs de la trame (commande, données, CRC, acquittement, fin de trame)
    }
}

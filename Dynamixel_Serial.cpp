/*

Version 2.2

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */

#include "Dynamixel_Serial.h"


//##############################################################################
//############################ Public Methods ##################################
//##############################################################################

void DynamixelClass::begin(long baud){

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
    Serial1.begin(baud);  // Set up Serial for Leonardo and Mega
    _serial = &Serial1;
#else
    Serial.begin(baud);   // Set up Serial for all others (Uno, etc)
    _serial = &Serial;
#endif

}

void DynamixelClass::begin(HardwareSerial &HWserial, long baud){

    HWserial.begin(baud); // Set up Serial for a specified Serial object
    _serial = &HWserial;

}

void DynamixelClass::begin(Stream &serial){

    _serial = &serial;  // Set a reference to a specified Stream object (Hard or Soft Serial)

}

void DynamixelClass::end(){

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
    Serial1.end();
#else
    Serial.end();
#endif

}


void DynamixelClass::setDirectionPin(byte D_Pin){

    Direction_Pin = D_Pin;
    pinMode(Direction_Pin,OUTPUT);

}

unsigned int DynamixelClass::reset(byte ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = RESET_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_RESET;

    clearRXbuffer();

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }

}

unsigned int DynamixelClass::ping(byte ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = PING_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_PING;

    clearRXbuffer();

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
        return (Status_Packet_Array[0]);            // Return SERVO ID
    }else{
        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
    }

}

unsigned int DynamixelClass::setRegister1(byte ID, byte address, byte value) {
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_REGISTER1_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = address;
    Instruction_Packet_Array[4] = value;

    clearRXbuffer();

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return 0;
    } else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {              // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        } else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}

unsigned int DynamixelClass::setStatusPacketReturnDelay(byte ID, byte ReturnDelay){
    return setRegister1(ID, EEPROM_RETURN_DELAY_TIME, byte(ReturnDelay/2));
}

unsigned int DynamixelClass::setID(byte ID, byte New_ID){
    return setRegister1(ID, EEPROM_ID, New_ID);
}

unsigned int DynamixelClass::setBaudRate(byte ID, long Baud) {
    byte baudChar = 0;
    switch (Baud) {
        case 1000000:
            baudChar = 0x01;
            break;
        case 2250000:
            baudChar = 0xFA;
            break;
        case 2500000:
            baudChar = 0xFB;
            break;
        case 3000000:
            baudChar = 0xFC;
            break;
        default:
            baudChar = byte((2000000 / Baud) - 1);
    }
    
    return setRegister1(ID, EEPROM_BAUD_RATE, baudChar);
}

unsigned int DynamixelClass::setRegister2(byte ID, byte address, unsigned int value) {
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_REGISTER1_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = address;
    Instruction_Packet_Array[4] = byte(value);
    Instruction_Packet_Array[5] = byte(value >> 8);

    clearRXbuffer();

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return 0;
    } else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {              // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        } else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}

unsigned int DynamixelClass::setMaxTorque( byte ID, int Torque){
    return setRegister2(ID, EEPROM_MAX_TORQUE_L, Torque);
}

unsigned int DynamixelClass::setTorqueLimit( byte ID, int Torque){
    return setRegister2(ID, RAM_TORQUE_LIMIT_L, Torque);
}

unsigned int DynamixelClass::setHoldingTorque(byte ID, bool Set){
    return setRegister1(ID, RAM_TORQUE_ENABLE, Set);
}

unsigned int DynamixelClass::setAlarmShutdown(byte ID, byte Set){
    return setRegister1(ID, EEPROM_ALARM_SHUTDOWN, Set);
}

unsigned int DynamixelClass::setStatusPacket(byte  ID,byte Set){
    return setRegister1(ID, EEPROM_RETURN_LEVEL, Set);
}

unsigned int DynamixelClass::setMode(byte ID, bool Dynamixel_Mode, unsigned int Dynamixel_CW_Limit,unsigned int Dynamixel_CCW_Limit){
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_MODE_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_CW_ANGLE_LIMIT_L;
    if (Dynamixel_Mode == WHEEL) {                                    // Set WHEEL mode, this is done by setting both the clockwise and anti-clockwise angle limits to ZERO
        Instruction_Packet_Array[4] = 0x00;
        Instruction_Packet_Array[5] = 0x00;
        Instruction_Packet_Array[6] = 0x00;
        Instruction_Packet_Array[7] = 0x00;
    }else {                                                             // Else set SERVO mode
        Instruction_Packet_Array[4] = byte(Dynamixel_CW_Limit);
        Instruction_Packet_Array[5] = byte((Dynamixel_CW_Limit & 0x0F00) >> 8);
        Instruction_Packet_Array[6] = byte(Dynamixel_CCW_Limit);
        Instruction_Packet_Array[7] = byte((Dynamixel_CCW_Limit & 0x0F00) >> 8);
    }

    clearRXbuffer();

    transmitInstructionPacket();

    if (Status_Return_Value == ALL){
        readStatusPacket();
        if (Status_Packet_Array[2] != 0){
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
 }

 unsigned int DynamixelClass::setPunch(byte ID,unsigned int Punch){
    return setRegister2(ID, RAM_PUNCH_L, Punch);
 }

 unsigned int DynamixelClass::setPID(byte ID, byte P, byte I, byte D){
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_PID_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_DERIVATIVE_GAIN;
    Instruction_Packet_Array[4] = D;
    Instruction_Packet_Array[5] = I;
    Instruction_Packet_Array[6] = P;

    clearRXbuffer();

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
 }

unsigned int DynamixelClass::setTemp(byte ID, byte temp){
    return setRegister1(ID, EEPROM_LIMIT_TEMPERATURE, temp);
}

unsigned int DynamixelClass::setVoltage(byte ID, byte Volt_L, byte Volt_H){
    return setRegister2(ID, EEPROM_LOW_LIMIT_VOLTAGE, Volt_H << 8 | Volt_L);
}

unsigned int DynamixelClass::servo(byte ID,unsigned int Position,unsigned int Speed){
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet_Array[4] = byte(Position);
    Instruction_Packet_Array[5] = byte((Position & 0x0F00) >> 8);
    Instruction_Packet_Array[6] = byte(Speed);
    Instruction_Packet_Array[7] = byte((Speed & 0x0F00) >> 8);

    clearRXbuffer();

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}

unsigned int DynamixelClass::wheel(byte ID, bool Rotation,unsigned int Speed){
    byte Speed_H,Speed_L;
    Speed_L = Speed;
    if (Rotation == 0){                         // Move Left
        Speed_H = Speed >> 8;

    }else if (Rotation == 1){                    // Move Right
        Speed_H = (Speed >> 8)+4;
    }
    
    return setRegister2(ID, RAM_GOAL_SPEED_L, Speed_H << 8 | Speed_L);
}

unsigned int DynamixelClass::action(byte ID){
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = RESET_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_ACTION;

    clearRXbuffer();

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}

unsigned int DynamixelClass::ledState(byte ID, bool Status){
    return setRegister1(ID, RAM_LED, Status);
}

unsigned int DynamixelClass::readRegister(byte ID, byte address, byte length) {
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_REGISTER_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = address;
    Instruction_Packet_Array[4] = length;

    clearRXbuffer();

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0 {               // If there is no status packet error return value
        if (length == 1) {
            return Status_Packet_Array[3];
        } else {
            return Status_Packet_Array[4] << 8 | Status_Packet_Array[3];
        }
    } else {
        return (Status_Packet_Array[2] | 0xF000); // error
    }
}

unsigned int DynamixelClass::readTemperature(byte ID) {
    return readRegister(ID, RAM_PRESENT_TEMPERATURE, 1);
}

unsigned int DynamixelClass::readPosition(byte ID) {
    return readRegister(ID, RAM_PRESENT_POSITION_L, 2);
}

unsigned int DynamixelClass::readLoad(byte ID) {
    return readRegister(ID, RAM_PRESENT_LOAD_L, 2);
}

unsigned int DynamixelClass::readSpeed(byte ID) {
    return readRegister(ID, RAM_PRESENT_SPEED_L, 2);
}

unsigned int DynamixelClass::readVoltage(byte ID) {
    return readRegister(ID, RAM_PRESENT_VOLTAGE, 1);
}

unsigned int DynamixelClass::readGoalPosition(byte ID) {
    return readRegister(ID, RAM_GOAL_POSITION_L, 2);
}

unsigned int DynamixelClass::readGoalSpeed(byte ID) {
    return readRegister(ID, RAM_GOAL_SPEED_L, 2);
}

unsigned int DynamixelClass::readMaxTorque(byte ID) {
    return readRegister(ID, EEPROM_MAX_TORQUE_L, 2);
}

unsigned int DynamixelClass::readTorqueLimit(byte ID) {
    return readRegister(ID, RAM_TORQUE_LIMIT_L, 2);
}

unsigned int DynamixelClass::readHoldingTorque(byte ID) {
    return readRegister(ID, RAM_TORQUE_ENABLE, 1);
}

unsigned int DynamixelClass::checkRegistered(byte ID) {
    return readRegister(ID, RAM_REGISTERED, 1);
}

unsigned int DynamixelClass::checkMovement(byte ID){
    return readRegister(ID, RAM_MOVING, 1);
}

unsigned int DynamixelClass::checkLock(byte ID){
    return readRegister(ID, RAM_LOCK, 1);
}


//##############################################################################
//########################## Private Methods ###################################
//##############################################################################

void DynamixelClass::transmitInstructionPacket(void){                                   // Transmit instruction packet to Dynamixel
    if (Direction_Pin > -1){
        digitalWrite(Direction_Pin,HIGH);                                               // Set TX Buffer pin to HIGH
    }

    _serial->write(HEADER);                                                             // 1 Write Header (0xFF) data 1 to serial
    _serial->write(HEADER);                                                             // 2 Write Header (0xFF) data 2 to serial
    _serial->write(Instruction_Packet_Array[0]);                                        // 3 Write Dynamixal ID to serial
    _serial->write(Instruction_Packet_Array[1]);                                        // 4 Write packet length to serial
    _serial->write(Instruction_Packet_Array[2]);                                        // 5 Write instruction type to serial

    unsigned int checksum_packet = Instruction_Packet_Array[0] + Instruction_Packet_Array[1] + Instruction_Packet_Array[2];

    for (byte i = 3; i <= Instruction_Packet_Array[1]; i++){
        _serial->write(Instruction_Packet_Array[i]);                                    // Write Instuction & Parameters (if there are any) to serial
        checksum_packet += Instruction_Packet_Array[i];
    }

    noInterrupts();

    _serial->write(~checksum_packet & 0xFF);                                            // Write low bit of checksum to serial

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__) // Leonardo and Mega use Serial1
    if ((UCSR1A & B01100000) != B01100000){                                             // Wait for TX data to be sent
        _serial->flush();
    }

#elif defined(__SAM3X8E__)

    //if(USART_GetFlagStatus(USART1, USART_FLAG_TC) != RESET)
        _serial->flush();
    //}

#else
    if ((UCSR0A & B01100000) != B01100000){                                             // Wait for TX data to be sent
        _serial->flush();
    }

#endif

    if (Direction_Pin > -1){
        digitalWrite(Direction_Pin,LOW);                                                //Set TX Buffer pin to LOW after data has been sent
    }

    interrupts();
}

unsigned int DynamixelClass::readStatusPacket(void){
    byte Counter = 0x00;
    byte First_Header = 0x00;

    Status_Packet_Array[0] = 0x00;
    Status_Packet_Array[1] = 0x00;
    Status_Packet_Array[2] = 0x00;
    Status_Packet_Array[3] = 0x00;

    Time_Counter = STATUS_PACKET_TIMEOUT + millis();                                    // Setup time out error

    while(STATUS_FRAME_BUFFER >= _serial->available()){                                     // Wait for " header + header + frame length + error " RX data

        if (millis() >= Time_Counter){
            return Status_Packet_Array[2] = B10000000;                                      // Return with Error if Serial data not received with in time limit
        }
    }

    if (_serial->peek() == 0xFF && First_Header != 0xFF){
        First_Header = _serial->read();                                                 // Clear 1st header from RX buffer
    }else if (_serial->peek() == -1){
        return Status_Packet_Array[2] = B10000000;                                      // Return with Error if two headers are not found
    }
    if(_serial->peek() == 0xFF && First_Header == 0xFF){
        _serial->read();                                                                // Clear 2nd header from RX buffer
        Status_Packet_Array[0] = _serial->read();                                   // ID sent from Dynamixel
        Status_Packet_Array[1] = _serial->read();                                       // Frame Length of status packet
        Status_Packet_Array[2] = _serial->read();                                       // Error byte

        Time_Counter = STATUS_PACKET_TIMEOUT + millis();
        while(Status_Packet_Array[1] - 2 >= _serial->available()){              // Wait for wait for "Para1 + ... Para X" received data

            if (millis() >= Time_Counter){
                return Status_Packet_Array[2] = B10000000;                          // Return with Error if Serial data not received with in time limit
            }
        }
        do{
            Status_Packet_Array[3 + Counter] = _serial->read();
            Counter++;
        }while(Status_Packet_Array[1] > Counter);                           // Read Parameter(s) into array

        Status_Packet_Array[Counter + 4] = _serial->read();                         // Read Check sum

    }else{
        return Status_Packet_Array[2] = B10000000;                                      // Return with Error if two headers are not found
    }
}

void DynamixelClass::clearRXbuffer(void){
    while (_serial->read() != -1);  // Clear RX buffer;
}

DynamixelClass Dynamixel;

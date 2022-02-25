

/*
add DRAM_ATTR and IRAM_ATTR
*/

//note: lidar stops working correctly below 4.7V (400mA)
//at (norminal) 5V, it uses 400-420mA (fluctuation is the result of the PID loop on the motor speed control)


unsigned long printTimer;
unsigned long speedTimer[10];


//#include "camsense_X1.h"

#define lidarDebugSerial Serial

//#define DACdisplay
#define ILI9341displayAdafruit
//#define ILI9341displayTFT_eSPI

#define CAMSENSE_X1_SYNC_BYTE_0 0x55        //byte at the start of each packet
#define CAMSENSE_X1_SYNC_BYTE_1 0xAA        //byte at the start of each packet
#define CAMSENSE_X1_SYNC_BYTE_2 0x03        //byte at the start of each packet
#define CAMSENSE_X1_SYNC_BYTE_3 0x08        //byte at the start of each packet
#define CAMSENSE_X1_ANGLE_SUBTRACT_16 0xA000  //angle in degrees is ((16bit value)-0xA000)/64.00 or (((buffer[H]-0xA0)<<8) + buffer[L])/64.00
#define CAMSENSE_X1_ANGLE_SUBTRACT_8  0xA0
////#define CAMSENSE_X1_ANGLE_ROLLOVER 23040  //= 360 * 64  to be used in rlvrDif() with the raw (int, not float) angle value (NOT USED)
//const uint8_t CAMSENSE_X1_SYNC_BYTE[4] = {0x55, 0xAA, 0x03, 0x08};
//const uint16_t CAMSENSE_X1_ANGLE_SUBTRACT_16 = 0xA000;
//const uint8_t CAMSENSE_X1_ANGLE_SUBTRACT_8 = 0xA0;

#define lidarArrayErrorMessages //enable error messages in data array system

DRAM_ATTR struct lidarPacket
{
  uint16_t RPM = 0;
  uint16_t startAngle = 0;
  uint16_t endAngle = 0;
  uint16_t smallTimestamp = 0;
  uint16_t measurements[8] = {0,0,0,0, 0,0,0,0};
  uint8_t reservedData[8] = {0,0,0,0, 0,0,0,0};
  bool CRCpassed = true; //default to true, because it doesnt really matter that much
} __attribute__((packed)); //34 bytes becomes 33 by packing. i did notice that some functions become a few (5-10) clock cycles faster, and some about the same slower
//do however note that pointers to individual entries are probably fricked up, i think

DRAM_ATTR const uint16_t lidarDataArrayMaxSize = 80;

template<class T> class camsense_X1
{
  private:
    T& _serialPort;
    
    volatile uint8_t _fillUpBuff = 0; //counter to keep track of how many (serial) bytes of current packet are received (1 means start byte is received, 36 means all bytes received)
    volatile uint8_t _sixDegRawBuf[32]; //a buffer to hold the serial data for the 
    volatile bool _sixDegRawBufWriting = false; //for multi-threading/core and interrupt safety, DONT access _sixDegRawBuf[volatileDegree] if _sixDegRawBufWriting is TRUE
    //volatile bool _sixDegRawBufReady = false; //flag to let another core/interrupt know that the data needs to be processed/translated
    const uint32_t _runPacketTimeout = 500; //if no new packets have been received in this time, it's probably no longer spinning
    uint32_t _packetTimeoutTimer = 0;
    
    uint16_t _lastAngles[2]; //hold the angles of the last packet, to compare to the new ones
    
    //the data is found in the array at lidarDataUnordered[lidarDataIndexes[i]] because the contents of lidarDataUnordered[] are out of order and lidarDataIndexes[] holds that order
    #define _lidarData(index)  lidarDataUnordered[lidarDataIndexes[index]]
    #define lidarData(classObject, index)  classObject.lidarDataUnordered[classObject.lidarDataIndexes[index]]
    
    //the following functions describe the way the lidarData array works. long story short: to read out the data, use lidarData(index)
    //one data array (lidarDataUnordered[]), one dynamic array that holds the order of the data (lidarDataIndexes[])
    //and finally, one semi-dynamic array to hold the free indices of the data array (which can be anywhere).
    volatile uint16_t _lidarDataIndexesFree[lidarDataArrayMaxSize]; //this is also a dynamic array (but simpler), with length (lidarDataArrayMaxSize-lidarDataDynamicSize),  (inverse length of lidarDataIndexes[])
    volatile uint16_t _lidarDataFreeDynamicSize = 0; //number of free indexes available (length of _lidarDataIndexesFree)

    void _initLidarDataStructure() {
      for(uint16_t i=0; i<lidarDataArrayMaxSize; i++) {
        lidarDataIndexes[i] = lidarDataArrayMaxSize; //set it to an illigal position, for debugging purpouses
        _releaseLidarDataIndex(lidarDataArrayMaxSize-1-i);
      }
    }
    
    IRAM_ATTR uint16_t _getFreeLidarDataIndex() {
      #ifdef lidarArrayErrorMessages
        if(_lidarDataFreeDynamicSize == 0) { log_e("_lidarDataFreeDynamicSize == 0 in _getFreeLidarDataIndex()"); return(_lidarDataIndexesFree[_lidarDataFreeDynamicSize]); } //error check
      #endif
      _lidarDataFreeDynamicSize--;
      return(_lidarDataIndexesFree[_lidarDataFreeDynamicSize]);
    }

    #ifdef lidarArrayErrorMessages
      IRAM_ATTR uint16_t _peekFreeLidarDataIndex() {
        if(_lidarDataFreeDynamicSize == 0) { log_e("_lidarDataFreeDynamicSize == 0 in _peekFreeLidarDataIndex()"); return(_lidarDataIndexesFree[_lidarDataFreeDynamicSize]); } //error check
        //_lidarDataFreeDynamicSize--; //dont actually assign the index
        return(_lidarDataIndexesFree[_lidarDataFreeDynamicSize]); //just show what index
      }
      //define is faster, but doesnt have error checking
    #else
      #define _peekFreeLidarDataIndex  _lidarDataIndexesFree[_lidarDataFreeDynamicSize-1]
    #endif

    #ifdef lidarArrayErrorMessages
      IRAM_ATTR void _releaseLidarDataIndex(uint16_t indexToRelease) {
        _lidarDataIndexesFree[_lidarDataFreeDynamicSize] = indexToRelease;
        _lidarDataFreeDynamicSize++;
        if(_lidarDataFreeDynamicSize >= lidarDataArrayMaxSize) { log_e("lidarDataDynamicSize >= lidarDataArrayMaxSize"); delay(1); } //error check
      }
      //define is faster, but doesnt have error checking
    #else
      #define _releaseLidarDataIndex(indexToRelease)  _lidarDataIndexesFree[_lidarDataFreeDynamicSize]=indexToRelease; _lidarDataFreeDynamicSize++
    #endif

    //this function (nearly efficiently) moves data in the array around (note: 'leaveNull' only affects 
    IRAM_ATTR bool _shiftLidarData(uint16_t placesToShift=1, uint16_t whereToShift=0, uint16_t newArraySize=lidarDataArrayMaxSize, bool shiftLeft=false, bool leaveNull=true) {
      #ifdef lidarArrayErrorMessages
        if(placesToShift == 0) { return(false); }
        if(placesToShift > newArraySize) { log_e("placesToShift > newArraySize, %u,%u,%u,%u,%u", placesToShift, whereToShift, newArraySize, shiftLeft, leaveNull); return(false); }
        else if(newArraySize > lidarDataArrayMaxSize) { log_e("newArraySize > lidarDataArrayMaxSize   make array larger"); return(false); }
      #endif
      if(shiftLeft) { //shift left (delete gap)
        for(uint16_t i=whereToShift; i<(whereToShift+placesToShift); i++) { //for all the items that are about to be deleted
          if(lidarDataIndexes[i] < lidarDataArrayMaxSize) { //if there is an object still there (normal)
            if(leaveNull) { //really not needed, but whatever
              memset(&lidarDataUnordered[lidarDataIndexes[i]], 0, sizeof(lidarPacket));
              //lidarDataUnordered[lidarDataIndexes[i]] = emptyLidarPacket; // slower
            }
            _releaseLidarDataIndex(lidarDataIndexes[i]); //put the indexes in the soon-to-be-deleted entries back into the free-pile (_lidarDataIndexesFree)
          }
        }
        //shifting to left to fill a gap (left by shifting to the right earlier), newArraySize must be smaller than old array size
        for(uint16_t i=whereToShift; i<newArraySize; i++) {
          lidarDataIndexes[i] = lidarDataIndexes[i+placesToShift]; //perform shift (left)
        }
        if(leaveNull) {
          //for(uint16_t i=newArraySize; i<constrain(newArraySize+placesToShift, 0, lidarDataArrayMaxSize); i++) { //constrain not really needed
          for(uint16_t i=newArraySize; i<(newArraySize+placesToShift); i++) {
            lidarDataIndexes[i] = lidarDataArrayMaxSize; //leave behind invalid indexes
          }
        }
      } else { //shift right (make gap)
        for(uint16_t i=newArraySize-1; i>(whereToShift+(placesToShift-1)); i--) {
          lidarDataIndexes[i] = lidarDataIndexes[i-placesToShift]; //perform shift (right)
        }
        for(uint16_t i=whereToShift; i<(whereToShift+placesToShift); i++) {
          lidarDataIndexes[i] = _getFreeLidarDataIndex(); //the newly made gap needs to be filled with new free indexes
          if(leaveNull) {
            memset(&lidarDataUnordered[lidarDataIndexes[i]], 0, sizeof(lidarPacket));
            //lidarDataUnordered[lidarDataIndexes[i]] = emptyLidarPacket; // slower
          }
        }
      }
      return(true);
    }
    
    #define _expandInbetweenLidarData(howManyToExpand, whereToStartExpansion, nullifyNewSpace)  _shiftLidarData(howManyToExpand, whereToStartExpansion, lidarDataDynamicSize+=howManyToExpand, false, nullifyNewSpace)
    
    IRAM_ATTR bool _insertOneLidarDataReturn(lidarPacket dataToInsert, uint16_t whereToInsert=0) {
      if(_expandInbetweenLidarData(1, whereToInsert, false)) { //make room (but dont bother nullifying the data in the new gap, cuz that takes time and it's about to be filled anyways)
        _lidarData(whereToInsert) = dataToInsert;
      } else {
        return(false);
      }
    }
    // define is faster(?), but doesnt return a bool. for that, use _insertOneLidarDataReturn()
    #define _insertOneLidarData(dataToInsert, whereToInsert)  _expandInbetweenLidarData(1, whereToInsert, false);  _lidarData(whereToInsert) = dataToInsert
    
    IRAM_ATTR bool _insertLidarData(lidarPacket dataToInsert[], uint16_t insertArraySize=1, uint16_t whereToInsert=0) {
      if(_expandInbetweenLidarData(insertArraySize, whereToInsert, false)) { //make room (but dont bother nullifying the data in the new gap, cuz that takes time and it's about to be filled anyways)
        for(uint16_t i=0; i<insertArraySize; i++) {
          _lidarData(whereToInsert+i) = dataToInsert[i];
        }
      } else {
        return(false);
      }
    }
    
    #define _deleteInbetweenLidarData(howManyToDelete, whereToStartDelete, nullifyOldData)  _shiftLidarData(howManyToDelete, whereToStartDelete, lidarDataDynamicSize-=howManyToDelete, true, nullifyOldData)
    
    IRAM_ATTR void _deleteTailLidarData(uint16_t newLidarDataDynamicSize) {
      for(uint16_t i=newLidarDataDynamicSize; i<lidarDataDynamicSize; i++) {
        _releaseLidarDataIndex(lidarDataIndexes[i]);
      }
      lidarDataDynamicSize = newLidarDataDynamicSize;
    }

    IRAM_ATTR int16_t _parseFullSixDegBuf(uint8_t bufferProgress) {
      if(bufferProgress == 36) { //full package
        lidarPacket newPacket;
        newPacket.smallTimestamp = ((uint16_t) millis());
//      if(bufferProgress >= 6) { //angle-index bytes received
        newPacket.RPM = (((uint16_t) _sixDegRawBuf[1]) << 8) + _sixDegRawBuf[0]; //combine bytes to make number
//        if(bufferProgress >= 8) { //RPM bytes received
          newPacket.startAngle = (((uint16_t) _sixDegRawBuf[3]) << 8) + _sixDegRawBuf[2]; //combine bytes to make number
          if(newPacket.startAngle == 0) { //when booting up, the lidar will send packets with 0000, then A000 (CAMSENSE_X1_ANGLE_SUBTRACT_16), then real data.
            return(0);
          } //else
          newPacket.startAngle -= CAMSENSE_X1_ANGLE_SUBTRACT_16;
//        }
//        if(bufferProgress >= 11) { //at least one measurement point
          //for(uint8_t i=4; i<constrain(bufferProgress-6, 0, 26); i+=3) { //measurement data, 8 points per packet
          //for(uint8_t i=0; i<(((bufferProgress<32) ? (bufferProgress-8) : 24)/3); i++) { //measurement data, 8 points per packet
          for(uint8_t i=0; i<8; i++) { //measurement data, 8 points per packet //only works in _parse FULL SixDegBuf
            newPacket.measurements[i] = (((uint16_t) _sixDegRawBuf[(i*3)+5]) << 8) + _sixDegRawBuf[(i*3)+4]; //combine bytes to make number
            newPacket.reservedData[i] = _sixDegRawBuf[(i*3)+6]; //i still dont know what that last byte holds, but sometimes it's nonzero, so better store it to be safe
          }
//        }
//        if(bufferProgress >= 34) { //endAngle bytes received
          newPacket.endAngle = (((uint16_t) _sixDegRawBuf[29]) << 8) + _sixDegRawBuf[28]; //combine bytes to make number
          newPacket.endAngle -= CAMSENSE_X1_ANGLE_SUBTRACT_16;
          if(newPacket.startAngle == newPacket.endAngle) { //if both angles are equal, the packet is invalid (device still spinning up). These packets do have real distance measurements, but no associated angle
            return(0);
          }
//        }
//        if(bufferProgress == 36) { //full packet received
          newPacket.CRCpassed = true;
          while(dataArrayWriting) {} //wait (only needed in specific multicore/interrupt type situations)
          dataArrayWriting = true; //for absolute multi core/threading and interrupt safety
          //determine how to put the new data in the existing array
          if((newPacket.startAngle < _lastAngles[0]) && (volatileIndex > 0)) {//if the new packet is the first one of the new rotation (except the endAngle of the last packet can already be rolled over)
            if(volatileIndex < lidarDataDynamicSize) {
              _deleteTailLidarData(volatileIndex); //release the indexes from volatileIndex onwards and then set lidarDataDynamicSize = volatileIndex
            }
            volatileIndex = 0;
            rotationCount += 1;
          }
          //if(!rotationCount) { //for the first rotation, just insert data. (inserting data at the end of the array (even if the current array size is 0) is perfectly fine)
          if((!rotationCount) || (volatileIndex >= lidarDataDynamicSize)) { //for the first rotation, just insert data. (inserting data at the end of the array (even if the current array size is 0) is perfectly fine)
            _insertOneLidarData(newPacket, volatileIndex);
          } else {
            bool keepSearchingArray = true;
            uint16_t packetsToDelete = 0;
            bool insertNewPacket = false;
            uint16_t nextAngles[2];
            for(uint16_t i=volatileIndex; i<(lidarDataDynamicSize*keepSearchingArray); i++) {
              nextAngles[0] = _lidarData(i).startAngle;  nextAngles[1] = _lidarData(i).endAngle;
              if(nextAngles[0] <= newPacket.startAngle) { //if next startAngle is below 
                if((nextAngles[1] <= newPacket.startAngle) && (nextAngles[0] < nextAngles[1])) {//if the old packet is completely behind the new one, then it must be deleted altogether (e.g: old{0,6} new{7,13})
                  packetsToDelete++;
                  if((lidarDataDynamicSize-packetsToDelete) <= volatileIndex) {
                    insertNewPacket = true; //if it intends to delete all entries between volatileIndex and the end of the list, then it must append (a.k.a. insert at lidarDataDynamicSize)
                  }
                } else if((nextAngles[1] <= newPacket.endAngle) || ((newPacket.endAngle < newPacket.startAngle) && (nextAngles[0] < nextAngles[1]))) {//if next endAngle falls in within angle-range of the new packet
                  keepSearchingArray = false; //the packet to overwrite has been found, stop searching
                } else {//if this is true, the old packet completely encompasses the new packet, which is strange
                  if(nextAngles[0] < nextAngles[1]) {//in normal non-rollover conditions (so old{5,15} is fine, but old{355,10} is not)
                    insertNewPacket = true; //will preserve more data (at the cost of array complexety, and therefore time)
                  } //in rollover conditions, the last packet should simply be overwritten (at the cost of minor data loss) because it will not get deleted/overwritten correctly once obsolete
                  keepSearchingArray = false;
                }
              } else { //the next startAngle is obove the current one
                if((nextAngles[1] <= newPacket.endAngle) && (nextAngles[0] < nextAngles[1])) { //if this is true, the new packet completely encompasses the old packet, which is strange
                  //(very niche scenario) just overwrite the old packet with the new (encompassing) one.  (BTW, the (nextAngles[0] < nextAngles[1]) is to avoid old{358,5} packets being misinterpreted)
                  keepSearchingArray = false;
                } else {
                  insertNewPacket = true;
                  keepSearchingArray = false;
                }
              }
            }
            #ifdef lidarArrayErrorMessages
              if((keepSearchingArray) && (packetsToDelete==0)) {
                log_e("keepSearchingArray but no deleted packets (%d,%d)", volatileIndex, lidarDataDynamicSize);
              }
            #endif
            if((!insertNewPacket) && (packetsToDelete==0)) {
              //no weird stuff, just overwrite the existing packet at volatileIndex
              _lidarData(volatileIndex) = newPacket;
            } else if(insertNewPacket) {
              if(packetsToDelete > 0) { //no need to delete AND insert, if you just delete one less and overwrite the leftover one, you can save a lot of time
                packetsToDelete--;
                if(packetsToDelete > 0) { //if multiple packets 
                  _deleteInbetweenLidarData(packetsToDelete, volatileIndex, false); //delete packets (only obsolete data is deleted)
                }
                _lidarData(volatileIndex) = newPacket;
              } else {
                _insertOneLidarData(newPacket, volatileIndex);
              }
            } else if(packetsToDelete > 0) {
              _deleteInbetweenLidarData(packetsToDelete, volatileIndex, false); //delete packets (only obsolete data is deleted)
              _lidarData(volatileIndex) = newPacket;
            }
          }
          volatileIndex++;
          if(newPacket.endAngle < newPacket.startAngle) {
            if(volatileIndex < lidarDataDynamicSize) { //volatileIndex should be at the max (technically an illigal position) by now. if it isn't, that means there is old data at the end
              _deleteTailLidarData(volatileIndex); //release the indexes from volatileIndex onwards and then set lidarDataDynamicSize = volatileIndex
            }
            volatileIndex = 0; //not 100% necessary, but this does make sure volatileIndex is never equal to lidarDataDynamicSize (i think)
            rotationCount += 1;
          }
          _lastAngles[0] = newPacket.startAngle;
          _lastAngles[1] = newPacket.endAngle;
          RPMraw = newPacket.RPM
          dataArrayWriting = false; //for absolute multi core/threading and interrupt safety
          _packetTimeoutTimer = millis();
//        }
        return(8);
      } else {
        return(0);
      }
    }
    
  public:
    volatile bool spinning = false; //set to true once the first sync byte comes through
    volatile uint32_t rotationCount = 0; //mostly used as an indicator that the first rotation is done, but also usefull to know when a new set of datapoints is available

    volatile uint16_t RPMraw = 0; //divide by 64 to get RPM
    
    //const uint16_t lidarDataArrayMaxSize = 80; global var
    volatile uint16_t lidarDataDynamicSize = 0;
    lidarPacket lidarDataUnordered[lidarDataArrayMaxSize]; //(static) array to hold the data
    volatile uint16_t lidarDataIndexes[lidarDataArrayMaxSize]; //(dynamic) array to hold the indexes of lidarData in the correct order
    volatile uint16_t volatileIndex = 0; //this index is the next index to be filled 
    volatile bool dataArrayWriting = false; //for multi-threading/core and interrupt safety, DONT access dataArray[volatileDegree] if dataArrayWriting is TRUE

    void (*postParseCallback)(camsense_X1* self, int16_t dataPointsAdded) = NULL;
    
    camsense_X1(T& serialPort) : _serialPort(serialPort) {} //init function, pass serial port
    
    #if defined (ESP32) //i'd recommend using Serial1. (the ESP32 has 3 HW serials and by defualt, Serial1 uses pins 9 and 10, which are internal flash pins)
      void begin(uint8_t rx=16, uint8_t tx=17) {
        pinMode(rx, INPUT_PULLUP);
        pinMode(tx, OUTPUT);
        _serialPort.begin(115200, SERIAL_8N1, rx, tx);
        _initLidarDataStructure();
      }
    #else
      void begin() {
        _serialPort.begin(115200);
        _initLidarDataStructure();
      }
    #endif
    
    
    IRAM_ATTR int8_t run(bool allowCallback=true) {
      bool didSomething=false;
      while(_serialPort.available()) {
        while(_sixDegRawBufWriting) {}; //wait (only needed in specific multicore/interrupt type situations)
        _sixDegRawBufWriting = true;
        if(_fillUpBuff >= 4) { //if the starting bytes have been received
          _sixDegRawBuf[_fillUpBuff-4] = _serialPort.read();
          _fillUpBuff++;
          if(_fillUpBuff == 36) {
            //_sixDegRawBufReady = true; //flag to let another core/interrupt know that the data needs to be processed/translated
            speedTimer[8] = ESP.getCycleCount();
            int16_t pointsAdded = _parseFullSixDegBuf(_fillUpBuff); //note, function can also be called intermittendly.
            speedTimer[9] = ESP.getCycleCount();
            _fillUpBuff = 0;
            if((postParseCallback) && allowCallback) { postParseCallback(this, pointsAdded); }
          }
          didSomething=true;
        } else {
          uint8_t syncByteRead = _serialPort.read();
          //start sequence
          switch(syncByteRead) {
            case(CAMSENSE_X1_SYNC_BYTE_0):
              _fillUpBuff = 1; //first sync byte received, start collecting the remaining 35 bytes
              break;
            case(CAMSENSE_X1_SYNC_BYTE_1):
              #ifdef lidarArrayErrorMessages
                if(_fillUpBuff != 1) { log_w("sync byte fault after 1"); }
              #endif
              (_fillUpBuff == 1) ? _fillUpBuff++ : _fillUpBuff=0;
              break;
            case(CAMSENSE_X1_SYNC_BYTE_2):
              #ifdef lidarArrayErrorMessages
                if(_fillUpBuff != 2) { log_w("sync byte fault after 2"); }
              #endif
              (_fillUpBuff == 2) ? _fillUpBuff++ : _fillUpBuff=0;
              break;
            case(CAMSENSE_X1_SYNC_BYTE_3):
              #ifdef lidarArrayErrorMessages
                if(_fillUpBuff != 3) { log_w("sync byte fault after 3"); }
              #endif
              if(_fillUpBuff == 3) {
                _fillUpBuff++;
                spinning = true;
                didSomething=true;
              } else { _fillUpBuff=0; }
              break;
          }
        }
        _sixDegRawBufWriting = false;
      }
      if(!didSomething) { if((millis() - _packetTimeoutTimer) > _runPacketTimeout) { spinning = false; } }
      return(_fillUpBuff);
    }
    
    IRAM_ATTR float RPM() {
      return(RPMraw/64.0);
    }

    //kinda obsolete
    uint16_t dataStructureCheck() {
      return(lidarDataDynamicSize+_lidarDataFreeDynamicSize);
    }
    bool dataStructureCheckBool() {
      return((lidarDataDynamicSize+_lidarDataFreeDynamicSize)==lidarDataArrayMaxSize);
    }
};



camsense_X1<HardwareSerial> lidar(Serial2);


void setup() {
  Serial.begin(250000);
  lidar.begin();
  //lidar.begin(16, 17); //on the ESP32, the (three) serial interfaces can use any (most) pins, so for pin reassignment, fill in (rxPin, txPin) , p.s. i'd recommend using Serial1 for this, as its default pins are 9&10, which are connected to internal flash

  camsense_X1_display_init();
}

void loop() {
  speedTimer[0] = ESP.getCycleCount();
  lidar.run(); //needs to be ran more than 300 times per second, otherwise the serial buffer will fill up
//  speedTimer[1] = ESP.getCycleCount();
//  speedTimer[2] = max(speedTimer[2], speedTimer[1]-speedTimer[0]);
//  speedTimer[3] = min(speedTimer[3], speedTimer[1]-speedTimer[0]);
//  speedTimer[4] = (speedTimer[4] + (speedTimer[1]-speedTimer[0]))/2; //first average
//  speedTimer[5] = (speedTimer[5] + speedTimer[4])/2;                 //second average
//  speedTimer[6] = (speedTimer[6] + speedTimer[5])/2;                 //third average

  if(millis() > printTimer) {
    printTimer = millis() + 500;
    
////    Serial.print((speedTimer[1]-speedTimer[0])/((float)ESP.getCpuFreqMHz()), 3); Serial.print(' ');
////    Serial.print(speedTimer[2]/((float)ESP.getCpuFreqMHz()), 3); Serial.print(' ');
////    Serial.print(speedTimer[3]/((float)ESP.getCpuFreqMHz()), 3); Serial.print(' ');
////    Serial.print(speedTimer[6]/((float)ESP.getCpuFreqMHz()), 3); Serial.print(' ');
////    Serial.print((speedTimer[9]-speedTimer[8])/((float)ESP.getCpuFreqMHz()), 3); Serial.print(' ');
////    speedTimer[3] = speedTimer[2]; //reset min
////    speedTimer[2] = 0; //reset max
////    Serial.println();
//    
    Serial.print(lidar.spinning); Serial.print(' ');
    Serial.print(lidar.rotationCount); Serial.print(' ');
    while(lidar.dataArrayWriting) {} //wait (only for multicore/interrupt safety)
    Serial.print(lidar.lidarDataDynamicSize); Serial.print(' ');
//
////    //print first- and last 5 datapoints
////    if(lidar.lidarDataDynamicSize > 10) {
////      for(int i=0; i<5; i++) {
////        Serial.print(lidarData(lidar,i).startAngle/64.00); Serial.print(','); Serial.print(lidarData(lidar,i).endAngle/64.00); Serial.print(' ');
////      }
////      Serial.print("  ");
////      for(int i=lidar.lidarDataDynamicSize-6; i<lidar.lidarDataDynamicSize; i++) {
////        Serial.print(lidarData(lidar,i).startAngle/64.00); Serial.print(','); Serial.print(lidarData(lidar,i).endAngle/64.00); Serial.print(' ');
////      }
////    }
//    
////    Serial.print(lidar.dataArray[0][0]); Serial.print('\t');
////    Serial.print(lidar.dataArray[0][1]); Serial.print(' ');
////    Serial.print(lidar.dataArray[179][1]);
//
    Serial.println();
  }
}

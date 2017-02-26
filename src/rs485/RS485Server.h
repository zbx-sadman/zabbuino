#ifndef RS485SERVER_H
#define RS485SERVER_H

#include "RS485Structs.h"
#include "RS485-conf.h"
//#include "service.h"

class RS485Class;
class RS485ClientClass;

class RS485ServerClass {
  private:
     RS485Class* RS485;

  public:
    RS485ServerClass(RS485Class&);
    RS485ServerClass(RS485Class*);
    ~RS485ServerClass();

    void begin(uint32_t);
    void begin(uint8_t, uint8_t, uint8_t, uint8_t);

    RS485ClientClass available();

  friend RS485Class;
   //friend RS485ClientClass;

};

#endif // RS485SERVER_H


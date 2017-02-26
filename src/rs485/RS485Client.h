#ifndef RS485CLIENT_H
#define RS485CLIENT_H

#include "RS485Structs.h"
#include "RS485-conf.h"
//#include "service.h"


// Make forward declaration to able use pointers to class instances & friend keyword
class RS485Class;
class RS485ServerClass;

class RS485ClientClass {
  private:
    RS485Class* RS485;

  public:
    //RS485ClientClass();
    RS485ClientClass(RS485Class&);
    RS485ClientClass(RS485Class*);
    ~RS485ClientClass();

    void stop();
    void flush() {}
    uint8_t connect(const NetworkAddress);
    uint8_t connected();
    int16_t available();
    int16_t read();
    int32_t println(int32_t&, int);
    int32_t println(const char* _src);

    operator bool();
    bool const operator==(const RS485ClientClass&);
    bool const operator!=(const RS485ClientClass& rhs) { return !this->operator==(rhs); };


    friend RS485Class;
    friend RS485ServerClass;

};

#endif // RS485CLIENT_H


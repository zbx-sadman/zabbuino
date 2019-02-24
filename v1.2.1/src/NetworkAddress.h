#ifndef NetworkAddress_H
#define NetworkAddress_H

#include <Arduino.h>
#include <IPAddress.h>

class NetworkAddress {
private:
   union {
     uint8_t bytes[4];  // IPv4 address
     uint32_t dword;
   } _address;

    uint8_t* raw_address() { return _address.bytes; };
public:
    // Constructors
    NetworkAddress();
    NetworkAddress(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet);
    NetworkAddress(uint32_t address);
    NetworkAddress(const uint8_t *address);

    // Overloaded cast operator to allow IPAddress objects to be used where a pointer
    // to a four-byte uint8_t array is expected
    operator uint32_t() const { return _address.dword; };

    bool operator==(const NetworkAddress& addr) const { return _address.dword == addr._address.dword; };
    bool operator==(const uint8_t* addr) const;

    // Overloaded index operator to allow getting and setting individual octets of the address
    uint8_t operator[](int index) const { return _address.bytes[index]; };
    uint8_t& operator[](int index) { return _address.bytes[index]; };

    // Overloaded copy operators to allow initialisation of IPAddress objects from other types
    NetworkAddress& operator=(const uint8_t *address);
    NetworkAddress& operator=(uint32_t address);
};
#endif // NetworkAddress_H


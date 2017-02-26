#include "NetworkAddress.h"

NetworkAddress::NetworkAddress()
{
    _address.dword = 0;
}

NetworkAddress::NetworkAddress(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet)
{
    _address.bytes[0] = first_octet;
    _address.bytes[1] = second_octet;
    _address.bytes[2] = third_octet;
    _address.bytes[3] = fourth_octet;
}

NetworkAddress::NetworkAddress(uint32_t address)
{
    _address.dword = address;
}

NetworkAddress::NetworkAddress(const uint8_t *address)
{
    memcpy(_address.bytes, address, sizeof(_address.bytes));
}


NetworkAddress& NetworkAddress::operator=(const uint8_t *address)
{
    memcpy(_address.bytes, address, sizeof(_address.bytes));
    return *this;
}

NetworkAddress& NetworkAddress::operator=(uint32_t address)
{
    _address.dword = address;
    return *this;
}

bool NetworkAddress::operator==(const uint8_t* addr) const
{
    return memcmp(addr, _address.bytes, sizeof(_address.bytes)) == 0;
}





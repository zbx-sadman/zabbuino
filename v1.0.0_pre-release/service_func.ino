void gatherMetrics(){
  correctVCCMetrics(MeasureVoltage(ANALOG_CHAN_VBG));
//  correctVCCMetrics(0);
}

void correctVCCMetrics(uint32_t _currVCC) {
  sysMetrics[SYS_METRIC_IDX_VCCMIN] = min(_currVCC, sysMetrics[SYS_METRIC_IDX_VCCMIN]);
  sysMetrics[SYS_METRIC_IDX_VCCMAX] = max(_currVCC, sysMetrics[SYS_METRIC_IDX_VCCMAX]);
}

void setDefaults(netconfig_t& _configStruct)
{
  char hostname[] = ZBX_AGENT_DEFAULT_HOSTNAME;  
  //  memcpy(_configStruct.hostname, hostname, ZBX_AGENT_HOSTNAME_MAXLEN-1);
  //_configStruct.hostname[ZBX_AGENT_HOSTNAME_MAXLEN]='\0';
  sethostname(_configStruct.hostname, hostname);
  uint8_t mac[] = SYS_DEFAULT_MAC_ADDRESS;
  memcpy(_configStruct.macAddress, mac, sizeof(netConfig.macAddress));
  _configStruct.useDHCP = SYS_DEFAULT_USE_DHCP;
  _configStruct.ipAddress = IPAddress(SYS_DEFAULT_IP_ADDRESS);
  _configStruct.ipNetmask = IPAddress(SYS_DEFAULT_NETMASK);
  _configStruct.ipGateway = IPAddress(SYS_DEFAULT_GATEWAY);
//#ifdef PASSWORD_PROTECTION_FEATURE_ENABLE
  _configStruct.password = SYS_DEFAULT_PASSWORD;
  _configStruct.useProtection = SYS_DEFAULT_PROTECTION;
//#endif

   //printArray(_configStruct.macAddress,6,1);
}


void sethostname(char* _dest, const char* _src){
   strncpy(_dest, _src, ZBX_AGENT_HOSTNAME_MAXLEN-1);
   _dest[ZBX_AGENT_HOSTNAME_MAXLEN]='\0';
//   return _dest;

}

// convert _len chars (exclude 0x prefix) of hex string to byte array
uint8_t hstoba(uint8_t* _array, const char* _data, uint8_t _len)
{
  // don't fill _array and return false if mailformed string detected
  if (!isHexString(_data)) { return false; }
  // skip prefix
  _data += 2;
  // for all bytes do...
  while (_len--)  {
     *_array = (htod(*_data) << 4);
     _data++;
     *_array += htod(*_data);
      _data++; _array++;      
  };
  return true;
}

// 
uint8_t isHexString(const char* _source) 
{
  if (_source[0] == '0' && _source[1] == 'x') { return true; }
  return false;
}



/* ****************************************************************************************************************************
*
*  Функция преобразования шестнадцатеричного символа (lowcase) в десятичное число
*
**************************************************************************************************************************** */
uint8_t htod(int8_t _hex)
{
  if (_hex >= 'a' && _hex <= 'f')
  {
    return (10 + _hex - 'a');
  } else if (_hex >= '0' && _hex <= '9')
  {
    return (_hex - '0');
  } else {
    return 0;
  }
}


/* ****************************************************************************************************************************
*
*  Функция преобразования 32-битного (long) числа в строку с десятичным разделителем на заданном месте
*
**************************************************************************************************************************** */
char* ltoaf(int32_t _number, char* _dst, uint8_t _num_after_dot)
//  _number is long because may contain int*2 => [-]int.int
{
  int8_t k, r;
  uint8_t s, dp, bp, ep, o, sl;

#ifdef DEBUG_MSG_TO_ETHCLIENT
  ethClient.print("number:");
  ethClient.println(_number);
  ethClient.print("num_after_dot:");
  ethClient.println(_num_after_dot);
#endif
  ltoa(_number, _dst, 10);

  sl = strlen(_dst);
  // offset =1 for shifting with making one free cell in char array where will be placed '.'
  o = 1;

  // check for negative sign is exist
  s = 0;
  if (_dst[0] == '-') s = 1;

  k = sl - _num_after_dot;
  // Begin position of shifted part is (0 + neg_sign_len) if need to shift right all string except neg. sigh (
  // or (str_num_len - future_fract_part_len)
  //
  bp = max(s, k);
  // dot position is begin position of shift part. Dot is replace shifted to right symb.
  dp = bp;
  // if need to shift right more that string length (-1234 -> -0.1234) begin pos always eq lengt of neg. sign
  // is calculated in max() func.
  // if (k <= s )
  if (bp == s )
  {
    //printf("sl <= num\n");
    // shift offset is more that one cell, because need to insert "0.[0]" to new string
    // offset is lengt of "0." + number of zeroes which filled gap:
    //                         1 + 1 + neg_sign_len + (future_fract_part_len - str_num_len)
    o += 1 + s + (_num_after_dot - sl);
    // Correction of dot position because need to insert additional "0" before dot.
    // if this block is executed, bp = b =s
    dp += 1;
  }

  // end of new string eq length_of_old_string + length of appended part (just '.' or "0.[0]")
  ep = sl + o;

  // Use neg sign flag as symbols counter, which corrected for exepting neg symbol length
  //i=s;

#ifdef DEBUG_MSG_TO_ETHCLIENT
  ethClient.print("bp: ");
  ethClient.println(bp);
  ethClient.print("ep: ");
  ethClient.println(ep);
  ethClient.print("sl: ");
  ethClient.println(sl);
  ethClient.print("s: ");
  ethClient.println(s);
  ethClient.print("o: ");
  ethClient.println(o);
  ethClient.print("dp: ");
  ethClient.println(dp);
  ethClient.print("dst: ");
  ethClient.println(_dst);
#endif

  // Shift strings symbols by going from end to begin (reverse move) and fill current cell with value from
  // previous cell, which placed on Offset distantion.
  for (r = ep; r >= bp ; r--)
  {
    // if not all symbols from unshifted string was copied - to that
    // if (i <= sl)
    if (s <= sl)
    {
      _dst[r] = _dst[r - o];
    }
    // otherwise just fill cells by '0'
    else
    {
      _dst[r] = '0';
    }
    // count how much symbols copied
    //i++;
    s++;
  }
  // place Dot on this position
  _dst[dp] = '.';
  // finalize string
  // finalized by ltoa()
  //_dst[ep]='\0';

  return _dst;
}

/* ****************************************************************************************************************************
*
*  Convert dec number to hex char
*
**************************************************************************************************************************** */
char dtoh(uint8_t _dec)
{
//  if (_dec > 0xF) {return '0';}
  if (_dec > 0x9) {return 'A'+_dec-0xA;}
  return '0'+_dec;
}

/* ****************************************************************************************************************************
*
*  Convert N bytes from Source to HEX chars and put its to Destination
*
**************************************************************************************************************************** */
// pointer => hex string
void ptonhs(char *_dstptr, uint8_t *_srcptr, uint8_t _len) {
  // write to destination prefix 0x 
  _dstptr[0] = '0'; _dstptr[1] = 'x';  
  // move pointer to begin+2
  _dstptr+=2;
  // count _len bytes of memory pointed with _srcptr
  while (_len--) {
    // Take high nibble and convert to hex char
    *_dstptr++ = dtoh(*_srcptr >> 4); 
    // Take low nibble and convert to hex char, increase source pointer to 1;
    *_dstptr++ = dtoh(*_srcptr++ & 0x0F);
  }
  // terminate C-string
  *_dstptr='\0'; 
}

//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but much smaller, than the lookup table.
//
// This function placed here to aviod compilation error for "no OneWire devices used" case
uint8_t dallas_crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;
	
	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

void SerialPrint_P (const char* _src) {
  char currChar;
  while ((currChar = pgm_read_byte(_src++)) != 0)
  Serial.print(currChar);
}
void SerialPrintln_P (const char* _src) {
  SerialPrint_P(_src);
  Serial.println();
}


// pointer-lenght array of char to uint32_t variable
uint32_t pl_atol(char const *_src, uint8_t _len) {
  long result = 0;
  uint8_t i;
  int8_t sign = 1;

  // Check lengh of buffer and work if its no more max digits of int_32 type (11 chars)
  if ( 11 < _len ) {return 0; }
  
  // Sign of number is exist? 
  if ('-' == (*_src)) {sign = -1; _src++; _len--; }

  // Walk thru buffer and calculate int32_t number
  for (i = 0 ; i < _len ; i++) {
    if ((*_src < '0') or (*_src > '9')) { return 0; }
    result = (result << 3) + ( result<<1 ) + *_src -'0';
   _src++;
  }

  // If number with sign
  return sign * result;
}


void printArray(uint8_t *_src, uint8_t _len, uint8_t _type)
{
  while (_len--) {
    if (DBG_PRINT_AS_MAC == _type) {
       Serial.print(*_src, HEX);
       if (1 <= _len) { Serial.print(':'); }
    } else if (DBG_PRINT_AS_IP == _type) {
       Serial.print(*_src, DEC);
       if (1 <= _len) Serial.print('.');
    }
    _src++;
  }
  Serial.println();
}

/*
uint32_t pton(const char* _ipString) 
{
  uint8_t i = 0, k = 0, n = 0;
  char* buff = "\1\2\3\4";
  uint32_t currentOctet, result = 0;
  do {
    buff[k] = _ipString[i];
    if ('.' == _ipString[i] || '\0' == _ipString[i]) {
       buff[k]='\0';
       currentOctet = atoi(buff);
       result += currentOctet << n;
       // every new octet must be shift to right for n*8 bytes
       n += 8;
       k = 0;
       if ('\0' == _ipString[i]) {break;}
    } else { 
       k++;
    }
    i++;
  } while (true);
  return result;
}


*/


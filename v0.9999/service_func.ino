
/* ****************************************************************************************************************************
*
*  Функция копирования фрагмента одной строки от заданного символа до ее конца  в другую
*
**************************************************************************************************************************** */
void copyChars(char *dst, char *src, int16_t from, int16_t count)
{
  int16_t i;
  for (i = 0; i < count; i++)
  {
    dst[i] = src[from + i];
  }
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



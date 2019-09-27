#pragma once

#define DFPLAYER_MINI_UART_SPEED                   (9600) // DFPlayer Mini works on 9600 bps

#define DFPLAYER_MINI_PACKET_SIZE                  (0x0A)
#define DFPLAYER_MINI_DEFAULT_READ_TIMEOUT         (1000UL)

#define DFPLAYER_MINI_BYTE_HEADER                  (0x00)
#define DFPLAYER_MINI_BYTE_VERSION                 (0x01)
#define DFPLAYER_MINI_BYTE_LENGHT                  (0x02)
#define DFPLAYER_MINI_BYTE_COMMAND                 (0x03)
#define DFPLAYER_MINI_BYTE_FEEDBACK                (0x04)
#define DFPLAYER_MINI_BYTE_PARAM_00                (0x05)
#define DFPLAYER_MINI_BYTE_PARAM_01                (0x06)
#define DFPLAYER_MINI_BYTE_CRC_00                  (0x07)
#define DFPLAYER_MINI_BYTE_CRC_01                  (0x08)
#define DFPLAYER_MINI_BYTE_END                     (0x09)

#define DFPLAYER_MINI_HEADER                       (0x7E)
#define DFPLAYER_MINI_VERSION                      (0xFF)
#define DFPLAYER_MINI_FEEDBACK                     (0x01) // Need feedback (ACK must be returned)
#define DFPLAYER_MINI_PAYLOAD_LENGHT               (0x06) 
#define DFPLAYER_MINI_END                          (0xEF)
                                                  
#define DFPLAYER_MINI_INACTIVE_INTERVAL            (50)   // ms

// Command codes
#define DFPLAYER_CMD_PLAYBACK_TRACK                (0x02) // Specify playback of a track
#define DFPLAYER_CMD_SET_VOLUME                    (0x06) // Specify volume
#define DFPLAYER_CMD_RESET                         (0x0C) // 
#define DFPLAYER_CMD_PLAY                          (0x0D) // 
#define DFPLAYER_CMD_PAUSE                         (0x0E) // 
#define DFPLAYER_CMD_STOP                          (0x16) // 
#define DFPLAYER_CMD_SET_DAC                       (0x1A) // Set DAC
#define DFPLAYER_CMD_GET_STATUS                    (0x42) // 

#define DFPLAYER_MINI_REPLY_SUCCESS                (0x41)


/*****************************************************************************************************************************
*
*  Run specified commad with specified volume on DFPlayer Mini
*
*  Returns: 
*    - DEVICE_ERROR_TIMEOUT        if device stop talking
*    - DEVICE_ERROR_CHECKSUM       when bad CRC is recieved
*    - DEVICE_ERROR_WRONG_ANSWER   on DFPlayer Mini answer "fail" (not 0x41)
*    - DEVICE_ERROR_NOT_SUPPORTED  when wrong command is given
*    - RESULT_IS_FAIL              on other fails
*
*****************************************************************************************************************************/
int8_t runDFPlayerMini(const uint8_t _rxPin, const uint8_t _txPin, uint8_t _command, uint16_t _option, int16_t _volume, uint8_t* _dst);
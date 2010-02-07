//M4MO BY PGH

#ifndef M4MO_H
#define M4MO_H



#include <mach/board.h> 
#include "pgh_debug.h" 


#define FLASH_CMD           200

#define FLASH_CAMERA       0
#define FLASH_MOVIE        1  

#define FLASH_CMD_ON      1
#define FLASH_CMD_OFF     0

#define RESET_FOR_FW            202

int32_t m4mo_i2c_write_8bit_external(unsigned char category, unsigned char byte, unsigned char value);




#endif /* M4MO_H */


/*
 * I2C1 user function(s) file
 * Master and BLDC boards use I2C1 to send/receive information
 * January, 2018
 * Author: Rodrigo A. Romano
 */

#include "mcc_generated_files/mcc.h"
#include "phase_control.h"

#define M_BYTES_TO_WORD(byteH, byteL) ((uint16_t)(((uint16_t)byteL & 0x00FF) | (((uint16_t)byteH << 8) & 0xFF00)))

uint8_t i2c1Buffer[64] =
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };

bool I2C1_LogicHandleF(I2C1_SLAVE_DRIVER_STATUS status)
{
    static uint8_t  i2c1BufferAddress = 0;
    static uint8_t  i2c1ReadBuffer;
    static uint8_t  i2c1WriteBuffer;
    static bool     addressState = true;
    
    switch (status)
    {
        case I2C1_SLAVE_TRANSMIT_REQUEST_DETECTED:

            // set up the slave driver buffer transmit pointer
            I2C1_ReadPointerSet(&i2c1ReadBuffer);

            break;

        case I2C1_SLAVE_RECEIVE_REQUEST_DETECTED:

            addressState = true;
            // set up the slave driver buffer receive pointer
            I2C1_WritePointerSet(&i2c1BufferAddress);

            break;

        case I2C1_SLAVE_RECEIVED_DATA_DETECTED:

            if (addressState == true)
            {
                // get the address of the memory being accessed
                i2c1ReadBuffer = i2c1Buffer[i2c1BufferAddress];
                addressState = false;
                
                // In case of write operation points to write buffer
                I2C1_WritePointerSet(&i2c1WriteBuffer);
                // In case of read operation points to read buffer
                I2C1_ReadPointerSet(&i2c1ReadBuffer);
            }
            else // if (addressState == false)
            {
                // set the I2C1 buffer with the received data
                i2c1Buffer[i2c1BufferAddress] = i2c1WriteBuffer;
                
                if(i2c1BufferAddress == 20)
                {
                    uint16_t new_MDC = 
                            M_BYTES_TO_WORD(i2c1Buffer[i2c1BufferAddress],
                            i2c1Buffer[i2c1BufferAddress+1]);
                    if(new_MDC < 750)   // Check validity of new MDC
                    {
                        PWM_MasterDutyCycleSet(new_MDC);
                        Commut_Phase();
                    }
                    
                    // Just for debug
                    i2c1Buffer[30] = i2c1Buffer[20];
                    i2c1Buffer[31] = i2c1Buffer[21];
                        
                }
            }

            break;

        case I2C1_SLAVE_10BIT_RECEIVE_REQUEST_DETECTED:

            // do something here when 10-bit address is detected

            // 10-bit address is detected

            break;

        default:
            break;

    }

    return true;
}

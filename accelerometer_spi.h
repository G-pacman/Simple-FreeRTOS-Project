/* ************************************************************************** */
/** accelerometer_spi.h

  @Description
    Need this file to access functions from library.
 
 */
/* ************************************************************************** */

#ifndef _ACCELEROMETER_SPI_H    /* Guard against multiple inclusion */
#define _ACCELEROMETER_SPI_H 

void spi2_setup(void);
void spi2_write_register(uint8_t address, uint8_t data);
int16_t spi2_read_register(uint8_t address);
void accel_setup(void);               
float accel_read_x(void);
float accel_read_y(void);   
float accel_read_z(void);


#endif /* _ACCELEROMETER_SPI_H  */

/* *****************************************************************************
 End of File
 */

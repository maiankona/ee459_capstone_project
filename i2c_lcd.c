#include <avr/io.h>
#include <stdio.h>

#define DEBUG

/*
  i2c_io - write and read bytes to an I2C device

  This funtions writes "wn" bytes from array "wp" to I2C device at
  bus address "device_addr".  It then reads "rn" bytes from the same device
  to array "rp".

  Return values (might not be a complete list):
        0    - Success
        0x20 - NAK received after sending device address for writing
        0x30 - NAK received after sending data
        0x38 - Arbitration lost with address or data
        0x48 - NAK received after sending device address for reading

  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

This "i2c_io" I2C routine is an attempt to provide an I/O function for both
reading and writing, rather than have separate functions.

I2C writes consist of sending a stream of bytes to the slave device.  In some
cases the first few bytes may be the internal address in the device, and then
the data to be stored follows.  For example, EEPROMs like the 24LC256 require a
two-byte address to precede the data.  The DS1307 RTC requires a one-byte
address.

I2C reads often consist of first writing one or two bytes of internal address
data to the device and then reading back a stream of bytes starting from that
address.  Some devices appear to claim that that reads can be done without
first doing the address writes, but so far I haven't been able to get any to
work that way.

This function does writing and reading by using pointers to two arrays
"wp", and "rp".  The function performs the following actions in this order:
    If "wn" is greater then zero, then "wn" bytes are written from array "wp"
    If "rn" is greater then zero, then "rn" byte are read into array "rp"
Either of the "wn" or "rn" can be zero.

A typical write with a 2-byte address and 50 data bytes is done with

    i2c_io(0xA0, wbuf, 52, NULL, 0);

A typical read of 20 bytes with a 1-byte address is done with

    i2c_io(0xD0, wbuf, 1, rbuf, 20);
*/

uint8_t i2c_io(uint8_t device_addr,
               uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn)
{
  uint8_t status, send_stop, wrote, start_stat;

  status = 0;
  wrote = 0;
  send_stop = 0;

  if (wn > 0)
  {
    wrote = 1;
    send_stop = 1;

    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA); // Send start condition
    while (!(TWCR & (1 << TWINT)))
      ; // Wait for TWINT to be set
    status = TWSR & 0xf8;
    if (status != 0x08) // Check that START was sent OK
      return (status);

    TWDR = device_addr & 0xfe;         // Load device address and R/W = 0;
    TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
    while (!(TWCR & (1 << TWINT)))
      ; // Wait for TWINT to be set
    status = TWSR & 0xf8;
    if (status != 0x18)
    {                     // Check that SLA+W was sent OK
      if (status == 0x20) // Check for NAK
        goto nakstop;     // Send STOP condition
      return (status);    // Otherwise just return the status
    }

    // Write "wn" data bytes to the slave device
    while (wn-- > 0)
    {
      TWDR = *wp++;                      // Put next data byte in TWDR
      TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
      while (!(TWCR & (1 << TWINT)))
        ; // Wait for TWINT to be set
      status = TWSR & 0xf8;
      if (status != 0x28)
      {                     // Check that data was sent OK
        if (status == 0x30) // Check for NAK
          goto nakstop;     // Send STOP condition
        return (status);    // Otherwise just return the status
      }
    }

    status = 0; // Set status value to successful
  }

  if (rn > 0)
  {
    send_stop = 1;

    // Set the status value to check for depending on whether this is a
    // START or repeated START
    start_stat = (wrote) ? 0x10 : 0x08;

    // Put TWI into Master Receive mode by sending a START, which could
    // be a repeated START condition if we just finished writing.
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
    // Send start (or repeated start) condition
    while (!(TWCR & (1 << TWINT)))
      ; // Wait for TWINT to be set
    status = TWSR & 0xf8;
    if (status != start_stat) // Check that START or repeated START sent OK
      return (status);

    TWDR = device_addr | 0x01;         // Load device address and R/W = 1;
    TWCR = (1 << TWINT) | (1 << TWEN); // Send address+r
    while (!(TWCR & (1 << TWINT)))
      ; // Wait for TWINT to be set
    status = TWSR & 0xf8;
    if (status != 0x40)
    {                     // Check that SLA+R was sent OK
      if (status == 0x48) // Check for NAK
        goto nakstop;
      return (status);
    }

    // Read all but the last of n bytes from the slave device in this loop
    rn--;
    while (rn-- > 0)
    {
      TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Read byte and send ACK
      while (!(TWCR & (1 << TWINT)))
        ; // Wait for TWINT to be set
      status = TWSR & 0xf8;
      if (status != 0x50) // Check that data received OK
        return (status);
      *rp++ = TWDR; // Read the data
    }

    // Read the last byte
    TWCR = (1 << TWINT) | (1 << TWEN); // Read last byte with NOT ACK sent
    while (!(TWCR & (1 << TWINT)))
      ; // Wait for TWINT to be set
    status = TWSR & 0xf8;
    if (status != 0x58) // Check that data received OK
      return (status);
    *rp++ = TWDR; // Read the data

    status = 0; // Set status value to successful
  }

nakstop: // Come here to send STOP after a NAK
  if (send_stop)
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); // Send STOP condition

  return (status);
}

/*
  i2c_init - Initialize the I2C port
*/
void i2c_init(uint8_t bdiv)
{
  TWSR = 0;    // Set prescalar for 1
  TWBR = bdiv; // Set bit rate register
}

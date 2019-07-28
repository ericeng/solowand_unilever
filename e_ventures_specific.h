

/*
The following four function declarations are included because they are missing
from standard library header file i2c-dev.h
*/
static inline __s32 _i2c_smbus_access( int file, char read_write, __u8 command, int size, union i2c_smbus_data *data )
{
    struct i2c_smbus_ioctl_data args;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;

    return ioctl( file, I2C_SMBUS, &args );
}

static inline __s32 _i2c_smbus_read_byte_data( int file, __u8 command )
{
    union i2c_smbus_data data;

    if ( _i2c_smbus_access( file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data ))
      {
      return -1;
      }
    else
      {
      return ( 0xFF & data.byte );
      }
}

static inline __s32 _i2c_smbus_write_byte_data( int file, __u8 command, __u8 value )
{
    union i2c_smbus_data data;

    data.byte = value;

    return _i2c_smbus_access( file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data );
}

static inline __s32 _i2c_smbus_read_word_data(int file, __u8 command)
{
    union i2c_smbus_data data;

    if( _i2c_smbus_access( file, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data ))
      {
      return -1;
      }
    else
      {
      return 0xFFFF & data.word;
      }
}

static inline __s32 _i2c_smbus_write_word_data( int file, __u8 command, __u16 value )
{
    union i2c_smbus_data data;

    data.word = value;
    return _i2c_smbus_access( file, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data );
}


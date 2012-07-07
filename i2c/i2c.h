#ifndef I2C_H
#define I2C_H 1


class I2C
{
  public:
    I2C(char * filename);

    void setDeviceAddress(const int &address);
    void write(const unsigned char &val);

    void i2cRead(char * buf, int size);

    void i2cEnd();

  private:
    int fd;
    int address;

    void i2cBeginLow(const int &address);


};


#endif

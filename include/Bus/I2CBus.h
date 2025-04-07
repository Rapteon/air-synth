#ifndef I2CBUS_H
#define I2CBUS_H

#include <cstdint>
class I2CBus {
public:
  I2CBus(const char *device_file);
  ~I2CBus();
  void register_slave(int address);
  void sendData(int address, uint8_t *buffer, int byte_count);
  long receiveData(int address, uint8_t *buffer, int byte_count);

private:
  int file;
};
#endif
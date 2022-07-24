#pragma once


#include <Windows.h>
#include <WinBase.h>
#include <stdint.h>


class SerialPortWrapper {
public:
  SerialPortWrapper();
  SerialPortWrapper(const char*, int);
  SerialPortWrapper(int, int);

  ~SerialPortWrapper();

  void begin(const char*, int);
  void begin(int, int);

  void end();

  int write(uint8_t*, size_t);
  int read(uint8_t*, size_t);
  bool is_empty();
  bool check();
  void flush();
  char get_char();
  void put_char(char);


private:
  HANDLE com_handle_ = INVALID_HANDLE_VALUE;
  DCB dcb_old_;
  DCB dcb_new_;
  COMMTIMEOUTS timeout_old_;
  COMMTIMEOUTS timeout_new_;
};

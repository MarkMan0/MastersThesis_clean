#include "SerialPortWrapper/SerialPortWrapper.h"
#include <cstdio>


SerialPortWrapper::SerialPortWrapper(void) {
}

SerialPortWrapper::SerialPortWrapper(const char* port, int baud) {
  begin(port, baud);
}

SerialPortWrapper::SerialPortWrapper(int port, int baud) {
  begin(port, baud);
}

SerialPortWrapper::~SerialPortWrapper(void) {
  end();
}

void SerialPortWrapper::begin(const char* port, int baud) {
  com_handle_ = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
  if (com_handle_ == INVALID_HANDLE_VALUE) return;

  GetCommTimeouts(com_handle_, &timeout_old_);

  timeout_new_ = timeout_old_;

  timeout_new_.ReadIntervalTimeout = MAXDWORD;
  timeout_new_.ReadTotalTimeoutConstant = 0;
  timeout_new_.ReadTotalTimeoutMultiplier = 0;
  timeout_new_.WriteTotalTimeoutMultiplier = 0;
  timeout_new_.WriteTotalTimeoutConstant = 0;

  SetCommTimeouts(com_handle_, &timeout_new_);

  GetCommState(com_handle_, &dcb_old_);

  dcb_new_ = dcb_old_;

  dcb_new_.BaudRate = baud;
  dcb_new_.ByteSize = 8;
  dcb_new_.fParity = 0;
  dcb_new_.Parity = 0;
  dcb_new_.StopBits = 0;

  SetCommState(com_handle_, &dcb_new_);
}

void SerialPortWrapper::begin(int port, int baud) {
  char port_str[20];

  snprintf(port_str, 19, "//.////COM%d", port);

  begin(port_str, baud);
}

void SerialPortWrapper::end() {
  if (com_handle_ == INVALID_HANDLE_VALUE) {
    return;
  }

  SetCommTimeouts(com_handle_, &timeout_old_);
  SetCommState(com_handle_, &dcb_old_);

  CloseHandle(com_handle_);

  com_handle_ = INVALID_HANDLE_VALUE;
}

int SerialPortWrapper::write(uint8_t* data, size_t sz) {
  if (com_handle_ == INVALID_HANDLE_VALUE) {
    return -1;
  }
  unsigned long count = 0;
  WriteFile(com_handle_, data, sz, &count, NULL);
  return count;
}

int SerialPortWrapper::read(uint8_t* data, size_t sz) {
  if (com_handle_ == INVALID_HANDLE_VALUE) {
    return -1;
  }
  unsigned long count = 0;
  ReadFile(com_handle_, data, sz, &count, NULL);
  return count;
}

bool SerialPortWrapper::is_empty() {
  return false;
}

bool SerialPortWrapper::check() {
  return com_handle_ != INVALID_HANDLE_VALUE;
}

void SerialPortWrapper::flush() {
  if (com_handle_ == INVALID_HANDLE_VALUE) {
    return;
  }
  FlushFileBuffers(com_handle_);
}

char SerialPortWrapper::get_char() {
  if (com_handle_ == INVALID_HANDLE_VALUE) {
    return -1;
  }
  unsigned long count;
  char c;
  ReadFile(com_handle_, &c, 1, &count, NULL);

  if (count == 1) {
    return c;
  }


  return -1;
}

void SerialPortWrapper::put_char(char C) {
  if (com_handle_ == INVALID_HANDLE_VALUE) {
    return;
  }

  unsigned long count;

  WriteFile(com_handle_, &C, 1, &count, NULL);
}

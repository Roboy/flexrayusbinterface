#pragma once

#include <type_traits>
#include <utility>
#include "ftd2xx.h"

#define EXPECT(_y_, _x_) do { auto&& _actual_value_ = (_x_); if (_actual_value_ == (_y_)) {} else { return _actual_value_; } } while (false)
#define TRY(_x_) EXPECT(FtResult::Message::OK, (_x_))

class FtResult
{
public:
  enum class Message : FT_STATUS
  {
    OK,
    INVALID_HANDLE,
    DEVICE_NOT_OPENED,
    IO_ERROR,
    INSUFFICIENT_RESOURCES,
    INVALID_PARAMETER,
    INVALID_BAUD_RATE,
    DEVICE_NOT_OPENED_FOR_ERASE,
    DEVICE_NOT_OPENED_FOR_WRITE,
    FAILED_TO_WRITE_DEVICE,
    EEPROM_READ_FAILED,
    EEPROM_WRITE_FAILED,
    EEPROM_ERASE_FAILED,
    EEPROM_NOT_PRESENT,
    EEPROM_NOT_PROGRAMMED,
    INVALID_ARGS,
    NOT_SUPPORTED,
    OTHER_ERROR,
    DEVICE_LIST_NOT_READY
  };

private:
  Message message;

public:
  inline FtResult(Message message) : message{ message }
  {
  }

  inline explicit FtResult(FT_STATUS status) : FtResult{ static_cast<Message>(status) }
  {
  }

  inline auto str() const -> char const*
  {
    switch (message)
    {
      case Message::OK:
        return  "it's all good" ;
      case Message::INVALID_HANDLE:
        return  "invalid handle" ;
      case Message::DEVICE_NOT_OPENED:
        return  "device not opened" ;
      case Message::IO_ERROR:
        return  "I/O error" ;
      case Message::INSUFFICIENT_RESOURCES:
        return  "insufficient resources" ;
      case Message::INVALID_PARAMETER:
        return  "invalid parameter" ;
      case Message::INVALID_BAUD_RATE:
        return  "invalid baud rate" ;
      case Message::DEVICE_NOT_OPENED_FOR_ERASE:
        return  "device not opened for erase" ;
      case Message::DEVICE_NOT_OPENED_FOR_WRITE:
        return  "device not opened for write" ;
      case Message::FAILED_TO_WRITE_DEVICE:
        return  "failed to write device" ;
      case Message::EEPROM_READ_FAILED:
        return  "EEPROM read failed" ;
      case Message::EEPROM_WRITE_FAILED:
        return  "EEPROM write failed" ;
      case Message::EEPROM_ERASE_FAILED:
        return  "EEPROM erase failed" ;
      case Message::EEPROM_NOT_PRESENT:
        return  "EEPROM not present" ;
      case Message::EEPROM_NOT_PROGRAMMED:
        return  "EEPROM not programmed" ;
      case Message::INVALID_ARGS:
        return  "invalid arguments" ;
      case Message::NOT_SUPPORTED:
        return  "not supported" ;
      case Message::OTHER_ERROR:
        return  "other error" ;
      case Message::DEVICE_LIST_NOT_READY:
        return  "device list not ready" ;
    }
  }

  inline operator Message() const {
      return message;
  }

  template <typename F, typename... Args>
  auto or_else(F&& f, Args&&... args) const ->
      typename std::enable_if<std::is_same<typename std::result_of<F(FtResult const&, Args...)>::type, FtResult>::value,
                              FtResult>::type
  {
    if (message == Message::OK)
      return *this;
    return std::forward<F>(f)(*this, std::forward<Args>(args)...);
  }
};

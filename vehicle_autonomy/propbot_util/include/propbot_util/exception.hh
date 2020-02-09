#pragma once

#include <exception>
#include <string>

namespace propbot {

class Exception : std::exception {
 public:
  Exception(std::string reason):
    reason_(reason) {}

  virtual ~Exception() = default;

  const char *what() const noexcept override;
 private:
  std::string reason_;
};

}

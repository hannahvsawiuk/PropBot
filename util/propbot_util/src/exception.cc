#include <propbot_util/exception.hh>

namespace propbot {

const char *Exception::what() const noexcept
{
  return reason_.c_str();
}

}

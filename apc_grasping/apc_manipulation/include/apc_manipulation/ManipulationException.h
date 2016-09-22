#ifndef __MANIPULATION_EXCEPTION__
#define __MANIPULATION_EXCEPTION__
#include <string>
#include <exception>

namespace apc
{
/*
    For exception handling
*/
class ManipulationException : public std::exception
{
public:
  ManipulationException(std::string text) : exception_text_(text)
  {
  }
  ~ManipulationException() throw()
  {
  }

  const char *what() const throw()
  {
    return exception_text_.c_str();
  }

private:
  std::string exception_text_;
};
}
#endif

#include "rv/XmlRpcSource.h"
#include "rv/XmlRpcSocket.h"
#include "XmlRpcUtil.h"

namespace rv {


  XmlRpcSource::XmlRpcSource(int fd /*= -1*/, bool deleteOnClose /*= false*/) 
    : _fd(fd), _deleteOnClose(deleteOnClose), _keepOpen(false)
  {
  }

  XmlRpcSource::~XmlRpcSource()
  {
  }


  void
  XmlRpcSource::close()
  {
    if (_fd != -1) {
      XmlRpc::XmlRpcUtil::log(2,"XmlRpcSource::close: closing socket %d.", _fd);
      XmlRpcSocket::close(_fd);
      XmlRpc::XmlRpcUtil::log(2,"XmlRpcSource::close: done closing socket %d.", _fd);
      _fd = -1;
    }
    if (_deleteOnClose) {
      XmlRpc::XmlRpcUtil::log(2,"XmlRpcSource::close: deleting this");
      _deleteOnClose = false;
      delete this;
    }
  }

} // namespace rv


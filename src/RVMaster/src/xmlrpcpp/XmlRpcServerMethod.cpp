
#include "rv/XmlRpcServerMethod.h"
#include "rv/XmlRpcServer.h"

namespace rv {


  XmlRpcServerMethod2::XmlRpcServerMethod2(std::string const& name, XmlRpcServer* server)
  {
    _name = name;
    _server = server;
    if (_server) _server->addMethod(this);
  }

  XmlRpcServerMethod2::~XmlRpcServerMethod2()
  {
    if (_server) _server->removeMethod(this);
  }


} // namespace XmlRpc


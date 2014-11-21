
#ifndef RVCPP_XMLRPCSERVERMETHOD2_H_
#define RVCPP_XMLRPCSERVERMETHOD2_H_



#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#include "XmlRpcDecl.h"
#include "rv/callInfo.h"
#include "XmlRpcValue.h"

#ifndef MAKEDEPEND
# include <string>
#endif

namespace rv {

  // Representation of a parameter or result value

  // The XmlRpcServer processes client requests to call RPCs
  class XmlRpcServer;

  //! Abstract class representing a single RPC method
  class XMLRPCPP_DECL XmlRpcServerMethod2 {
  public:
    //! Constructor
    XmlRpcServerMethod2(std::string const& name, XmlRpcServer* server = 0);
    //! Destructor
    virtual ~XmlRpcServerMethod2();

    //! Returns the name of the method
    std::string& name() { return _name; }

    //! Execute the method. Subclasses must provide a definition for this method.
    virtual void execute(XmlRpc::XmlRpcValue& params, rv::ClientInfo &ci, XmlRpc::XmlRpcValue& result) = 0;

    //! Returns a help string for the method.
    //! Subclasses should define this method if introspection is being used.
    virtual std::string help() { return std::string(); }

  protected:
    std::string _name;
    XmlRpcServer* _server;
  };
} // namespace XmlRpc

#endif // _XMLRPCSERVERMETHOD2_H_


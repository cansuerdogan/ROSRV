#ifndef RVCPP_CALLINFO_H
#define RVCPP_CALLINFO_H

#include <string>

namespace rv
{

struct ClientInfo
{
short family;
unsigned short port;
std::string ip;

};

}

#endif

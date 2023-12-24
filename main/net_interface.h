#ifndef __NET_INTERFACE_H__
#define __NET_INTERFACE_H__

#define net_iface_init(iface) void net_iface_##iface##_init(void)

net_iface_init(wifi);
#endif
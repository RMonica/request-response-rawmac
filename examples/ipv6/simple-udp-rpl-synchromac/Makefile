all: broadcast-example unicast-sender unicast-receiver request-response-server request-response-client
APPS=servreg-hack
CONTIKI=../../..

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS+= -DUIP_CONF_IPV6_RPL

include $(CONTIKI)/Makefile.include

// net.h - WiFi setup and the waypoint server client.

#ifndef NET_H
#define NET_H

#include "config.h"

// Joins the access point. Returns false if it does not associate or get an
// address within WIFI_MAX_ATTEMPTS. The original looped here forever, which
// made a typo in the SSID look the same as a dead access point.
bool netBegin();

bool netIsConnected();

// Reports arrival at a node and reads back the next destination.
//
// Returns true and writes to destinationOut only if the whole exchange worked:
// connected, response received before the timeout, and a valid node id in the
// body. On failure destinationOut is left alone so the caller can keep using
// the destination it already had.
//
// Retries up to HTTP_MAX_ATTEMPTS with a doubling delay. The original assumed
// every POST succeeded, so one dropped response left the robot with a
// destination the server never sent.
bool netReportArrival(uint8_t position, uint8_t *destinationOut);

#endif  // NET_H

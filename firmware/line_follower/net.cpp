#include "net.h"

#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <stdio.h>
#include <string.h>

#include "route.h"
#include "secrets.h"

namespace {

WiFiClient g_client;
bool g_connected = false;

// Energia's WiFi API takes char* rather than const char*, so the configured
// strings are copied into arrays instead of casting away const at each call.
char g_ssid[] = WIFI_SSID;
char g_password[] = WIFI_PASSWORD;
char g_server[] = SERVER_HOST;

// Reads until the server closes the socket or the timeout expires. Returns the
// number of bytes read.
//
// The original waited a fixed 1000 ms and then read whatever had arrived. That
// is slower than needed on a good link and not long enough on a busy one.
size_t readResponse(char *buffer, size_t capacity, uint16_t timeoutMs) {
  size_t length = 0;
  const uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (g_client.available() > 0 && length < capacity - 1) {
      buffer[length++] = (char)g_client.read();
    }
    if (length >= capacity - 1) {
      break;
    }
    if (!g_client.connected() && g_client.available() == 0) {
      break;  // server closed the connection, so the response is complete
    }
  }

  buffer[length] = '\0';
  return length;
}

// Pulls the node id out of the response body. Returns false if the headers are
// missing, the body is empty, or the number is not a node on this track.
bool parseDestination(const char *response, uint8_t *out) {
  const char *body = strstr(response, "\r\n\r\n");
  if (body == NULL) {
    return false;
  }
  body += 4;

  while (*body == ' ' || *body == '\r' || *body == '\n') {
    ++body;
  }
  if (*body < '0' || *body > '9') {
    return false;
  }

  int value = 0;
  while (*body >= '0' && *body <= '9') {
    value = (value * 10) + (*body - '0');
    if (value >= NODE_COUNT) {
      return false;  // not a node on this track
    }
    ++body;
  }

  *out = (uint8_t)value;
  return true;
}

bool attemptReport(uint8_t position, uint8_t *destinationOut) {
  if (!g_client.connect(g_server, SERVER_PORT)) {
    Serial.println("net: connect failed");
    return false;
  }

  char body[24];
  snprintf(body, sizeof(body), "position=%u", (unsigned)position);

  g_client.print("POST ");
  g_client.print(SERVER_ARRIVAL_PATH);
  g_client.print(" HTTP/1.1\r\n");
  g_client.print("Host: ");
  g_client.print(g_server);
  g_client.print("\r\n");
  g_client.print("Content-Type: application/x-www-form-urlencoded\r\n");
  g_client.print("Content-Length: ");
  g_client.print((unsigned int)strlen(body));
  g_client.print("\r\nConnection: close\r\n\r\n");
  g_client.print(body);

  char response[HTTP_BUFFER_SIZE];
  const size_t length =
      readResponse(response, sizeof(response), HTTP_RESPONSE_TIMEOUT_MS);
  g_client.stop();

  if (length == 0) {
    Serial.println("net: no response");
    return false;
  }

  uint8_t destination = 0;
  if (!parseDestination(response, &destination)) {
    Serial.println("net: unparseable response");
    return false;
  }

  *destinationOut = destination;
  return true;
}

}  // namespace

bool netBegin() {
  Serial.print("net: joining ");
  Serial.println(g_ssid);

  WiFi.begin(g_ssid, g_password);

  for (uint8_t attempt = 0; attempt < WIFI_MAX_ATTEMPTS; ++attempt) {
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    delay(WIFI_RETRY_DELAY_MS);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("net: association failed");
    return false;
  }

  for (uint8_t attempt = 0; attempt < WIFI_MAX_ATTEMPTS; ++attempt) {
    if (!(WiFi.localIP() == INADDR_NONE)) {
      break;
    }
    delay(WIFI_RETRY_DELAY_MS);
  }

  if (WiFi.localIP() == INADDR_NONE) {
    Serial.println("net: no address");
    return false;
  }

  Serial.print("net: address ");
  Serial.println(WiFi.localIP());
  g_connected = true;
  return true;
}

bool netIsConnected() {
  return g_connected && WiFi.status() == WL_CONNECTED;
}

bool netReportArrival(uint8_t position, uint8_t *destinationOut) {
  if (destinationOut == NULL) {
    return false;
  }

  uint16_t backoff = HTTP_RETRY_BASE_MS;

  for (uint8_t attempt = 1; attempt <= HTTP_MAX_ATTEMPTS; ++attempt) {
    Serial.print("net: reporting node ");
    Serial.print(position);
    Serial.print(" attempt ");
    Serial.println(attempt);

    if (attemptReport(position, destinationOut)) {
      Serial.print("net: next destination ");
      Serial.println(*destinationOut);
      return true;
    }

    if (attempt < HTTP_MAX_ATTEMPTS) {
      delay(backoff);
      backoff = (uint16_t)(backoff * 2);
    }
  }

  Serial.println("net: giving up, holding previous destination");
  return false;
}

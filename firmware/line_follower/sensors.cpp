#include "sensors.h"

#include "line_policy.h"

namespace {

// One moving average type, used once per channel. The original had five copies
// of this function that differed only in the name of the static buffer.
struct MovingAverage {
  uint16_t buffer[SENSOR_FILTER_LENGTH];
  uint8_t index;
  uint32_t sum;
  uint16_t count;
};

void filterReset(MovingAverage &filter) {
  for (uint8_t i = 0; i < SENSOR_FILTER_LENGTH; ++i) {
    filter.buffer[i] = 0;
  }
  filter.index = 0;
  filter.sum = 0;
  filter.count = 0;
}

uint16_t filterPush(MovingAverage &filter, uint16_t value) {
  filter.sum -= filter.buffer[filter.index];
  filter.sum += value;
  filter.buffer[filter.index] = value;
  filter.index = (uint8_t)((filter.index + 1) % SENSOR_FILTER_LENGTH);
  if (filter.count < SENSOR_FILTER_LENGTH) {
    ++filter.count;
  }
  // count is at least 1 here because it is incremented above.
  return (uint16_t)(filter.sum / filter.count);
}

const uint8_t kChannelPins[CHANNEL_COUNT] = {
    PIN_SENSOR_FAR_LEFT, PIN_SENSOR_LEFT, PIN_SENSOR_CENTRE, PIN_SENSOR_RIGHT,
    PIN_SENSOR_FAR_RIGHT};

MovingAverage g_channels[CHANNEL_COUNT];
uint16_t g_filtered[CHANNEL_COUNT];
MovingAverage g_distance;

}  // namespace

void sensorsBegin() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    pinMode(kChannelPins[i], INPUT);
    filterReset(g_channels[i]);
    g_filtered[i] = 0;
  }
  pinMode(PIN_DISTANCE, INPUT);
  filterReset(g_distance);
}

void sensorsUpdate() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    g_filtered[i] =
        filterPush(g_channels[i], (uint16_t)analogRead(kChannelPins[i]));
  }
}

uint16_t sensorsFiltered(SensorChannel channel) {
  if (channel >= CHANNEL_COUNT) {
    return 0;
  }
  return g_filtered[channel];
}

bool sensorsOnLine(SensorChannel channel) {
  // A shrouded TCRT5000 over white reflects more IR and pulls the output low.
  return sensorsFiltered(channel) < LINE_THRESHOLD;
}

uint8_t sensorsPattern() {
  return linePatternPack(
      sensorsOnLine(CHANNEL_FAR_LEFT), sensorsOnLine(CHANNEL_LEFT),
      sensorsOnLine(CHANNEL_CENTRE), sensorsOnLine(CHANNEL_RIGHT),
      sensorsOnLine(CHANNEL_FAR_RIGHT));
}

float sensorsDistanceCm() {
  const float volts = (float)analogRead(PIN_DISTANCE) * ADC_VOLTS_PER_COUNT;
  if (volts <= 0.0f) {
    return DISTANCE_MAX_CM;
  }

  const float centimetres = DISTANCE_CURVE_K / volts;
  if (centimetres < DISTANCE_MIN_CM) {
    return DISTANCE_MIN_CM;
  }
  if (centimetres > DISTANCE_MAX_CM) {
    return DISTANCE_MAX_CM;
  }
  return centimetres;
}

float sensorsDistanceAveragedCm(uint16_t windowMs) {
  const uint32_t start = millis();
  uint16_t averaged = 0;

  // Stored in tenths of a centimetre so the integer filter keeps enough
  // resolution over a 4 to 30 cm range.
  do {
    averaged = filterPush(g_distance, (uint16_t)(sensorsDistanceCm() * 10.0f));
  } while (millis() - start < windowMs);

  return (float)averaged / 10.0f;
}

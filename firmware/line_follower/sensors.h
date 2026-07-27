// sensors.h - reads the reflective array and the rangefinder.

#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

typedef enum {
  CHANNEL_FAR_LEFT = 0,
  CHANNEL_LEFT,
  CHANNEL_CENTRE,
  CHANNEL_RIGHT,
  CHANNEL_FAR_RIGHT,
  CHANNEL_COUNT
} SensorChannel;

// Sets up the ADC pins and clears the filters.
void sensorsBegin();

// Samples all five channels and updates their moving averages. Cheap enough to
// call every pass of the main loop. The control loop reads the filtered values
// on its own slower tick.
void sensorsUpdate();

// Filtered ADC counts for one channel.
uint16_t sensorsFiltered(SensorChannel channel);

// True if the channel is over the white line.
bool sensorsOnLine(SensorChannel channel);

// The five channels packed into the 5 bit pattern used by line_policy.h.
uint8_t sensorsPattern();

// One rangefinder reading in centimetres, clamped to the 4 to 30 cm range the
// datasheet specifies.
float sensorsDistanceCm();

// Averages rangefinder readings over windowMs. A single reading from the
// GP2Y0A41SK0F can be several centimetres out, which is the difference between
// stopping in front of the wall and hitting it.
float sensorsDistanceAveragedCm(uint16_t windowMs);

#endif  // SENSORS_H

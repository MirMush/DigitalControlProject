#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stddef.h>

struct LogData
{
  float time;
  int state;
  float desiredValue;
  float motorVel[2];
  float motorVolt[2];
  int32_t encoder[2];
  float motorCurrent[2];
  float battery;
  float poseX;
  float poseY;
  float heading;
  float distA;
  float forwardVel;
  float turnRate;
  float gyroX;
  float gyroY;
  float gyroZ;
  float desiredTurnRate;
};

class Logger
{
public:
  Logger(size_t bufferBytes, uint32_t sampleDivider);
  ~Logger();

  void startRun(
      const char *robotName,
      uint32_t sampleTimeUs,
      uint32_t logIntervalUs,
      float gear,
      float wheelRadius,
      int pulsesPerRevolution);
  bool shouldLog(bool active);
  void record(const LogData &data);
  void finish();
  void printAll(
      const char *robotName,
      uint32_t sampleTimeUs,
      uint32_t logIntervalUs,
      float gear,
      float wheelRadius,
      int pulsesPerRevolution);

private:
  void printHeader(
      const char *robotName,
      uint32_t sampleTimeUs,
      uint32_t logIntervalUs,
      float gear,
      float wheelRadius,
      int pulsesPerRevolution);
  void printRow(const LogData &data);

  LogData *records;
  size_t capacity;
  size_t recordCount;
  size_t streamedCount;
  uint32_t sampleDivider;
  uint32_t sampleCounter;
  bool bufferFullReported;
};

#endif
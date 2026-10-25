#include "logger.h"

#include <stdlib.h>
#include <Arduino.h>

Logger::Logger(size_t bufferBytes, uint32_t sampleDivider)
    : records(static_cast<LogData *>(malloc(bufferBytes))),
      capacity(bufferBytes / sizeof(LogData)),
      recordCount(0),
      streamedCount(0),
      sampleDivider(sampleDivider == 0 ? 1 : sampleDivider),
      sampleCounter(0),
      bufferFullReported(false)
{
}

Logger::~Logger()
{
  free(records);
}

void Logger::printHeader(
    const char *robotName,
    uint32_t sampleTimeUs,
    uint32_t logIntervalUs,
    float gear,
    float wheelRadius,
    int pulsesPerRevolution)
{
  Serial.print("% Basebot log for ");
  Serial.println(robotName);
  Serial.print("% Sample time "); Serial.print(sampleTimeUs);
  Serial.print(" us, log interval "); Serial.print(logIntervalUs);
  Serial.print(" us, Gear "); Serial.print(gear);
  Serial.print(", Wheels "); Serial.print(wheelRadius);
  Serial.print(" m, PPR "); Serial.println(pulsesPerRevolution);
  Serial.print("% maximum log samples "); Serial.print(capacity);
  Serial.print(" each sized "); Serial.print(sizeof(LogData)); Serial.println(" bytes");
  Serial.println("% 1 time (sec)");
  Serial.println("% 2 state");
  Serial.println("% 3 desired linear velocity (m/s)");
  Serial.println("% 4-5 motor voltage (left,right) (V)");
  Serial.println("% 6-7 Encoder count (left, right)");
  Serial.println("% 8-9 motor vel (left, right) (rad/s)");
  Serial.println("% 10  Battery voltage (V)");
  Serial.println("% 11-13 gyro x,y,z (deg/s)");
  Serial.println("% 14-15 motor current (left, right) (A)");
  Serial.println("% 16-17 pose x,y (m)");
  Serial.println("% 18 heading (rad)");
  Serial.println("% 19 distance (m)");
  Serial.println("% 20 forward velocity (m/s)");
  Serial.println("% 21 turn rate (rad/s)");
  Serial.println("% 22-23 filtered motor vel (left, right) (rad/s)");
}

void Logger::printRow(const LogData &d)
{
  Serial.print(d.time, 4);
  Serial.print(" "); Serial.print(d.state);
  Serial.print(" "); Serial.print(d.desiredValue, 2);
  Serial.print(" "); Serial.print(d.motorVolt[0], 2);
  Serial.print(" "); Serial.print(d.motorVolt[1], 2);
  Serial.print(" "); Serial.print(d.encoder[0]);
  Serial.print(" "); Serial.print(d.encoder[1]);
  Serial.print(" "); Serial.print(d.motorVel[0], 2);
  Serial.print(" "); Serial.print(d.motorVel[1], 2);
  Serial.print(" "); Serial.print(d.battery, 2);
  Serial.print(" "); Serial.print(d.gyroX, 2);
  Serial.print(" "); Serial.print(d.gyroY, 2);
  Serial.print(" "); Serial.print(d.gyroZ, 2);
  Serial.print(" "); Serial.print(d.motorCurrent[0], 2);
  Serial.print(" "); Serial.print(d.motorCurrent[1], 2);
  Serial.print(" "); Serial.print(d.poseX, 2);
  Serial.print(" "); Serial.print(d.poseY, 2);
  Serial.print(" "); Serial.print(d.heading, 2);
  Serial.print(" "); Serial.print(d.distA, 2);
  Serial.print(" "); Serial.print(d.forwardVel, 2);
  Serial.print(" "); Serial.print(d.turnRate, 2);
  Serial.print(" "); Serial.print(d.filteredMotorVel[0], 2);
  Serial.print(" "); Serial.print(d.filteredMotorVel[1], 2);
  Serial.println();
}

void Logger::startRun(
    const char *robotName,
    uint32_t sampleTimeUs,
    uint32_t logIntervalUs,
    float gear,
    float wheelRadius,
    int pulsesPerRevolution)
{
  recordCount = 0;
  streamedCount = 0;
  sampleCounter = 0;
  bufferFullReported = false;
  if (records == nullptr)
  {
    Serial.println("% ERROR: unable to allocate log buffer");
    return;
  }
  printHeader(robotName, sampleTimeUs, logIntervalUs, gear, wheelRadius, pulsesPerRevolution);
}

bool Logger::shouldLog(bool active)
{
  if (!active)
    return false;
  if (++sampleCounter < sampleDivider)
    return false;
  sampleCounter = 0;
  return true;
}

void Logger::record(const LogData &data)
{
  if (records == nullptr)
    return;

  if (recordCount < capacity)
    records[recordCount++] = data;
  else if (!bufferFullReported)
  {
    Serial.println("% ERROR: log buffer full; continuing to stream without buffering");
    bufferFullReported = true;
  }

  printRow(data);
  ++streamedCount;
}

void Logger::finish()
{
  Serial.print("% sent ");
  Serial.print(streamedCount);
  Serial.println(" log lines");
}

void Logger::printAll(
    const char *robotName,
    uint32_t sampleTimeUs,
    uint32_t logIntervalUs,
    float gear,
    float wheelRadius,
    int pulsesPerRevolution)
{
  printHeader(robotName, sampleTimeUs, logIntervalUs, gear, wheelRadius, pulsesPerRevolution);
  if (records == nullptr)
  {
    Serial.println("% ERROR: log buffer is unavailable");
    return;
  }

  for (size_t i = 0; i < recordCount; ++i)
    printRow(records[i]);

  Serial.print("% sent ");
  Serial.print(recordCount);
  Serial.println(" log lines");
}
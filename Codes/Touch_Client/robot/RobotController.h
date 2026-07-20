#pragma once

bool sendCoordinate(double deltaX, double deltaY, double deltaZ);
bool initRobot();
void shutdownRobot();
void checkAlarmPeriodically();
bool clearAlarm();

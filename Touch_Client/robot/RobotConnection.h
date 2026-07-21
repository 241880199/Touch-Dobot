#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>

bool robotConnect(const char* ip);
void robotDisconnect();
bool robotSendEnable(const char* cmd);
bool robotSendMotion(const char* cmd);
bool robotRecvMotion(char* buf, int len);
bool robotRecvEnable(char* buf, int len);
bool isRobotConnected();

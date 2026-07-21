#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>

bool connectToRelay();
void disconnectRelay();
bool sendToRelay(int targetPort, const char* cmd);
bool readFeedback(char* outBuf, int outLen, DWORD timeoutMs);
DWORD WINAPI tcpClientThreadProc(LPVOID param);
void startTcpClient();
void stopTcpClient();

#pragma once
#include <windows.h>

void startSenderThread();
void stopSenderThread();
DWORD WINAPI senderThreadProc(LPVOID param);

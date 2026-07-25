#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>

bool robotConnect(const char* ip);
void robotDisconnect();
bool robotSendEnable(const char* cmd);
bool robotSendMotion(const char* cmd);
bool robotRecvMotion(char* buf, int len);
bool robotRecvEnable(char* buf, int len);
// 非阻塞版本 (pollFeedback 使用, select 0-timeout, 不阻塞渲染)
bool robotRecvMotionPoll(char* buf, int len);
bool robotRecvEnablePoll(char* buf, int len);
bool isRobotConnected();
// 排空使能端口缓冲区 (丢弃所有残留数据)
void robotDrainEnable();
// 力传感器实时反馈 (30004, 125Hz binary)
bool robotConnectRealtime(const char* ip);
bool robotRecvRealtime(char* buf, int len);
void robotCloseRealtime();

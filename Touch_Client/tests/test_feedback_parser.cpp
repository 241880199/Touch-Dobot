// Standalone test: FeedbackParser — protocol parsing + error code mapping
// Build: build_feedback_parser_test.bat
// Run: test_feedback_parser.exe

#include <iostream>
#include <cstring>

// Real headers (need OpenHaptics SDK include paths — same as other tests)
#include "../core/AppState.h"
#include "../safety/RobotError.h"
#include "../relay/FeedbackParser.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

// ===== isSuccess =====
static void test_isSuccess_ok() {
    TEST(isSuccess_ok);
    CHECK(FeedbackParser::isSuccess("0,{100,200,300,0,0,0},GetPose();"));
    PASS();
}

static void test_isSuccess_error() {
    TEST(isSuccess_error);
    CHECK(!FeedbackParser::isSuccess("-1,{0x0002},ServoP();"));
    PASS();
}

static void test_isSuccess_null() {
    TEST(isSuccess_null);
    CHECK(!FeedbackParser::isSuccess(nullptr));
    PASS();
}

static void test_isSuccess_empty() {
    TEST(isSuccess_empty);
    CHECK(!FeedbackParser::isSuccess(""));
    PASS();
}

// ===== extractData =====
static void test_extractData_normal() {
    TEST(extractData_normal);
    char buf[256];
    CHECK(FeedbackParser::extractData("0,{100,200,300,0,0,0},GetPose();", buf, 256));
    CHECK(strcmp(buf, "100,200,300,0,0,0") == 0);
    PASS();
}

static void test_extractData_single() {
    TEST(extractData_single);
    char buf[64];
    CHECK(FeedbackParser::extractData("0,{9},RobotMode();", buf, 64));
    CHECK(strcmp(buf, "9") == 0);
    PASS();
}

static void test_extractData_no_braces() {
    TEST(extractData_no_braces);
    char buf[64];
    CHECK(!FeedbackParser::extractData("0123456789,ServoP();", buf, 64));
    PASS();
}

static void test_extractData_empty_braces() {
    TEST(extractData_empty_braces);
    char buf[64];
    CHECK(!FeedbackParser::extractData("0,{},ServoP();", buf, 64));
    PASS();
}

static void test_extractData_null() {
    TEST(extractData_null);
    char buf[64];
    CHECK(!FeedbackParser::extractData(nullptr, buf, 64));
    PASS();
}

// ===== parsePose =====
static void test_parsePose_valid() {
    TEST(parsePose_valid);
    AppState::RobotPose pose;
    CHECK(FeedbackParser::parsePose("0,{123.4,567.8,90.1,2.3,4.5,6.7},GetPose();", pose));
    CHECK(pose.x == 123.4);
    CHECK(pose.y == 567.8);
    CHECK(pose.z == 90.1);
    CHECK(pose.rx == 2.3);
    CHECK(pose.ry == 4.5);
    CHECK(pose.rz == 6.7);
    PASS();
}

static void test_parsePose_negative() {
    TEST(parsePose_negative);
    AppState::RobotPose pose;
    CHECK(FeedbackParser::parsePose("0,{-100,-200,-300,-1,-2,-3},GetPose();", pose));
    CHECK(pose.x == -100.0);
    CHECK(pose.y == -200.0);
    CHECK(pose.z == -300.0);
    CHECK(pose.rx == -1.0);
    CHECK(pose.ry == -2.0);
    CHECK(pose.rz == -3.0);
    PASS();
}

static void test_parsePose_invalid() {
    TEST(parsePose_invalid);
    AppState::RobotPose pose;
    CHECK(!FeedbackParser::parsePose("0,{1,2,3},GetPose();", pose));
    PASS();
}

static void test_parsePose_no_braces() {
    TEST(parsePose_no_braces);
    AppState::RobotPose pose;
    CHECK(!FeedbackParser::parsePose("not a valid response", pose));
    PASS();
}

// ===== parseAngle =====
static void test_parseAngle_valid() {
    TEST(parseAngle_valid);
    double angles[6] = {0};
    CHECK(FeedbackParser::parseAngle("0,{10.5,20.5,30.5,40.5,50.5,60.5},GetAngle();", angles));
    CHECK(angles[0] == 10.5);
    CHECK(angles[1] == 20.5);
    CHECK(angles[2] == 30.5);
    CHECK(angles[3] == 40.5);
    CHECK(angles[4] == 50.5);
    CHECK(angles[5] == 60.5);
    PASS();
}

static void test_parseAngle_invalid() {
    TEST(parseAngle_invalid);
    double angles[6] = {0};
    CHECK(!FeedbackParser::parseAngle("0,{1,2,3},GetAngle();", angles));
    PASS();
}

// ===== parseMode =====
static void test_parseMode_valid() {
    TEST(parseMode_valid);
    int mode = 0;
    CHECK(FeedbackParser::parseMode("0,{9},RobotMode();", mode));
    CHECK(mode == 9);
    PASS();
}

static void test_parseMode_invalid() {
    TEST(parseMode_invalid);
    int mode = 0;
    CHECK(!FeedbackParser::parseMode("not valid", mode));
    PASS();
}

// ===== extractErrorCode =====
static void test_extractErrorCode_hex() {
    TEST(extractErrorCode_hex);
    int code = 0;
    CHECK(FeedbackParser::extractErrorCode("-1,{0x0002},ServoP();", code));
    CHECK(code == 0x0002);
    PASS();
}

static void test_extractErrorCode_decimal() {
    TEST(extractErrorCode_decimal);
    int code = 0;
    CHECK(FeedbackParser::extractErrorCode("-1,{42},ServoP();", code));
    CHECK(code == 42);
    PASS();
}

static void test_extractErrorCode_success() {
    TEST(extractErrorCode_success);
    int code = -1;
    CHECK(FeedbackParser::extractErrorCode("0,{},ServoP();", code));
    CHECK(code == 0);
    PASS();
}

static void test_extractErrorCode_null() {
    TEST(extractErrorCode_null);
    int code = 0;
    CHECK(!FeedbackParser::extractErrorCode(nullptr, code));
    PASS();
}

// ===== mapRobotErrorCode =====
static void test_mapErrorCode_workspace() {
    TEST(mapErrorCode_workspace);
    CHECK(FeedbackParser::mapRobotErrorCode(0x0001) == RobotErrorCode::ERR_WORKSPACE_RADIUS);
    PASS();
}

static void test_mapErrorCode_jointlimit() {
    TEST(mapErrorCode_jointlimit);
    CHECK(FeedbackParser::mapRobotErrorCode(0x0002) == RobotErrorCode::ERR_JOINTLIMIT_EXCEED);
    PASS();
}

static void test_mapErrorCode_singular() {
    TEST(mapErrorCode_singular);
    CHECK(FeedbackParser::mapRobotErrorCode(0x0010) == RobotErrorCode::ERR_IK_SINGULAR);
    PASS();
}

static void test_mapErrorCode_collision() {
    TEST(mapErrorCode_collision);
    CHECK(FeedbackParser::mapRobotErrorCode(0x0020) == RobotErrorCode::ERR_COLLISION);
    PASS();
}

static void test_mapErrorCode_unknown() {
    TEST(mapErrorCode_unknown);
    CHECK(FeedbackParser::mapRobotErrorCode(0xFFFF) == RobotErrorCode::ERR_SERVOP_REJECTED);
    PASS();
}

// ===== Edge cases =====
static void test_error_response_hex_format() {
    TEST(error_response_hex_format);
    int code = 0;
    CHECK(FeedbackParser::extractErrorCode("-1,{0x0001},MovJ();", code));
    CHECK(code == 0x0001);
    PASS();
}

static void test_newline_in_response() {
    TEST(newline_in_response);
    AppState::RobotPose pose;
    bool ok = FeedbackParser::parsePose("0,{100,200,300,0,0,0},GetPose();\r\n", pose);
    CHECK(ok);
    CHECK(pose.x == 100.0);
    PASS();
}

// ===== MAIN =====
int main() {
    std::cout << "=== FeedbackParser Tests ===" << std::endl;

    test_isSuccess_ok();
    test_isSuccess_error();
    test_isSuccess_null();
    test_isSuccess_empty();

    test_extractData_normal();
    test_extractData_single();
    test_extractData_no_braces();
    test_extractData_empty_braces();
    test_extractData_null();

    test_parsePose_valid();
    test_parsePose_negative();
    test_parsePose_invalid();
    test_parsePose_no_braces();

    test_parseAngle_valid();
    test_parseAngle_invalid();

    test_parseMode_valid();
    test_parseMode_invalid();

    test_extractErrorCode_hex();
    test_extractErrorCode_decimal();
    test_extractErrorCode_success();
    test_extractErrorCode_null();

    test_mapErrorCode_workspace();
    test_mapErrorCode_jointlimit();
    test_mapErrorCode_singular();
    test_mapErrorCode_collision();
    test_mapErrorCode_unknown();

    test_error_response_hex_format();
    test_newline_in_response();

    std::cout << std::endl;
    std::cout << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed > 0 ? 1 : 0;
}

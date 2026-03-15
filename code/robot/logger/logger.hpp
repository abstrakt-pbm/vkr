#pragma once

#define ENABLE_TEST_LOGS

#include <cstdio>

#ifdef ENABLE_TEST_LOGS
    #define LOG_INFO(fmt, ...)   std::printf("[INFO]  " fmt "\n", ##__VA_ARGS__)
    #define LOG_WARN(fmt, ...)   std::printf("[WARN]  " fmt "\n", ##__VA_ARGS__)
    #define LOG_ERROR(fmt, ...)  std::printf("[ERROR] " fmt "\n", ##__VA_ARGS__)
#else
    #define LOG_INFO(fmt, ...)   
    #define LOG_WARN(fmt, ...)   
    #define LOG_ERROR(fmt, ...)  std::printf("[ERROR] " fmt "\n", ##__VA_ARGS__)
#endif


/**
* @file test_main.cpp
* @author Raghav Sood (raghav2sood@gmail.com)
* @brief Main file for running all tests
**/

#include <gtest/gtest.h>
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#include <gtest/gtest.h>

#include "bridge.h"
#include "slam.h"

#include <QApplication>

namespace{
    TEST(SlamSystemThreadTest, AssignsAndStopsIndependentWorkerThreads){
        int argc = 1;
        char application_name[] = "test_cpp_2d_slam";
        char* argv[] = {application_name, nullptr};
        QApplication application(argc, argv);
        Bridge bridge;

        {
            rcl_slam::SlamSystem slam_system(&bridge);
            EXPECT_TRUE(slam_system.hasExpectedThreadAffinity());
        }

        SUCCEED();
    }
}

#include <gtest/gtest.h>

#include "my_pose_graph.h"

#include <cmath>
#include <limits>

namespace{
    using rcl_pose_graph::PoseGraph;
    using rcl_pose_graph_type::Edge;
    using rcl_pose_graph_type::Node;

    TEST(PoseGraphTest, RejectsNonFinitePoses){
        PoseGraph graph;
        graph.addPose(0.0, 0.0, 0.0);
        graph.addPose(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0);

        EXPECT_EQ(graph.getPoseCount(), 1U);

        graph.setPose(0, Node{std::numeric_limits<double>::infinity(), 0.0, 0.0});
        const auto pose = graph.getPose(0);
        EXPECT_DOUBLE_EQ(pose.tx, 0.0);
        EXPECT_DOUBLE_EQ(pose.ty, 0.0);
        EXPECT_DOUBLE_EQ(pose.theta, 0.0);
    }

    TEST(PoseGraphTest, RejectsInvalidEdgesBeforeOptimization){
        PoseGraph graph;
        graph.addPose(0.0, 0.0, 0.0);
        graph.addPose(1.0, 0.0, 0.0);

        Edge out_of_range(0, 4, 1.0, 1.0, 1.0, true);
        out_of_range.set_relative_pose(Node{1.0, 0.0, 0.0});
        graph.addEdge(out_of_range);

        EXPECT_FALSE(graph.loopOptimize());
        EXPECT_FALSE(graph.errorCompute(out_of_range).allFinite());

        const auto poses = graph.getPoseSnapshot();
        ASSERT_EQ(poses.size(), 2U);
        EXPECT_TRUE(std::isfinite(poses[0].tx));
        EXPECT_TRUE(std::isfinite(poses[1].tx));
    }

    TEST(PoseGraphTest, OptimizesFiniteConstraintAndKeepsAnchorFixed){
        PoseGraph graph;
        graph.addPose(0.0, 0.0, 0.0);
        graph.addPose(2.0, 0.0, 0.0);

        Edge edge(0, 1, 100.0, 100.0, 100.0, true);
        edge.set_relative_pose(Node{1.0, 0.0, 0.0});
        graph.addEdge(edge);

        ASSERT_TRUE(graph.loopOptimize(20, 1e-8));
        const auto poses = graph.getPoseSnapshot();
        ASSERT_EQ(poses.size(), 2U);
        EXPECT_NEAR(poses[0].tx, 0.0, 1e-4);
        EXPECT_NEAR(poses[0].ty, 0.0, 1e-4);
        EXPECT_NEAR(poses[1].tx, 1.0, 1e-3);
        EXPECT_TRUE(std::isfinite(poses[1].ty));
        EXPECT_TRUE(std::isfinite(poses[1].theta));
    }

    TEST(PoseGraphTest, PreservesPosesWhenParametersAreInvalid){
        PoseGraph graph;
        graph.addPose(0.0, 0.0, 0.0);
        graph.addPose(1.0, 0.0, 0.0);
        graph.addEdge(0, 1, 1.0, 1.0, 1.0);
        const auto before = graph.getPoseSnapshot();

        EXPECT_FALSE(graph.loopOptimize(0, 1e-6));
        EXPECT_FALSE(graph.loopOptimize(20, std::numeric_limits<double>::quiet_NaN()));

        const auto after = graph.getPoseSnapshot();
        ASSERT_EQ(after.size(), before.size());
        for(std::size_t i = 0; i < before.size(); ++i){
            EXPECT_DOUBLE_EQ(after[i].tx, before[i].tx);
            EXPECT_DOUBLE_EQ(after[i].ty, before[i].ty);
            EXPECT_DOUBLE_EQ(after[i].theta, before[i].theta);
        }
    }

    TEST(PoseGraphTest, RobustLoopDoesNotOverwhelmOdometryChain){
        PoseGraph graph;
        graph.addPose(0.0, 0.0, 0.0);
        graph.addPose(1.0, 0.0, 0.0);
        graph.addPose(2.0, 0.0, 0.0);

        Edge odom_01(0, 1, 25.0, 25.0, 50.0, false);
        odom_01.set_relative_pose(Node{1.0, 0.0, 0.0});
        graph.addEdge(odom_01);
        Edge odom_12(1, 2, 25.0, 25.0, 50.0, false);
        odom_12.set_relative_pose(Node{1.0, 0.0, 0.0});
        graph.addEdge(odom_12);

        Edge false_loop(0, 2, 30.0, 30.0, 60.0, true);
        false_loop.set_relative_pose(Node{20.0, 0.0, 0.0});
        graph.addEdge(false_loop);

        ASSERT_TRUE(graph.loopOptimize(20, 1e-8));
        const auto poses = graph.getPoseSnapshot();
        ASSERT_EQ(poses.size(), 3U);
        EXPECT_LT(poses[2].tx, 5.0);
        EXPECT_GT(poses[2].tx, 1.5);
    }
}

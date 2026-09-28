#include <gtest/gtest.h>

#include "latest_scan_mailbox.h"

namespace{
    TEST(LatestScanMailboxTest, CoalescesPendingScansAndRestartsAfterCatchUp){
        rcl_scan_match_backend::LatestScanMailbox mailbox;

        EXPECT_TRUE(mailbox.submit(ScanAxis{1.0}, ScanAxis{10.0}));
        EXPECT_FALSE(mailbox.submit(ScanAxis{2.0}, ScanAxis{20.0}));
        EXPECT_FALSE(mailbox.submit(ScanAxis{3.0}, ScanAxis{30.0}));

        ScanAxis xs;
        ScanAxis ys;
        ASSERT_TRUE(mailbox.takeLatest(xs, ys));
        EXPECT_EQ(xs, ScanAxis({3.0}));
        EXPECT_EQ(ys, ScanAxis({30.0}));

        // A scan arriving during processing requests exactly one follow-up.
        EXPECT_FALSE(mailbox.submit(ScanAxis{4.0}, ScanAxis{40.0}));
        EXPECT_TRUE(mailbox.completeProcessing());
        ASSERT_TRUE(mailbox.takeLatest(xs, ys));
        EXPECT_EQ(xs, ScanAxis({4.0}));
        EXPECT_EQ(ys, ScanAxis({40.0}));

        // Once the consumer catches up, the next producer schedules work.
        EXPECT_FALSE(mailbox.completeProcessing());
        EXPECT_TRUE(mailbox.submit(ScanAxis{5.0}, ScanAxis{50.0}));
    }
}

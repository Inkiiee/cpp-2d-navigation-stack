#ifndef CPP_2D_SLAM_LATEST_SCAN_MAILBOX_H
#define CPP_2D_SLAM_LATEST_SCAN_MAILBOX_H

#include "bridge.h"

#include <mutex>
#include <utility>

namespace rcl_scan_match_backend{
    // Keeps at most one pending scan. Producers overwrite stale data while a
    // backend task is scheduled or running, so the Qt event queue cannot grow
    // with one heavy scan-matching task per sensor message.
    class LatestScanMailbox{
    private:
        std::mutex mutex_;
        ScanAxis pending_x_;
        ScanAxis pending_y_;
        bool has_pending_ = false;
        bool task_scheduled_ = false;

    public:
        bool submit(const ScanAxis& xs, const ScanAxis& ys){
            std::lock_guard<std::mutex> lock(mutex_);
            pending_x_ = xs;
            pending_y_ = ys;
            has_pending_ = true;

            if(task_scheduled_){
                return false;
            }

            task_scheduled_ = true;
            return true;
        }

        bool takeLatest(ScanAxis& xs, ScanAxis& ys){
            std::lock_guard<std::mutex> lock(mutex_);
            if(!has_pending_){
                return false;
            }

            xs = std::move(pending_x_);
            ys = std::move(pending_y_);
            has_pending_ = false;
            return true;
        }

        // Called after one backend task finishes. Keeping task_scheduled_ set
        // while pending data exists closes the producer/consumer race: either
        // this caller schedules the follow-up task, or a later producer does.
        bool completeProcessing(){
            std::lock_guard<std::mutex> lock(mutex_);
            if(has_pending_){
                return true;
            }

            task_scheduled_ = false;
            return false;
        }
    };
}

#endif

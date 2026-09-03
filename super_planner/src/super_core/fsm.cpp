#include <cmath>
using std::isnan;
/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fsm/fsm.h>
#include <memory>

using namespace super_utils;

namespace fsm {
    Fsm::~Fsm() {
        write_time_.close();
    }

    void Fsm::WriteTimeToLog() {
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    void Fsm::callReplanOnce() {
        if (stop) {
            return;
        }

        if (machine_state_ != FOLLOW_TRAJ) {
            return;
        }

        if (finish_plan) {
            return;
        }

        // Static-goal settle (works with continuous_following too). Orbit
        // carrots stay ~lookahead away so this only fires at a real endpoint.
        if (arrivedAtGoal()) {
            finish_plan = true;
            gi_.new_goal = false;
            ChangeState("ReplanTimerCallback", WAIT_GOAL);
            cout << GREEN
                 << " -- [Fsm] Arrived at goal (settle) — stop continuous replan."
                 << RESET << endl;
            return;
        }

        if (plan_from_rest_) {
            plan_from_rest_ = false;
            return;
        }

        planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, gi_.goal_p, gi_.goal_p, 3.0);

        TimeConsuming replan_once_time("replan_once_time", false);

        RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        if (ret_code == FAILED) {
            onPlanFailure("ReplanOnce");
        } else if (ret_code == SUCCESS || ret_code == FINISH) {
            onPlanSuccess();
            cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl;
        }

        if (ret_code == EMER) {
            ChangeState("ReplanTimerCallback", EMER_STOP);
        } else if (ret_code == NEW_TRAJ) {
            ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
        } else if (ret_code == SUCCESS || ret_code == FINISH) {
            gi_.new_goal = false;
            // FINISH near a static goal: don't keep feeding OMMPC tiny loops.
            if (ret_code == FINISH && arrivedAtGoal()) {
                finish_plan = true;
                ChangeState("ReplanTimerCallback", WAIT_GOAL);
            } else {
                publishPolyTraj();
            }
        }

        planner_ptr_->getModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        // save on log
        replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
        WriteTimeToLog();
    }

    void Fsm::callMainFsmOnce() {
        if (stop) {
            return;
        }
        static double fsm_start_time = ros_ptr_->getSimTime();
        double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);
        static double last_print_t = 0.0;
        planner_ptr_->getRobotState(robot_state_);


        if (cur_t - last_print_t > 1.0) {
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                return;
            }
            if (!started_) {
                cout << YELLOW << " -- [Fsm] Wait for goal." << RESET << endl;
            }
            cout << std::fixed << std::setprecision(3);
            cout << GREEN << " -- [Fsm " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_]
                 << RESET << endl;
        }

        switch (machine_state_) {
            case INIT: {
                if (!started_) {
                    return;
                }
                if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            case WAIT_GOAL: {
                if (!gi_.new_goal) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                resetVisualizedPath();
                break;
            }
            case GENERATE_TRAJ: {
                // Settle on arrival even when continuous_following is on
                // (otherwise PlanFromRest keeps minting short non-zero-terminal
                // trajectories and the drone oscillates at the goal).
                if (arrivedAtGoal() || (!cfg_.continuous_following && closeToGoal(0.1))) {
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    finish_plan = true;
                    cout << GREEN
                         << " -- [Fsm] GENERATE_TRAJ: already at goal, not planning."
                         << RESET << endl;
                    return;
                }
                int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
                if (!planner_ptr_->goalValid()) {
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    return;
                }
                if (retcode == SUCCESS || retcode == FINISH) {
                    onPlanSuccess();
                    gi_.new_goal = false;
                    plan_from_rest_ = true;
                    finish_plan = false;
                    if (retcode == FINISH) {
                        finish_plan = true;
                    }

                    publishPolyTraj();

                    ChangeState("MainFsmCallback", FOLLOW_TRAJ);
                } else {
                    cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan." << RESET << endl;
                    onPlanFailure("PlanFromRest");
                }
                replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
                break;
            }
            case FOLLOW_TRAJ: {
                publishCurPoseToPath();
                break;
            }
            case EMER_STOP: {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
        }
    }

    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    bool Fsm::arrivedAtGoal() {
        // Distance + speed: need both so a fast fly-through of a moving carrot
        // (orbit) does not falsely "arrive", while a static endpoint settles.
        const double dis = (robot_state_.p - gi_.goal_p).norm();
        const double speed = robot_state_.v.norm();
        return dis < cfg_.goal_arrive_dis && speed < cfg_.goal_arrive_vel;
    }

    void Fsm::onPlanSuccess() {
        plan_fail_streak_ = 0;
        if (frontend_relaxed_) {
            planner_ptr_->setFrontendInKnownFree(frontend_known_free_default_);
            frontend_relaxed_ = false;
            cout << GREEN
                 << " -- [Fsm] Plan ok — restored frontend_in_known_free="
                 << (frontend_known_free_default_ ? "true" : "false")
                 << RESET << endl;
        }
    }

    void Fsm::onPlanFailure(const std::string & context) {
        plan_fail_streak_++;
        cout << YELLOW << " -- [Fsm] " << context << " fail streak="
             << plan_fail_streak_ << RESET << endl;

        // Level 1: re-snap goal into free space with a larger search radius
        // (phantom occupancy / mover trails often sit on the requested goal).
        if (plan_fail_streak_ == cfg_.stuck_fail_count) {
            Vec3f snapped = gi_.goal_p;
            if (planner_ptr_->getMap()->getNearestInfCellNot(
                    GridType::OCCUPIED, gi_.goal_p, snapped, 5.0)) {
                gi_.goal_p = snapped;
                gi_.new_goal = true;
                cout << YELLOW
                     << " -- [Fsm] Stuck recovery L1: re-snapped goal to free "
                     << gi_.goal_p.transpose() << RESET << endl;
            }
        }

        // Level 2: temporarily allow A* through unknown (ghost trails that
        // still read occupied/unknown while FreeDOM has already cleared them
        // from OctoMap). Restored on next successful plan.
        if (plan_fail_streak_ == cfg_.stuck_relax_count) {
            if (!frontend_relaxed_) {
                frontend_known_free_default_ = planner_ptr_->frontendInKnownFree();
                planner_ptr_->setFrontendInKnownFree(false);
                frontend_relaxed_ = true;
                gi_.new_goal = true;
                cout << YELLOW
                     << " -- [Fsm] Stuck recovery L2: frontend_in_known_free=false "
                        "(unknown traversable until plan succeeds)"
                     << RESET << endl;
            }
        }

        // Level 2.5: hard-reset the in-process ROG local map (phantom dynamics /
        // stale inflation). ROG is not a separate server — it lives in fsm_node;
        // clearing + recentering is the equivalent of a "rog map restart".
        if (plan_fail_streak_ == cfg_.stuck_reset_map_count) {
            const double now_s = ros_ptr_->getSimTime();
            const bool cooled = (last_rog_reset_wall_s_ < 0.0) ||
                ((now_s - last_rog_reset_wall_s_) >= cfg_.stuck_reset_map_cooldown);
            if (cooled) {
                planner_ptr_->getRobotState(robot_state_);
                planner_ptr_->getMap()->hardResetLocalMap(robot_state_.p);
                last_rog_reset_wall_s_ = now_s;
                gi_.new_goal = true;
                cout << YELLOW
                     << " -- [Fsm] Stuck recovery L2.5: ROG local map HARD RESET "
                        "at robot — waiting for LiDAR to refill free space"
                     << RESET << endl;
            } else {
                cout << YELLOW
                     << " -- [Fsm] Stuck recovery L2.5: ROG reset skipped "
                        "(cooldown " << cfg_.stuck_reset_map_cooldown << "s)"
                     << RESET << endl;
            }
        }

        // Level 3: give up this goal so global_planner / exploration can
        // pick a new one (or the user can re-click). Stay ready for a new goal.
        if (plan_fail_streak_ >= cfg_.stuck_give_up_count) {
            finish_plan = true;
            gi_.new_goal = false;
            plan_fail_streak_ = 0;
            has_giveup_goal_ = true;
            last_giveup_goal_ = gi_.goal_p;
            last_giveup_wall_s_ = ros_ptr_->getSimTime();
            if (frontend_relaxed_) {
                planner_ptr_->setFrontendInKnownFree(frontend_known_free_default_);
                frontend_relaxed_ = false;
            }
            ChangeState(context, WAIT_GOAL);
            cout << YELLOW
                 << " -- [Fsm] Stuck recovery L3: giving up goal, WAIT_GOAL "
                    "(replan/global path needed)"
                 << RESET << endl;
        }
    }

    void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q) {

        auto click_point = p;
        if (cfg_.click_height > -5) {
            click_point.z() = cfg_.click_height;
        }

        if (planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, click_point, gi_.goal_p, 3.0)) {
            cout << GREEN << " -- [Fsm] Get goal at " << RESET << gi_.goal_p.transpose() << endl;
        } else {
            fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
            return;
        }

        if ((robot_state_.p - gi_.goal_p).norm() <
            0.1) {
            //                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            return;
        }

        // A goal we just gave up on (L3) after exhausting the recovery ladder
        // is not reachable from here right now. If the upstream source (e.g. a
        // global_planner carrot) re-publishes the same point, accepting it
        // immediately would just restart the identical fail cycle from streak
        // zero. Hold it off for a short cooldown instead of thrashing.
        if (has_giveup_goal_) {
            const double since_giveup = ros_ptr_->getSimTime() - last_giveup_wall_s_;
            if ((gi_.goal_p - last_giveup_goal_).norm() < 0.3 &&
                since_giveup < cfg_.stuck_giveup_regoal_cooldown) {
                cout << YELLOW
                     << " -- [Fsm] Ignoring re-request for goal just given up on "
                     << since_giveup << "s ago (cooldown "
                     << cfg_.stuck_giveup_regoal_cooldown << "s)" << RESET << endl;
                return;
            }
            has_giveup_goal_ = false;
        }

        if (cfg_.click_yaw_en) {
            if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())) {
                gi_.goal_yaw = NAN;
                ros_ptr_->info(" -- [Fsm] Receive click goal at: [{}, {}, {}]; goal yaw disabled",
                               gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
            } else {
                gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
                cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw: "
                     << gi_.goal_yaw * 57.3 << " deg" << RESET << endl;
            }

        } else {
            gi_.goal_yaw = NAN;
            cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }

        started_ = true;
        gi_.new_goal = true;
        // A fresh goal must re-enable continuous replan (finish_plan may still
        // be set from the previous arrival settle).
        finish_plan = false;
        plan_fail_streak_ = 0;
        if (frontend_relaxed_) {
            planner_ptr_->setFrontendInKnownFree(frontend_known_free_default_);
            frontend_relaxed_ = false;
        }
    }

    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
        machine_state_ = new_state;
    }
}

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


#ifndef SUPER_FSM_CONFIG_HPP
#define SUPER_FSM_CONFIG_HPP


#include <super_core/config.hpp>
#include <vector>
#include <cstring>
#include <utils/header/yaml_loader.hpp>

namespace fsm {
    using namespace traj_opt;
    using namespace super_planner;
    static constexpr int MPC_PVAJ_MODE = 1;
    static constexpr int MPC_POLYTRAJ_MODE = 2;

    class Config {
    public:
        bool timer_en{true};

        // Fsm Params
        bool click_goal_en{},visualization_en{};
        double replan_rate{}, resolution{};
        double click_height{};

        bool click_yaw_en{};
        bool continuous_following{false};
        // Arrival settle: even with continuous_following, stop replanning once
        // the robot is near a *static* goal and slow enough. Orbit carrots stay
        // ~lookahead away so this does not fire mid-orbit.
        double goal_arrive_dis{0.60};
        double goal_arrive_vel{0.30};
        // Stuck recovery: consecutive PlanFromRest / ReplanOnce failures before
        // escalating (main FSM + replan timer both count).
        int stuck_fail_count{15};
        int stuck_relax_count{30};   // temporarily allow unknown as free
        int stuck_reset_map_count{40}; // hard-reset in-process ROG local map
        int stuck_give_up_count{60}; // abandon goal → WAIT_GOAL for upper layer
        double stuck_reset_map_cooldown{20.0}; // [s] min time between ROG resets
        string cmd_topic, mpc_cmd_topic, click_goal_topic;
        double yaw_dot_max{};

        Config() = default;

        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            vector<double> tem_gain;
            loader.LoadParam("fsm/timer_en", timer_en, false);
            loader.LoadParam("fsm/click_goal_en", click_goal_en, false);
            loader.LoadParam("fsm/click_yaw_en", click_yaw_en, false);
            loader.LoadParam("fsm/replan_rate", replan_rate, 10.0);
            loader.LoadParam("fsm/click_height", click_height, 1.5);
            loader.LoadParam("super_planner/continuous_following", continuous_following, false);
            loader.LoadParam("super_planner/goal_arrive_dis", goal_arrive_dis, 0.60);
            loader.LoadParam("super_planner/goal_arrive_vel", goal_arrive_vel, 0.30);
            loader.LoadParam("super_planner/stuck_fail_count", stuck_fail_count, 15);
            loader.LoadParam("super_planner/stuck_relax_count", stuck_relax_count, 30);
            loader.LoadParam("super_planner/stuck_reset_map_count", stuck_reset_map_count, 40);
            loader.LoadParam("super_planner/stuck_give_up_count", stuck_give_up_count, 60);
            loader.LoadParam("super_planner/stuck_reset_map_cooldown", stuck_reset_map_cooldown, 20.0);
            loader.LoadParam("fsm/cmd_topic", cmd_topic, string("/planning/pos_cmd"));
            loader.LoadParam("fsm/mpc_cmd_topic", mpc_cmd_topic, string("/planning_cmd/mpc"));
            loader.LoadParam("fsm/click_goal_topic", click_goal_topic, string("/planning/click_goal_topic"));


            loader.LoadParam("super_planner/yaw_dot_max", yaw_dot_max, 1.0, true);
            loader.LoadParam("super_planner/visualization_en", visualization_en, false, true);
            loader.LoadParam("rog_map/resolution", resolution, 0.1, true);

        }
    };
}

#endif //SUPER_FSM_CONFIG_H

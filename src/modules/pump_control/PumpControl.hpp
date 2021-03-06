/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <commander/px4_custom_mode.h>

#include <systemlib/mavlink_log.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/pump_status.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>

using namespace time_literals;

//static constexpr uint32_t PUMP_SCHEDULE_INTERVAL{100_ms};    /**< The schedule interval in usec (10 Hz) */


class PumpControl : public ModuleBase<PumpControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	PumpControl();
	~PumpControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Parameters
	// DEFINE_PARAMETERS(
	// 	(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
	// 	(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	// )
        bool _low_flow{false};
	bool low_flow_flag{false};

	bool armd_flag{false};
	bool mission_on_flag{false};
	bool RTL_on_flag{false};
	bool pump_on_flag{false};

        // pump_control publications
	pump_status_s   pump_status;

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _rpm_update_sub{this, ORB_ID(rpm)};
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates

	uORB::Subscription                 _act_output_sub{ORB_ID(actuator_outputs)};
        uORB::Subscription	           _actuator_controls_3_sub{ORB_ID(actuator_controls_3)};
	// uORB::Subscription                 _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	uORB::Subscription	           _vehicle_cmd_sub{ORB_ID(vehicle_command)};            // get the param from the mission manager
	uORB::SubscriptionData<position_setpoint_triplet_s>      _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
        uORB::SubscriptionData<vehicle_status_s>                 _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<vehicle_local_position_s>         _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};


	// Publications
	uORB::Publication<pump_status_s>   _pump_status_pub{ORB_ID(pump_status)};

	orb_advert_t _mavlink_log_pub{nullptr};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};


};

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

/**
 * @file PumpControl.cpp
 *
 * Control and deal with the pump when the multipilot is in the misson mode
 *
 * @author ChenKang
 */

#include "PumpControl.hpp"

static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN, const float param3 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}


PumpControl::PumpControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

PumpControl::~PumpControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool PumpControl::init()
{
	// execute Run() on every rpm publication
	if (!_rpm_update_sub.registerCallback()) {
		PX4_ERR("rpm callback registration failed");
		return false;
	}

	//ScheduleOnInterval(PUMP_SCHEDULE_INTERVAL);
	return true;
}

void PumpControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	//得到rpm频率并计算流量
	if (_rpm_update_sub.updated()) {
                rpm_s rpm_pcf8583;

		if (_rpm_update_sub.copy(&rpm_pcf8583)) {
                        //脉冲特性 F=Q*23 +-10%
			pump_status.pump_flow = rpm_pcf8583.indicated_frequency_rpm / 23;
	 		// mavlink_vasprintf(_MSG_PRIO_INFO, &_mavlink_log_pub, "pump_flow:%lf", double(pump_status.pump_flow));
                        const bool low_flow = (pump_status.pump_flow > 0.1f) ? true : false;

			if (low_flow && !_low_flow) {
				low_flow_flag = true;

			} else if (!low_flow && _low_flow) {
				low_flow_flag = false;
			}

			_low_flow = low_flow;
		}
	}

	//是否处于返航或任务模式
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			// armd_flag = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			RTL_on_flag = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
		        mission_on_flag = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
		}
	}

	//水泵是否在运行，pwm_min = 1075
	if (_act_output_sub.updated()) {
		actuator_outputs_s act;

		if (_act_output_sub.copy(&act)) {
	 	        //mavlink_vasprintf(_MSG_PRIO_INFO, &_mavlink_log_pub, "aux1:%.2f", double(act.output[4]));
			pump_on_flag = (act.output[4] > 1150) ? true : false;
		}
	}

        if (low_flow_flag && !RTL_on_flag && pump_on_flag) {
                uORB::SubscriptionData<vehicle_constraints_s>  _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
		// if (armd_flag) {
		if (mission_on_flag && (_vehicle_constraints_sub.get().speed_xy > 0.5f)) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL);
		}

	}

	perf_end(_loop_perf);
}

int PumpControl::task_spawn(int argc, char *argv[])
{
	PumpControl *instance = new PumpControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int PumpControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int PumpControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PumpControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pump_control_main(int argc, char *argv[])
{
	return PumpControl::main(argc, argv);
}

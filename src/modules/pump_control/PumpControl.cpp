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

	// Example
	//  update vehicle_status to check arming state
	// if (_vehicle_status_sub.updated()) {
	// 	vehicle_status_s vehicle_status;

	// 	if (_vehicle_status_sub.copy(&vehicle_status)) {

	// 		const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	// 		if (armed && !_armed) {
	// 			PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

	// 		} else if (!armed && _armed) {
	// 			PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
	// 		}

	// 		_armed = armed;
	// 	}
	// }

	if (_rpm_update_sub.updated()) {
                rpm_s rpm_pcf8583;
	        last_flag = pump_status.pump_flag;

		if (_rpm_update_sub.copy(&rpm_pcf8583)) {

			pump_status.pump_flow = rpm_pcf8583.indicated_frequency_rpm / 23;
	 		// mavlink_vasprintf(_MSG_PRIO_INFO, &_mavlink_log_pub, "pump_flow:%lf", double(pump_status.pump_flow));

			if (pump_status.pump_flow > 1.0f)
			{
                                pump_status.pump_flag = 1;
			}
			else {
				pump_status.pump_flag = 0;
			}

			if (last_flag ^ pump_status.pump_flag) {
				pump_status.timestamp = hrt_absolute_time();
				_pump_status_pub.publish(pump_status);
				// mavlink_vasprintf(_MSG_PRIO_INFO, &_mavlink_log_pub, "pub");

			}


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

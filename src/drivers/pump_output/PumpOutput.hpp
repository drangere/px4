/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

// /**
//  * @file PumpOutput.hpp
//  *
//  */

// #pragma once

// #include <drivers/drv_hrt.h>
// #include <drivers/drv_input_capture.h>
// #include <drivers/drv_pwm_output.h>
// #include <lib/parameters/param.h>
// #include <px4_platform_common/px4_config.h>
// #include <px4_platform_common/defines.h>
// #include <px4_platform_common/module.h>
// #include <px4_platform_common/tasks.h>
// #include <px4_platform_common/workqueue.h>
// #include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
// #include <uORB/Publication.hpp>
// #include <uORB/Subscription.hpp>
// #include <uORB/topics/camera_trigger.h>
// #include <uORB/topics/vehicle_command.h>
// #include <uORB/topics/vehicle_command_ack.h>

// #define PX4FMU_DEVICE_PATH	"/dev/px4fmu"


// class PumpOutput : public px4::ScheduledWorkItem
// {
// public:
// 	/**
// 	 * Constructor
// 	 */
// 	PumpOutput();

// 	/**
// 	 * Destructor, also kills task.
// 	 */
// 	~PumpOutput();

// 	/**
// 	 * Start the task.
// 	 */
// 	int			start();

// 	/**
// 	 * Stop the task.
// 	 */
// 	void			stop();

// 	void 			status();

// 	// Low-rate command handling loop
// 	void			Run() override;

// 	static void		capture_trampoline(void *context, uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state,
// 			uint32_t overflow);

// 	void 			set_capture_control(bool enabled);

// 	void			reset_statistics(bool reset_seq);

// 	void			publish_trigger();


// 	static struct work_s	_work_publisher;

// private:

// 	// Publishers
// 	uORB::Publication<vehicle_command_ack_s>	_command_ack_pub{ORB_ID(vehicle_command_ack)};
// 	uORB::Publication<camera_trigger_s>		_trigger_pub{ORB_ID(camera_trigger)};

// 	// Subscribers
// 	uORB::Subscription				_command_sub{ORB_ID(vehicle_command)};

// 	// Trigger Buffer
// 	struct _trig_s {
// 		uint32_t chan_index;
// 		hrt_abstime edge_time;
// 		uint32_t edge_state;
// 		uint32_t overflow;
// 	} _trigger{};

// 	bool			_capture_enabled{false};
// 	bool			_gpio_capture{false};

// 	// Parameters
// 	param_t 		_p_strobe_delay{PARAM_INVALID};
// 	float			_strobe_delay{0.0f};
// 	param_t			_p_camera_capture_mode{PARAM_INVALID};
// 	int32_t			_camera_capture_mode{0};
// 	param_t			_p_camera_capture_edge{PARAM_INVALID};
// 	int32_t			_camera_capture_edge{0};

// 	// Signal capture statistics
// 	uint32_t		_capture_seq{0};
// 	hrt_abstime		_last_trig_begin_time{0};
// 	hrt_abstime		_last_exposure_time{0};
// 	hrt_abstime		_last_trig_time{0};
// 	uint32_t 		_capture_overflows{0};

// 	// Signal capture callback
// 	void			capture_callback(uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);

// 	// GPIO interrupt routine
// 	static int		gpio_interrupt_routine(int irq, void *context, void *arg);

// 	// Signal capture publish
// 	static void		Run_trampoline(void *arg);

// };

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/ecl/geo/geo.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

static constexpr uint32_t PUMP_SCHEDULE_INTERVAL{100_ms};    /**< The schedule interval in usec (10 Hz) */


class PumpOutput : public ModuleBase<PumpOutput>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	PumpOutput();
	~PumpOutput() override;

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

	// void parameters_update();

        // DEFINE_PARAMETERS(
	// 	(ParamFloat<px4::parameters_volatile::PUMP_ACC_RAD>) _pump_acc_rad
	// )
	// Subscriptions
	uORB::SubscriptionInterval                               _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates

        uORB::SubscriptionData<vehicle_status_s>                 _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<position_setpoint_triplet_s>      _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::SubscriptionData<vehicle_local_position_s>         _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionData<vehicle_gps_position_s>           _vehicle_gps_pos_sub{ORB_ID(vehicle_gps_position)};

	// Publications
        uORB::Publication<actuator_controls_s>    _actuator_controls_3_pub{ORB_ID(actuator_controls_3)};

	orb_advert_t _mavlink_log_pub{nullptr};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};


};

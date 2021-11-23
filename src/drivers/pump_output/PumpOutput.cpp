// #include "PumpOutput.hpp"

// namespace pump_output
// {
// PumpOutput *g_camera_capture{nullptr};
// }

// PumpOutput::PumpOutput() :
// {

// }

// PumpOutput::~PumpOutput()
// {
//     pump_output::g_camera_capture = nullptr;
// }

// void
// PumpOutput::Run_trampoline(void *arg)
// {
//     PumpOutput *dev = reinterpret_cast<PumpOutput *>(arg);
//     dev->Run();
// }


// void
// PumpOutput::Run()
// {
//     //订阅命令消息，处理命令
//     if (_command_sub == 0) {
//         _command_sub == orb_subscribe(ORB_ID(vehicle_command));
//     }

//     bool updated = false;
//     orb_check(_command_sub, &updated);
//     // Command handling
//     if (updated) {
//         vehicle_command_s cmd;
//         orb_copy(ORB_ID(vehicle_command), _command_sub, &cmd);
//         // TODO : this should eventuallly be a capture control command
//         if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL) {
//             // Enable/disable signal capture
//             if (commandParamToInt(cmd.param1) == 1) {
//                 set_capture_control(true);
//             } else if (commandParamToInt(cmd.param1) == 0) {
//                 set_capture_control(false);
//             }
//             // Reset capture sequence
//             if (commandParamToInt(cmd.param2) == 1) {
//                 reset_statistics(true);
//             }
//         }
//     }

//     //低速循环队列，比开进程好多了
//     work_queue(LPWORK, &amp;_work, (worker_t)&amp;PumpOutput::Run_trampoline, pump_output::g_camera_capture,
//            USEC2TICK(100000)); // 100ms
// }


// int
// PumpOutput::start()
// {
//     /* allocate basic report buffers */
//     _trig_buffer = new ringbuffer::RingBuffer(2, sizeof(_trig_s));

//     if (_trig_buffer == nullptr) {
//         return PX4_ERROR;
//     }

//     // start to monitor at low rates for capture control commands
//     work_queue(LPWORK, &_work, (worker_t)&PumpOutput::Run_trampoline, this,
//            USEC2TICK(1));

//     return PX4_OK;
// }

// void
// PumpOutput::stop()
// {
//     work_cancel(LPWORK, &_work);
//     work_cancel(HPWORK, &_work_publisher);

//     if (pump_output::g_camera_capture != nullptr) {
//         delete (pump_output::g_camera_capture);
//     }
// }

// static int usage()
// {
// 	PX4_INFO("usage: camera_capture {start|stop|on|off|reset|status}\n");
// 	return 1;
// }

// extern "C" __EXPORT int pump_output_main(int argc, char *argv[]);
// int pump_output_main(int argc, char *argv[])
// {
//     if (argc < 2) {
//         return usage();
//     }

//     if (!strcmp(argv[1], "start")) {
//         pump_output::g_camera_capture = new PumpOutput();
//         return 0;
//     } else if (!strcmp(argv[1], "stop")) {
//         pump_output::g_camera_capture->stop();

//     } else {
//         return usage();
//     }
//     return 0;
// }


#include "PumpOutput.hpp"

PumpOutput::PumpOutput() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

PumpOutput::~PumpOutput()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool PumpOutput::init()
{
	// execute Run() on every rpm publication
	// if (!_rpm_update_sub.registerCallback()) {
	// 	PX4_ERR("rpm callback registration failed");
	// 	return false;
	// }

	ScheduleOnInterval(PUMP_SCHEDULE_INTERVAL);
	return true;
}

void PumpOutput::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);
	parameters_update();

	if (_vehicle_gps_pos_sub.updated()) {
		const vehicle_gps_position_s &gpos = _vehicle_gps_pos_sub.get();
                const position_setpoint_triplet_s &pos_sp_triplet = _position_setpoint_triplet_sub.get();

		float points_dis = get_distance_to_next_waypoint(pos_sp_triplet.current.lat, pos_sp_triplet.current.lon,
								pos_sp_triplet.next.lat, pos_sp_triplet.next.lon);
		double s_limit = points_dis * 5 / 2;  //默认限制为5m
		double hypotenuse = sqrt(points_dis * points_dis + 5 * 5);  //斜边长

		float local_pre_dis = get_distance_to_next_waypoint(gpos.lat, gpos.lon, pos_sp_triplet.previous.lat, pos_sp_triplet.previous.lon);
		float local_next_dis = get_distance_to_next_waypoint(gpos.lat, gpos.lon, pos_sp_triplet.next.lat, pos_sp_triplet.next.lon);

		double p = (points_dis + local_pre_dis + local_next_dis) / 2;
		double s = sqrt(p * (p - points_dis) * (p - local_pre_dis) * (p - local_next_dis));  //海伦公式

		if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION &&
			_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION &&
			s < s_limit && local_next_dis < hypotenuse) {

			const vehicle_local_position_s &lpos = _vehicle_local_position_sub.get();
			double speed_xy = sqrt(lpos.vx * lpos.vx + lpos.vy * lpos.vy);

			actuator_controls_s _actuator_controls_3;
			_actuator_controls_3.control[5] = speed_xy / 4 - 1;  //max:8m/s pwm:100% | min:0m/s pwm:0%
			_actuator_controls_3.timestamp = hrt_absolute_time();
			_actuator_controls_3_pub.publish(_actuator_controls_3);  //输出通道为main 5

		}

	}


	perf_end(_loop_perf);
}

void PumpOutput::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

int PumpOutput::task_spawn(int argc, char *argv[])
{
	PumpOutput *instance = new PumpOutput();

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

int PumpOutput::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int PumpOutput::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PumpOutput::print_usage(const char *reason)
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

extern "C" __EXPORT int pump_output_main(int argc, char *argv[])
{
	return PumpOutput::main(argc, argv);
}

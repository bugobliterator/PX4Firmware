/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file ekf2_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>
#include <drivers/drv_hrt.h>
#include <controllib/uorb/blocks.hpp>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ekf2_innovations.h>

#include <ros/ros.h>

extern "C" __EXPORT int roshk_main(int argc, char *argv[]);


class Roshk;

namespace roshk
{
Roshk *instance = nullptr;
}


class Roshk
{
public:
	/**
	 * Constructor
	 */
	Roshk();

	/**
	 * Destructor, also kills task.
	 */
	~Roshk();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

	void print();

	void print_status();

	void exit() { _task_should_exit = true; }
private:
	int		_control_task = -1;			/**< task handle for task */
	bool	_task_should_exit;
};

Roshk::Roshk()
{
}

Roshk::~Roshk()
{

}

void Roshk::print()
{
	printf("Hello World\n");
}

void Roshk::print_status()
{
}

void Roshk::task_main()
{
	while (!_task_should_exit) {

	}
}

void Roshk::task_main_trampoline(int argc, char *argv[])
{
	roshk::instance->task_main();
}

int Roshk::start()
{
	ASSERT(_control_task == -1);
	ros::init();
	/* start the task */
	_control_task = px4_task_spawn_cmd("roshk",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   9000,
					   (px4_main_t)&Roshk::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

int roshk_main(int argc, char *argv[])
{
	if (argc < 1) {
		PX4_WARN("usage: roshk {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		printf("I was called\n");
		if (roshk::instance != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		roshk::instance = new Roshk();

		if (roshk::instance == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (OK != roshk::instance->start()) {
			delete roshk::instance;
			roshk::instance = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (roshk::instance == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		roshk::instance->exit();

		// wait for the destruction of the instance
		while (roshk::instance != nullptr) {
			usleep(50000);
		}

		return 0;
	}

	if (!strcmp(argv[1], "print")) {
		if (roshk::instance != nullptr) {

			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		if (roshk::instance) {
			PX4_WARN("running");
			roshk::instance->print_status();
			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

	PX4_WARN("unrecognized command");
	return 1;
}
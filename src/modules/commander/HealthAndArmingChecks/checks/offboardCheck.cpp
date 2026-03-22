/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "offboardCheck.hpp"

using namespace time_literals;

void OffboardChecks::checkAndReport(const Context &context, Report &reporter)
{
	reporter.failsafeFlags().offboard_control_signal_lost = true;

	enum class OffboardFailureReason {
		NoSignal,
		SignalLost,
		NoSetpointType,
		LocalPositionInvalid,
		LocalVelocityInvalid,
		None,
	};

	offboard_control_mode_s offboard_control_mode;
	OffboardFailureReason failure_reason = OffboardFailureReason::NoSignal;
	bool offboard_available = false;

	if (_offboard_control_mode_sub.copy(&offboard_control_mode)) {
		const bool has_active_setpoints = offboard_control_mode.position || offboard_control_mode.velocity
					  || offboard_control_mode.acceleration || offboard_control_mode.attitude || offboard_control_mode.body_rate
					  || offboard_control_mode.thrust_and_torque || offboard_control_mode.direct_actuator;

		const bool data_is_recent = hrt_absolute_time() < offboard_control_mode.timestamp
					    + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s);

		offboard_available = has_active_setpoints && data_is_recent;

		if (!has_active_setpoints) {
			offboard_available = false;
			failure_reason = OffboardFailureReason::NoSetpointType;

		} else if (!data_is_recent) {
			offboard_available = false;
			failure_reason = OffboardFailureReason::SignalLost;

		} else if (offboard_control_mode.position && reporter.failsafeFlags().local_position_invalid) {
			offboard_available = false;
			failure_reason = OffboardFailureReason::LocalPositionInvalid;

		} else if (offboard_control_mode.velocity && reporter.failsafeFlags().local_velocity_invalid) {
			offboard_available = false;
			failure_reason = OffboardFailureReason::LocalVelocityInvalid;

		} else if (offboard_control_mode.acceleration && reporter.failsafeFlags().attitude_invalid) {
			// OFFBOARD acceleration handled by position controller
			offboard_available = false;
			failure_reason = OffboardFailureReason::None;

		} else {
			failure_reason = OffboardFailureReason::None;
		}
	}

	reporter.failsafeFlags().offboard_control_signal_lost = !offboard_available;

	if (!offboard_available) {
		const NavModes required_modes = (NavModes)reporter.failsafeFlags().mode_req_offboard_signal;

		if (required_modes != NavModes::None) {
			switch (failure_reason) {
			case OffboardFailureReason::NoSignal:
				/* EVENT
				 * @description
				 * The offboard component is not sending offboard control updates.
				 */
				reporter.armingCheckFailure(required_modes, health_component_t::system,
							    events::ID("check_modes_offboard_no_signal"),
							    events::Log::Error, "No offboard signal");
				break;

			case OffboardFailureReason::SignalLost:
				/* EVENT
				 * @description
				 * The offboard component stopped sending updates in time. Check the companion link and stream rate.
				 */
				reporter.armingCheckFailure(required_modes, health_component_t::system,
							    events::ID("check_modes_offboard_signal_lost"),
							    events::Log::Error, "Offboard signal lost");
				break;

			case OffboardFailureReason::NoSetpointType:
				/* EVENT
				 * @description
				 * Enable at least one offboard control mode in the companion computer stream.
				 */
				reporter.armingCheckFailure(required_modes, health_component_t::system,
							    events::ID("check_modes_offboard_no_setpoint_type"),
							    events::Log::Error, "No offboard setpoint type enabled");
				break;

			case OffboardFailureReason::LocalPositionInvalid:
				/* EVENT
				 * @description
				 * Offboard position control requires a valid local position estimate.
				 */
				reporter.armingCheckFailure(required_modes, health_component_t::local_position_estimate,
							    events::ID("check_modes_offboard_local_position"),
							    events::Log::Error, "Offboard position control requires valid local position estimate");
				break;

			case OffboardFailureReason::LocalVelocityInvalid:
				/* EVENT
				 * @description
				 * Offboard velocity control requires a valid local velocity estimate.
				 */
				reporter.armingCheckFailure(required_modes, health_component_t::local_position_estimate,
							    events::ID("check_modes_offboard_local_velocity"),
							    events::Log::Error, "Offboard velocity control requires valid local velocity estimate");
				break;

			case OffboardFailureReason::None:
				break;
			}

			reporter.clearCanRunBits(required_modes);
		}
	}
}

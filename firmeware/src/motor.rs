use cortex_m::prelude::_embedded_hal_Pwm;
use embassy_stm32::{
	gpio::Output,
	timer::{Channel, simple_pwm::SimplePwm},
};

use crate::MotorPwmTim;

pub struct Motor {
	in1: Output<'static>,
	in2: Output<'static>,
	standby: Output<'static>,
	pwm_channel: Channel,
}

impl Motor {
	pub fn new(
		in1: Output<'static>,
		in2: Output<'static>,
		standby: Output<'static>,
		pwm_channel: Channel,
	) -> Self {
		Self {
			in1,
			in2,
			standby,
			pwm_channel,
		}
	}

	pub fn enable(&mut self) {
		self.standby.set_high();
	}

	pub fn disable(&mut self) {
		self.standby.set_low();
	}

	pub fn set_speed(&mut self, pwm: &mut SimplePwm<'static, MotorPwmTim>, speed: i32) {
		let speed = speed.clamp(-1000, 1000);

		if speed > 0 {
			self.in1.set_high();
			self.in2.set_low();
		} else if speed < 0 {
			self.in1.set_low();
			self.in2.set_high();
		} else {
			self.in1.set_low();
			self.in2.set_low();
		}

		pwm.set_duty(self.pwm_channel, speed.unsigned_abs());
	}
}

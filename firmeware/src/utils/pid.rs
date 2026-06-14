pub struct Pid {
	pub kp: f32,
	pub ki: f32,
	pub kd: f32,

	pub integral: f32,
	pub derivative: f32,
	pub previous_error: f32,

	pub integral_limit: f32,
	pub output_limit: f32,
}

impl Pid {
	/// Creates a new PID controller.
	///
	/// # Arguments
	///
	/// * `kp` - Proportional gain
	/// * `ki` - Integral gain (set to 0.0 to disable integral term)
	/// * `kd` - Derivative gain (set to 0.0 to disable derivative term)
	/// * `integral_limit` - Maximum absolute value for the integral term (0.0 for unlimited)
	/// * `output_limit` - Maximum absolute value for the output (0.0 for unlimited)
	///
	/// # Example
	///
	/// ```
	/// let mut pid = PID::new(1.0, 0.1, 0.01, 10.0, 100.0);
	/// let output = pid.update(100.0, 90.0, 0.1);
	/// println!("PID output: {}", output);
	///
	/// let mut pd = PID::new(1.0, 0.0, 0.01, 0.0, 100.0);
	/// let output = pd.update(100.0, 90.0, 0.1);
	/// println!("PD output: {}", output);
	/// ```
	pub fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32, output_limit: f32) -> Self {
		Self {
			kp,
			ki,
			kd,
			integral: 0.0,
			derivative: 0.0,
			previous_error: 0.0,
			integral_limit,
			output_limit,
		}
	}

	pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
		let error = setpoint - measurement;

		if self.ki != 0.0 {
			self.integral += error * dt;

			if self.integral_limit > 0.0 {
				self.integral = self
					.integral
					.clamp(-self.integral_limit, self.integral_limit);
			}
		}

		// TODO: add filter
		if self.kd != 0.0 {
			self.derivative = (error - self.previous_error) / dt;
		}

		self.previous_error = error;

		let output = self.kp * error + self.ki * self.integral + self.kd * self.derivative;
		output.clamp(-self.output_limit, self.output_limit)
	}
}

use std::cmp::Ordering;

/// The type of time step to use for the physics simulation.
pub enum TimeStep {
    /// Physics will always use the given timestep.
    Fixed(f32),
    /// Physics use one of the given timesteps, changing when physics are falling behind.
    SemiFixed(TimeStepConstraint),
}

impl Default for TimeStep {
    fn default() -> Self {
        TimeStep::Fixed(1. / 120.)
    }
}

/// Error when trying to change the actual timestep for a semi-fixed timestep.
#[derive(Debug)]
pub enum TimeStepChangeError {
    /// No smaller timestep available.
    MinimumTimestepReached,
    /// No larger timestep available.
    MaximumTimestepReached,
}

impl std::fmt::Display for TimeStepChangeError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            TimeStepChangeError::MaximumTimestepReached => {
                write!(f, "No larger timestep available!")
            }
            TimeStepChangeError::MinimumTimestepReached => {
                write!(f, "No smaller timestep available!")
            }
        }
    }
}

/// Constraints for a semi-fixed timestep.
pub struct TimeStepConstraint {
    /// Vector of possible timesteps to use.
    time_steps: Vec<f32>,
    /// Index of the currently used timestep.
    current_index: usize,
}

impl TimeStepConstraint {
    /// Creates a new `TimeStepConstraint` from the specified timesteps to use and the minimum time of running
    /// slowly before changing timesteps. The timestep will be increased if physics have been lagging behind for
    /// the last `minimum_slow_time`.
    ///
    /// # Panics
    ///
    /// This constructor will panic if no timesteps are given or if any negative timesteps are specified.
    pub fn new(time_steps: impl Into<Vec<f32>>) -> Self {
        let mut time_steps = time_steps.into();
        assert!(
            !time_steps.is_empty(),
            "No timesteps given in TimeStepConstraint"
        );
        time_steps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        time_steps.dedup();
        assert!(time_steps[0] > 0., "Negative timesteps are not allowed");

        Self {
            time_steps,
            current_index: 0,
        }
    }

    /// Increase the timestep. This corresponds to fewer updates per second.
    ///
    /// Shouldn't be called from outside the `PhysicsStepperSystem`, otherwise bad things may happen.
    pub fn increase_timestep(&mut self) -> Result<f32, TimeStepChangeError> {
        if self.current_index >= self.time_steps.len() - 1 {
            return Err(TimeStepChangeError::MaximumTimestepReached);
        }
        self.current_index += 1;
        Ok(self.current_timestep())
    }

    /// Decrease the timestep. This corresponds to more updates per second.
    ///
    /// Shouldn't be called from outside the `PhysicsStepperSystem`, otherwise bad things may happen.
    pub fn decrease_timestep(&mut self) -> Result<f32, TimeStepChangeError> {
        if self.current_index <= 0 {
            return Err(TimeStepChangeError::MinimumTimestepReached);
        }
        self.current_index -= 1;
        Ok(self.current_timestep())
    }

    /// Get the currently used timestep.
    pub fn current_timestep(&self) -> f32 {
        self.time_steps[self.current_index]
    }

    /// Get next smaller timestep.
    pub fn smaller_timestep(&self) -> Option<f32> {
        if self.current_index <= 0 {
            None
        } else {
            Some(self.time_steps[self.current_index - 1])
        }
    }
}

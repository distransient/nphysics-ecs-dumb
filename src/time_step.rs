use std::{
    cmp::Ordering,
    time::{Duration, Instant},
};

/// The type of time step to use for the physics simulation.
pub enum TimeStep {
    /// Physics will always use the given timestep.TimeStepChangeError.
    Fixed(f32),
    /// Physics use one of the given timesteps, chaning when physics are falling behind.
    SemiFixed(TimeStepConstraint),
}

/// Error when trying to change the actual timestep for a semi-fixed timestep.
#[derive(Debug)]
pub enum TimeStepChangeError {
    MinimumTimestepReached,
    MaximumTimestepReached,
    MinimumAgeNotReached(Duration),
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
            TimeStepChangeError::MinimumAgeNotReached(remaining) => {
                let time = remaining.as_secs() as f32 + remaining.subsec_nanos() as f32 * 1e-9;
                write!(f, "Timestep can be changed in {:.4} seconds", time)
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
    /// Time when the timestep was last changed.
    change_time: Instant,
    /// Minimum time after changing the timestep before it can be changed again.
    minimum_age: Duration,
}

impl TimeStepConstraint {
    /// Creates a new `TimeStepConstraint` from the specified timesteps to use and the minimum time between changing timesteps.
    ///
    /// # Panics
    ///
    /// This constructor will panic if no timesteps are given or if any negative timesteps are specified.
    pub fn new(time_steps: impl Into<Vec<f32>>, minimum_age: Duration) -> Self {
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
            change_time: Instant::now(),
            minimum_age,
        }
    }

    /// Increase the timestep. This corresponds to fewer updates per seconds.
    pub fn increase_timestep(&mut self) -> Result<(), TimeStepChangeError> {
        if self.current_index >= self.time_steps.len() - 1 {
            return Err(TimeStepChangeError::MaximumTimestepReached);
        }
        if let Some(remaining) = self.minimum_age.checked_sub(self.change_time.elapsed()) {
            return Err(TimeStepChangeError::MinimumAgeNotReached(remaining));
        }
        self.current_index += 1;
        self.change_time = Instant::now();
        Ok(())
    }

    /// Get the currently used timestep.
    pub fn current_timestep(&self) -> f32 {
        self.time_steps[self.current_index]
    }
}

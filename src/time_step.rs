use std::{
    cmp::Ordering,
    time::{Duration, Instant},
};

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
    /// Physics haven't been running slow for long enough to change the timestep.
    MinimumTimeNotReached(Duration),
    /// Physics aren't running slow.
    NotRunningSlow,
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
            TimeStepChangeError::MinimumTimeNotReached(remaining) => {
                let time = remaining.as_secs() as f32 + remaining.subsec_nanos() as f32 * 1e-9;
                write!(f, "Timestep can be changed in {:.4} seconds", time)
            }
            TimeStepChangeError::NotRunningSlow => write!(f, "Simulation is not running slow"),
        }
    }
}

/// Constraints for a semi-fixed timestep.
pub struct TimeStepConstraint {
    /// Vector of possible timesteps to use.
    time_steps: Vec<f32>,
    /// Index of the currently used timestep.
    current_index: usize,
    /// Time when the simulation started running slow.
    running_slow_since: Option<Instant>,
    /// Minimum time the simulation has to be running slow before the timestep is changed.
    minimum_slow_time: Duration,
}

impl TimeStepConstraint {
    /// Creates a new `TimeStepConstraint` from the specified timesteps to use and the minimum time of running
    /// slowly before changing timesteps. The timestep will be increased if physics have been lagging behind for
    /// the last `minimum_slow_time`.
    ///
    /// # Panics
    ///
    /// This constructor will panic if no timesteps are given or if any negative timesteps are specified.
    pub fn new(time_steps: impl Into<Vec<f32>>, minimum_slow_time: Duration) -> Self {
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
            minimum_slow_time,
            running_slow_since: None,
        }
    }

    /// Increase the timestep. This corresponds to fewer updates per second.
    ///
    /// Shouldn't be called from outside the `PhysicsStepperSystem`, otherwise bad things may happen.
    pub fn increase_timestep(&mut self) -> Result<f32, TimeStepChangeError> {
        if self.current_index >= self.time_steps.len() - 1 {
            return Err(TimeStepChangeError::MaximumTimestepReached);
        }
        let running_slow_since = match self.running_slow_since {
            Some(time) => time,
            None => return Err(TimeStepChangeError::NotRunningSlow),
        };
        if let Some(remaining) = self
            .minimum_slow_time
            .checked_sub(running_slow_since.elapsed())
        {
            return Err(TimeStepChangeError::MinimumTimeNotReached(remaining));
        }
        self.current_index += 1;
        self.running_slow_since = None;
        Ok(self.current_timestep())
    }

    /// Decrease the timestep. This corresponds to more updates per second.
    ///
    /// Shouldn't be called from outside the `PhysicsStepperSystem`, otherwise bad things may happen.
    pub fn decrease_timestep(&mut self) -> Result<f32, TimeStepChangeError> {
        if self.current_index <= 0 {
            return Err(TimeStepChangeError::MinimumTimestepReached);
        }
        let running_slow_since = match self.running_slow_since {
            Some(time) => time,
            None => return Err(TimeStepChangeError::NotRunningSlow),
        };
        if let Some(remaining) = self
            .minimum_slow_time
            .checked_sub(running_slow_since.elapsed())
        {
            return Err(TimeStepChangeError::MinimumTimeNotReached(remaining));
        }
        self.current_index -= 1;
        self.running_slow_since = None;
        Ok(self.current_timestep())
    }

    /// Get the currently used timestep.
    pub fn current_timestep(&self) -> f32 {
        self.time_steps[self.current_index]
    }

    /// Set whether physics are currently running slow or not. Intended to be called every frame as changing
    /// values only happens when `is_running_slow` changes between calls.
    ///
    /// Shouldn't be called from outside the `PhysicsStepperSystem`, otherwise bad things may happen.
    pub fn set_running_slow(&mut self, is_running_slow: bool) {
        self.running_slow_since = match (self.running_slow_since, is_running_slow) {
            (None, true) => {
                warn!("Physics seem to be running slow! Timestep will be changed if we keep running slow.");
                Some(Instant::now())
            }
            (Some(_), false) => {
                debug!("Physics aren't running slow anymore.");
                None
            }
            (_, _) => self.running_slow_since,
        }
    }

    /// Get whether physics have been running slow for the specified minimum time and thus the timestep should
    /// be increased.
    pub fn should_increase_timestep(&self) -> bool {
        match self.running_slow_since {
            None => false,
            Some(time) => time.elapsed() > self.minimum_slow_time,
        }
    }
}

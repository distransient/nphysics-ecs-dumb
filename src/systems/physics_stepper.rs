use crate::time_step::TimeStep;
use crate::PhysicsWorld;
use amethyst::core::Time;
use amethyst::ecs::{Read, System, WriteExpect};
use std::time::Instant;

/// Falloff factor for calculating the moving average step time.
const AVERAGE_STEP_TIME_FALLOFF: f32 = 0.33;

/// Simulates a step of the physics world.
pub struct PhysicsStepperSystem {
    intended_timestep: TimeStep,
    max_timesteps: i32,
    time_accumulator: f32,
    avg_step_time: Option<f32>,
}

impl Default for PhysicsStepperSystem {
    fn default() -> Self {
        PhysicsStepperSystem {
            intended_timestep: TimeStep::Fixed(1. / 120.),
            max_timesteps: 10,
            time_accumulator: 0.,
            avg_step_time: None,
        }
    }
}

impl PhysicsStepperSystem {
    pub fn new(intended_timestep: TimeStep, max_timesteps: i32) -> Self {
        PhysicsStepperSystem {
            intended_timestep,
            max_timesteps,
            time_accumulator: 0.,
            avg_step_time: None,
        }
    }
}

impl<'a> System<'a> for PhysicsStepperSystem {
    type SystemData = (WriteExpect<'a, PhysicsWorld>, Read<'a, Time>);

    // Simulate world using the current time frame
    fn run(&mut self, (mut physical_world, time): Self::SystemData) {
        let (timestep, mut change_timestep) = match &mut self.intended_timestep {
            TimeStep::Fixed(timestep) => (*timestep, false),
            TimeStep::SemiFixed(constraint) => {
                let mut timestep = (constraint.current_timestep(), false);
                if let Some(avg_step) = self.avg_step_time {
                    // If the timestep is smaller than it takes to simulate that step, we have a problem.
                    // As simulated time is affected by the time scale, simulated time step / time scale
                    // is the maximum real time the step may take, so we take that into account here. We
                    // also take into account the maximum fraction of time physics are allowed to take
                    let adjusted_step_time =
                        avg_step * time.time_scale() / constraint.max_physics_time_fraction();
                    if constraint.current_timestep() < adjusted_step_time {
                        match constraint.increase_timestep() {
                            Err(error) => {
                                warn!("Failed to increase physics timestep! Error: {}", error);
                            }
                            Ok(new_timestep) => {
                                info!("Increasing physics timestep to {:.8} seconds", new_timestep);
                                timestep = (new_timestep, true);
                            }
                        }
                    } else if let Some(smaller_timestep) = constraint.smaller_timestep() {
                        // Check if we have enough time to simulate with a smaller timestep.
                        if smaller_timestep > adjusted_step_time {
                            match constraint.decrease_timestep() {
                                Err(error) => {
                                    warn!("Failed to decrease physics timestep! Error: {}", error);
                                }
                                Ok(new_timestep) => {
                                    info!(
                                        "Decreasing physics timestep to {:.8} seconds",
                                        new_timestep
                                    );
                                    timestep = (new_timestep, true);
                                }
                            }
                        }
                    }
                }
                timestep
            }
        };
        if physical_world.timestep() != timestep && !change_timestep {
            warn!("Physics world timestep out of sync with intended timestep! Leave me alone!!!");
            change_timestep = true;
        }
        if change_timestep {
            // reset average when changing timestep
            self.avg_step_time = None;
            physical_world.set_timestep(timestep);
        }

        self.time_accumulator += time.delta_seconds();
        let mut steps = 0;

        while steps <= self.max_timesteps && self.time_accumulator >= timestep {
            let physics_time = Instant::now();

            physical_world.step();

            let physics_time = physics_time.elapsed();
            let physics_time =
                physics_time.as_secs() as f32 + physics_time.subsec_nanos() as f32 * 1e-9;
            self.avg_step_time = Some(match self.avg_step_time {
                None => physics_time,
                Some(avg) => {
                    // calculate exponentially weighted moving average
                    // basic formula: AVG_n = alpha * value_n + (1 - alpha) * AVG_n-1
                    avg + AVERAGE_STEP_TIME_FALLOFF * (physics_time - avg)
                }
            });
            self.time_accumulator -= timestep;
            steps += 1;
        }

        trace!(
            "Average time per physics step: {:.8} seconds",
            self.avg_step_time.unwrap_or_default()
        );
    }
}

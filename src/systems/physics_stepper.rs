use crate::time_step::TimeStep;
use crate::PhysicsWorld;
use amethyst::core::Time;
use amethyst::ecs::{Read, System, WriteExpect};

/// Simulates a step of the physics world.
pub struct PhysicsStepperSystem {
    intended_timestep: TimeStep,
    max_timesteps: i32,
    time_accumulator: f32,
}

impl Default for PhysicsStepperSystem {
    fn default() -> Self {
        PhysicsStepperSystem {
            intended_timestep: TimeStep::Fixed(1. / 120.),
            max_timesteps: 10,
            time_accumulator: 0.,
        }
    }
}

impl PhysicsStepperSystem {
    pub fn new(intended_timestep: TimeStep, max_timesteps: i32) -> Self {
        PhysicsStepperSystem {
            intended_timestep,
            max_timesteps,
            time_accumulator: 0.,
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
                if constraint.should_increase_timestep() {
                    match constraint.increase_timestep() {
                        Err(error) => {
                            warn!("Failed to raise physics timestep! Error: {}", error);
                            (constraint.current_timestep(), false)
                        }
                        Ok(new_timestep) => {
                            info!("Raising physics timestep to {} seconds", new_timestep);
                            (new_timestep, true)
                        }
                    }
                } else {
                    (constraint.current_timestep(), false)
                }
            }
        };
        if physical_world.timestep() != timestep && !change_timestep {
            warn!("Physics world timestep out of sync with intended timestep! Leave me alone!!!");
            change_timestep = true;
        }
        if change_timestep {
            physical_world.set_timestep(timestep);
        }

        self.time_accumulator += time.delta_seconds();
        let mut steps = 0;

        while steps <= self.max_timesteps && self.time_accumulator >= timestep {
            physical_world.step();
            self.time_accumulator -= timestep;
            steps += 1;
        }

        if let TimeStep::SemiFixed(constraint) = &mut self.intended_timestep {
            let is_running_slow = steps > self.max_timesteps;
            constraint.set_running_slow(is_running_slow);
        }
    }
}

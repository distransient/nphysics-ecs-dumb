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
        let timestep = match &self.intended_timestep {
            TimeStep::Fixed(timestep) => *timestep,
            TimeStep::SemiFixed(constraint) => constraint.current_timestep(),
        };
        if physical_world.timestep() != timestep {
            warn!("Physics world timestep out of sync with intended timestep! Leave me alone!!!");
            physical_world.set_timestep(timestep);
        }

        self.time_accumulator += time.delta_seconds();
        let mut steps = 0;

        while steps <= self.max_timesteps && self.time_accumulator >= timestep {
            physical_world.step();
            self.time_accumulator -= timestep;
            steps += 1;
        }

        if steps > self.max_timesteps {
            if let TimeStep::SemiFixed(constraint) = &mut self.intended_timestep {
                if let Err(error) = constraint.increase_timestep() {
                    warn!("Failed to raised physics timestep! Error: {}", error);
                } else {
                    physical_world.set_timestep(constraint.current_timestep());
                    info!("Raised physics timestep");
                }
            }
        }
    }
}

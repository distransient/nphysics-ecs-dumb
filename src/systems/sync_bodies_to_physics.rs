use crate::bodies::DynamicBody;
use crate::PhysicsWorld;
use amethyst::core::GlobalTransform;
use amethyst::ecs::storage::{ComponentEvent, GenericReadStorage, MaskedStorage};
use amethyst::ecs::{
    BitSet, Component, Entities, Join, ReadStorage, ReaderId, Resources, Storage, System,
    SystemData, Tracked, WriteExpect, WriteStorage,
};
use core::ops::Deref;
use nalgebra::try_convert;
use nphysics3d::math::{Inertia, Force, Isometry};

#[derive(Default)]
pub struct SyncBodiesToPhysicsSystem {
    transforms_reader_id: Option<ReaderId<ComponentEvent>>,
    physics_bodies_reader_id: Option<ReaderId<ComponentEvent>>,
}

impl SyncBodiesToPhysicsSystem {
    pub fn new() -> Self {
        Default::default()
    }
}

impl<'a> System<'a> for SyncBodiesToPhysicsSystem {
    type SystemData = (
        WriteExpect<'a, PhysicsWorld>,
        Entities<'a>,
        ReadStorage<'a, GlobalTransform>,
        WriteStorage<'a, DynamicBody>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut physical_world, entities, transforms, mut physics_bodies) = data;

        let mut inserted_transforms = BitSet::new();
        let mut modified_transforms = BitSet::new();
        let mut inserted_physics_bodies = BitSet::new();
        let mut modified_physics_bodies = BitSet::new();

        // Get change flag events for transforms, removing deleted ones from the physics world.
        trace!("Iterating transform storage events.");
        iterate_events(
            &transforms,
            self.transforms_reader_id.as_mut().unwrap(),
            &mut inserted_transforms,
            &mut modified_transforms,
            &mut physical_world,
            &entities,
            &physics_bodies,
        );

        // Get change flag events for physics bodies, removing deleted ones from the physics world.
        trace!("Iterating physics body storage events.");
        iterate_events(
            &physics_bodies,
            self.physics_bodies_reader_id.as_mut().unwrap(),
            &mut inserted_physics_bodies,
            &mut modified_physics_bodies,
            &mut physical_world,
            &entities,
            &physics_bodies,
        );

        // Update simulation world with the value of Components flagged as changed
        #[allow(unused_mut)]
        for (_entity, transform, mut body, id) in (
            &entities,
            &transforms,
            &mut physics_bodies,
            &modified_transforms
                | &inserted_transforms
                | &modified_physics_bodies
                | &inserted_physics_bodies,
        )
            .join()
        {
            if inserted_transforms.contains(id) || inserted_physics_bodies.contains(id) {
                trace!("Detected inserted dynamics body with id {}", id);

                match body {
                    DynamicBody::RigidBody(ref mut rigid_body) => {
                        // Just inserted. Remove old one and insert new.
                        if let Some(handle) = rigid_body.handle {
                            if physical_world.rigid_body(handle).is_some() {
                                trace!("Removing body marked as inserted that already exists with handle: {:?}", handle);
                                physical_world.remove_bodies(&[handle]);
                            }
                        }

                        rigid_body.handle = Some(physical_world.add_rigid_body(
                            try_convert(transform.0).unwrap(),
                            Inertia::new(rigid_body.mass, rigid_body.angular_mass),
                            rigid_body.center_of_mass,
                        ));

                        trace!("Inserted rigid body to world with values: {:?}", rigid_body);

                        let physical_body = physical_world
                            .rigid_body_mut(rigid_body.handle.unwrap())
                            .unwrap();

                        physical_body.set_velocity(rigid_body.velocity);
                        physical_body.apply_force(&rigid_body.external_forces);
                        rigid_body.external_forces = Force::<f32>::zero();

                        physical_body.set_status(rigid_body.body_status);

                        trace!("Velocity and external forces applied, external forces reset to zero, for body with handle: {:?}", rigid_body.handle);
                    }
                    DynamicBody::Multibody(_) => {
                        // TODO
                        error!("Multibody found; not implemented currently, sorry!")
                    }
                }
            } else if modified_transforms.contains(id) || modified_physics_bodies.contains(id) {
                trace!("Detected changed dynamics body with id {}", id);
                match body {
                    DynamicBody::RigidBody(ref mut rigid_body) => {
                        match physical_world.rigid_body_mut(rigid_body.handle.unwrap()) {
                            Some(physical_body) => {
                                trace!("Updating rigid body in physics world with isometry: {}", position);
                                match try_convert(transform.0) {
                                    Some(p) => {
                                        let position: Isometry<f32> = p;
                                        physical_body.set_position(position);
                                        physical_body.set_velocity(rigid_body.velocity);
                                        physical_body.apply_force(&rigid_body.external_forces);
                                        physical_body.set_status(rigid_body.body_status);
                                    },
                                    None => error!("Failed to convert entity position from `Transform` to physics systems"),
                                }

                                // if you changed the mass properties at all... too bad!
                            },
                            None => {

                            }
                        }
                    }
                    DynamicBody::Multibody(_) => {
                        // TODO
                        error!("Multibody found; not implemented currently, sorry!")
                    }
                }
            }
        }
    }

    fn setup(&mut self, res: &mut Resources) {
        Self::SystemData::setup(res);

        let mut transform_storage: WriteStorage<GlobalTransform> = SystemData::fetch(&res);
        self.transforms_reader_id = Some(transform_storage.register_reader());

        let mut physics_body_storage: WriteStorage<DynamicBody> = SystemData::fetch(&res);
        self.physics_bodies_reader_id = Some(physics_body_storage.register_reader());
    }
}

fn iterate_events<'a, T, D, S>(
    tracked_storage: &Storage<T, D>,
    reader: &mut ReaderId<ComponentEvent>,
    inserted: &mut BitSet,
    modified: &mut BitSet,
    world: &mut PhysicsWorld,
    entities: &Entities,
    bodies: &S,
) where
    T: Component,
    T::Storage: Tracked,
    D: Deref<Target = MaskedStorage<T>>,
    S: GenericReadStorage<Component = DynamicBody>,
{
    let events = tracked_storage.channel().read(reader);

    for event in events {
        match event {
            ComponentEvent::Modified(id) => {
                modified.add(*id);
            }
            ComponentEvent::Inserted(id) => {
                inserted.add(*id);
            }
            ComponentEvent::Removed(id) => {
                match bodies.get(entities.entity(*id)) {
                    Some(body) => {
                        match body.handle() {
                            Some(handle) => {
                                trace!("Removing body with id: {}", id);

                                world.remove_bodies(&[handle]);
                            }
                            None => {
                                error!("Missing handle in body: {}", id);
                            }
                        };
                    }
                    None => {
                        error!("Missing body with id: {}", id);
                    }
                };
            }
        };
    }
}

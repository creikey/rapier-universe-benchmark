use rapier2d::dynamics::{RigidBody, RigidBodyHandle};
use rapier2d::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase, SharedShape};
use rapier2d::na::Isometry2;
use rapier2d::na::Vector2 as phVector2;
use rapier2d::pipeline::PhysicsPipeline;
use rapier2d::{
    dynamics::{CCDSolver, IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet},
    na::Translation2,
};
use raylib::prelude::*;
use serde::{Deserialize, Serialize};
use std::hash::{Hash, Hasher};

const GRAV_CONSTANT: f32 = 0.0001;
const FRAMES_IN_HASH: u32 = 60 * 5;

fn pos_of(b: &RigidBody) -> Vector2 {
    let position = b.position().translation;
    Vector2::new(position.x, position.y)
}

fn into_ph(v: Vector2) -> phVector2<f32> {
    phVector2::new(v.x, v.y)
}

// objects are serialized so they can be hashed. it is more of a PITA to implement a custom
// hash for floats than to just hash the serialized data
#[derive(Serialize, Deserialize)]
#[serde(remote = "Vector2")]
struct Vector2Def {
    x: f32,
    y: f32,
}

#[derive(Serialize, Deserialize)]
struct Planet {
    #[serde(with = "Vector2Def")]
    pos: Vector2,
    #[serde(with = "Vector2Def")]
    velocity: Vector2,

    rotation: f32,
    rotational_velocity: f32,
    radius: f32,

    #[serde(skip, default = "RigidBodyHandle::invalid")]
    body_ref: RigidBodyHandle,
}

#[derive(Serialize, Deserialize)]
struct UniverseState {
    planets: Vec<Planet>,
}

impl UniverseState {
    fn new() -> Self {
        UniverseState {
            planets: Vec::new(),
        }
    }
    /// Should be called after deserializing to instantiate and apply physical body characteristics
    /// read from a file. Should also be called after adding planets so that they are simulated
    /// physically.
    fn apply_physically(&mut self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
        for o in self.planets.iter_mut() {
            match bodies.get_mut(o.body_ref) {
                // body already exists, update from my value
                Some(b) => {
                    b.set_position(Isometry2::new(into_ph(o.pos), o.rotation), true);
                    b.set_linvel(into_ph(o.velocity), true);
                    b.set_angvel(o.rotational_velocity, true);
                }
                None => {
                    let planet = RigidBodyBuilder::new_dynamic()
                        .position(Isometry2::new(into_ph(o.pos), 0.0))
                        .linvel(o.velocity.x, o.velocity.y)
                        .rotation(o.rotation)
                        .angvel(o.rotational_velocity)
                        .build();
                    o.body_ref = bodies.insert(planet);
                    let planet_collider = ColliderBuilder::new(SharedShape::ball(o.radius))
                        .restitution(1.0)
                        .build();
                    let _planet_collider_handle =
                        colliders.insert(planet_collider, o.body_ref, bodies);
                }
            }
        }
    }
    fn update_from_physics(&mut self, bodies: &RigidBodySet, colliders: &ColliderSet) {
        for o in self.planets.iter_mut() {
            let body = bodies.get(o.body_ref).unwrap();
            o.pos = pos_of(body);
            let linvel = body.linvel();
            o.velocity = Vector2::new(linvel.x, linvel.y);
            o.rotation = body.position().rotation.re;
            o.rotational_velocity = body.angvel();
        }
    }
    fn new_planet(&mut self, radius: f32, pos: Vector2) {
        self.planets.push(Planet {
            pos: pos,
            velocity: Vector2::zero(),
            rotation: 0.0,
            rotational_velocity: 0.0,
            radius: radius,
            body_ref: RigidBodyHandle::invalid(),
        });
    }
}

#[derive(Hash)]
struct HashedStr<'a>(&'a str);

fn main() {
    let program_git_hash = env!("GIT_HASH");

    // Here the gravity is -9.81 along the y axis.
    let mut pipeline = PhysicsPipeline::new();
    let gravity = phVector2::new(0.0, 0.0);
    let integration_parameters = IntegrationParameters::default();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();
    let mut ccd_solver = CCDSolver::new();
    // We ignore physics hooks and contact events for now.
    let physics_hooks = ();
    let event_handler = ();

    let mut universe = UniverseState::new();

    universe.new_planet(1000.0, Vector2::new(0.0, -1100.0));
    universe.new_planet(10.0, Vector2::zero());

    universe.apply_physically(&mut bodies, &mut colliders);

    let screen_size = Vector2::new(900.0, 900.0);
    let (mut rl, thread) = raylib::init()
        .size(screen_size.x as i32, screen_size.y as i32)
        .title("Rapier Physics Benchmark")
        .build();

    rl.set_target_fps(60);

    let mut camera = Camera2D {
        zoom: 0.5,
        offset: screen_size / 2.0,
        target: Vector2::new(0.0, 0.0),
        rotation: 0.0,
    };

    let mut frames_simulated = 0;
    let mut world_state_hash: String = String::from("processing...");
    let mut world_state_hashed = false;
    while !rl.window_should_close() {
        pipeline.step(
            &gravity,
            &integration_parameters,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut joints,
            &mut ccd_solver,
            &physics_hooks,
            &event_handler,
        );

        frames_simulated += 1;

        if world_state_hashed && rl.is_key_pressed(KeyboardKey::KEY_C) {
            rl.set_clipboard_text(&world_state_hash).unwrap();
        }

        if frames_simulated < FRAMES_IN_HASH {
            world_state_hash = format!("frames left: {}", FRAMES_IN_HASH - frames_simulated);
        } else if frames_simulated == FRAMES_IN_HASH {
            // TODO create hash here
            let mut hasher = std::collections::hash_map::DefaultHasher::new();
            universe.update_from_physics(&bodies, &colliders);
            let universe_ron_string = ron::to_string(&universe).unwrap();
            universe = ron::from_str(&universe_ron_string).unwrap();
            universe_ron_string.as_str().hash(&mut hasher);
            world_state_hash = format!("{}", hasher.finish());
            world_state_hashed = true;

            // now that I updated the universe from the serialized state, I need to clear the
            // physics stuff and remake it from the deserialized object
            let mut to_remove = Vec::with_capacity(bodies.len());
            for (handle, _body) in bodies.iter() {
                to_remove.push(handle);
            }
            for body_handle in to_remove.into_iter() {
                bodies.remove(body_handle, &mut colliders, &mut joints);
            }
            universe.apply_physically(&mut bodies, &mut colliders);
        }

        // gravitational force
        let planet_handle = universe.planets[0].body_ref;
        let cur_planet = bodies.get(planet_handle).unwrap();
        let planet_center = pos_of(cur_planet);
        let planet_mass = cur_planet.mass();
        for (handle, body) in bodies.iter_mut() {
            if handle == planet_handle {
                continue;
            }
            let body_to_planet = (planet_center - pos_of(body)).normalized();
            let distance_sqr = body_to_planet.length_sqr();
            let gravity_magnitude = GRAV_CONSTANT * ((planet_mass * body.mass()) / distance_sqr);
            body.apply_force(into_ph(body_to_planet * gravity_magnitude), true);
        }

        // camera zooming in and out
        camera.zoom = ((rl.get_time().sin() + 1.1) / 3.0) as f32;

        // -- DRAWING START
        // two drawing objects so camera can be used
        let mut global_canvas = rl.begin_drawing(&thread);
        let mut d = global_canvas.begin_mode2D(camera);

        d.clear_background(Color::WHITE);

        // draw circular bodies
        let mut drawn_bodies = 0;
        for (_handle, body) in bodies.iter() {
            if body.colliders().len() <= 0 {
                continue;
            }
            drawn_bodies += 1;
            match colliders
                .get(body.colliders()[0])
                .unwrap()
                .shape()
                .as_ball()
            {
                Some(ball) => {
                    let body_translation = body.position().translation;
                    d.draw_circle_v(
                        Vector2::new(body_translation.x, -body_translation.y),
                        ball.radius,
                        Color::BLACK,
                    );
                }
                None => (),
            }
        }
        //println!("Drawn bodies: {}", drawn_bodies);

        // draw axis
        d.draw_line_v(
            Vector2::new(0.0, -100.0),
            Vector2::new(0.0, 100.0),
            Color::GREEN,
        );
        d.draw_line_v(
            Vector2::new(100.0, 0.0),
            Vector2::new(-100.0, 0.0),
            Color::BLUE,
        );

        drop(d);

        // draw hash information
        let info_font_size = 16;
        global_canvas.draw_text(
            &format!("program hash: {}", program_git_hash),
            0,
            0,
            info_font_size,
            Color::RED,
        );
        global_canvas.draw_text(
            &format!("world state hash: {}", world_state_hash),
            0,
            30,
            info_font_size,
            Color::RED,
        );
        if world_state_hashed {
            global_canvas.draw_text(
                "Press c to copy world hash to clipboard",
                0,
                60,
                info_font_size,
                Color::RED,
            );
        }
    }
}

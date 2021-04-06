use rapier2d_f64::dynamics::{RigidBody, RigidBodyHandle};
use rapier2d_f64::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase, SharedShape};
use rapier2d_f64::na::{Isometry2, ComplexField};
use rapier2d_f64::na::Vector2;
use rapier2d_f64::pipeline::PhysicsPipeline;
use rapier2d_f64::{
    dynamics::{CCDSolver, IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet},
    na::Translation2,
};
use raylib::prelude::Vector2 as rlVector2;
use raylib::prelude::*;
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};
use std::{
    f64::consts::PI,
    hash::{Hash, Hasher},
};

type V = Vector2<f64>;

const GRAV_CONSTANT: f64 = 0.00001;
const FRAMES_IN_HASH: u32 = 60 * 5;

fn pos_of(b: &RigidBody) -> V {
    let position = b.position().translation;
    V::new(position.x, position.y)
}

fn from_raylib(v: rlVector2) -> V {
    V::new(v.x as f64, v.y as f64)
}

fn into_raylib(v: V) -> rlVector2 {
    rlVector2::new(v.x as f32, v.y as f32)
}

fn v_lerp(from: V, to: V, weight: f64) -> V {
    from + (to - from) * weight
}

// objects are serialized so they can be hashed. it is more of a PITA to implement a custom
// hash for floats than to just hash the serialized data
#[derive(Serialize, Deserialize)]
struct Planet {
    pos: V,
    velocity: V,

    rotation: f64,
    rotational_velocity: f64,
    radius: f64,

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
                    b.set_position(Isometry2::new(o.pos, o.rotation), true);
                    b.set_linvel(o.velocity, true);
                    b.set_angvel(o.rotational_velocity, true);
                }
                None => {
                    let planet = RigidBodyBuilder::new_dynamic()
                        .position(Isometry2::new(o.pos, 0.0))
                        .linvel(o.velocity.x, o.velocity.y)
                        .rotation(o.rotation)
                        .angvel(o.rotational_velocity)
                        .build();
                    o.body_ref = bodies.insert(planet);
                    let planet_collider = ColliderBuilder::new(SharedShape::ball(o.radius))
                        .restitution(0.8)
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
            o.velocity = V::new(linvel.x, linvel.y);
            o.rotation = body.position().rotation.re;
            o.rotational_velocity = body.angvel();
        }
    }
    fn new_with_vel(&mut self, radius: f64, pos: V, vel: V) {
        self.planets.push(Planet {
            pos: pos,
            velocity: vel,
            rotation: 0.0,
            rotational_velocity: 0.0,
            radius: radius,
            body_ref: RigidBodyHandle::invalid(),
        });
    }
    fn new_planet(&mut self, radius: f64, pos: V) {
        self.planets.push(Planet {
            pos: pos,
            velocity: V::default(),
            rotation: 0.0,
            rotational_velocity: 0.0,
            radius: radius,
            body_ref: RigidBodyHandle::invalid(),
        });
    }
}
fn spawn_layer_of_planets(
    universe: &mut UniverseState,
    planet_size: f64,
    radius: f64,
    tangential_speed: f64,
    num_planets_in_layer: i32,
) {
    for i in 0..num_planets_in_layer {
        let angle = ((i as f64) / (num_planets_in_layer as f64)) * (2.0 * PI);
        let pos = V::new(ComplexField::sin(angle), ComplexField::cos(angle));
        let vel_angle = pos.angle(&V::x()) + PI / 2.0;
        let vel = V::new(ComplexField::cos(vel_angle), ComplexField::sin(vel_angle)) * tangential_speed;
        universe.new_with_vel(planet_size, pos * radius, vel);
    }
}

fn main() {
    let program_git_hash = env!("GIT_HASH");

    // -- PHYSICS STUFF --
    let mut pipeline = PhysicsPipeline::new();
    let gravity = V::new(0.0, 0.0);
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

    // -- MAKING THE UNIVERSE --
    let mut universe = UniverseState::new();

    universe.new_planet(10.0, V::default());
    universe.new_planet(100.0, V::new(600.0, 100.0));
    universe.new_planet(100.0, V::new(600.0, 300.0));
    universe.new_planet(10_000.0, V::new(13_000.0, 0.0));
    //universe.new_planet(100_000.0, V::new(0.0, -150_000.0));
    spawn_layer_of_planets(&mut universe, 500.0, 10_000.0, 1000.0, 5);
    spawn_layer_of_planets(&mut universe, 300.0, 6_000.0, 4000.0, 9);
    spawn_layer_of_planets(&mut universe, 300.0, 3_000.0, -4000.0, 9);
    spawn_layer_of_planets(&mut universe, 300.0, 4_000.0, -4000.0, 13);

    universe.apply_physically(&mut bodies, &mut colliders);

    // -- INITIALIZING RAYLIB STUFF --
    let screen_size = V::new(900.0, 900.0);
    let (mut rl, thread) = raylib::init()
        .size(screen_size.x as i32, screen_size.y as i32)
        .title("Rapier Physics Benchmark")
        .build();
    let background_texture = rl
        .load_texture_from_image(
            &thread,
            &Image::gen_image_checked(512, 512, 256, 256, Color::WHITE, Color::GRAY),
        )
        .unwrap();
    rl.set_target_fps(60);

    let mut camera = Camera2D {
        zoom: 0.5,
        offset: into_raylib(screen_size / 2.0),
        target: into_raylib(V::new(0.0, 0.0)),
        rotation: 0.0,
    };

    // -- VARS RELATED TO THE WORLD HASH 5 SECONDS IN --
    let mut frames_simulated = 0;
    let mut world_state_hash_message: String = String::from("processing...");
    let mut world_state_hashed = false;
    let mut world_state_ron_string: String = String::new();

    while !rl.window_should_close() {
        rl.get_time();
        // simulate physics
        let physics_instant = Instant::now();
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
        let physics_processing_micros = physics_instant.elapsed().as_micros();

        frames_simulated += 1;

        // process user input
        if world_state_hashed && rl.is_key_pressed(KeyboardKey::KEY_C) {
            rl.set_clipboard_text(&world_state_hash_message).unwrap();
        }
        if world_state_hashed && rl.is_key_pressed(KeyboardKey::KEY_S) {
            rl.set_clipboard_text(&&world_state_ron_string).unwrap();
        }
        let zoom_change = (rl.is_key_down(KeyboardKey::KEY_Q) as u8 as f64)
            - (rl.is_key_down(KeyboardKey::KEY_E) as u8 as f64);
        camera.zoom += (zoom_change as f32) * rl.get_frame_time() * 0.5;
        camera.zoom = camera.zoom.clamp(0.01, 10.0) as f32;

        // set hash stuff if countdown is done
        if frames_simulated < FRAMES_IN_HASH {
            world_state_hash_message =
                format!("frames left: {}", FRAMES_IN_HASH - frames_simulated);
        } else if frames_simulated == FRAMES_IN_HASH {
            universe.update_from_physics(&bodies, &colliders);

            world_state_ron_string = ron::ser::to_string(&universe).unwrap();

            let mut hasher = std::collections::hash_map::DefaultHasher::new();
            world_state_ron_string.hash(&mut hasher);
            world_state_hash_message = format!("{}", hasher.finish());

            world_state_hashed = true;

            // deserialize the universe from the serialized string to double check that serializing
            // works
            universe = ron::from_str(&world_state_ron_string).unwrap();
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
        let mut rigid_body_gravity_properties: Vec<(f64, V)> = Vec::with_capacity(bodies.len());
        // TODO use the handles to check if they're the same instead of the bad distance_to call
        for (_handle, b) in bodies.iter() {
            rigid_body_gravity_properties.push((b.mass(), pos_of(b)));
        }
        // sum this up over time as iterating over each body anyways
        let mut average_position = V::default();

        for (_cur_handle, body) in bodies.iter_mut() {
            average_position += pos_of(body);
            for (other_body_mass, other_body_position) in rigid_body_gravity_properties.iter() {
                let body_to_planet = (*other_body_position - pos_of(body)).normalize();
                let distance_sqr = body_to_planet.magnitude_squared();
                if distance_sqr < 0.1 || distance_sqr.is_nan() {
                    // on top of eachother or the same body
                    continue;
                }
                let gravity_magnitude =
                    GRAV_CONSTANT * ((other_body_mass * body.mass()) / distance_sqr);
                body.apply_force(body_to_planet * gravity_magnitude, true);
            }
        }

        average_position /= bodies.len() as f64;

        // follow a tiny ball to see if physics still works
        let target = pos_of(bodies.get(universe.planets[0].body_ref).unwrap())
            .component_mul(&V::new(1.0, -1.0));
        //camera.target = v_lerp(camera.target, target, rl.get_frame_time() * 13.0);
        camera.target = into_raylib(target);

        // -- DRAWING START
        // two drawing objects so camera can be used while still drawing UI
        let mut global_canvas = rl.begin_drawing(&thread);

        global_canvas.clear_background(Color::WHITE);

        let mut d = global_canvas.begin_mode2D(camera);

        d.draw_texture_tiled(
            &background_texture,
            Rectangle::new(0.0, 0.0, 512.0, 512.0),
            Rectangle::new(-5000.0, -5000.0, 10000.0, 10000.0),
            into_raylib(V::default()),
            0.0,
            1.0,
            Color::GRAY,
        );

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
                        into_raylib(V::new(body_translation.x, -body_translation.y)),
                        ball.radius as f32,
                        Color::BLACK,
                    );
                }
                None => (),
            }
        }

        // draw axis
        let axis_origin = from_raylib(camera.target);
        d.draw_line_v(
            into_raylib(axis_origin + V::new(0.0, -100.0)),
            into_raylib(axis_origin + V::new(0.0, 100.0)),
            Color::GREEN,
        );
        d.draw_line_v(
            into_raylib(axis_origin + V::new(100.0, 0.0)),
            into_raylib(axis_origin + V::new(-100.0, 0.0)),
            Color::BLUE,
        );

        drop(d);

        // draw hash information and other help
        let info_font_size = 16;
        let info_texts = [
            format!(
                "physics processing millis: {}",
                (physics_processing_micros as f32) / 1000.0
            ),
            format!("program hash: {}", program_git_hash),
            format!("world state hash: {}", world_state_hash_message),
            String::from("use the q and e keys to zoom in and out"),
            String::from("Press c to copy world hash to clipboard"),
            String::from("Press s to copy world state serialized to clipboard"),
        ];
        let mut height = 0;
        for text in info_texts.iter() {
            global_canvas.draw_text(text, 0, height, info_font_size, Color::RED);
            height += 30;
        }
    }
}

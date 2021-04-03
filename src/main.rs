use rapier2d::dynamics::RigidBody;
use rapier2d::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase, SharedShape};
use rapier2d::na::Isometry2;
use rapier2d::na::Vector2 as phVector2;
use rapier2d::pipeline::PhysicsPipeline;
use rapier2d::{
    dynamics::{CCDSolver, IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet},
    na::Translation2,
};
use raylib::prelude::*;
use std::ops::Deref;

const GRAV_CONSTANT: f32 = 0.0000001;

fn pos_of(b: &RigidBody) -> Vector2 {
    let position = b.position().translation;
    Vector2::new(position.x, position.y)
}

fn into_ph(v: Vector2) -> phVector2<f32> {
    phVector2::new(v.x, v.y)
}

fn main() {
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

    let ball = RigidBodyBuilder::new_dynamic().build();
    let ball_handle = bodies.insert(ball);

    let ball_collider = ColliderBuilder::new(SharedShape::ball(10.0)).restitution(1.0).build();
    let _ball_collider_handle = colliders.insert(ball_collider, ball_handle, &mut bodies);

    let planet = RigidBodyBuilder::new_dynamic()
        .position(Isometry2::new(phVector2::new(0.0, -2100.0), 0.0))
        .build();
    let planet_handle = bodies.insert(planet);
    let planet_collider = ColliderBuilder::new(SharedShape::ball(2000.0))
        .restitution(1.0)
        .build();
    let _planet_collider_handle = colliders.insert(planet_collider, planet_handle, &mut bodies);

    let screen_size = Vector2::new(900.0, 900.0);
    let (mut rl, thread) = raylib::init()
        .size(screen_size.x as i32, screen_size.y as i32)
        .title("Rapier Physics Benchmark")
        .build();

    let mut camera = Camera2D {
        zoom: 0.5,
        offset: screen_size / 2.0,
        target: Vector2::new(0.0, 0.0),
        rotation: 0.0,
    };

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

        // gravitational force
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
        //println!("Drawn bodies: {}", drawn_bodies);
    }
}

use itertools::Itertools;
use piston_window::*;
use std::fmt;

#[derive(Debug)]
struct Vector {
    x: f64,
    y: f64,
}

#[derive(Debug, PartialEq)]
struct Position {
    x: f64,
    y: f64,
}

impl fmt::Display for Position {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({},{})", self.x, self.y)
    }
}

fn v(x: f64, y: f64) -> Vector {
    Vector { x, y }
}

fn pos(x: f64, y: f64) -> Position {
    Position { x, y }
}

macro_rules! positions {
    ($(($x:expr, $y:expr)),*) => {
        [$(crate::Position{ x: $x, y: $y }),*]
    }
}

#[derive(Debug)]
struct UniformBody {
    mass: f64,
    pos: Position,
    acceleration: Vector,
    velocity: Vector,
}

impl UniformBody {
    fn still_body(m: f64, p: Position) -> UniformBody {
        UniformBody {
            mass: m,
            pos: p,
            acceleration: v(0.0, 0.0),
            velocity: v(0.0, 0.0),
        }
    }

    fn apply_force(&mut self, fx: f64, fy: f64) {
        self.acceleration.x = self.acceleration.x + (fx / self.mass);
        self.acceleration.y = self.acceleration.y + (fy / self.mass);
    }

    fn set_resulting_force(&mut self, fx: f64, fy: f64) {
        self.acceleration.x = fx / self.mass;
        self.acceleration.y = fy / self.mass;
    }
}

struct Engine {
    bodies: Vec<UniformBody>,
    ga: f64,
}

impl Engine {
    fn create(g: f64) -> Engine {
        Engine {
            bodies: vec![],
            ga: g,
        }
    }
}

struct ViewPort {
    /// Origin of viewport. It's the top left corner of
    /// the view port in meters.
    origin: Position,
    /// Ratio meter/pixel.
    ratio: f64,
}

impl ViewPort {
    fn translate_pos(&self, real_pos: &Position) -> Position {
        Position {
            x: (real_pos.x - self.origin.x) / self.ratio,
            y: (self.origin.y - real_pos.y) / self.ratio,
        }
    }

    fn translate_size(&self, size: f64) -> f64 {
        size / self.ratio
    }
}

impl Engine {
    fn tick(&mut self, dt: f64) {
        for body in self.bodies.iter_mut() {
            let ax = body.acceleration.x;
            let ay = body.acceleration.y - self.ga;
            let vx = body.velocity.x + (ax * dt);
            let vy = body.velocity.y + (ay * dt);
            let sx = (dt / 2.0) * (vx + body.velocity.x);
            let sy = (dt / 2.0) * (vy + body.velocity.y);
            body.velocity = v(vx, vy);
            body.pos = pos(body.pos.x + sx, body.pos.y + sy);
        }
    }

    fn add_body(&mut self, b: UniformBody) {
        self.bodies.push(b);
    }

    fn get_bodies(&self) -> &[UniformBody] {
        self.bodies.as_slice()
    }

    fn get_bodies_mut(&mut self) -> &mut [UniformBody] {
        self.bodies.as_mut_slice()
    }
}

// Projects point `position` onto a line with gradient
// `line_gradient` and y-interception 0.
fn project(position: &Position, line_gradient: f64) -> Position {
    let p = position;
    let a = line_gradient;

    if line_gradient == f64::INFINITY {
        pos(0.0, p.y)
    } else if line_gradient.abs() < f64::EPSILON {
        pos(p.x, 0.0)
    } else {
        let a_orth = -1.0 / a;
        let b_orth = p.y - (a_orth * p.x);
        let projected_x = (-b_orth) / (a_orth - a);
        let projected_y = a * projected_x;
        pos(projected_x, projected_y)
    }
}

// Checks for collision between two convex polygons using
// the "separating axis theorem" approach.
fn collided(shape1: &[Position], shape2: &[Position]) -> bool {
    shape1.iter().circular_tuple_windows().all(|(p1, p2)| {
        let a = (p1.y - p2.y) / (p1.x - p2.x);
        let a_orth = -1.0 / a;

        let shape1_projections = shape1
            .iter()
            .map(|p| project(&p, a_orth))
            .collect::<Vec<_>>();

        let shape2_projections = shape2
            .iter()
            .map(|p| project(&p, a_orth))
            .collect::<Vec<_>>();

        let shape1_min = shape1_projections
            .iter()
            .fold(pos(f64::MAX, f64::MAX), |min_p, p| {
                pos(min_p.x.min(p.x), min_p.y.min(p.y))
            });

        let shape1_max = shape1_projections
            .iter()
            .fold(pos(f64::MIN, f64::MIN), |max_p, p| {
                pos(max_p.x.max(p.x), max_p.y.max(p.y))
            });

        let shape2_min = shape2_projections
            .iter()
            .fold(pos(f64::MAX, f64::MAX), |min_p, p| {
                pos(min_p.x.min(p.x), min_p.y.min(p.y))
            });

        let shape2_max = shape2_projections
            .iter()
            .fold(pos(f64::MIN, f64::MIN), |max_p, p| {
                pos(max_p.x.max(p.x), max_p.y.max(p.y))
            });

        (shape1_max.x >= shape2_min.x || shape1_max.y >= shape2_min.y)
            && (shape2_max.x >= shape1_min.x && shape2_max.y >= shape1_min.y)
    })
}

fn main() {
    let viewport = ViewPort {
        origin: pos(0.0, 100.0),
        ratio: 0.15,
    };

    let mut window: PistonWindow = WindowSettings::new(
        "Hello Piston!",
        [
            viewport.translate_size(100.0),
            viewport.translate_size(100.0),
        ],
    )
    .exit_on_esc(true)
    .build()
    .unwrap();

    let mut engine = Engine::create(1.625);
    engine.add_body(UniformBody::still_body(10.0, pos(50.0, 100.0)));

    while let Some(event) = window.next() {
        window.draw_2d(&event, |context, graphics, _device| {
            clear([1.0; 4], graphics);
            let border = Ellipse::new_border([1.0, 0.0, 0.0, 1.0], viewport.translate_size(1.0));
            let body = &engine.get_bodies()[0];
            let pos = viewport.translate_pos(&body.pos);
            border.draw(
                rectangle::centered_square(pos.x, pos.y, viewport.translate_size(1.0)),
                &context.draw_state,
                context.transform,
                graphics,
            );
        });

        if let Some(update_args) = event.update_args() {
            engine.tick(update_args.dt);
        }

        if let Some(button_args) = event.button_args() {
            let body = engine.get_bodies_mut().first_mut().unwrap();
            match button_args {
                ButtonArgs {
                    state,
                    button: Button::Keyboard(Key::Down),
                    ..
                } => match state {
                    ButtonState::Press => body.apply_force(0.0, 100.0),
                    ButtonState::Release => body.apply_force(0.0, -100.0),
                },
                ButtonArgs {
                    state,
                    button: Button::Keyboard(Key::Right),
                    ..
                } => match state {
                    ButtonState::Press => body.apply_force(-100.0, 0.0),
                    ButtonState::Release => body.apply_force(100.0, 0.0),
                },
                ButtonArgs {
                    state,
                    button: Button::Keyboard(Key::Left),
                    ..
                } => match state {
                    ButtonState::Press => body.apply_force(100.0, 0.0),
                    ButtonState::Release => body.apply_force(-100.0, 0.0),
                },
                _ => body.set_resulting_force(0.0, 0.0),
            }
        }
    }
}

#[cfg(test)]
mod test {
    use crate::*;

    #[test]
    fn view_port_tests() {
        let vp = ViewPort {
            origin: pos(0.0, 480.0),
            ratio: 1.0,
        };

        assert_eq!(vp.translate_pos(&pos(0.0, 0.0)), pos(0.0, 480.0));
        assert_eq!(vp.translate_pos(&pos(640.0, 0.0)), pos(640.0, 480.0));
        assert_eq!(vp.translate_pos(&pos(0.0, 480.0)), pos(0.0, 0.0));
        assert_eq!(vp.translate_pos(&pos(640.0, 480.0)), pos(640.0, 0.0));

        let vp = ViewPort {
            origin: pos(0.0, 480.0),
            ratio: 2.0,
        };

        assert_eq!(vp.translate_pos(&pos(0.0, 0.0)), pos(0.0, 240.0));
        assert_eq!(vp.translate_pos(&pos(640.0, 0.0)), pos(320.0, 240.0));
        assert_eq!(vp.translate_pos(&pos(0.0, 480.0)), pos(0.0, 0.0));
        assert_eq!(vp.translate_pos(&pos(640.0, 480.0)), pos(320.0, 0.0));

        let vp = ViewPort {
            origin: pos(0.0, 100.0),
            ratio: 0.10,
        };

        assert_eq!(vp.translate_pos(&pos(50.0, 50.0)), pos(500.0, 500.0));
    }

    #[test]
    fn create_still_body() {
        let mut engine = Engine::create(10.0);
        engine.add_body(UniformBody::still_body(10.0, pos(100.0, 100.0)));
        engine.tick(1.0);
        {
            let body = &engine.get_bodies()[0];
            assert_eq!(body.pos, pos(100.0, 95.0));
        }
        engine.tick(1.0);
        {
            let body = &engine.get_bodies()[0];
            assert_eq!(body.pos, pos(100.0, 80.0));
        }
    }

    #[test]
    fn free_fall_on_moon() {
        let mut engine = Engine::create(1.625);
        engine.add_body(UniformBody::still_body(10.0, pos(100.0, 100.0)));
        engine.tick(1.0);
        {
            let body = &engine.get_bodies()[0];
            assert_eq!(body.pos, pos(100.0, 99.1875));
        }
    }

    #[test]
    fn force_opposite_to_gravity() {
        let mut engine = Engine::create(10.0);
        engine.add_body(UniformBody::still_body(10.0, pos(100.0, 100.0)));
        engine
            .get_bodies_mut()
            .first_mut()
            .unwrap()
            .apply_force(0.0, 100.0);

        engine.tick(1.0);

        let body = engine.get_bodies().first().unwrap();

        assert_eq!(body.pos, pos(100.0, 100.0));
    }

    #[test]
    fn set_resulting_force() {
        let mut engine = Engine::create(10.0);
        engine.add_body(UniformBody::still_body(10.0, pos(100.0, 100.0)));
        engine
            .get_bodies_mut()
            .first_mut()
            .unwrap()
            .set_resulting_force(0.0, 100.0);

        engine.tick(1.0);

        let body = engine.get_bodies().first().unwrap();

        assert_eq!(body.pos, pos(100.0, 100.0));
    }

    #[test]
    fn forces_on_both_axis() {
        let mut engine = Engine::create(0.0);
        engine.add_body(UniformBody::still_body(10.0, pos(100.0, 100.0)));

        {
            let body = engine.get_bodies_mut().first_mut().unwrap();

            body.apply_force(100.0, 0.0);
            body.apply_force(0.0, 100.0);
        }

        engine.tick(1.0);

        let body = engine.get_bodies_mut().first_mut().unwrap();
        assert_eq!(body.pos, pos(105.0, 105.0));
    }

    #[test]
    fn point_projection() {
        assert_eq!(project(&pos(5.0, 0.0), 0.0), pos(5.0, 0.0));
        assert_eq!(project(&pos(0.0, 5.0), 0.0), pos(0.0, 0.0));
        assert_eq!(project(&pos(2.0, 0.0), 1.0), pos(1.0, 1.0));
        assert_eq!(project(&pos(0.0, 2.0), 1.0), pos(1.0, 1.0));
        assert_eq!(project(&pos(2.0, 0.0), -1.0), pos(1.0, -1.0));
        assert_eq!(project(&pos(0.0, -2.0), -1.0), pos(1.0, -1.0));
    }

    #[test]
    fn collision_two_non_intersecting_triangles() {
        let triangle1 = positions![(1.0, 1.0), (3.0, 1.0), (2.0, 3.0)];
        let triangle2 = positions![(3.0, 3.0), (4.0, 1.0), (5.0, 3.0)];
        assert!(!collided(&triangle1, &triangle2));
    }

    #[test]
    fn collision_two_triangles_sharing_one_edge() {
        let triangle1 = positions![(1.0, 1.0), (3.0, 1.0), (2.0, 3.0)];
        let triangle2 = positions![(2.0, 3.0), (3.0, 1.0), (4.0, 3.0)];
        assert!(collided(&triangle1, &triangle2));
    }

    #[test]
    fn collision_two_triangles_overlapping() {
        let triangle1 = positions![(1.0, 1.0), (3.0, 1.0), (2.0, 3.0)];
        let triangle2 = positions![(2.0, 2.0), (1.0, 4.0), (3.0, 4.0)];
        assert!(collided(&triangle1, &triangle2));
    }
}

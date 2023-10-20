use itertools::Itertools;
use piston_window::*;
use rand::Rng;
use std::fmt;

#[derive(Debug)]
struct Vector {
    x: f64,
    y: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Position {
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

#[allow(unused_macros)]
macro_rules! positions {
    ($(($x:expr, $y:expr)),*) => {
        [$(crate::Position{ x: $x, y: $y }),*]
    }
}

#[derive(Debug)]
struct ConvexBody {
    mass: f64,
    mesh: Vec<Position>,
    acceleration: Vector,
    velocity: Vector,
    fixed: bool,
    report_collision: bool,
}

impl ConvexBody {
    fn still_body(m: f64, mesh: &[Position]) -> ConvexBody {
        ConvexBody {
            mass: m,
            mesh: Vec::from(mesh),
            acceleration: v(0.0, 0.0),
            velocity: v(0.0, 0.0),
            fixed: false,
            report_collision: false,
        }
    }

    fn fixed_body(mesh: &[Position]) -> ConvexBody {
        ConvexBody {
            mass: 0.0,
            mesh: Vec::from(mesh),
            acceleration: v(0.0, 0.0),
            velocity: v(0.0, 0.0),
            fixed: true,
            report_collision: false,
        }
    }

    fn apply_force(&mut self, fx: f64, fy: f64) {
        self.acceleration.x += fx / self.mass;
        self.acceleration.y += fy / self.mass;
    }

    fn set_resulting_force(&mut self, fx: f64, fy: f64) {
        self.acceleration.x = fx / self.mass;
        self.acceleration.y = fy / self.mass;
    }

    fn report_collision(mut self) -> Self {
        self.report_collision = true;
        self
    }
}

struct Engine {
    bodies: Vec<ConvexBody>,
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
    fn update_body_position(body: &mut ConvexBody, ga: f64, dt: f64) -> bool {
        let ax = body.acceleration.x;
        let ay = body.acceleration.y - ga;
        let vx = body.velocity.x + (ax * dt);
        let vy = body.velocity.y + (ay * dt);
        let sx = (dt / 2.0) * (vx + body.velocity.x);
        let sy = (dt / 2.0) * (vy + body.velocity.y);
        body.velocity = v(vx, vy);
        body.mesh.iter_mut().for_each(|pos| {
            pos.x += sx;
            pos.y += sy;
        });
        sx != 0.0 || sy != 0.0
    }

    fn tick(&mut self, dt: f64) {
        self.bodies.iter_mut().for_each(|body|{
            Self::update_body_position(body, self.ga, dt);
        });
        let collisions = self.bodies.iter().cartesian_product(self.bodies.iter()).filter(|pair|{
            std::ptr::addr_of!(*pair.0) != std::ptr::addr_of!(*pair.1)
        }).filter(|pair|{
            !check_for_separating_axis(pair.0.mesh.as_slice(), pair.1.mesh.as_slice())
        }).collect::<Vec<_>>();
        if !collisions.is_empty(){
            //println!("{collisions:?}");
        }
    }

    fn add_body(&mut self, b: ConvexBody) {
        self.bodies.push(b);
    }

    fn get_bodies(&self) -> &[ConvexBody] {
        self.bodies.as_slice()
    }

    fn get_bodies_mut(&mut self) -> &mut [ConvexBody] {
        self.bodies.as_mut_slice()
    }
}

/// Projects point `position` onto a line with gradient
/// `line_gradient` and y-interception 0.
fn project(position: &Position, line_gradient: f64) -> Position {
    let p = position;
    let a = line_gradient;

    if line_gradient.is_infinite() {
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

/// Checks for collision between two convex polygons using
/// the "separating axis theorem" approach.
fn collided(shape1: &[Position], shape2: &[Position]) -> bool {
    check_for_separating_axis(shape1, shape2) && check_for_separating_axis(shape2, shape1)
}

/// Checks for a separating axis between `shape1` and `shape2`. It does that
/// based on the shape projections onto lines that are solely perpendicular to
/// the edges of `shape1`.
fn check_for_separating_axis(shape1: &[Position], shape2: &[Position]) -> bool {
    shape1.iter().circular_tuple_windows().all(|(p1, p2)| {
        let a = (p1.y - p2.y) / (p1.x - p2.x);
        let a_orth = -1.0 / a;

        let shape1_projections = shape1
            .iter()
            .map(|p| project(p, a_orth))
            .collect::<Vec<_>>();

        let shape2_projections = shape2
            .iter()
            .map(|p| project(p, a_orth))
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

        (shape1_max.x >= shape2_min.x && shape2_max.x >= shape1_min.x)
            && (shape1_max.y >= shape2_min.y && shape2_max.y >= shape1_min.y)
    })
}

fn generate_terrain() -> Vec<Position> {
    let left_limit = 0.0;
    let right_limit = 100.0;

    let mut rng = rand::thread_rng();
    let mut x = left_limit;
    let mut terrain = vec![];

    for _ in 0..21 {
        terrain.push(pos(x, rng.gen_range(2.0..20.0)));
        x += right_limit / 20.0;
    }

    let landing_site_index = rng.gen_range(0..terrain.len() - 1);

    let landing_site_height = terrain.get(landing_site_index).unwrap().y;
    terrain.get_mut(landing_site_index + 1).unwrap().y = landing_site_height;

    terrain
}

/// Partitions `terrain` onto non-convex polygons so they can
/// be used later in collision detection.
fn partition_terrain(terrain: &[Position]) -> Vec<[Position; 4]> {
    terrain
        .iter()
        .tuple_windows()
        .map(|(p1, p2)| {
            [
                p1.clone(),
                p2.clone(),
                pos(p2.x, p2.y - 10.0),
                pos(p1.x, p1.y - 10.0),
            ]
        })
        .collect()
}

fn main() {
    let viewport = ViewPort {
        origin: pos(0.0, 100.0),
        ratio: 0.15,
    };

    let mut window: PistonWindow = WindowSettings::new(
        "Lander",
        [
            viewport.translate_size(100.0),
            viewport.translate_size(100.0),
        ],
    )
    .exit_on_esc(true)
    .build()
    .unwrap();

    let g = 1.625;

    let mut engine = Engine::create(g);
    let mut lander = ConvexBody::still_body(
        10.0,
        &[
            pos(49.0, 100.0),
            pos(51.0, 100.0),
            pos(51.0, 98.0),
            pos(49.0, 98.0),
        ],
    )
    .report_collision();
    engine.add_body(lander);

    let terrain = generate_terrain();
    partition_terrain(terrain.as_slice())
        .iter()
        .for_each(|polygon| {
            engine.add_body(ConvexBody::fixed_body(polygon));
        });

    while let Some(event) = window.next() {
        window.draw_2d(&event, |context, graphics, _device| {
            clear([1.0; 4], graphics);
            let body = &engine.get_bodies()[0];
            let p = Polygon::new([1.0, 0.0, 0.0, 1.0]);
            let polygon_mesh: Vec<[f64; 2]> = body
                .mesh
                .iter()
                .map(|p| {
                    let t = viewport.translate_pos(p);
                    [t.x, t.y]
                })
                .collect();
            p.draw(
                polygon_mesh.as_slice(),
                &context.draw_state,
                context.transform,
                graphics,
            );
            let line = Line::new([0.0, 0.0, 0.0, 1.0], 1.0);
            terrain
                .iter()
                .map(|p| viewport.translate_pos(p))
                .tuple_windows()
                .for_each(|(p1, p2)| {
                    line.draw(
                        [p1.x, p1.y, p2.x, p2.y],
                        &context.draw_state,
                        context.transform,
                        graphics,
                    );
                });
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
        engine.add_body(ConvexBody::still_body(10.0, &[pos(100.0, 100.0)]));
        engine.tick(1.0);
        {
            let body = &engine.get_bodies()[0];
            assert_eq!(body.mesh, [pos(100.0, 95.0)]);
        }
        engine.tick(1.0);
        {
            let body = &engine.get_bodies()[0];
            assert_eq!(body.mesh, [pos(100.0, 80.0)]);
        }
    }

    #[test]
    fn free_fall_on_moon() {
        let mut engine = Engine::create(1.625);
        engine.add_body(ConvexBody::still_body(10.0, &[pos(100.0, 100.0)]));
        engine.tick(1.0);
        {
            let body = &engine.get_bodies()[0];
            assert_eq!(body.mesh, [pos(100.0, 99.1875)]);
        }
    }

    #[test]
    fn force_opposite_to_gravity() {
        let mut engine = Engine::create(10.0);
        engine.add_body(ConvexBody::still_body(10.0, &[pos(100.0, 100.0)]));
        engine
            .get_bodies_mut()
            .first_mut()
            .unwrap()
            .apply_force(0.0, 100.0);

        engine.tick(1.0);

        let body = engine.get_bodies().first().unwrap();

        assert_eq!(body.mesh, [pos(100.0, 100.0)]);
    }

    #[test]
    fn set_resulting_force() {
        let mut engine = Engine::create(10.0);
        engine.add_body(ConvexBody::still_body(10.0, &[pos(100.0, 100.0)]));
        engine
            .get_bodies_mut()
            .first_mut()
            .unwrap()
            .set_resulting_force(0.0, 100.0);

        engine.tick(1.0);

        let body = engine.get_bodies().first().unwrap();

        assert_eq!(body.mesh, [pos(100.0, 100.0)]);
    }

    #[test]
    fn forces_on_both_axis() {
        let mut engine = Engine::create(0.0);
        engine.add_body(ConvexBody::still_body(10.0, &[pos(100.0, 100.0)]));

        {
            let body = engine.get_bodies_mut().first_mut().unwrap();

            body.apply_force(100.0, 0.0);
            body.apply_force(0.0, 100.0);
        }

        engine.tick(1.0);

        let body = engine.get_bodies_mut().first_mut().unwrap();
        assert_eq!(body.mesh, [pos(105.0, 105.0)]);
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

    #[test]
    fn collision_two_rectangles() {
        let mesh1 = [
            pos(0.0, 20.0),
            pos(100.0, 20.0),
            pos(100.0, 10.0),
            pos(0.0, 10.0),
        ];

        let mesh2 = [
            pos(40.0, 20.0),
            pos(50.0, 20.0),
            pos(50.0, 30.0),
            pos(40.0, 30.0),
        ];

        assert!(collided(&mesh1, &mesh2));
    }

    #[test]
    fn collision_trapezoid_and_rectangle() {
        let trapezoid = [
            pos(0.0, 0.0),
            pos(20.0, 0.0),
            pos(10.0, 10.0),
            pos(0.0, 10.0),
        ];

        let rectangle = [
            pos(16.0, 5.0),
            pos(25.0, 5.0),
            pos(25.0, 15.0),
            pos(16.0, 15.0),
        ];

        assert!(!collided(&rectangle, &trapezoid));
    }

    #[test]
    fn partition_terrain_test() {
        let terrain = positions![(0.0, 5.0), (1.0, 6.0), (2.0, 4.0), (3.0, 4.0)];
        let polygons = partition_terrain(&terrain);
        assert_eq!(
            polygons.as_slice(),
            &[
                positions![(0.0, 5.0), (1.0, 6.0), (1.0, -4.0), (0.0, -5.0)],
                positions![(1.0, 6.0), (2.0, 4.0), (2.0, -6.0), (1.0, -4.0)],
                positions![(2.0, 4.0), (3.0, 4.0), (3.0, -6.0), (2.0, -6.0)]
            ]
        );
    }

    #[test]
    fn collision_between_non_fixed_and_fixed_body() {
        let mut engine = Engine::create(10.0);
        engine.add_body(ConvexBody::fixed_body(&positions![
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, -1.0),
            (0.0, -1.0)
        ]));
        engine.add_body(ConvexBody::still_body(
            10.0,
            &positions![(1.0, 1.0), (2.0, 1.0), (1.5, 2.0)],
        ));
        engine.tick(0.5);
    }
}

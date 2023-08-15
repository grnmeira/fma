use piston_window::ellipse::circle;
use piston_window::*;

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

fn v(x: f64, y: f64) -> Vector {
    Vector { x, y }
}

fn pos(x: f64, y: f64) -> Position {
    Position { x, y }
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
    origin: Position,
    width: f64,
    height: f64,
    ratio: f64,
}

impl ViewPort {
    fn translate(&self, real_pos: &Position) -> Position {
        Position {
            x: (real_pos.x - self.origin.x) * self.ratio,
            y: (self.origin.y - real_pos.y) * self.ratio,
        }
    }
}

impl Engine {
    fn tick(&mut self, dt: f64) {
        for body in self.bodies.iter_mut() {
            let ax = 0.0;
            let ay = -self.ga;
            let vx = body.velocity.x + (ax / dt);
            let vy = body.velocity.y + (ay / dt);
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
}

fn main() {
    let mut window: PistonWindow = WindowSettings::new("Hello Piston!", [640, 480])
        .exit_on_esc(true)
        .build()
        .unwrap();

    let mut engine = Engine::create(1.625);
    engine.add_body(UniformBody::still_body(10.0, pos(300.0, 500.0)));

    while let Some(event) = window.next() {
        window.draw_2d(&event, |context, graphics, _device| {
            if let Some(render_args) = event.render_args() {
                engine.tick(render_args.ext_dt);
            }
            clear([1.0; 4], graphics);
            let border = Ellipse::new_border([1.0, 0.0, 0.0, 1.0], 10.0);
            let body = &engine.get_bodies()[0];
            border.draw(
                rectangle::centered_square(300.0, 480.0 - body.pos.y, 10.0),
                &context.draw_state,
                context.transform,
                graphics,
            );
        });
    }
}

#[cfg(test)]
mod test {
    use crate::*;

    #[test]
    fn view_port_tests() {
        let vp = ViewPort {
            origin: pos(0.0, 480.0),
            height: 480.0,
            width: 640.0,
            ratio: 1.0,
        };

        assert_eq!(vp.translate(&pos(0.0, 0.0)), pos(0.0, 480.0));
        assert_eq!(vp.translate(&pos(640.0, 0.0)), pos(640.0, 480.0));
        assert_eq!(vp.translate(&pos(0.0, 480.0)), pos(0.0, 0.0));
        assert_eq!(vp.translate(&pos(640.0, 480.0)), pos(640.0, 0.0));

        let vp = ViewPort {
            origin: pos(0.0, 480.0),
            height: 480.0,
            width: 640.0,
            ratio: 0.5,
        };

        assert_eq!(vp.translate(&pos(0.0, 0.0)), pos(0.0, 240.0));
        assert_eq!(vp.translate(&pos(640.0, 0.0)), pos(320.0, 240.0));
        assert_eq!(vp.translate(&pos(0.0, 480.0)), pos(0.0, 0.0));
        assert_eq!(vp.translate(&pos(640.0, 480.0)), pos(320.0, 0.0));
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
}

use piston_window::*;

struct Vector {
    x: f64,
    y: f64,
}

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

impl Engine {
    fn tick(&mut self, dt: f64) {
        let gf = v(0.0, -self.gf);
        for mut body in self.bodies {
            let ax = gf.x;
            let ay = gf.y;
            let vx = ax / dt;
            let vy = ay / dt;
            let sx = dt / 2 * (vx - body.velocity.x);
            let sy = dt / 2 * (vy - body.velocity.y);
            body.velocity = v(vx, vy);
            body.pos = pos(body.pos.x + sx, body.pos.y + sy);
        }
    }

    fn add_body(&mut self, b: UniformBody) {
        self.bodies.push(b);
    }
}

fn main() {
    let mut window: PistonWindow = WindowSettings::new("Hello Piston!", [640, 480])
        .exit_on_esc(true)
        .build()
        .unwrap();
    while let Some(event) = window.next() {
        window.draw_2d(&event, |context, graphics, _device| {
            clear([1.0; 4], graphics);
            rectangle(
                [1.0, 0.0, 0.0, 1.0], // red
                [0.0, 0.0, 100.0, 100.0],
                context.transform,
                graphics,
            );
        });
    }
}

#[cfg(test)]
mod test {
    use crate::UniformBody;

    #[test]
    fn create_still_body() {
        let engine = Engine::create(10.0);
        engine.add_body(UniformBody::still_body(10.0, pos(100.0, 100.0)));
        engine.tick(1.0);
    }
}

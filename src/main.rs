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
    /// Width of the viewport in meters.
    width: f64,
    /// Height of the viewport in meters.
    height: f64,
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

fn main() {
    let viewport = ViewPort {
        origin: pos(0.0, 100.0),
        ratio: 0.15,
        height: 100.0,
        width: 100.0,
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
            height: 480.0,
            width: 640.0,
            ratio: 1.0,
        };

        assert_eq!(vp.translate_pos(&pos(0.0, 0.0)), pos(0.0, 480.0));
        assert_eq!(vp.translate_pos(&pos(640.0, 0.0)), pos(640.0, 480.0));
        assert_eq!(vp.translate_pos(&pos(0.0, 480.0)), pos(0.0, 0.0));
        assert_eq!(vp.translate_pos(&pos(640.0, 480.0)), pos(640.0, 0.0));

        let vp = ViewPort {
            origin: pos(0.0, 480.0),
            height: 480.0,
            width: 640.0,
            ratio: 2.0,
        };

        assert_eq!(vp.translate_pos(&pos(0.0, 0.0)), pos(0.0, 240.0));
        assert_eq!(vp.translate_pos(&pos(640.0, 0.0)), pos(320.0, 240.0));
        assert_eq!(vp.translate_pos(&pos(0.0, 480.0)), pos(0.0, 0.0));
        assert_eq!(vp.translate_pos(&pos(640.0, 480.0)), pos(320.0, 0.0));

        let vp = ViewPort {
            origin: pos(0.0, 100.0),
            height: 100.0,
            width: 100.0,
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
}

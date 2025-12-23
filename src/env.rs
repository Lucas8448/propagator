use glam::Vec3;

pub trait Environment {
    fn gravity(&self, pos_world: Vec3) -> Vec3;
    fn wind(&self, pos_world: Vec3, t: f32) -> Vec3;
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FlatEarth {
    pub gravity_magnitude: f32,
}

impl FlatEarth {
    pub const STANDARD_GRAVITY: f32 = 9.81;

    pub fn new() -> Self {
        Self {
            gravity_magnitude: Self::STANDARD_GRAVITY,
        }
    }

    pub fn with_gravity(gravity_magnitude: f32) -> Self {
        Self { gravity_magnitude }
    }
}

impl Environment for FlatEarth {
    fn gravity(&self, _pos_world: Vec3) -> Vec3 {
        Vec3::new(0.0, 0.0, -self.gravity_magnitude)
    }

    fn wind(&self, _pos_world: Vec3, _t: f32) -> Vec3 {
        Vec3::ZERO
    }
}

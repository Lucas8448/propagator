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

#[derive(Debug, Clone, Copy)]
pub struct ConstantWind {
    pub wind_world: Vec3,
    pub gravity_magnitude: f32,
}

impl ConstantWind {
    pub fn new(wind_world: Vec3) -> Self {
        Self {
            wind_world,
            gravity_magnitude: FlatEarth::STANDARD_GRAVITY,
        }
    }
}

impl Environment for ConstantWind {
    fn gravity(&self, _pos_world: Vec3) -> Vec3 {
        Vec3::new(0.0, 0.0, -self.gravity_magnitude)
    }

    fn wind(&self, _pos_world: Vec3, _t: f32) -> Vec3 {
        self.wind_world
    }
}

#[cfg(test)]
mod tests {
    use super::{ConstantWind, Environment};
    use glam::Vec3;

    #[test]
    fn constant_wind_returns_configured_vector() {
        let env = ConstantWind::new(Vec3::new(3.0, -1.0, 0.5));

        assert_eq!(env.wind(Vec3::ZERO, 12.0), Vec3::new(3.0, -1.0, 0.5));
        assert_eq!(env.gravity(Vec3::ZERO), Vec3::new(0.0, 0.0, -9.81));
    }
}

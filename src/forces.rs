use crate::env::Environment;
use crate::state::RigidBodyState;
use glam::Vec3;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Wrench {
    pub force: Vec3,
    pub moment: Vec3,
}

impl Wrench {
    pub fn zero() -> Self {
        Self {
            force: Vec3::ZERO,
            moment: Vec3::ZERO,
        }
    }

    pub fn from_force(force: Vec3) -> Self {
        Self {
            force,
            moment: Vec3::ZERO,
        }
    }

    pub fn from_moment(moment: Vec3) -> Self {
        Self {
            force: Vec3::ZERO,
            moment,
        }
    }

    pub fn new(force: Vec3, moment: Vec3) -> Self {
        Self { force, moment }
    }
}

impl std::ops::Add for Wrench {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            force: self.force + other.force,
            moment: self.moment + other.moment,
        }
    }
}

impl std::ops::AddAssign for Wrench {
    fn add_assign(&mut self, other: Self) {
        self.force += other.force;
        self.moment += other.moment;
    }
}

pub trait ForceModel {
    fn wrench(&self, state: &RigidBodyState, env: &dyn Environment, t: f32) -> Wrench;
}

#[derive(Debug, Clone, Copy)]
pub struct Gravity {
    pub mass: f32,
}

impl Gravity {
    pub fn new(mass: f32) -> Self {
        Self { mass }
    }
}

impl ForceModel for Gravity {
    fn wrench(&self, state: &RigidBodyState, env: &dyn Environment, _t: f32) -> Wrench {
        let gravity_accel = env.gravity(state.pos);
        Wrench::from_force(gravity_accel * self.mass)
    }
}

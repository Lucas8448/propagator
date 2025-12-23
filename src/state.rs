use glam::{Vec3, Quat};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyState {
    pub pos: Vec3,
    pub vel: Vec3,
    pub orient: Quat,
    pub ang_vel: Vec3,
}

impl RigidBodyState {
    pub fn new() -> Self {
        Self {
            pos: Vec3::ZERO,
            vel: Vec3::ZERO,
            orient: Quat::IDENTITY,
            ang_vel: Vec3::ZERO,
        }
    }

    pub fn with_values(pos: Vec3, vel: Vec3, orient: Quat, ang_vel: Vec3) -> Self {
        Self {
            pos,
            vel,
            orient: orient.normalize(),
            ang_vel,
        }
    }

    pub fn as_derivative(dpos: Vec3, dvel: Vec3, dang_vel: Vec3) -> Self {
        Self {
            pos: dpos,
            vel: dvel,
            orient: Quat::IDENTITY,
            ang_vel: dang_vel,
        }
    }
}

impl Default for RigidBodyState {
    fn default() -> Self {
        Self::new()
    }
}

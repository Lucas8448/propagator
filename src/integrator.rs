use crate::state::RigidBodyState;
use glam::Quat;

pub trait Integrator {
    fn step(&self, state: &RigidBodyState, deriv: &RigidBodyState, dt: f32) -> RigidBodyState;
}

#[derive(Debug, Clone, Copy, Default)]
pub struct SemiImplicitEuler;

impl SemiImplicitEuler {
    pub fn new() -> Self {
        Self
    }
}

impl Integrator for SemiImplicitEuler {
    fn step(&self, state: &RigidBodyState, deriv: &RigidBodyState, dt: f32) -> RigidBodyState {
        let new_vel = state.vel + deriv.vel * dt;
        let new_pos = state.pos + new_vel * dt;

        let new_ang_vel = state.ang_vel + deriv.ang_vel * dt;

        let rotation_increment = Quat::from_scaled_axis(new_ang_vel * dt);
        let new_orient = (state.orient * rotation_increment).normalize();

        RigidBodyState {
            pos: new_pos,
            vel: new_vel,
            orient: new_orient,
            ang_vel: new_ang_vel,
        }
    }
}

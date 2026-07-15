use crate::env::Environment;
use crate::forces::{ForceModel, Wrench};
use crate::integrator::Integrator;
use crate::state::RigidBodyState;
use glam::Vec3;

pub struct Sim<'a> {
    pub state: RigidBodyState,
    env: &'a dyn Environment,
    forces: Vec<&'a dyn ForceModel>,
    integrator: &'a dyn Integrator,
    mass: f32,
    inertia_diag: Vec3,
}

impl<'a> Sim<'a> {
    pub fn new(
        state: RigidBodyState,
        env: &'a dyn Environment,
        integrator: &'a dyn Integrator,
        mass: f32,
        inertia_diag: Vec3,
    ) -> Self {
        Self {
            state,
            env,
            forces: Vec::new(),
            integrator,
            mass,
            inertia_diag,
        }
    }

    pub fn add_force(&mut self, force: &'a dyn ForceModel) {
        self.forces.push(force);
    }

    pub fn state(&self) -> &RigidBodyState {
        &self.state
    }

    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn inertia_diag(&self) -> Vec3 {
        self.inertia_diag
    }

    pub fn compute_wrench(&self, t: f32) -> Wrench {
        let mut total = Wrench::zero();
        for force in &self.forces {
            total += force.wrench(&self.state, self.env, t);
        }
        total
    }

    pub fn compute_derivatives(&self, t: f32) -> RigidBodyState {
        let wrench = self.compute_wrench(t);
        let accel = wrench.force / self.mass;
        let ang_accel = wrench.moment / self.inertia_diag;
        RigidBodyState::as_derivative(self.state.vel, accel, ang_accel)
    }

    pub fn step(&mut self, dt: f32, t: f32) {
        let deriv = self.compute_derivatives(t);
        self.state = self.integrator.step(&self.state, &deriv, dt);
        self.state.orient = self.state.orient.normalize();
    }

    pub fn run(&mut self, dt: f32, steps: usize, t_start: f32) -> f32 {
        let mut t = t_start;
        for _ in 0..steps {
            self.step(dt, t);
            t += dt;
        }
        t
    }
}

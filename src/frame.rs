use glam::{Vec3, Quat};

#[inline]
pub fn body_to_world(v_body: Vec3, orient: Quat) -> Vec3 {
    orient * v_body
}

#[inline]
pub fn world_to_body(v_world: Vec3, orient: Quat) -> Vec3 {
    orient.inverse() * v_world
}

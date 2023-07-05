use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height]);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    /*
     * A rectangle on a motor
     */

    let x_pos = 0.0 as f32;
    let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x_pos, 2.0]);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(0.1, 0.5);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let joint = RevoluteJointBuilder::new()
        .local_anchor2(point![x_pos, 1.5])
        .local_anchor1(point![0.0, -0.5])
        .motor_position(-150f32.to_radians(), 30.0, 1.0);
    impulse_joints.insert(handle, ground_handle, joint, true);
    

    // set limits
    for (_, joint) in impulse_joints.iter_mut(){
        joint.data.set_limits(
            JointAxis::AngX,
            [-45f32.to_radians(), 45f32.to_radians()]
        ); 
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world_with_params(
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        vector![0.0, 0.0],
        (),
    );
    testbed.look_at(point![0.0, 0.0], 40.0);
}
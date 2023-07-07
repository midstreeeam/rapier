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
        .motor_position(-150f32.to_radians(), 300.0, 1.0)
        .limits([-145f32.to_radians(), 145f32.to_radians()]);
    impulse_joints.insert(handle, ground_handle, joint, true);


    /*
     * 2 rectangls on a motor
     */

     let x_pos = 1.0 as f32;
     let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x_pos, 1.0]);
     let handle1 = bodies.insert(rigid_body);
     let collider = ColliderBuilder::cuboid(0.5, 0.1);
     colliders.insert_with_parent(collider, handle1, &mut bodies);

     let x_pos = 2.0 as f32;
     let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x_pos, 1.0]);
     let handle2 = bodies.insert(rigid_body);
     let collider = ColliderBuilder::cuboid(0.5, 0.1);
     colliders.insert_with_parent(collider, handle2, &mut bodies);
 
     let joint = RevoluteJointBuilder::new()
         .local_anchor2(point![0.5, 0.0])
         .local_anchor1(point![-0.5, 0.0])
         .motor_position(-150f32.to_radians(), 300.0, 0.0)
         .contacts_enabled(false)
         .limits([-120f32.to_radians(),120f32.to_radians()]);
    impulse_joints.insert(handle1, handle2, joint, true);
    

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
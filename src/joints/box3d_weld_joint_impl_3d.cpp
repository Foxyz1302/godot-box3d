#include "box3d_weld_joint_impl_3d.hpp"

#include "../misc/type_conversions.hpp"

#include <box3d/box3d.h>

b3JointId Box3DWeldJointImpl3D::_create_joint_id(b3WorldId p_world_id, b3BodyId p_body_a, b3BodyId p_body_b, b3Transform p_local_frame_a, b3Transform p_local_frame_b) {
	b3WeldJointDef def = b3DefaultWeldJointDef();
	def.base.bodyIdA = p_body_a;
	def.base.bodyIdB = p_body_b;
	def.base.localFrameA = p_local_frame_a;

	// The weld enforces relQ == identity against the two local frames, so frame B's rotation
	// is pre-corrected with the bodies' CURRENT relative rotation: the joint then locks the
	// relative pose at creation time, no matter how the joint node was oriented.
	const b3Quat quat_a = b3Body_GetRotation(p_body_a);
	const b3Quat quat_b = b3Body_GetRotation(p_body_b);
	const b3Quat world_frame_a = b3MulQuat(quat_a, p_local_frame_a.q);
	def.base.localFrameB.q = b3InvMulQuat(quat_b, world_frame_a);
	def.base.localFrameB.p = p_local_frame_b.p;

	// hertz 0 = maximum stiffness (Box3D falls back to its engine-clamped world constraint
	// softness); bias > 0 softens the weld: hertz = bias * 30, mirroring the pin joint.
	def.linearHertz = (float)bias * 30.0f;
	def.angularHertz = (float)bias * 30.0f;
	def.linearDampingRatio = 1.0f;
	def.angularDampingRatio = 1.0f;
	return b3CreateWeldJoint(p_world_id, &def);
}

real_t Box3DWeldJointImpl3D::get_param(PhysicsServer3D::ConeTwistJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer3D::CONE_TWIST_JOINT_BIAS:
			return bias;
		default:
			return 0.0;
	}
}

void Box3DWeldJointImpl3D::set_param(PhysicsServer3D::ConeTwistJointParam p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer3D::CONE_TWIST_JOINT_BIAS:
			bias = p_value;
			if (has_joint_id()) {
				b3WeldJoint_SetLinearHertz(get_joint_id(), (float)bias * 30.0f);
				b3WeldJoint_SetAngularHertz(get_joint_id(), (float)bias * 30.0f);
			}
			break;
		case PhysicsServer3D::CONE_TWIST_JOINT_SWING_SPAN:
		case PhysicsServer3D::CONE_TWIST_JOINT_TWIST_SPAN:
			if (p_value > 0.0) {
				WARN_PRINT_ONCE("Box3D: ConeTwistJoint3D's swing/twist spans have no Box3D equivalent (the weld joint has no limits) and are treated as fully rigid.");
			}
			break;
		case PhysicsServer3D::CONE_TWIST_JOINT_SOFTNESS:
			WARN_PRINT_ONCE("Box3D: ConeTwistJoint3D's SOFTNESS parameter has no Box3D equivalent and is ignored; use BIAS to soften the weld.");
			break;
		case PhysicsServer3D::CONE_TWIST_JOINT_RELAXATION:
			WARN_PRINT_ONCE("Box3D: ConeTwistJoint3D's RELAXATION parameter has no Box3D equivalent and is ignored.");
			break;
		default:
			break;
	}
}

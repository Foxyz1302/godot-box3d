#pragma once

#include "box3d_joint_impl_3d.hpp"

// CONE_TWIST_JOINT mapped to Box3D's weld joint: a solver-owned rigid weld that locks the
// two bodies at the relative pose captured when the joint is created. Godot's
// ConeTwistJoint3D with swing_span 0 + twist_span 0 is therefore a rigid grab weld; nonzero
// spans have no Box3D equivalent (the weld has no limits) and are warned + treated as rigid.
class Box3DWeldJointImpl3D final : public Box3DJointImpl3D {
public:
	Box3DWeldJointImpl3D(
			Box3DBodyImpl3D* p_body_a,
			Box3DBodyImpl3D* p_body_b,
			const Transform3D& p_local_frame_a,
			const Transform3D& p_local_frame_b) :
			Box3DJointImpl3D(p_body_a, p_body_b, p_local_frame_a, p_local_frame_b) {
	}

	PhysicsServer3D::JointType get_type() const override { return PhysicsServer3D::JOINT_TYPE_CONE_TWIST; }

	real_t get_param(PhysicsServer3D::ConeTwistJointParam p_param) const;

	void set_param(PhysicsServer3D::ConeTwistJointParam p_param, real_t p_value);

protected:
	b3JointId _create_joint_id(b3WorldId p_world_id, b3BodyId p_body_a, b3BodyId p_body_b, b3Transform p_local_frame_a, b3Transform p_local_frame_b) override;

private:
	real_t bias = 0.0;
};

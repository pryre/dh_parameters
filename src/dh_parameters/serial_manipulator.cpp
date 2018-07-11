#include <dh_parameters/dh_parameters.h>
#include <dh_parameters/serial_manipulator.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <assert.h>

SerialManipulator::SerialManipulator( void ) {
}


SerialManipulator::~SerialManipulator( void ) {
}

bool SerialManipulator::add_joint( const dh_parameters::JointDescription& description ) {
	DHParameters dh( description );

	return add_joint(dh);
}

bool SerialManipulator::add_joint( const DHParameters& joint ) {
	bool success = false;

	if( joint.is_valid() ) {
		joints_.push_back(joint);
		success = true;
	}

	return success;
}

bool SerialManipulator::add_body( const double center_of_mass, const int parent_joint ) {
	bool success = false;
	bool used_joint = false;

	for(int i=0; i<body_map_.size(); i++) {
		if(body_map_[i].parent_joint == parent_joint) {
			used_joint = true;
			ROS_ERROR("Body already defined for parent joint %i", parent_joint);
			break;
		}
	}

	//XXX: The parent joint represents the last joint before the body,
	//such that the body is aligned on the axis of joint: parent_joint + 1
	if( (parent_joint < int(joints_.size()-1)) && (parent_joint >= -1) && !used_joint) {
		if( (joints_[parent_joint + 1].jt() == DHParameters::JointType::TranslateX) ||
			(joints_[parent_joint + 1].jt() == DHParameters::JointType::TranslateZ) ) {

			ROS_WARN("Body added to linear joint, this may produce undesired results");
		}

		body_map_t body;
		body.center_of_mass = center_of_mass;
		body.parent_joint = parent_joint;
		body_map_.push_back(body);

		success = true;
	} else {
		ROS_ERROR("Invalid parent joint %i", parent_joint);
	}

	return success;
}

bool SerialManipulator::is_valid( void ) {
	bool valid = true;

	for(int i=0; i<num_joints(); i++) {
		valid &= joints_[i].is_valid();
	}

	return valid;
}

void SerialManipulator::reset( void ) {
	joints_.clear();
	body_map_.clear();
}

int SerialManipulator::num_joints( void ) {
	joints_.size();
}

void SerialManipulator::get_dynamic_joint_map( std::vector<bool>& map ) {
	map.resize( num_joints() );

	for(int i=0; i<num_joints(); i++) {
		map[i] = (joints_[i].jt() != DHParameters::JointType::Static);
	}
}

DHParameters& SerialManipulator::joint( int i ) {
	assert(i < num_joints());

	return joints_[i];
}


void SerialManipulator::get_q(Eigen::VectorXd &q) {
	get_qxn(q, 0, num_joints());
}

void SerialManipulator::get_qxn(Eigen::VectorXd &qxn, const unsigned int x, const unsigned int n ) {
	assert( (x+n) <= joints_.size() );

	qxn = Eigen::VectorXd(n);

	for(int i=x; i<(x+n); i++) {
		qxn(i) = joints_[i].q();
	}
}

void SerialManipulator::get_qd(Eigen::VectorXd &qd) {
	get_qdxn(qd, 0, num_joints());
}

void SerialManipulator::get_qdxn(Eigen::VectorXd &qdxn, const unsigned int x, const unsigned int n ) {
	assert( (x+n) <= joints_.size() );

	qdxn = Eigen::VectorXd(n);

	for(int i=x; i<(x+n); i++) {
		qdxn(i) = joints_[i].qd();
	}
}

void SerialManipulator::calculate_Jxy( Eigen::MatrixXd &J, const unsigned int x, const unsigned int y ) {
	assert( ( x <= joints_.size() ) && ( y <= joints_.size() ) && ( x < y ) );

	unsigned int nj = y - x;
	unsigned int jc = 0;
	J = Eigen::MatrixXd(6,nj);

	Eigen::Affine3d gn;
	calculate_gxy(gn, x, y);

	for(int i=x; i<y; i++) {
		bool is_rot = false;
		Eigen::Vector3d axis;
		Eigen::VectorXd Ji = Eigen::VectorXd::Zero(6);
		Eigen::Affine3d gi;
		calculate_gxy(gi, x, i);

		if(joints_[i].jt() == DHParameters::JointType::TwistX) {
			is_rot = true;
			axis = gi.linear()*Eigen::Vector3d(1.0,0.0,0.0);
		} else if(joints_[i].jt() == DHParameters::JointType::TwistZ) {
			is_rot = true;
			axis = gi.linear()*Eigen::Vector3d(0.0,0.0,1.0);
		} else if(joints_[i].jt() == DHParameters::JointType::TranslateX) {
			axis = gi.linear()*Eigen::Vector3d(1.0,0.0,0.0);
		} else if(joints_[i].jt() == DHParameters::JointType::TranslateZ) {
			axis = gi.linear()*Eigen::Vector3d(0.0,0.0,1.0);
		} else {
			axis = Eigen::Vector3d::Zero();
		}

		if(is_rot) {
			//Ji = [ zi X (pn - pi); zi]
			//ROS_INFO_STREAM("dg:\n" << (gn.translation() - gi.translation()));
			Ji.segment<3>(0) = axis.cross(gn.translation() - gi.translation());

			Ji.segment<3>(3) = axis;
		} else {
			//Ji = [ zi; 0]
			Ji.segment<3>(0) = axis;
		}

		J.block<6,1>(0,jc) = Ji;

		jc++;
	}
}

void SerialManipulator::calculate_Je( Eigen::MatrixXd &Je ) {
	assert(num_joints() > 0);

	calculate_Jxy( Je, 0, num_joints() );
}

void SerialManipulator::calculate_J_body( Eigen::MatrixXd &Jbi, const unsigned int bi ) {
	//XXX: This Jacobian returns the joint jacobian of a body in the body frame,
	//	as apposed to the base frame!

	unsigned int x = body_map_[bi].parent_joint + 2; //XXX: +1 for parent joint, and +1 for body joint
	assert( x <= joints_.size() );

	//Check to see if the base link has not been requested
	if(body_map_[x].parent_joint >=0) {
		unsigned int nj = x;
		unsigned int jc = 0;
		Jbi = Eigen::MatrixXd(6,nj);

		Eigen::Affine3d gn;
		calculate_g_body(gn, bi);

		for(int i=0; i<x; i++) {
			bool is_rot = false;
			Eigen::Vector3d axis;
			Eigen::VectorXd Ji = Eigen::VectorXd::Zero(6);
			Eigen::Affine3d gi;
			calculate_gxy(gi, 0, i);

			if(joints_[i].jt() == DHParameters::JointType::TwistX) {
				is_rot = true;
				axis = gi.linear()*Eigen::Vector3d::UnitX();
			} else if(joints_[i].jt() == DHParameters::JointType::TwistZ) {
				is_rot = true;
				axis = gi.linear()*Eigen::Vector3d::UnitZ();
			} else if(joints_[i].jt() == DHParameters::JointType::TranslateX) {
				axis = gi.linear()*Eigen::Vector3d::UnitX();
			} else if(joints_[i].jt() == DHParameters::JointType::TranslateZ) {
				axis = gi.linear()*Eigen::Vector3d::UnitZ();
			} else {
				axis = Eigen::Vector3d::Zero();
			}

			if(is_rot) {
				//Ji = [ zi X (pn - pi); zi]
				//XXX: The gn.linear().inverse() is the difference from the usual calculation
				Ji.segment<3>(0) = gn.linear().inverse()*axis.cross(gn.translation() - gi.translation());

				Ji.segment<3>(3) = gn.linear().inverse()*axis;
			} else {
				//Ji = [ zi; 0]
				Ji.segment<3>(0) = gn.linear().inverse()*axis;
			}

			Jbi.block<6,1>(0,jc) = Ji;

			jc++;
		}
	}
}

void SerialManipulator::calculate_gxy( Eigen::Affine3d &g, const unsigned int x, const unsigned int y ) {
	assert( ( x <= joints_.size() ) && ( y <= joints_.size() ) );

	bool inverse = false;
	unsigned int p;
	unsigned int q;

	g = Eigen::Affine3d::Identity();

	//The inverse calculation was requested
	if(x > y) {
		p = y;
		q = x;

		inverse = true;
	} else {
		p = x;
		q = y;
	}

	for(int i=p; i<q; i++) {
		g = g * joints_[i].transform();
	}

	if(inverse)
		g = g.inverse();
}

void SerialManipulator::calculate_gbe( Eigen::Affine3d &gbe ) {
	assert(num_joints() > 0);

	calculate_gxy( gbe, 0, joints_.size() );
}

void SerialManipulator::calculate_g_body( Eigen::Affine3d &gbi, const unsigned int i ) {
	assert(i < body_map_.size());

	//Check to see if the base link has not been requested
	if(body_map_[i].parent_joint >=0) {
		DHParameters bdh = joints_[body_map_[i].parent_joint + 1];
		bdh.set_r(body_map_[i].center_of_mass);

		//Calculate the parent transform
		Eigen::Affine3d gp;
		calculate_gxy( gp, 0, body_map_[i].parent_joint + 1);

		//Calculate the body transform
		gbi = gp * bdh.transform();
	} else {
		gbi = Eigen::Affine3d::Identity();
	}
}

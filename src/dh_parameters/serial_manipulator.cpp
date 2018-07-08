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

bool SerialManipulator::is_valid( void ) {
	bool valid = true;

	for(int i=0; i<num_joints(); i++) {
		valid &= joints_[i].is_valid();
	}

	return valid;
}

void SerialManipulator::reset( void ) {
	joints_.clear();
}

int SerialManipulator::num_joints( void ) {
	joints_.size();
}

void SerialManipulator::get_dynamic_map( std::vector<bool>& map ) {
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

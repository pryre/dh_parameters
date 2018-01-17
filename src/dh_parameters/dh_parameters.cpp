#include <dh_parameters/dh_parameters.h>

#include <eigen3/Eigen/Dense>

#include <math.h>
#include <string>


DHParameters::DHParameters( ros::NodeHandle *nh ) :
							nh_(nh),
							d_(0.0),
							t_(0.0),
							r_(0.0),
							a_(0.0),
							jt_(JointType::Static),
							q_(0.0),
							parameters_changed_(true),
							transform_valid_(true),
							transform_(Eigen::Affine3d::Identity()) {
}

DHParameters::DHParameters( ros::NodeHandle *nh, std::string parameter_name ) :
							nh_(nh),
							d_(0.0),
							t_(0.0),
							r_(0.0),
							a_(0.0),
							jt_(JointType::Static),
							q_(0.0),
							parameters_changed_(true),
							transform_valid_(true),
							transform_(Eigen::Affine3d::Identity()) {
	load(parameter_name);
}

DHParameters::DHParameters( ros::NodeHandle *nh, const double d, const double t, const double r, const double a, const JointType jt, const double q ) :
							nh_(nh),
							d_(0.0),
							t_(0.0),
							r_(0.0),
							a_(0.0),
							jt_(JointType::Static),
							q_(0.0),
							parameters_changed_(true),
							transform_valid_(true),
							transform_(Eigen::Affine3d::Identity()) {
	set(d,t,r,a,jt,q);
}

DHParameters::~DHParameters( void ) {

}

bool DHParameters::load( std::string parameter_name ) {
	double success = false;

	double d = 0.0;
	double t = 0.0;
	double r = 0.0;
	double a = 0.0;
	JointType jt = JointType::Static;
	double q = 0.0;

	if( nh_->getParam(parameter_name + "/d", d) &&
		nh_->getParam(parameter_name + "/t", t) &&
		nh_->getParam(parameter_name + "/r", r) &&
		nh_->getParam(parameter_name + "/a", a) ) {

		std::string type;
		nh_->getParam(parameter_name + "/type", type);

		if(type == "twist_x") {
			jt = JointType::TwistX;
		} else if(type == "twist_z") {
			jt = JointType::TwistZ;
		} else if(type == "translate_x") {
			jt = JointType::TranslateX;
		} else if(type == "translate_z") {
			jt = JointType::TranslateZ;
		} else if(type != "static") {
			ROS_WARN("Unknown joint type (%s), setting static", type.c_str());
		} //Else it's static, and that's already set

		//Try to get joint variable, but no issue if we can't
		nh_->getParam(parameter_name + "/q", q);

		success = set(d, t, r, a, jt, q);
	}

	return success;
}

bool DHParameters::set( const double d, const double t, const double r, const double a, const JointType jt, const double q) {
	transform_valid_ = set_d(d) &&
					   set_t(t) &&
					   set_r(r) &&
					   set_a(a) &&
					   set_jt(jt) &&
					   set_q(q);

	parameters_changed_ = true;

	return transform_valid_;
}

bool DHParameters::is_valid( void ) {
	return transform_valid_;
}

double DHParameters::d( void ) {
	return d_;
}

double DHParameters::t( void ) {
	return t_;
}

double DHParameters::r( void ) {
	return r_;
}

double DHParameters::a( void ) {
	return a_;
}

DHParameters::JointType DHParameters::jt( void ) {
	return jt_;
}

double DHParameters::q( void ) {
	return q_;
}

bool DHParameters::update(const double q) {
	bool success = set_q(q);

	parameters_changed_ = success;

	return success;
}

Eigen::Affine3d DHParameters::transform( void ) {
	if(is_valid()) {
		if(parameters_changed_)
			generate_transform();

		return transform_;
	}

	return Eigen::Affine3d::Identity();
}

bool DHParameters::set_d( const double d ) {
	d_ = d;

	return true;
}

bool DHParameters::set_t( const double t ) {
	t_ = t;

	return true;
}

bool DHParameters::set_r( const double r ) {
	r_ = r;

	return true;
}

bool DHParameters::set_a( const double a ) {
	a_ = a;

	return true;
}

bool DHParameters::set_jt( DHParameters::JointType jt ) {
	bool success = false;

	if( (jt >= 0) && (jt < JointType::NUM_TYPES) ) {
		jt_ = jt;
		success = true;
	}

	return success;
}

bool DHParameters::set_q( const double q ) {
	q_ = q;

	return true;
}

void DHParameters::generate_transform( void ) {
	Eigen::Affine3d Td = Eigen::Affine3d::Identity();
	Eigen::Affine3d Rt = Eigen::Affine3d::Identity();
	Eigen::Affine3d Tr = Eigen::Affine3d::Identity();
	Eigen::Affine3d Ra = Eigen::Affine3d::Identity();

	double d = this->d();
	double t = this->t();
	double r = this->r();
	double a = this->a();

	switch(jt()) {
		case JointType::Static: {
			break;
		}
		case JointType::TwistX: {
			a += q();

			break;
		}
		case JointType::TwistZ: {
			t += q();

			break;
		}
		case JointType::TranslateX: {
			r += q();

			break;
		}
		case JointType::TranslateZ: {
			d += q();

			break;
		}
		default: {
			ROS_ASSERT_MSG( false, "Unhandled joint type %i", jt() );
		}
	}

    //Translation of d
	//Td = [1, 0, 0, 0;
    //		0, 1, 0, 0;
	//		0, 0, 1, d;
    //		0, 0, 0, 1];
	Td.translation() << Eigen::Vector3d(0.0, 0.0, d);

	//Rotation of theta
	//Rt = [cos(theta), -sin(theta), 0, 0;
	//		sin(theta),  cos(theta), 0, 0;
	//				 0,			  0, 1, 0;
	//				 0,			  0, 0, 1];
	Rt.linear().block(0,0,2,2) << std::cos(t), -std::sin(t),
								  std::sin(t),  std::cos(t);

	//Translation of r
	//Tr = [1, 0, 0, r;
    //		0, 1, 0, 0;
	//		0, 0, 1, 0;
    //		0, 0, 0, 1];
	Tr.translation() << Eigen::Vector3d(r, 0.0, 0.0);

    //Rotation of alpha
    //Ra = [1,			0,			 0, 0; ...
    //		0, cos(alpha), -sin(alpha), 0; ...
    //		0, sin(alpha),  cos(alpha), 0; ...
    //		0,			0,			 0, 1];
	Ra.linear().block(1,1,2,2) << std::cos(a), -std::sin(a),
								  std::sin(a),  std::cos(a);

    //Formulate transformation
    transform_ = Rt*Td*Tr*Ra;
}

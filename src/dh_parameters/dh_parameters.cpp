#include <dh_parameters/dh_parameters.h>

#include <eigen3/Eigen/Dense>

#include <math.h>
#include <string>


DHParameters::DHParameters( void ) :
							d_(0.0),
							t_(0.0),
							r_(0.0),
							a_(0.0),
							name_(""),
							jt_(JointType::Static),
							q_(0.0),
							qd_(0.0),
							qdd_(0.0),
							beta_(0.0),
							parameters_changed_(true),
							transform_valid_(false),
							transform_(Eigen::Affine3d::Identity()) {
}

DHParameters::DHParameters( ros::NodeHandle &nh, std::string parameter_name ) :
							d_(0.0),
							t_(0.0),
							r_(0.0),
							a_(0.0),
							name_(""),
							jt_(JointType::Static),
							q_(0.0),
							qd_(0.0),
							qdd_(0.0),
							beta_(0.0),
							parameters_changed_(true),
							transform_valid_(false),
							transform_(Eigen::Affine3d::Identity()) {
	load( nh, parameter_name);
}

DHParameters::DHParameters( const dh_parameters::JointDescription &description ) :
							d_(0.0),
							t_(0.0),
							r_(0.0),
							a_(0.0),
							name_(""),
							jt_(JointType::Static),
							q_(0.0),
							qd_(0.0),
							qdd_(0.0),
							beta_(0.0),
							parameters_changed_(true),
							transform_valid_(false),
							transform_(Eigen::Affine3d::Identity()) {
	load(description);
}

DHParameters::DHParameters( const double d, const double t, const double r, const double a, const std::string name, const JointType jt, const double q, const double beta) :
							d_(0.0),
							t_(0.0),
							r_(0.0),
							a_(0.0),
							name_(""),
							jt_(JointType::Static),
							q_(0.0),
							qd_(0.0),
							qdd_(0.0),
							beta_(0.0),
							parameters_changed_(true),
							transform_valid_(false),
							transform_(Eigen::Affine3d::Identity()) {
	set(d,t,r,a,name,jt,q,beta);
}

DHParameters::~DHParameters( void ) {

}

bool DHParameters::load( ros::NodeHandle& nh, std::string parameter_name ) {
	double success = false;

	double d = 0.0;
	double t = 0.0;
	double r = 0.0;
	double a = 0.0;
	std::string name;
	JointType jt = JointType::Static;
	double q = 0.0;
	double beta = 0.0;

	if( nh.getParam(parameter_name + "/d", d) &&
		nh.getParam(parameter_name + "/t", t) &&
		nh.getParam(parameter_name + "/r", r) &&
		nh.getParam(parameter_name + "/a", a) ) {

		std::string type;
		nh.getParam(parameter_name + "/type", type);

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

		//Try to get additional info, but no issue if we can't
		nh.getParam(parameter_name + "/name", name);
		nh.getParam(parameter_name + "/q", q);
		nh.getParam(parameter_name + "/beta", beta);

		success = set(d, t, r, a, name, jt, q, beta);
	}

	return success;
}

bool DHParameters::load( const dh_parameters::JointDescription &description ) {
	JointType jt = JointType::Static;

	if(description.type == "twist_x") {
		jt = JointType::TwistX;
	} else if(description.type == "twist_z") {
		jt = JointType::TwistZ;
	} else if(description.type == "translate_x") {
		jt = JointType::TranslateX;
	} else if(description.type == "translate_z") {
		jt = JointType::TranslateZ;
	} else if(description.type != "static") {
		ROS_WARN("Unknown joint type (%s), setting static", description.type.c_str());
	} //Else it's static, and that's already set

	bool success = set(description.d,
					   description.t,
					   description.r,
					   description.a,
					   description.name,
					   jt,
					   description.q,
					   description.beta);

	return success;
}

bool DHParameters::set( const double d, const double t, const double r, const double a, const std::string name, const JointType jt, const double q, const double beta) {
	transform_valid_ = set_d(d) &&
					   set_t(t) &&
					   set_r(r) &&
					   set_a(a) &&
					   set_name(name) &&
					   set_jt(jt) &&
					   set_q(q) &&
					   set_accel_filter(beta);

	parameters_changed_ = true;

	return transform_valid_;
}

bool DHParameters::set_accel_filter( const double b ) {
	bool success = false;

	if( (b >= 0.0) && (b <= 1.0) ) {
		beta_ = b;
		success = true;
	}

	return success;
}

bool DHParameters::is_valid( void ) const {
	return transform_valid_;
}

double DHParameters::d( void ) const {
	return d_;
}

double DHParameters::t( void ) const {
	return t_;
}

double DHParameters::r( void ) const {
	return r_;
}

double DHParameters::a( void ) const {
	return a_;
}


std::string DHParameters::name( void ) const {
	return name_;
}

DHParameters::JointType DHParameters::jt( void ) const {
	return jt_;
}

double DHParameters::q( void ) const {
	return q_;
}

double DHParameters::qd( void ) const {
	return qd_;
}

double DHParameters::qdd( void ) const {
	return qdd_;
}

bool DHParameters::update(const double q, const double qd, const double dt) {
	bool success = true;

	//Only estimate accel if dt is valid
	double a = (dt > 0.0) ? (qd - qd_) / dt : 0.0;

	//If the same velocity value comes in, as last time, bypass the filter
	a = (qd == qd_) ? 0.0 : lpf(a, qdd_);

	success &= set_qdd(a);
	success &= set_qd(qd);
	success &= set_q(q);

	parameters_changed_ = success;

	return success;
}

const Eigen::Affine3d& DHParameters::transform( void ) {
	if(is_valid()) {
		if(parameters_changed_)
			generate_transform();

	} else {
		transform_ = Eigen::Affine3d::Identity();
	}

	return transform_;
}

bool DHParameters::set_d( const double d ) {
	d_ = d;
	parameters_changed_ = true;

	return true;
}

bool DHParameters::set_t( const double t ) {
	t_ = t;
	parameters_changed_ = true;

	return true;
}

bool DHParameters::set_r( const double r ) {
	r_ = r;
	parameters_changed_ = true;

	return true;
}

bool DHParameters::set_a( const double a ) {
	a_ = a;
	parameters_changed_ = true;

	return true;
}

bool DHParameters::set_name( const std::string name ) {
	name_ = name;
	parameters_changed_ = true;

	return true;
}

bool DHParameters::set_jt( DHParameters::JointType jt ) {
	bool success = false;

	if( (jt >= 0) && (jt < JointType::NUM_TYPES) ) {
		jt_ = jt;
		parameters_changed_ = true;
		success = true;
	}

	return success;
}

bool DHParameters::set_q( const double q ) {
	q_ = q;

	return true;
}

bool DHParameters::set_qd( const double qd ) {
	qd_ = qd;

	return true;
}

bool DHParameters::set_qdd( const double qdd ) {
	qdd_ = qdd;

	return true;
}

double DHParameters::lpf(const double v, const double vp) {
	return ((1.0 - beta_) * vp) + (beta_ * v);
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

#pragma once

#include <ros/ros.h>

#include <dh_parameters/JointDescription.h>
#include <eigen3/Eigen/Dense>

#include <string>

class DHParameters {
	public:
		enum JointType {
			Static = 0,	//Non-moving joint
			TwistX,		//Pure twist around X axis
			TwistZ,		//Pure twist around Z axis
			TranslateX,	//Pure twist around Z axis
			TranslateZ,	//Pure twist around Z axis
			NUM_TYPES
			//TODO: screw/combination joint types
		};

	private:
		std::string name_;

		double d_;	//Z offset distance
		double t_;	//Z rotation
		double r_;	//X offset distance
		double a_;	//X rotation

		JointType jt_;	//Joint type
		double q_;	//Joint parameter position
		double qd_;	//Joint parameter velocity
		double qdd_;//Joint parameter acceleration estimate

		double beta_;

		bool parameters_changed_;
		bool transform_valid_;
		Eigen::Affine3d transform_;

	public:
		DHParameters( void );
		DHParameters( ros::NodeHandle *nh, std::string parameter_name );
		DHParameters( const dh_parameters::JointDescription &description );
		DHParameters( const double d,
					  const double t,
					  const double r,
					  const double a,
					  const std::string name = "",
					  const JointType jt = JointType::Static,
					  const double q = 0.0,
					  const double beta = 0.0 );

		~DHParameters( void );

		//Load in a configuration
		bool load( ros::NodeHandle *nh, std::string parameter_name );
		bool load( const dh_parameters::JointDescription &description );

		//Set the joint parameters
		bool set( const double d,
				  const double t,
				  const double r,
				  const double a,
				  const std::string name = "",
				  const JointType jt = JointType::Static,
				  const double q = 0.0,
				  const double beta = 0.0);
		bool set_accel_filter( const double b );

		bool is_valid( void ) const;

		//Get the joint parameters
		std::string name( void ) const;

		double d( void ) const;
		double t( void ) const;
		double r( void ) const;
		double a( void ) const;

		JointType jt( void ) const;
		double q( void ) const;
		double qd( void ) const;
		double qdd( void ) const;

		//Update the joint variable
		bool update(const double q, const double qd = 0.0, const double dt = 0.0);

		//Get the joint transformation
		const Eigen::Affine3d& transform( void );

	private:
		bool set_d( const double d );
		bool set_t( const double t );
		bool set_r( const double r );
		bool set_a( const double a );

		bool set_name( const std::string name );

		bool set_jt( const JointType jt );
		bool set_q( const double q );
		bool set_qd( const double qd );
		bool set_qdd( const double qdd );

		double lpf( const double v, const double vp );

		void generate_transform( void );
};

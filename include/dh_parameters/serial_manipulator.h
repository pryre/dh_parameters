#pragma once

#include <dh_parameters/dh_parameters.h>
#include <dh_parameters/JointDescription.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include <vector>

typedef struct {
	double center_of_mass;
	int parent_joint;
} body_map_t;

class SerialManipulator {
	private:
		std::vector<DHParameters,Eigen::aligned_allocator<DHParameters> > joints_;
		std::vector<body_map_t> body_map_;

	public:
		SerialManipulator( void );
		~SerialManipulator( void );

		bool add_joint( const dh_parameters::JointDescription& description );
		bool add_joint( const DHParameters& joint );
		bool add_body( const double center_of_mass, const int parent_joint );

		bool is_valid( void );
		void reset( void );

		int num_joints( void );
		void get_dynamic_joint_map( std::vector<bool>& map );
		DHParameters& joint( int i );

		void get_q(Eigen::VectorXd &q);
		void get_qxn(Eigen::VectorXd &q, const unsigned int x, const unsigned int n );
		void get_qd(Eigen::VectorXd &qd);
		void get_qdxn(Eigen::VectorXd &qd, const unsigned int x, const unsigned int n );

		void calculate_Jxy( Eigen::MatrixXd &J, const unsigned int x, const unsigned int y );
		void calculate_Je( Eigen::MatrixXd &Je );
		void calculate_J_body( Eigen::MatrixXd &Jbi, const int unsigned bi );

		void calculate_gxy( Eigen::Affine3d &g, const unsigned int x, const unsigned int y );
		void calculate_gbe( Eigen::Affine3d &gbe );
		void calculate_g_body( Eigen::Affine3d &gbi, const unsigned int i );
};

/*
 * This source file was written to control the fully-actuated hexrotor platform
 * within the optitrack environment using rc_override and the APM flight stack.
 *
 * written by: Jameson Lee
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <keyboard/Key.h>
#include <math.h>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "marvelmind_nav/hedge_pos.h"

//////////////////////
// Keyboard Defines //
//////////////////////

#define Q_KEY 113
#define A_KEY 97
#define W_KEY 119
#define S_KEY 115
#define E_KEY 101
#define D_KEY 100
#define R_KEY 114
#define F_KEY 102
#define T_KEY 116
#define G_KEY 103
#define Y_KEY 121
#define H_KEY 104
#define ONE_KEY 49
#define TWO_KEY 50
#define THREE_KEY 51
#define FOUR_KEY 52

#define POINT_ONE 1
#define POINT_TWO 2
#define POINT_THREE 3
#define POINT_FOUR 4
#define EXIT_COLLECT 0

// sync type defines
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
typedef message_filters::Subscriber<geometry_msgs::PoseStamped> marvel_sub_type;
typedef message_filters::Subscriber<geometry_msgs::PoseStamped> opti_sub_type;

class Calib{
	public:
		Calib();

	private:

        ros::NodeHandle nh;       			// ROS Node Handle

		message_filters::Subscriber<geometry_msgs::PoseStamped> *marvel_sub;	// create subscriber for marvelmind (stamped)
		message_filters::Subscriber<geometry_msgs::PoseStamped> *opti_sub;		// create subscriber for optitrack (stamped)
		message_filters::Synchronizer<MySyncPolicy> *sync;						// define sync policy to generate callback trigger on sync

        ros::Subscriber keydown_sub;        // subscriber for keyboard downstroke
        ros::Subscriber keyup_sub;          // subscriber for keyboard upstroke
		ros::Subscriber marvel_raw_sub;     // subscriber for marvelmind_nav raw message

		ros::Publisher hedge_pub;           // publish PoseStamped message for marvelmind_nav
		ros::Publisher debug_pub;           // debug publisher (if needed)

    	tf::TransformBroadcaster br;        // tf broadcaster object
        tf::TransformListener Li;           // tf Listener object
		tf::Transform TM;                   // Global to Marvelmind Frame Transform
		tf::Transform TO;                   // Global to Optitrack Frame Transform

		geometry_msgs::PoseStamped hedge_bag;	// sub msg to convert marvelmind /hedge_pos topic to /hedge_pos_stamped topic
		ros::Time hedge_time_bag;				// ros::Time for pose stamped message

		std_msgs::String msg;					// debug string msg
	    std::ostringstream os;					// debug string stream data

		int collect_mode;						// define if you are capturing a point
		bool catch_mask[5];						// defines if a message has been captured

		int k, k_last;										// defines current and last capture mode
		bool publish_transforms, publish_transforms_last;	// defines current and last transform publish state
		std::vector < std::vector <double> > debug_M;		// debug matrix
		std::vector < std::vector <double> > M;				// container for transform solver matrix
		std::vector <double> Y;								// container for transform solver output
		std::vector <double> X;								// container for transform solver solution
		double ox[5], oy[5], oz[5];							// store optitrack data
		double mx[5], my[5], mz[5];							// store marvelmind_nav data

	    //callback defines
		void sync_cb(const geometry_msgs::PoseStampedConstPtr& marvel_stamp, const geometry_msgs::PoseStampedConstPtr& opti_stamp);
		void marvel_cb(const marvelmind_nav::hedge_posConstPtr& marvel_stamp_raw);
        void keydown_cb(const keyboard::KeyConstPtr& keydown);
		void keyup_cb(const keyboard::KeyConstPtr& keyup);

		double det(std::vector <std::vector <double> > M);
		double cofactor(double a, double b, double c, double d);
		std::vector < std::vector <double> > Transpose(std::vector < std::vector <double> > M);
		std::vector < std::vector <double> > GenCofactor(std::vector < std::vector <double> > M);
		std::vector < std::vector <double> > Inverse(std::vector < std::vector <double> > M);
		std::vector < std::vector <double> > initMatrix(std::vector < std::vector <double> > M, int row, int col);

		std::vector <double> push3(double x, int k);
};

Calib::Calib(){
	marvel_sub = new marvel_sub_type(nh, "/hedge_pos_stamped", 1);
	opti_sub = new opti_sub_type(nh, "/vrpn/marvel/pose", 1);

	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *marvel_sub, *opti_sub);
  	sync -> registerCallback(boost::bind(&Calib::sync_cb, this, _1, _2));

	marvel_raw_sub = nh.subscribe<marvelmind_nav::hedge_pos>("/hedge_pos", 1, &Calib::marvel_cb, this);
	keyup_sub = nh.subscribe<keyboard::Key>("/keyboard/keyup", 1, &Calib::keyup_cb, this);
	keydown_sub = nh.subscribe<keyboard::Key>("/keyboard/keydown", 1, &Calib::keydown_cb, this);

	hedge_pub = nh.advertise<geometry_msgs::PoseStamped>("/hedge_pos_stamped", 1);
	debug_pub = nh.advertise<std_msgs::String>("/debug", 1);

	collect_mode = EXIT_COLLECT;

	k = 0;
	k_last = 0;
	publish_transforms = false;
	publish_transforms_last = false;
	for(int i = 0; i < 5; i++){
		catch_mask[i] = false;
		ox[i] = 0.0f;
		oy[i] = 0.0f;
		oz[i] = 0.0f;
		mx[i] = 0.0f;
		my[i] = 0.0f;
		mz[i] = 0.0f;
	}

	/*debug_M = Calib::initMatrix(debug_M, 3, 3);

	debug_M[0][0] = 3.0f;
	debug_M[0][1] = 2.0f;
	debug_M[0][2] = 17.0f;
	debug_M[1][0] = -9.0f;
	debug_M[1][1] = 2.0f;
	debug_M[1][2] = 7.0f;
	debug_M[2][0] = 4.0f;
	debug_M[2][1] = 18.0f;
	debug_M[2][2] = -12.0f;

	debug_M = Calib::Inverse(debug_M);
	for(int i = 0; i < debug_M.size(); i++){
		for(int j = 0; j < debug_M.size(); j++){
			os << debug_M[i][j] << ",";
		}
	}
	msg.data = os.str();
	*/
}

void Calib::sync_cb(const geometry_msgs::PoseStampedConstPtr& marvel_stamp, const geometry_msgs::PoseStampedConstPtr& opti_stamp){

	switch(collect_mode){
		case EXIT_COLLECT:
			k = 0;
			break;
		case POINT_ONE:
			k = 1;
			if(k != k_last) ROS_INFO_STREAM("Point 1 Collected...");
			break;
		case POINT_TWO:
			k = 2;
			if(k != k_last) ROS_INFO_STREAM("Point 2 Collected...");
			break;
		case POINT_THREE:
			k = 3;
			if(k != k_last) ROS_INFO_STREAM("Point 3 Collected...");
			break;
		case POINT_FOUR:
			k = 4;
			if(k != k_last) ROS_INFO_STREAM("Point 4 Collected...");
			break;
	}

	k_last = k;
	catch_mask[k] = true;
	ox[k] = opti_stamp -> pose.position.x;
	oy[k] = opti_stamp -> pose.position.y;
	oz[k] = opti_stamp -> pose.position.z;
	mx[k] = marvel_stamp -> pose.position.x;
	my[k] = marvel_stamp -> pose.position.y;
	mz[k] = marvel_stamp -> pose.position.z;

	if((collect_mode == EXIT_COLLECT) && (publish_transforms_last == false) && (catch_mask[1] == catch_mask[2] == catch_mask[3] == catch_mask[4] == true)){
		publish_transforms = true;
		std::vector <double> x;
		std::vector <double> x_;
		std::vector <double> y;
		std::vector <double> y_;
		std::vector <double> z;
		std::vector <double> z_;
		for(int i = 0; i < 4; i++){	//sets of p1xp1yp1z
			x.clear();
			y.clear();
			z.clear();
			Y.push_back(ox[i + 1]);
			Y.push_back(oy[i + 1]);
			Y.push_back(oz[i + 1]);
			for(int j = 0; j < 4; j++){
				if(i == 3){
					x_ = push3(1.0f,i);
					y_ = push3(1.0f,i);
					z_ = push3(1.0f,i);
				}
				else{
					x_ = push3(mx[j + 1],i);
					y_ = push3(my[j + 1],i);
					z_ = push3(mz[j + 1],i);
				}
				for(int r = 0; r < 3; k++){
					x.push_back(x_[r]);
					y.push_back(y_[r]);
					z.push_back(z_[r]);
				}
			}
			M.push_back(x);
			M.push_back(y);
			M.push_back(z);
		}
		M = Calib::Inverse(M);
		double q;
		for(int i = 0; i < 12; i++){
			q = 0.0f;
			for(int j = 0; j < 12; j++){
				q = q + M[i][j] * Y[j];
			}
			X.push_back(q);
		}
		if(publish_transforms == true ) ROS_INFO_STREAM("TRANSFORM ACQUIRED...");
	}

	if(publish_transforms == true){
		double mx_, my_, mz_;
		mx_ = X[0]*mx[0] + X[1]*my[0] + X[2]*mz[0] + X[9];
		my_ = X[3]*mx[0] + X[4]*my[0] + X[5]*mz[0] + X[10];
		mz_ = X[6]*mx[0] + X[7]*my[0] + X[8]*mz[0] + X[11];
		TM.setOrigin(tf::Vector3(mx_,my_,mz_));
		TO.setOrigin(tf::Vector3(ox[0],oy[0],oz[0]));
		TM.setRotation(tf::Quaternion(marvel_stamp -> pose.orientation.x,marvel_stamp -> pose.orientation.y,marvel_stamp -> pose.orientation.z,marvel_stamp -> pose.orientation.w));
		TO.setRotation(tf::Quaternion(opti_stamp -> pose.orientation.x,opti_stamp -> pose.orientation.y,opti_stamp -> pose.orientation.z,opti_stamp -> pose.orientation.w));

	    br.sendTransform(tf::StampedTransform(TM,opti_stamp -> header.stamp,"UNIVERSAL","MARVEL_POS"));
		br.sendTransform(tf::StampedTransform(TO,opti_stamp -> header.stamp,"UNIVERSAL","OPTI_POS"));
	}

}

void Calib::marvel_cb(const marvelmind_nav::hedge_posConstPtr& marvel_stamp_raw){

	hedge_time_bag.sec = uint32_t(marvel_stamp_raw -> timestamp_ms / 1000);
	hedge_time_bag.nsec = uint32_t(marvel_stamp_raw -> timestamp_ms * 1000);

	hedge_bag.header.stamp = hedge_time_bag;
    hedge_bag.header.frame_id = "marvelmind_nav";
	hedge_bag.pose.position.x = marvel_stamp_raw -> x_m;
	hedge_bag.pose.position.y = marvel_stamp_raw -> y_m;
	hedge_bag.pose.position.z = marvel_stamp_raw -> z_m;

	hedge_bag.pose.orientation.x = 0.0f;
	hedge_bag.pose.orientation.y = 0.0f;
	hedge_bag.pose.orientation.z = 0.0f;
	hedge_bag.pose.orientation.w = 1.0f;

	hedge_pub.publish(hedge_bag);
}

std::vector <double> Calib::push3(double x, int k){
	std::vector <double> x_;
	for(int i = 0; i < 3; i++){
		x_.push_back(0.0f);
	}
	x_[k] = x;
	return x_;
}

std::vector < std::vector <double> > Calib::initMatrix(std::vector < std::vector <double> > Q, int row, int col){
	std::vector <double> de;
	for(int j = 0; j < row; j++){
		de.push_back(0.0f);
	}
	for(int i = 0; i < col; i++){
		Q.push_back(de);
	}
	return Q;
}

double Calib::det(std::vector <std::vector <double> > M){
	if(M.size() == 2) return Calib::cofactor(M[0][0],M[0][1],M[1][0],M[1][1]);
	else{
		double _sign = 1.0f;
		double determinant = 0.0f;
		for(int i = 0; i < M.size(); i++){
			if(i % 2 == 0) _sign = 1.0f;
			else _sign = -1.0f;
			std::vector <std::vector <double> > M_;
			M_ = Calib::initMatrix(M_,M.size() - 1, M.size() - 1);
			for(int j = 0; j < M.size(); j++){
				for(int r = 0; r < M.size(); r++){
					if(j > 0 && r < i)      M_[j- 1][r]      = M[j][r];
					else if(j > 0 && r > i) M_[j - 1][r - 1] = M[j][r];
				}
			}
			determinant = determinant + _sign * M[0][i] * Calib::det(M_);
		}
		return determinant;
	}
}

double Calib::cofactor(double a, double b, double c, double d){
	return (a * d) - (b * c);
}

std::vector < std::vector <double> > Calib::Transpose(std::vector < std::vector <double> > M){
	std::vector < std::vector <double> > M_;
	M_ = Calib::initMatrix(M_, M.size(), M.size());
	for(int i = 0; i < M.size(); i++){
		for(int j = 0; j < M.size(); j++){
			M_[i][j] = M[j][i];
		}
	}
	return M_;
}

std::vector < std::vector <double> > Calib::GenCofactor(std::vector < std::vector <double> > M){
	if(M.size() == 2) return M;
	else{
		double _sign_r = 1.0f;
		double _sign_c = 1.0f;
		std::vector < std::vector <double> > C;
		C = Calib::initMatrix(C, M.size(), M.size());
		for(int i = 0; i < M.size(); i++){
			for(int j = 0; j < M.size(); j++){
				if(i % 2 == 0) _sign_r = 1.0f;
				else _sign_r = -1.0f;
				if(j % 2 == 0) _sign_c = 1.0f;
				else _sign_c = -1.0f;
				std::vector <std::vector <double> > C_;
				C_ = Calib::initMatrix(C_, M.size() - 1, M.size() - 1);
				for(int k = 0; k < M.size(); k++){
					for(int r = 0; r < M.size(); r++){
						if(k < i && r < j)      C_[k][r]         = M[k][r];
						else if(k < i && r > j) C_[k][r - 1]     = M[k][r];
						else if(k > i && r < j) C_[k - 1][r]     = M[k][r];
						else if(k > i && r > j) C_[k - 1][r - 1] = M[k][r];
					}
				}
				C[i][j] = _sign_r * _sign_c * Calib::det(C_);
			}
		}
		return C;
	}
}

std::vector < std::vector <double> > Calib::Inverse(std::vector < std::vector <double> > M){
	std::vector < std::vector <double> > I;
	I = Calib::initMatrix(I, M.size(), M.size());
	I = Calib::GenCofactor(M);
	I = Calib::Transpose(I);
	double D = Calib::det(M);
	for(int i = 0; i < M.size(); i++){
		for(int j = 0; j < M.size(); j ++){
			I[i][j] = I[i][j]/D;
		}
	}
	return I;
}


void Calib::keydown_cb(const keyboard::KeyConstPtr& keydown){
	int x = keydown -> code;
	switch(x){
		case Q_KEY:
			debug_pub.publish(msg);
			break;
		case A_KEY:

			break;
		case W_KEY:

			break;
		case S_KEY:

			break;
		case E_KEY:

			break;
		case D_KEY:

			break;
		case R_KEY:

			break;
		case F_KEY:

			break;
		case T_KEY:

			break;
		case G_KEY:

			break;
		case Y_KEY:

			break;
		case H_KEY:
			break;
        case ONE_KEY:
			collect_mode = POINT_ONE;
            break;
        case TWO_KEY:
			collect_mode = POINT_TWO;
            break;
        case THREE_KEY:
			collect_mode = POINT_THREE;
            break;
		case FOUR_KEY:
			collect_mode = POINT_FOUR;
	        break;
	}
}

void Calib::keyup_cb(const keyboard::KeyConstPtr& keyup){
    int x = keyup -> code;
    switch(x){
        case Q_KEY:

            break;
        case A_KEY:

            break;
        case W_KEY:

            break;
        case S_KEY:

            break;
        case E_KEY:

            break;
        case D_KEY:

            break;
        case R_KEY:

            break;
        case F_KEY:

            break;
        case T_KEY:

            break;
        case G_KEY:

            break;
        case Y_KEY:

            break;
        case H_KEY:

            break;
		case ONE_KEY:
			collect_mode = EXIT_COLLECT;
	        break;
	    case TWO_KEY:
			collect_mode = EXIT_COLLECT;
	        break;
	    case THREE_KEY:
			collect_mode = EXIT_COLLECT;
	        break;
		case FOUR_KEY:
			collect_mode = EXIT_COLLECT;
	        break;
    }
}


int main(int argc, char **argv){
	ros::init(argc, argv, "Calib");
	ROS_INFO_STREAM("Calibrate Optitrack and MarvelMind Sensors!");
	Calib calib;

	while(ros::ok()){
		ros::spinOnce();
	}
}

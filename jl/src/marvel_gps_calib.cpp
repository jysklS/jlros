/*
 * This source file was written to sync gps and marvelmind position data
 *
 * written by: Jameson Lee
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

// sync type defines (here we use approximate Time sync policy)
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseStamped> MySyncPolicy;
typedef message_filters::Subscriber<geometry_msgs::PoseStamped> marvel_sub_type;
typedef message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> gps_sub_type;

class Calib{
	public:
		Calib();

	private:

        ros::NodeHandle nh;       			// ROS Node Handle

		message_filters::Subscriber<geometry_msgs::PoseStamped> *marvel_sub;				// create subscriber for marvelmind (stamped)
		message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *gps_sub;		// create subscriber for gps (stamped)
		message_filters::Synchronizer<MySyncPolicy> *sync;									// define sync policy to generate callback trigger on sync

        ros::Subscriber keydown_sub;        // subscriber for keyboard downstroke
        ros::Subscriber keyup_sub;          // subscriber for keyboard upstroke
		ros::Subscriber marvel_raw_sub;     // subscriber for marvelmind_nav raw message

		ros::Publisher hedge_pub;           // publish PoseStamped message for marvelmind_nav
		ros::Publisher debug_pub;           // debug publisher (if needed)

    	tf::TransformBroadcaster br;        // tf broadcaster object
        tf::TransformListener Li;           // tf Listener object
		tf::Transform TM;                   // Global to Marvelmind Frame Transform
		tf::Transform TG;                   // Global to GPS Frame Transform

		geometry_msgs::PoseStamped hedge_bag;	// sub msg to convert marvelmind /hedge_pos topic to /hedge_pos_stamped topic
		ros::Time hedge_time_bag;				// ros::Time for pose stamped message

		std_msgs::String msg;					// debug string msg
	    std::ostringstream os;					// debug string stream data

		double gx,gy,gz,gqx,gqy,gqz,gqw;		// bin for gps data
		double mx,my,mz,mqx,mqy,mqz,mqw;		// bin for marvelmind data
		double cov[36];							// covariance matrix

	    //callback defines
		void sync_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& gps_stamp, const geometry_msgs::PoseStampedConstPtr& marvel_stamp);
		void marvel_cb(const marvelmind_nav::hedge_posConstPtr& marvel_stamp_raw);
        void keydown_cb(const keyboard::KeyConstPtr& keydown);
		void keyup_cb(const keyboard::KeyConstPtr& keyup);
};

Calib::Calib(){
	marvel_sub = new marvel_sub_type(nh, "/hedge_pos_stamped", 1);
	gps_sub = new gps_sub_type(nh, "/mavros/global_position/local", 1);

	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *gps_sub, *marvel_sub);
  	sync -> registerCallback(boost::bind(&Calib::sync_cb, this, _1, _2));

	marvel_raw_sub = nh.subscribe<marvelmind_nav::hedge_pos>("/hedge_pos", 1, &Calib::marvel_cb, this);
	keyup_sub = nh.subscribe<keyboard::Key>("/keyboard/keyup", 1, &Calib::keyup_cb, this);
	keydown_sub = nh.subscribe<keyboard::Key>("/keyboard/keydown", 1, &Calib::keydown_cb, this);

	hedge_pub = nh.advertise<geometry_msgs::PoseStamped>("/hedge_pos_stamped", 1);
	debug_pub = nh.advertise<std_msgs::String>("/debug", 1);

}

void Calib::sync_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& gps_stamp, const geometry_msgs::PoseStampedConstPtr& marvel_stamp){

	// This callback syncs the data incoming from stamped hedge_pos and mavros gps estimation. Below is the unpacked data manipulate as
	// you want...

	gx = gps_stamp -> pose.pose.position.x;			// gps global x estimation
	gy = gps_stamp -> pose.pose.position.x;			// gps global y estimation
	gz = gps_stamp -> pose.pose.position.x;			// gps global z estimation
	gqx = gps_stamp -> pose.pose.orientation.x;		// gps global x quaternion estimation
	gqy = gps_stamp -> pose.pose.orientation.y;		// gps global y quaternion estimation
	gqz = gps_stamp -> pose.pose.orientation.z;		// gps global z quaternion estimation
	gqw = gps_stamp -> pose.pose.orientation.w;		// gps global w quaternion estimation

	for(int i = 0; i < 36; i++){
		cov[i] = gps_stamp -> pose.covariance[i];	// covariance matrix elements (you can replace with vector, but this is how you unpack)
	}

	mx = marvel_stamp -> pose.position.x;			// marvelmind x position
	my = marvel_stamp -> pose.position.y;			// marvelmind y position
	mz = marvel_stamp -> pose.position.z;			// marvelmind z position

}

void Calib::marvel_cb(const marvelmind_nav::hedge_posConstPtr& marvel_stamp_raw){

	//hedge_time_bag.sec = uint32_t(marvel_stamp_raw -> timestamp_ms / 1000);			// The real time stamp is here, placed into ros::Time but time is not correct
	//hedge_time_bag.nsec = uint32_t(marvel_stamp_raw -> timestamp_ms * 1000);

	hedge_bag.header.stamp = ros::Time::now();				// assign current ros time to marvelmind (we assume they are close...
    hedge_bag.header.frame_id = "marvelmind_nav";			// we have to fix on marvelmind driver side because they set time to zero at node start.
	hedge_bag.pose.position.x = marvel_stamp_raw -> x_m;	// Fix maybe with ros::Time::now() at their instantiation)
	hedge_bag.pose.position.y = marvel_stamp_raw -> y_m;
	hedge_bag.pose.position.z = marvel_stamp_raw -> z_m;

	hedge_bag.pose.orientation.x = 0.0f;					// Fill with no rotation quaternion
	hedge_bag.pose.orientation.y = 0.0f;
	hedge_bag.pose.orientation.z = 0.0f;
	hedge_bag.pose.orientation.w = 1.0f;

	hedge_pub.publish(hedge_bag);							// publish to stamped hedge pose
}


void Calib::keydown_cb(const keyboard::KeyConstPtr& keydown){
	int x = keydown -> code;
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

            break;
        case TWO_KEY:

            break;
        case THREE_KEY:

            break;
		case FOUR_KEY:

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

	        break;
	    case TWO_KEY:

	        break;
	    case THREE_KEY:

	        break;
		case FOUR_KEY:

	        break;
    }
}


int main(int argc, char **argv){
	ros::init(argc, argv, "Calib");
	ROS_INFO_STREAM("Calibrate GPS and MarvelMind Sensors!");
	Calib calib;

	while(ros::ok()){
		ros::spinOnce();
	}
}

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
#include <keyboard/Key.h>
#include <math.h>

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

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
typedef message_filters::Subscriber<geometry_msgs::PoseStamped> marvel_sub_type;
typedef message_filters::Subscriber<geometry_msgs::PoseStamped> opti_sub_type;

class Calib{
	public:
		Calib();

	private:

        ros::NodeHandle nh;       			// ROS Node Handle

		message_filters::Subscriber<geometry_msgs::PoseStamped> *marvel_sub;
		message_filters::Subscriber<geometry_msgs::PoseStamped> *opti_sub;
		message_filters::Synchronizer<MySyncPolicy> *sync;

        ros::Subscriber keydown_sub;        // subscriber for keyboard downstroke
        ros::Subscriber keyup_sub;          // subscriber for keyboard upstroke
		ros::Subscriber marvel_raw_sub;     // subscriber for keyboard upstroke

        tf::TransformBroadcaster br;        // tf broadcaster object
        tf::TransformListener Li;           // tf Listener object

		int collect_mode;

	    //callback defines
		void sync_cb(const geometry_msgs::PoseStampedConstPtr& marvel_stamp, const geometry_msgs::PoseStampedConstPtr& opti_stamp);
		void marvel_cb(const marvelmind_nav::hedge_posConstPtr& marvel_stamp_raw);
        void keydown_cb(const keyboard::KeyConstPtr& keydown);
		void keyup_cb(const keyboard::KeyConstPtr& keyup);
};

Calib::Calib(){
	marvel_sub = new marvel_sub_type(nh, "/hedge_pos_stamped", 1);
	opti_sub = new opti_sub_type(nh, "/vrpn/marvel/pose", 1);

	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *marvel_sub, *opti_sub);
  	sync -> registerCallback(boost::bind(&Calib::sync_cb, this, _1, _2));

	marvel_raw_sub = nh.subscribe<marvelmind_nav::hedge_pos>("/hedge_pos", 1, &Calib::marvel_cb, this);
	keyup_sub = nh.subscribe<keyboard::Key>("/keyboard/keyup", 1, &Calib::keyup_cb, this);
	keydown_sub = nh.subscribe<keyboard::Key>("/keyboard/keydown", 1, &Calib::keydown_cb, this);


	collect_mode = EXIT_COLLECT;

}

void Calib::sync_cb(const geometry_msgs::PoseStampedConstPtr& marvel_stamp, const geometry_msgs::PoseStampedConstPtr& opti_stamp){





}

void Calib::marvel_cb(const marvelmind_nav::hedge_posConstPtr& marvel_stamp_raw){





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

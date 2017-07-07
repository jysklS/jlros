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
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <keyboard/Key.h>
#include <math.h>
#include <vector>
#include <iostream>

#define RATE_LOOP 0.025			// timer callback loop rate
#define TAKEOFF_HEIGHT 1.0f		// Desired Take off height
#define LANDING_HEIGHT 0.0f		// Desired Landing height
#define GROUNDED_HEIGHT 0.1f    // Desired Landed Height
#define HOME_X 0.0f				// desired home x
#define HOME_Y 0.0f				// desired home y
#define HOME_Z 1.0f				// desired home z
#define HOME_YAW 0.0f			// desired home yaw

// Control Modes
#define TAKEOFF 1				// from current x,y,psi, take off to desired takeoff height
#define HOME 2					// travel to home position
#define CONTROLLED 3			// allow reference changes
#define LANDING 4				// from current x,y,psi, return to ground
#define LANDED 5

// data collection mode
#define OUTPUT_CONTROL 1		// accumulate PID and output control effort
#define STANDBY 2				// set reference to current location, 0 PID elements

// reference mode
#define RELATIVE 1				// edit reference from current x,y,psi
#define GLOBAL 2				// edit reference from home position

// reference filter values for smooth reference transitions
#define takeoff_d1 1.0f
#define takeoff_d2 10.0f
#define landing_d1 1.0f
#define landing_d2 10.0f
#define control_d1 1.0f
#define control_d2 10.0f

// PID gains for reference edits
#define kp 1.0f
#define ki 0.01f
#define kd 0.01f

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

class Hex{
	public:
		Hex();

	private:

        ros::NodeHandle nh;       			// ROS Node Handle

        ros::Subscriber keydown_sub;        // subscriber for keyboard downstroke
        ros::Subscriber keyup_sub;          // subscriber for keyboard upstroke
		ros::Subscriber opti_sub;           // subscriber for optitrack
		ros::Subscriber ref_sub;            // subscriber for optitrack
		ros::Subscriber mavros_sub;         // subscriber for optitrack


		ros::Publisher rc_pub;              // Publishes rc output (1000-2000)
		ros::Publisher rc0_pub;             // Publishes rc output (1000-2000)
		ros::Publisher rc1_pub;             // Publishes rc output (1000-2000)
		ros::Publisher rc2_pub;             // Publishes rc output (1000-2000)
		ros::Publisher rc3_pub;             // Publishes rc output (1000-2000)

        ros::Timer timer;                   //ROS timer object

        tf::TransformBroadcaster br;        //tf broadcaster object
        tf::TransformListener Li;           //tf Listener object

		ros::Time data_time_now;
		ros::Time data_time_last;
		ros::Time ref_time_now;
		ros::Time ref_time_last;

		mavros_msgs::OverrideRCIn rc_msg;
		std_msgs::UInt16 rc0_msg;
		std_msgs::UInt16 rc1_msg;
		std_msgs::UInt16 rc2_msg;
		std_msgs::UInt16 rc3_msg;

		double pos[6];
		double ref[6];
		double ref_m1[6];
		double ref_in[6];

		double e[6];
		double ei[6];
		double ed[6];
		double e_last[6];

		double u[6];
		double rc_out[8];
		double RC_MIN[8];
		double RC_MAX[8];

		double data_dt;
		double ref_dt;

		int mode;
		int data_mode;
		int ref_mode;

        //callback defines
        void keydown_cb(const keyboard::KeyConstPtr& keydown);
		void keyup_cb(const keyboard::KeyConstPtr& keyup);
		void opti_cb(const geometry_msgs::PoseStampedConstPtr& pos_fb);
		void ref_cb(const geometry_msgs::PoseStampedConstPtr& ref_fb);
		void timer_cb(const ros::TimerEvent& event);

		void mavros_cb(const mavros_msgs::RCInConstPtr& rc_fb);

		double Quaternion2Roll(double qx, double qy, double qz, double qw);
		double Quaternion2Pitch(double qx, double qy, double qz, double qw);
		double Quaternion2Yaw(double qx, double qy, double qz, double qw);
};

Hex::Hex(){
    timer = nh.createTimer(ros::Duration(RATE_LOOP), &Hex::timer_cb, this);
	ref_sub = nh.subscribe<geometry_msgs::PoseStamped>("/hex_ref", 1, &Hex::ref_cb, this);
	opti_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vprn/hex/pose", 1, &Hex::opti_cb, this);
	keyup_sub = nh.subscribe<keyboard::Key>("/keyboard/keyup", 1, &Hex::keyup_cb, this);
	keydown_sub = nh.subscribe<keyboard::Key>("/keyboard/keydown", 1, &Hex::keydown_cb, this);
	mavros_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, &Hex::mavros_cb, this);

	rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
	rc0_pub = nh.advertise<std_msgs::UInt16>("/matlabsucks/rc0", 1);
	rc1_pub = nh.advertise<std_msgs::UInt16>("/matlabsucks/rc1", 1);
	rc2_pub = nh.advertise<std_msgs::UInt16>("/matlabsucks/rc2", 1);
	rc3_pub = nh.advertise<std_msgs::UInt16>("/matlabsucks/rc3", 1);

	for(int i = 0; i < 6; i++){
		pos[i] = 0;
		ref[i] = 0;
		ref_m1[i] = 0;
		ref_in[i] = 0;
		e_last[i] = 0;
		ei[i] = 0;
	}

	for(int i = 0; i < 8; i++){
		RC_MIN[i] = 1000.0f;
		RC_MAX[i] = 2000.0f;
		rc_out[i] = RC_MIN[i];
	}

	data_time_last = ros::Time::now();
	ref_time_last = ros::Time::now();

	mode = LANDED;
	data_mode = STANDBY;
	ref_mode = GLOBAL;
}

void Hex::timer_cb(const ros::TimerEvent& event){
	ref_time_now = ros::Time::now();
	ref_dt = (data_time_now - data_time_last).toSec();
	ref_time_last = data_time_now;

	switch(mode){
		case TAKEOFF:
			data_mode = OUTPUT_CONTROL;
			ref_m1[2] = ref[2];
			ref[2] = (takeoff_d1/(takeoff_d1 + takeoff_d2*ref_dt))*ref_m1[2] + (takeoff_d1*takeoff_d2*ref_dt/(takeoff_d1 + takeoff_d2*ref_dt))*(TAKEOFF_HEIGHT);
			break;
		case CONTROLLED:
			data_mode = OUTPUT_CONTROL;
			for(int i = 0; i < 6; i++){
				ref_m1[i] = ref[i];
				ref[i] = (control_d1/(control_d1 + control_d2*ref_dt))*ref_m1[i] + (control_d1*control_d2*ref_dt/(control_d1 + control_d2*ref_dt))*ref_in[i];
			}
			break;
		case LANDING:
			if(ref[2] < GROUNDED_HEIGHT) data_mode = STANDBY;
			else data_mode = OUTPUT_CONTROL;
			ref_m1[2] = ref[2];
			ref[2] = (landing_d1/(landing_d1 + landing_d2*ref_dt))*ref_m1[2] + (landing_d1*landing_d2*ref_dt/(landing_d1 + landing_d2*ref_dt))*(LANDING_HEIGHT);
			break;
		case LANDED:
			data_mode = STANDBY;
			break;
		default:
			data_mode = STANDBY;
			break;
	}

	double cos_y = cos(pos[5]);
	double sin_y = sin(pos[5]);

	switch(data_mode){
		case OUTPUT_CONTROL:
			rc_out[0] = int(RC_MIN[0] + (RC_MAX[0] - RC_MIN[0]) * (1.0f + u[0]*cos_y - u[1]*sin_y) / 2.0f);
			rc_out[1] = int(RC_MIN[1] + (RC_MAX[1] - RC_MIN[1]) * (1.0f + u[0]*sin_y + u[1]*cos_y) / 2.0f);
			rc_out[2] = int(RC_MIN[2] + (RC_MAX[2] - RC_MIN[2]) * (u[2]));
			rc_out[3] = int(RC_MIN[3] + (RC_MAX[2] - RC_MIN[2]) * (1.0f + u[5]) / 2.0f);
			break;
		case STANDBY:
			for(int i = 0; i < 4; i++){
				rc_out[i] = rc_msg.CHAN_NOCHANGE;
			}
			break;
		default:
			break;
	}

	for(int i = 0; i < 5; i++){
		rc_msg.channels[i] = rc_out[i];
	}
	rc_pub.publish(rc_msg);
}

void Hex::opti_cb(const geometry_msgs::PoseStampedConstPtr& pos_fb){
	data_time_now = ros::Time::now();
	data_dt = (data_time_now - data_time_last).toSec();
	data_time_last = data_time_now;

	double qx = pos_fb -> pose.orientation.x;
	double qy = pos_fb -> pose.orientation.y;
	double qz = pos_fb -> pose.orientation.z;
	double qw = pos_fb -> pose.orientation.w;

	pos[0] = pos_fb -> pose.position.x;
	pos[1] = pos_fb -> pose.position.y;
	pos[2] = pos_fb -> pose.position.z;
	pos[3] = Hex::Quaternion2Roll(qx,qy,qz,qw);
	pos[4] = Hex::Quaternion2Pitch(qx,qy,qz,qw);
	pos[5] = Hex::Quaternion2Yaw(qx,qy,qz,qw);

	switch(data_mode){
		case OUTPUT_CONTROL:
			for(int i = 0; i < 6; i++){
				e_last[i] = e[i];
				e[i] = ref[i] - pos[i];
				ed[i] = (e[i] - e_last[i])/data_dt;
				ei[i] = ei[i] + e[i]*data_dt;
				u[i] = kp*e[i] + ki*ei[i] + kd*ed[i];
				if(u[i] > 1.0f) u[i] = 1.0f;
				if(u[i] < -1.0f) u[i] = -1.0f;
			}
			if(u[2] < 0.0f) u[2] = 0.0f;
			break;
		case STANDBY:
			for(int i = 0; i < 6; i++){
				ref[i] = pos[i];
				e_last[i] = 0;
				e[i] = 0;
				ed[i] = 0;
				ei[i] = 0;
				u[i] = 0;
			}
			break;
		default:
			break;
	}
}

void Hex::ref_cb(const geometry_msgs::PoseStampedConstPtr& ref_fb){
	double qx = ref_fb -> pose.orientation.x;
	double qy = ref_fb -> pose.orientation.y;
	double qz = ref_fb -> pose.orientation.z;
	double qw = ref_fb -> pose.orientation.w;

	ref_in[0] = ref_fb -> pose.position.x;
	ref_in[1] = ref_fb -> pose.position.y;
	ref_in[2] = ref_fb -> pose.position.z;
	ref_in[3] = Hex::Quaternion2Roll(qx,qy,qz,qw);
	ref_in[4] = Hex::Quaternion2Pitch(qx,qy,qz,qw);
	ref_in[5] = Hex::Quaternion2Yaw(qx,qy,qz,qw);
}

void Hex::mavros_cb(const mavros_msgs::RCInConstPtr& rc_fb){
	rc0_msg.data = rc_fb -> channels[0];
	rc1_msg.data = rc_fb -> channels[1];
	rc2_msg.data = rc_fb -> channels[2];
	rc3_msg.data = rc_fb -> channels[3];

	rc0_pub.publish(rc0_msg);
	rc1_pub.publish(rc1_msg);
	rc2_pub.publish(rc2_msg);
	rc3_pub.publish(rc3_msg);
}

double Hex::Quaternion2Roll(double qx, double qy, double qz, double qw){
	double t0 = 2.0*(qw*qx + qy*qz);
	double t1 = 1.0 - 2.0*(qx*qx + qy*qy);
	return atan2(t0, t1);
}

double Hex::Quaternion2Pitch(double qx, double qy, double qz, double qw){
	double t2 = 2.0*(qw*qy - qz*qx);
	t2 = ((t2 > 1.0) ? 1.0 : t2);
	t2 = ((t2 < -1.0) ? -1.0 : t2);
	return asin(t2);
}

double Hex::Quaternion2Yaw(double qx, double qy, double qz, double qw){
	double t3 = 2.0*(qw*qz + qx*qy);
	double t4 = 1.0 - 2.0*(qy*qy + qz*qz);
	return atan2(t3, t4);
}

void Hex::keydown_cb(const keyboard::KeyConstPtr& keydown){
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
			mode = TAKEOFF;
            break;
        case TWO_KEY:
			mode = LANDING;
            break;
        case THREE_KEY:
            break;
	}
}

void Hex::keyup_cb(const keyboard::KeyConstPtr& keyup){
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
    }
}


int main(int argc, char **argv){
	ros::init(argc, argv, "Hex");
	ROS_INFO_STREAM("Generate Hex Reference Generation Node!");
	Hex hex;

	while(ros::ok()){
		ros::spinOnce();
	}
}

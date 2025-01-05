#include <ros/ros.h>
#include <assignment2_exprob/SetVelocityControl.h>
#include <geometry_msgs/Twist.h>
#include <assignment2_exprob/laser_direction.h>
#include <assignment2_exprob/Marker_id_pos.h>
ros::Publisher cmd_vel_pub;  // Publisher per il controllo della velocità
ros::Subscriber laser_dir_sub_;  // Subscriber per la direzione del laser
ros::Subscriber found_sub_;
std::string global_direction;
float global_distance;
bool marker_found_ = false;
bool direction = false;

// Funzioni per il controllo del movimento del robot
void rotate_rosbot(double ang_z)
{
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z = ang_z;
    cmd_vel_pub.publish(cmd_vel_msg);
}

void move_rosbot(double lin_x)
{
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = lin_x;
    cmd_vel_pub.publish(cmd_vel_msg);
}

// Callback del servizio
bool setVelocityControl(assignment2_exprob::SetVelocityControl::Request &req,
                        assignment2_exprob::SetVelocityControl::Response &res)
{
 
    if (req.control) {
      
        if ((global_direction == "left" || global_direction == "fleft") && global_distance < 1.4) {
            // Ruota a destra se la distanza è inferiore a 1.5
            rotate_rosbot(0.4);
            //ROS_INFO("ruota a sinistra.");
        } 
        else if ((global_direction == "right" || global_direction == "fright") && global_distance < 1.4) {
            // Ruota a sinistra se la distanza è inferiore a 1.5
            //ROS_INFO("ruota a destra.");
            rotate_rosbot(-0.4);
        } 
        else if (global_direction == "front" && global_distance < 1.4 ) {
            // Ferma il robot se la distanza è inferiore a 1.8
            rotate_rosbot(0.0);
            if (global_distance < 0.6 )
            {
                // Muovi il robot all'indietro per 2 secondi
                ros::Time start_time = ros::Time::now();
                while (ros::Time::now() - start_time < ros::Duration(3.0)) {
                    move_rosbot(-0.1);
                    ros::Duration(0.1).sleep();  // Dorme per 100ms tra i comandi
                }
            }
            else if ( global_distance > 0.6 )
            {
                // Se il marker non è stato trovato, ruota il robot a sinistra
               //ROS_INFO("indietro.");
               // Muovi il robot all'indietro per 2 secondi
                ros::Time start_time = ros::Time::now();
                while (ros::Time::now() - start_time < ros::Duration(3.0)) {
                    move_rosbot(0.1);
                    ros::Duration(0.1).sleep();  // Dorme per 100ms tra i comandi
               
                }
            }

        }
        else {
            // Muovi il robot in avanti
            rotate_rosbot(0.4);
        }
        res.success = true;
    }
    else {
        // Disabilita il controllo della velocità
        ROS_INFO("Velocity control disabled.");
        // Logica per fermare il robot
        move_rosbot(0.0);
        rotate_rosbot(0.0);
       
        res.success = true;
    }

    return true;
}

// Callback per la direzione e distanza del laser
void laserCallback(const assignment2_exprob::laser_direction::ConstPtr& msg)
{
    // Salva i valori del messaggio nelle variabili globali
    global_direction = msg->direction;
    global_distance = msg->distance;
}
 void foundCallback(const assignment2_exprob::Marker_id_pos msg) {
        if (msg.marker_id != -1) {
            marker_found_ = true;
            // Additional actions on marker found can go here
        }
    }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_control_server");
    ros::NodeHandle nh;

    // Publisher per il controllo della velocità
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Creazione del server di servizio
    ros::ServiceServer service = nh.advertiseService("/set_velocity_control", setVelocityControl);

    // Creazione del subscriber per la direzione del laser
    laser_dir_sub_ = nh.subscribe("/laser_direction", 1, laserCallback);
    found_sub_ = nh.subscribe("/robot4_xacro/marker_id_detected", 1, foundCallback);
    ROS_INFO("Velocity control service ready.");

    // Esegui il loop ros per il servizio
    ros::spin();

    return 0;
}

#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>
#include <visualization_msgs/Marker.h>
#include <rwsfi2016_msgs/GameQuery.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
using namespace std;
using namespace ros;
int n=0;

/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */


class MyPlayer: public rwsfi2016_libs::Player
{
  public:
        Publisher publ;
        visualization_msgs::Marker marker;
        ros::ServiceServer service;
        Subscriber subs;
        typedef pcl::PointXYZRGB PointT;
        pcl::PointCloud<PointT> last_pcl;

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name)
    {
        publ =node.advertise<visualization_msgs::Marker>( "/bocas", 0 );
        subs =node.subscribe("/object_point_cloud", 1, &MyPlayer::PclCallback,this);
        marker.header.frame_id = name;
        marker.header.stamp = ros::Time();
        marker.ns = name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.z = 0.4;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        service = node.advertiseService(name + "/game_query", &MyPlayer::serviceCallback,this);
    };

    void PclCallback(const sensor_msgs::PointCloud2& msg)
    {
        pcl::fromROSMsg(msg,last_pcl);
    }

    bool serviceCallback(rwsfi2016_msgs::GameQuery::Request &req, rwsfi2016_msgs::GameQuery::Response &res )
    {

        switch (last_pcl.points.size()) {
        case 3979:
            res.resposta="banana";
            break;
        case 1570:
            res.resposta="tomato";
            break;
        case 3468:
            res.resposta="onion";
            break;
        default:
            res.resposta="soda_can";
            break;
        }
        return true;
    }

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
        //Custom play behaviour. Now I will win the game

        double distance_to_arena = getDistanceToArena();

        //Behaviour follow the closest prey
        //move(msg.max_displacement, getAngleToPLayer(preys_team->players[0]));
        //move(msg.max_displacement, M_PI/30);
        double dist_to_preys_min = INFINITY;
        int chosen_pl_to_hunt=0;
        int chosen_pl_to_run_away_from=0;
        double min_admissible_dist_to_hunters = 10;
        double displacement = msg.max_displacement;
        bool escape = 0;
        for(int pl_prey=0;pl_prey<preys_team->players.size();pl_prey++)
        {
            escape = 0;
            double dist_to_prey = getDistanceToPlayer(preys_team->players[pl_prey]);
            if (dist_to_prey<dist_to_preys_min)
            {
                for(int pl_hunter=0;pl_hunter<hunters_team->players.size();pl_hunter++)
                {
                    double dist_to_hunter = getDistanceToPlayer(hunters_team->players[pl_hunter]);
                    if (dist_to_hunter < min_admissible_dist_to_hunters)
                    {
                        escape = 1;
                        chosen_pl_to_run_away_from =  pl_hunter;
                        break;
                    }
                }
                dist_to_preys_min = dist_to_prey;
                chosen_pl_to_hunt = pl_prey;
            }
        }
        if (distance_to_arena < 7)
        {
            if (escape == 0)
            {
                move(displacement,getAngleToPLayer(preys_team->players[chosen_pl_to_hunt]));
            }
            else
            {
                move(displacement,getAngleToPLayer(hunters_team->players[chosen_pl_to_run_away_from])+2*M_PI);
            }
        }
        else
        {
            string arena = "/map";
            move(displacement, getAngleToPLayer(arena) );
        }
    }
};


/**
 * @brief The main function. All you need to do here is enter your name and your pets name
 * @param argc number of command line arguments
 * @param argv values of command line arguments
 * @return result
 */
int main(int argc, char** argv)
{
  // ------------------------
  //Replace this with your name
  // ------------------------
  string my_name = "idomingues";
  string my_pet = "/cat";

  //initialize ROS stuff
  ros::init(argc, argv, my_name);
  //Creating an instance of class MyPlayer
  MyPlayer my_player(my_name, my_pet);

  //Infinite spinning (until ctrl-c)
  ros::spin();
}

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */
#include <ros/ros.h>
#include <rwsfi2016_libs/player.h>

/* _________________________________
   |                                 |
   |              CODE               |
   |_________________________________| */
using namespace std;
using namespace ros;


/**
 * @brief MyPlayer extends class Player, i.e., there are additional things I can do with MyPlayer and not with any Player, e.g., to order a movement.
 */
class MyPlayer: public rwsfi2016_libs::Player
{
  public: 

    /**
     * @brief Constructor, nothing to be done here
     * @param name player name
     * @param pet_name pet name
     */
    MyPlayer(string player_name, string pet_name="/dog"): Player(player_name, pet_name){};

    void play(const rwsfi2016_msgs::MakeAPlay& msg)
    {
      //Custom play behaviour. Now I will win the game

      //Behaviour follow the closest prey
      //move(msg.max_displacement, getAngleToPLayer(preys_team->players[0]));
      //move(msg.max_displacement, M_PI/30);
	double dist_min = INFINITY; int pl_min=0;
	for(int pl=0;pl<preys_team->players.size();pl++)
	{
		double dist = getDistanceToPlayer(preys_team->players[pl]);
		if (dist<dist_min)
		{
			dist_min = dist; 
			pl_min = pl;
		} 
	}
	move(msg.max_displacement,getAngleToPLayer(preys_team->players[pl_min]));
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

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

	double distance_to_arena = getDistanceToArena();

      //Behaviour follow the closest prey
      //move(msg.max_displacement, getAngleToPLayer(preys_team->players[0]));
      //move(msg.max_displacement, M_PI/30);
	double dist_to_preys_min = INFINITY; 
	int chosen_pl_to_hunt=0;
	int chosen_pl_to_run_away_from=0;
	double min_admissible_dist_to_hunters = 1; 
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
	if (distance_to_arena < 8)
	{
		if (escape == 0)
		{
			move(msg.max_displacement,getAngleToPLayer(preys_team->players[chosen_pl_to_hunt]));
		}
		else
		{
			move(msg.max_displacement,-getAngleToPLayer(hunters_team->players[chosen_pl_to_run_away_from]));
		}
	}
	else 
	{
		string arena = "/map";
		move(msg.max_displacement, getAngleToPLayer(arena) );
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

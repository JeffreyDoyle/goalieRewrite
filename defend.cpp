#include "ai/hl/stp/evaluation/defense.h"
#include "ai/hl/stp/evaluation/ball.h"
#include "ai/hl/stp/evaluation/ball_threat.h"
#include "ai/hl/util.h"
#include "geom/util.h"
#include "util/dprint.h"

#include <cassert>
#include "../action/defend.h"
#include "defend.h"

#include "../action/goalie.h"
#include "../action/repel.h"

using namespace AI::HL::STP::Tactic;
using namespace AI::HL::W;
using namespace Geom;
namespace Evaluation = AI::HL::STP::Evaluation;
namespace Action = AI::HL::STP::Action;

namespace {
	BoolParam tdefend_goalie(u8"Whether or not Terence Defense should take the place of normal goalie", u8"AI/HL/STP/Tactic/defend", false);
	BoolParam tdefend_defender1(u8"Whether or not Terence Defense should take the place of normal defender 1", u8"AI/HL/STP/Tactic/defend", false);
	BoolParam tdefend_defender2(u8"Whether or not Terence Defense should take the place of normal defender 2", u8"AI/HL/STP/Tactic/defend", false);
	BoolParam tdefend_defender3(u8"Whether or not Terence Defense should take the place of normal defender 3", u8"AI/HL/STP/Tactic/defend", false);

Point destinations[6] = evaluateDefence(World world);
Point DefenceDestinations = &destinations;

//Goalie Class
class Goalie final: public Tactic {
  public: 
  //public variables and functions
    explicit Goalie(World world, Point *defenderDestinations ) : Tactic(world)), defenceWaypoints(defenderDestinations)  {
      }
  
   private:
  //private variables and functions
    void execute(caller_t& ca);         //co-routines thing                    
    Glib::ustring description() const override {
      if (defenders == 0)
        return u8"lone goalie";
      else
        return u8"goalie and defenders"
      }
    
    };
  
  //class for the friendly defender logic
  class Defender final : public Tactic {
    public:
      explicit Defender(World world, Point *defenderDestinations, unsigned defenderNum) : Tactic(world), defenceWaypoints(defenderDestinations, defenderNum(defenderNum)) {
      }
    private:
      Point defenceWaypoints;
      unsigned defenderNum;
      Player select(const std::set<Player> &player) const override;
      bool active_baller; //which team has the ball (true means our ball)
      void execute(caller_t& ca) override; //co-routines thing
      Glib::ustring description() const override {
        return u8"extra defender";
      }
  };
  
  bool dangerous(World world, const Player &player) {

    constexpr double dangerDist = 0.3; //Set the danger radius to 30cm
    constexpr double dangerDistGoal = 3.5; //set the danger proximity to the gosl to be 1 meter

    if ( abs(world.ball().position().len() - world.field.friendly_goal().len()) < dangerDistGoal )
      return true; //the ball is dangerously close to our net
    
    for ( auto i : world.enemy_team ){
      unsigned enemyProximityToBall = abs( world.ball().position.len - i.position().len() );
      if (enemyProximityToBall < dangerDist)
        return true; //there are enemy robots very close to the ball
    }
    return false; //ball isn't close to friendly net or enemy robots
  }

  void Goalie::execute(caller_t& ca) {
   
      for (auto i : world.enemy_team()) {     //if the enemy is in our goal, touch them for a pentalty kick
		
			if(AI::HL::Util::point_in_friendly_defense(world.field() , i.position())) {
				player().avoid_distance(AI::Flags::AvoidDistance::SHORT);
				Action::goalie_move(ca, world, player(), i.position());
				return;
			}
		}

    Action::goalie_move(ca, world, player(), defenceWaypoints[0]);  //move the goalie to the defence destination
    //defenceWaypoints[0] is always the waypoint for the goalie (highest priority defence)
  }

  void Defender::execute(caller_t& ca) {
    Action::defender_move(ca, world, player), defenceWaypoints(defenderNum); //defenderNum goes from 1-3
  }
    
  }

  Tactic::Ptr AI::HL::STP::Tactic::goalie_dynamic(World world, std::vector<Point> defendDestinations){
    Tactic::Ptr p(New Goalie(world, defendDestinations));
  }
}

  Tactic::Ptr AI::HL::STP::Tactic::defender(World world, std::vector<Point> defendDestinations), unsigned defenderNum) {
    Tactic::Ptr p(New Goalie(world, defendDestinations));
  }
}
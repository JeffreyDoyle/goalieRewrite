#ifndef AI_HL_STP_TACTIC_DEFEND_H
#define AI_HL_STP_TACTIC_DEFEND_H

#include "ai/hl/stp/tactic/tactic.h"

namespace AI {                  //
    namespace HL {              //setting up the namespaces (so I don't have to wright AI::HL::... ect everytime')
        namespace STP {         //
            namespace Tactic {  //
                //is the specifier necissary here?
                
                //Controls goalie position
                //world - game state
                //defendDestinations - vector of optimal defence positions determined by the evaluate_defence function 
                Tactic::Ptr AI::HL::STP::Tactic::goalie_dynamic(World world, std::vector<Point> defendDestinations)

                //Controls defender position
                //world - game state
                //defendDestinations - vector of optimal defence positions determined by the evaluate_defence function
                Tactic::Ptr AI::HL::STP::Tactic::defender(World world, std::vector<point> defendDestinations)

                }
            } 
        }
    }
#endif

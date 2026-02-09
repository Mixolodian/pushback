#pragma once
#include "vex.h"

extern const int AUTO;
extern const int OPCONTROL;
extern int GamePhase;

/*----------------------------------------------------------------------------*/
/*                           SUBSYSTEMS NAMESPACE                             */
/*----------------------------------------------------------------------------*/

namespace subsystems {
  void toggleWings();
  void toggleDescore();
  void init();
}

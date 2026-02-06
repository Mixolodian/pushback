#pragma once
#include "vex.h"

extern const int AUTO;
extern const  int OPCONTROL;
extern int GamePhase;
extern bool hoodState;

/*----------------------------------------------------------------------------*/
/*                           SUBSYSTEMS NAMESPACE                             */
/*----------------------------------------------------------------------------*/

namespace subsystems {

   // Pneumatics Toggles
  void toggleHood();
  void toggleWings();
  void toggleDescore();

  // Intake handler (call every loop)
  void handleIntake();

  // Attach button events (for pneumatics)
  void init();
}

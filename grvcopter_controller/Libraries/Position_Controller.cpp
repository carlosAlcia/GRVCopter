#include "Position_Controller.h"

Position PositionControl::run(Position *target, Position *current){
    Position error = Position::error(target, current);
    pid_x.update_pid(error.x());
}
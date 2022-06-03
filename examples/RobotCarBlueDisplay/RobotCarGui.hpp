/*
 * RobotCarGui.hpp
 *
 * includes all files of BlueDisplay GUI for robot car.
 *
 *  Copyright (C) 2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef _ROBOT_CAR_GUI_HPP
#define _ROBOT_CAR_GUI_HPP

#define USE_BLUE_DISPLAY_GUI

#include "RobotCarGui.h"

#include "RobotCarCommonGui.hpp"
#include "RobotCarHomePage.hpp"
#include "RobotCarTestPage.hpp"
#include "BTSensorDrivePage.hpp"
#if defined(ENABLE_AUTONOMOUS_DRIVE)
#include "AutonomousDrive.hpp"
#include "AutonomousDrivePage.hpp"
#endif
#if defined(ENABLE_PATH_INFO_PAGE)
#include "PathInfoPage.hpp"
#endif

#endif // _ROBOT_CAR_GUI_HPP

/* Include the controller definition */
#include "footbot_static.h"


/****************************************/
/****************************************/

CFootBotNest::CFootBotNest() :
	m_pcLEDs(NULL){}

/****************************************/
/****************************************/

void CFootBotNest::Init(TConfigurationNode& t_node) {
	/* Get sensor/actuator handles */
	m_pcLEDs   = GetActuator<CCI_LEDsActuator                          >("leds");

	/* LED color */
	int clrid = 0;
	GetNodeAttributeOrDefault(t_node, "colorid", clrid, clrid);
	switch(clrid)
	{
	case 1: //blue
		m_pcLEDs->SetAllColors(CColor::BLUE);
		break;
	case 2: //green
		m_pcLEDs->SetAllColors(CColor::GREEN);
		break;
	case 3: //yellow
		m_pcLEDs->SetAllColors(CColor::YELLOW);
		break;
	case 4: //red
		m_pcLEDs->SetAllColors(CColor::RED);
		break;
	default:
		break;
	}
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotNest, "footbot_static_controller")

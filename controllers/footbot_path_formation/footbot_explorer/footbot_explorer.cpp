/* Include the controller definition */
#include "footbot_explorer.h"

/****************************************/
/****************************************/

CFootBotExplorer::CFootBotExplorer() :
	m_pcLEDs(NULL),
	m_pcCamera(NULL),
	m_pcRNG(NULL),
	m_unCounter(0),
	m_cCountRange(0, 100) {}


/****************************************/
/****************************************/

void CFootBotExplorer::Init(TConfigurationNode& t_node) {
	/* Get sensor/actuator handles */
	m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
	m_pcLEDs   = GetActuator<CCI_LEDsActuator                          >("leds");
	m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
	/*
	 * Create a random number generator.
	 * We use the 'argos' category so that creation, reset, seeding and
	 * cleanup are managed by ARGoS.
	 */
	m_pcRNG = CRandom::CreateRNG("argos");
	/* To make all the robots initially out of sync, choose the value of
	 * the counter at random */
	m_unCounter = m_pcRNG->Uniform(m_cCountRange);
	/* Switch the camera on */
	m_pcCamera->Enable();

	behav = new Behaviour(this, setLED, get_IR_sensors, chain_found, nearest, chainstat);
	LED_clr = 0;
}

/****************************************/
/****************************************/

void CFootBotExplorer::ControlStep() {
	Vec2D vec = behav->getVec();
	
	double Vf = vel_forward(vec,MAX_FORWARD);
	double Vr = vel_rotate(vec,MAX_ROTATE)/180.0*M_PI;
	double L = 0.12;//meter
	m_pcWheels->SetLinearVelocity((Vf-Vr*L)*100.0,(Vf+Vr*L)*100.0);
}

/****************************************/
/****************************************/

void CFootBotExplorer::Reset() {
	/*
	 * Reset the controller.
	 * This means bringing the controller to the same state as when Init()
	 * finished its execution.
	 * Since we created the RNG in the 'argos' category, we don't need to
	 * reset it.
	 * The only thing we need to do here is resetting the counter.
	 */
	m_unCounter = m_pcRNG->Uniform(m_cCountRange);
}

/****************************************/
/****************************************/
//conversion functions for colors
CColor ind2clr(int index)
{
	if(index == 0)
		return CColor::BLACK;
	else if(index == 1)
		return CColor::BLUE;
	else if(index == 2)
		return CColor::GREEN;
	else if(index == 3)
		return CColor::YELLOW;
	else if(index == 4)
		return CColor::RED;
	else
		return CColor::BLACK;
}
int clr2ind(CColor clr)
{
	if(clr == CColor::CYAN || clr == CColor::BLUE)
		return 1;
	else if(clr == CColor::GREEN)
		return 2;
	else if(clr == CColor::YELLOW)
		return 3;
	else if(clr == CColor::RED)
		return 4;
	else
		return 0;
}

void setLED(void* dt,int clrind){
	CFootBotExplorer *robot = (CFootBotExplorer*) dt;
	robot->LED_clr = clrind;
	robot->m_pcLEDs->SetAllColors(ind2clr(clrind));
}
void get_IR_sensors(void* dt, double* senses, int* n){
	CFootBotExplorer *robot = (CFootBotExplorer*) dt;
	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads = robot->m_pcProximity->GetReadings();

	int max;
	if(*n < tProxReads.size())
	{
		max = *n;
	}
	else
	{
		max = tProxReads.size();
		*n = tProxReads.size();
	}

	for(size_t i = 0; i < max; ++i) {
		senses[i] = (1.0 - tProxReads[i].Value)*0.1;
	}
}
bool chain_found(void* dt){
	CFootBotExplorer *robot = (CFootBotExplorer*) dt;

	/* Get led color of nearby robots */
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = robot->m_pcCamera->GetReadings();
	/*
	 * Check whether someone sent a 1, which means 'flash'
	 */
	bool flashed = false;
	for(size_t i = 0; ! flashed && i < sBlobs.BlobList.size(); ++i) {
		if(sBlobs.BlobList[i]->Color != CColor::RED)
			flashed = true;
	}

	return flashed;
}
void nearest(void* dt, double* dist2obj, double* ang2obj, int* objclr, int* dir, bool* at_nest, bool* prey_found){
	CFootBotExplorer *robot = (CFootBotExplorer*) dt;

	*dist2obj = -1;
	*dir = 0;
	*at_nest = false;
	*prey_found = false;


	double dist = 100000.0;
	double dist_second = 100000.0;
	CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* closest = NULL;
	CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* closest_second = NULL;

	/* Get led color of nearby robots */
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = robot->m_pcCamera->GetReadings();
	/*
	 * Check whether someone sent a 1, which means 'flash'
	 */
	for(size_t i = 0; i < sBlobs.BlobList.size(); ++i) {
		if(sBlobs.BlobList[i]->Color == CColor::RED)
		{
			*prey_found = true; // prey detected
		}
		else if(sBlobs.BlobList[i]->Distance < dist)
		{
			if(closest != NULL)
			{
				if(sBlobs.BlobList[i]->Color != closest->Color)
				{
					closest_second = closest;
					dist_second = dist;
				}
			}
			closest = sBlobs.BlobList[i];
			dist = sBlobs.BlobList[i]->Distance;
		}
		else if(sBlobs.BlobList[i]->Distance < dist_second)
		{
			if(closest != NULL)
			{
				if(sBlobs.BlobList[i]->Color != closest->Color)
				{
					closest_second = sBlobs.BlobList[i];
					dist_second = sBlobs.BlobList[i]->Distance;
				}
			}
		}
	}

	// convert cm to meters
	dist = (dist + 9)/100.0;
	dist_second = (dist_second + 9)/ 100.0;

	// calculate direction of the path
	if(closest != NULL && closest_second != NULL)
	{
		double ang1 = adjang(closest->Angle.GetValue()/M_PI*180.0);
		double ang2 = adjang(closest_second->Angle.GetValue()/M_PI*180.0);
		
		double diff = ang1 - ang2;
		
		int clrind1 = clr2ind(closest->Color);
		int clrind2 = clr2ind(closest_second->Color);

		if(-15 < diff && diff < 15)
		{
			*dir = 0;
		}
		else if((0 <= diff && diff < 180.0) || diff <= -180.0) // L: ang1, R: ang2
		{
			if((clrind1 - clrind2 + 3)%3 == 1)
				*dir = 1;
			else if((clrind2 - clrind1 + 3)%3 == 1)
				*dir = -1;
		}
		else // L: ang2, R: ang1
		{
			if((clrind2 - clrind1 + 3)%3 == 1)
				*dir = 1;
			else if((clrind1 - clrind2 + 3)%3 == 1)
				*dir = -1;
		}
	}

	// infor about the nearest
	if(closest != NULL)
	{
		*dist2obj = (closest->Distance + 9)/100.0;
		*ang2obj = adjang(closest->Angle.GetValue()/M_PI*180.0);
		*objclr = clr2ind(closest->Color);
	}
}
void chainstat(void* dt, double* dist_prev, double* ang_prev, double* ang_next, bool* at_tail, double* preydist){
	CFootBotExplorer *robot = (CFootBotExplorer*) dt;

	*dist_prev = -1;
	*at_tail = false;
	*preydist = -1;

	int clrind = robot->LED_clr;
	int clrind_prev = ((clrind - 1) - 1 + 3) % 3 + 1;
	int clrind_next = ((clrind - 1) + 1 + 3) % 3 + 1;

	double distance_prev = 100000.0;
	double distance_next = 100000.0;
	double rad_prev=0;
	double rad_next=0;
	CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* prev = NULL;
	CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* next = NULL;

	/* Get led color of nearby robots */
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = robot->m_pcCamera->GetReadings();
	/*
	 * Check whether someone sent a 1, which means 'flash'
	 */
	for(size_t i = 0; i < sBlobs.BlobList.size(); ++i) {
		CColor clr = sBlobs.BlobList[i]->Color;

		if(clr == CColor::RED)
		{
			if(*preydist == -1 || *preydist > sBlobs.BlobList[i]->Distance)
				*preydist = sBlobs.BlobList[i]->Distance; // prey detected
		}
		if(clr != CColor::RED && clr2ind(clr)==clrind_prev && sBlobs.BlobList[i]->Distance < distance_prev)
		{
			distance_prev = sBlobs.BlobList[i]->Distance;
			rad_prev = sBlobs.BlobList[i]->Angle.GetValue();
			prev = sBlobs.BlobList[i];
		}
		else if((clr == CColor::RED||clr2ind(clr)==clrind_next) && sBlobs.BlobList[i]->Distance < distance_next)
		{
			distance_next = sBlobs.BlobList[i]->Distance;
			rad_next = sBlobs.BlobList[i]->Angle.GetValue();
			next = sBlobs.BlobList[i];
		}

	}

	// convert cm to meters
	distance_prev = (distance_prev + 9)/100.0;
	distance_next = (distance_next + 9)/ 100.0;
	if(*preydist != -1)
		*preydist = (*preydist + 9)/100.0;


	if(prev != NULL)
	{
		*dist_prev = distance_prev;
		*ang_prev = rad_prev/M_PI*180.0;
	}

	// if next is beyond prev, ignore it
	if(
		prev == NULL ||
		(
			next != NULL &&
			(
				distance_next / distance_prev > 1.0 && 
				diffang(adjang(rad_prev/M_PI*180.0),adjang(rad_next/M_PI*180.0)) < 60.0
			)
		)
    )
	{
		next = NULL;
	}
	
	if(next != NULL)
	{
		*ang_next = rad_next/M_PI*180.0;
	}
	else
	{
		*at_tail = true;
	}

}

REGISTER_CONTROLLER(CFootBotExplorer, "footbot_explorer_controller")

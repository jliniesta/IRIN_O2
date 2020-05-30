/******************* TRABAJO OBLIGATORIO 2 IRIN ******************/
/*****************************************************************/

// GRUPO 1:

// JAVIER LÓPEZ INIESTA DÍAZ DEL CAMPO
// JORQUE QUIJORNA SANTOS
// JORGE ROMEO TERCIADO

/*****************************************************************/

#include "irifitnessfunction.h"
#include "collisionmanager.h"

/******************************************************************************/
/******************************************************************************/

CIriFitnessFunction::CIriFitnessFunction(const char* pch_name, 
                                                                 CSimulator* pc_simulator, 
                                                                 unsigned int un_collisions_allowed_per_epuck)
    :
    CFitnessFunction(pch_name, pc_simulator)
{

	/* Check number of robots */
	m_pcSimulator = pc_simulator;
	TEpuckVector* pvecEpucks=m_pcSimulator->GetEpucks();
	
	if ( pvecEpucks->size() == 0 )
	{
		printf("No Robot, so fitness function can not be computed.\n Exiting...\n");
		fflush(stdout);
		exit(0);
	}
	else if  (pvecEpucks->size()>1)
	{
		printf("More than 1 robot, and fitness is not prepared for it.\n Exiting...\n");
	}
    
	m_pcEpuck=(*pvecEpucks)[0];
    batteryFlag = 0.0;
	m_unNumberOfSteps = 0;
	m_fComputedFitness = 0.0;

}

/******************************************************************************/
/******************************************************************************/

CIriFitnessFunction::~CIriFitnessFunction(){
}
/******************************************************************************/
/******************************************************************************/

double CIriFitnessFunction::GetFitness()
{    

	/* If you need to check the collisions of the robot, here are the total number of 
	 * collisions done by the robot in the simulations */
	int coll = (CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());

	/* Get the fitness divided by the number of steps */
	
	/*
	TODAS LAS PRUEBAS
	*/
	double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps ) * (1 - ((double) (fmin(coll,10.0)/10.0)));
 	
	/*
	ANN: PRUEBA 5, 6, RANDOM_POS Y RANDOM_POS_OBJ
	CTRNN: PRUEBA 1, 4 Y RANDOM_POS_OBJ
	*/
	fit *= ( (double) (fmin(m_unBatteryCounter, 5.0)/ 5.0 ));

	/* If fitness less than 0, put it to 0 */
	if ( fit < 0.0 ) fit = 0.0;

	return fit;
}

/******************************************************************************/
/******************************************************************************/
void CIriFitnessFunction::SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval)
{
	/* See Evolutionary Robotics Book */
	/* This is the function to be implemented */
	/* f = V * ( 1 - sqrt(Delta(v)) ) * (1 - i)
	 * V relates to the maximum speed
	 * Delta(v) relates to the movement on the same direction
	 * i relates to the maximum sensor value
	 */

	/* Get actual SPEED of the left and right wheel */
	double leftSpeed = 0.0;
	double rightSpeed = 0.0;
	m_pcEpuck->GetWheelSpeed(&leftSpeed,&rightSpeed);
	leftSpeed = 0.5 + ( leftSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );
	rightSpeed = 0.5 + ( rightSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );

	/* Eval maximum speed partial fitness */
	double maxSpeedEval = (fabs(leftSpeed - 0.5) + fabs(rightSpeed - 0.5));

	/* Eval same direction partial fitness */
	double sameDirectionEval = 1 - sqrt(fabs(leftSpeed - rightSpeed));
	
	/* Eval SENSORS */

	/* Where the Max PROXIMITY sensor will be stored*/
	double maxProxSensorEval 		= 0.0;
	/* Where the Max LIGHT sensor will be stored*/
	double maxLightSensorEval 	= 0.0;
	/* Where the Max BLUE LIGHT sensor will be stored*/
	double maxBlueLightSensorEval 	= 0.0;
	/* Where the Max RED LIGHT sensor will be stored*/
	double maxRedLightSensorEval 	= 0.0;
	/* Where the Max CONTACT sensor will be stored*/
	double maxContactSensorEval = 0.0;

	/* Where the GROUND MEMORY will be stored */
	double* groundMemory;
	/* Where the GROUND will be stored */
	double* ground;
	/* whre the BATTERY will be sotored */
	double *battery;
	/* whre the BLUE BATTERY will be sotored */
	double *blueBattery;
	/* whre the RED BATTERY will be sotored */
	double *redBattery;

	double blueLightS0=0;
	double blueLightS7=0;
	double lightS0=0;
	double lightS7=0;
	double lightS3=0;
	double lightS4=0;
	double redLightS0=0;
	double redLightS7=0;

	/* Auxiluar variables */
	unsigned int unThisSensorsNumberOfInputs; 
	double* pfThisSensorInputs; 
	
	/* Go in all the sensors */
	TSensorVector vecSensors = m_pcEpuck->GetSensors();
	for (TSensorIterator i = vecSensors.begin(); i != vecSensors.end(); i++)
	{
		/* Check type of sensor */
		switch ( (*i)->GetType() )
		{
			/* If sensor is PROXIMITY */
			case SENSOR_PROXIMITY:
				/* Get the number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxProxSensorEval )
					{	
						/* Store maximum value */
						maxProxSensorEval = pfThisSensorInputs[j];
					}
				}
				break;
				

			/* If sensor is GROUND_MEMORY */
			case SENSOR_GROUND_MEMORY:
				/* Get the actual value */
				groundMemory = (*i)->GetComputedSensorReadings();
				break;
	
			/* If sensor is GROUND */
			case SENSOR_GROUND:
				/* Get actual values */
				ground = (*i)->GetComputedSensorReadings();
				break;	
			/* If sensor is LIGHT */
			case SENSOR_REAL_LIGHT:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxLightSensorEval )
					{	
						/* Store maximum value */
						maxLightSensorEval = pfThisSensorInputs[j];
					}
					if (j==0)
						lightS0 = pfThisSensorInputs[j];
					else if (j==7)
						lightS7 = pfThisSensorInputs[j];
					else if (j==3)
						lightS3 = pfThisSensorInputs[j];
					else if (j==4)
						lightS4 = pfThisSensorInputs[j];
				}
				break;
			case SENSOR_REAL_BLUE_LIGHT:
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					if ( pfThisSensorInputs[j] > maxBlueLightSensorEval )
					{	
						maxBlueLightSensorEval = pfThisSensorInputs[j];
					}
					if (j==0)
						blueLightS0 = pfThisSensorInputs[j];
					else if (j==7)
						blueLightS7 = pfThisSensorInputs[j];
				}
				break;
			
			case SENSOR_REAL_RED_LIGHT:
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					if ( pfThisSensorInputs[j] > maxRedLightSensorEval )
					{	
						maxRedLightSensorEval = pfThisSensorInputs[j];
					}
					if (j==0)
						redLightS0 = pfThisSensorInputs[j];
					else if (j==7)
						redLightS7 = pfThisSensorInputs[j];
				}
				break;

			/* If sensor is BATTERY */
			case SENSOR_BATTERY:
        		 battery = (*i)->GetComputedSensorReadings();
				 break;
			
			case SENSOR_BLUE_BATTERY:
				blueBattery = (*i)->GetComputedSensorReadings();
				break;
			
			case SENSOR_RED_BATTERY:
				redBattery = (*i)->GetComputedSensorReadings();
				break;
			
			/* If sensor is CONTACT */
			case SENSOR_CONTACT:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxContactSensorEval )
					{
						/* Store maximum value */
						maxContactSensorEval = pfThisSensorInputs[j];
					}
				}
				break;
		}
	}

/* FROM HERE YOU NEED TO CREATE YOU FITNESS */	

	maxProxSensorEval = 1 - maxProxSensorEval;

	/*
	PRUEBA 1 ANN*/
	/*

    double coef1 = 0.25;
    double coef2 = 0.25;
    double coef3 = 0.50;

    double battery_threshold = 0.4;


	if(battery[0]<battery_threshold){
		batteryFlag = 1.0;
	}
	else if ((battery[0]<0.9) && (batteryFlag == 1.0)){
		batteryFlag = 1.0;
	}else{
		batteryFlag = 0.0;
	}


	double fitness = coef1 * maxSpeedEval * sameDirectionEval * maxProxSensorEval * (leftSpeed * rightSpeed);
	
	
	fitness+=coef2*battery[0];

 	if ( batteryFlag != 0.0 ){
		fitness += (coef3 * ( lightS0 + lightS7)/2);
	}
  	else{
    	fitness += (coef3 * ( blueLightS0 + blueLightS7)/2);

	}
*/

/*****************************************************************/

	/*
	PRUEBA 2 ANN*/
	/*

    double coef1 = 0.25;
    double coef2 = 0.75;
    double carga = 0.0;

    double battery_threshold = 0.5;


	if(battery[0]>battery_threshold){
		carga = 1.0;
	}
	else{
		carga = 0.0;
	}

	double fitness= coef1 * maxSpeedEval * sameDirectionEval;

 	if ( carga != 0.0 )
    	fitness += coef2 * (( blueLightS0 + blueLightS7)/2);
  	else
    	fitness += coef2 * (( lightS0 + lightS7)/2);
*/

/*****************************************************************/

	/*
	PRUEBA 3 ANN y PRUEBA 4 ANN*/
	/*

    double coef1 = 0.35;
    double coef2 = 0.65;

    double battery_threshold = 0.4;


	if(battery[0]<battery_threshold){
		batteryFlag = 1.0;
	}
	else if ((battery[0]<0.9) && (batteryFlag == 1.0)){
		batteryFlag = 1.0;
	}else{
		batteryFlag = 0.0;
	}

	double fitness= coef1 * maxSpeedEval * sameDirectionEval;

 	if ( batteryFlag != 0.0 ){
		fitness += coef2 * (( lightS0 + lightS7)/2);
	}
  	else{
    	fitness += coef2 * (( blueLightS0 + blueLightS7)/2);

	}
	*/

/*****************************************************************/

	/*
	PRUEBA 5 ANN*/
	/*
        double battery_threshold = 0.4;
	double coef1 = 0.25;
  	double coef2 = 0.75;

	if(battery[0]<battery_threshold)
	{
		batteryFlag = 1.0;
	}
	else if ((battery[0]<0.9) && (batteryFlag == 1.0))
	{
		batteryFlag = 1.0;
	}else
	{
		batteryFlag = 0.0;
	}
		
	double fitness = coef1 * maxSpeedEval * sameDirectionEval;

 	if ( batteryFlag != 0.0 ){
		fitness += coef2*((lightS0 + lightS7)/2);
	}
  	else
	  {
          if ( maxBlueLightSensorEval != 0.0 ){
    	     fitness += coef2*((blueLightS0 + blueLightS7)/2);
          }
          else {
             fitness += (coef2 * ( redLightS0 + redLightS7)/2);
          }
	}
	*/

/*****************************************************************/

	/*
	ANN:PRUEBA 6, RANDOM_POS Y RANDOM_POS_OBJ
	CTRNN: PRUEBA 4 Y RANDOM_POS_OBJ
	*/

    double battery_threshold = 0.4;
	double coef1 = 0.25;
  	double coef2 = 0.75;

	if(battery[0]<battery_threshold)
	{
		batteryFlag = 1.0;
	}
	else if ((battery[0]<0.9) && (batteryFlag == 1.0))
	{
		batteryFlag = 1.0;
	}else
	{
		batteryFlag = 0.0;
	}
	
	double fitness = coef1 * maxSpeedEval * sameDirectionEval * maxProxSensorEval;

 	if ( batteryFlag != 0.0 ){
		fitness += coef2*((lightS0 + lightS7)/2);
		if ((m_unBatteryFlag == 0))
    	{
      		m_unBatteryFlag = 1;
    	}
	}
  	else
	  {
          if ( maxBlueLightSensorEval != 0.0 ){
    	     fitness += coef2*((blueLightS0 + blueLightS7)/2);
          }
          else {
             fitness += (coef2 * ( redLightS0 + redLightS7)/2);
          }
		if (m_unBatteryFlag == 1)
    	{
      		m_unBatteryFlag = 0;
			m_unBatteryCounter++;

    	}
	}

/*****************************************************************/

	/*
	PRUEBA 1 CTRNN*/
	/*
    double battery_threshold = 0.4;
	double coef1 = 0.25;
  	double coef2 = 0.75;

	if(battery[0]<battery_threshold)
	{
		batteryFlag = 1.0;
	}
	else if ((battery[0]<0.9) && (batteryFlag == 1.0))
	{
		batteryFlag = 1.0;
	}else
	{
		batteryFlag = 0.0;
	}
		
	double fitness = coef1 * maxSpeedEval * sameDirectionEval * maxProxSensorEval;

 	if ( batteryFlag != 0.0 ){
		fitness += coef2*((lightS0 + lightS7)/2);
		if ((m_unBatteryFlag == 0))
    	{
      		m_unBatteryFlag = 1;
    	}
	}
  	else
	{
          if ( maxBlueLightSensorEval != 0.0 ){
    	     fitness += coef2*((blueLightS0 + blueLightS7)/2);
          }
          else {
             fitness += (coef2 * ( redLightS0 + redLightS7)/2);
          }
          if (m_unBatteryFlag == 1)
    	  {
      	     m_unBatteryFlag = 0;
	    	 m_unBatteryCounter++;
    	  }
	}
*/

/*****************************************************************/

	/*
	CTRNN: PRUEBA 2 y PRUEBA 3
	*/

/*
    double battery_threshold = 0.4;
	double coef1 = 0.25;
  	double coef2 = 0.75;

	if(battery[0]<battery_threshold)
	{
		batteryFlag = 1.0;
	}
	else if ((battery[0]<0.9) && (batteryFlag == 1.0))
	{
		batteryFlag = 1.0;
	}else
	{
		batteryFlag = 0.0;
	}
		
	double fitness = coef1 * maxSpeedEval * sameDirectionEval * maxProxSensorEval;

 	if ( batteryFlag != 0.0 ){
		fitness += coef2*((lightS0 + lightS7)/2);
		
                if ((m_unBatteryFlag == 0))
    	        {
      				m_unBatteryFlag = 1;
    	        }
	}
  	else
	{
    	fitness += coef2*((blueLightS0 + blueLightS7)/2);
          
	        if (m_unBatteryFlag == 1)
    	    {  		
                m_unBatteryFlag = 0;
				m_unBatteryCounter++;
    	    }
	}
*/
		
/* TO HERE YOU NEED TO CREATE YOU FITNESS */	

	m_unNumberOfSteps++;
	m_fComputedFitness += fitness;
}

/******************************************************************************/
/******************************************************************************/

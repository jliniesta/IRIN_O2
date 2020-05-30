/******************* TRABAJO OBLIGATORIO 2 IRIN ******************/
/*****************************************************************/

// GRUPO 1:

// JAVIER LÓPEZ INIESTA DÍAZ DEL CAMPO
// JORQUE QUIJORNA SANTOS
// JORGE ROMEO TERCIADO

/*****************************************************************/

#ifndef IRIFITNESSFUNCTION_H_
#define IRIFITNESSFUNCTION_H_

/******************************************************************************/
/******************************************************************************/

#include <math.h>
#include <vector>
#include <list>
#include "general.h"
using namespace std;

class CIriFitnessFunction;

#include "fitnessfunction.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

class CIriFitnessFunction : public CFitnessFunction
{
public:
    CIriFitnessFunction(const char* pch_name, CSimulator* pc_simulator,
                                    unsigned int un_collisions_allowed_per_epuck);
		~CIriFitnessFunction();
    virtual double GetFitness();
		virtual void SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval);

protected:
		unsigned int 	m_unNumberOfSteps;
		double 			m_fComputedFitness;
		double 			batteryFlag;
		unsigned int 	m_unBatteryFlag;
		unsigned int 	m_unBatteryCounter;

		CEpuck* m_pcEpuck;

};

/******************************************************************************/
/******************************************************************************/

#endif

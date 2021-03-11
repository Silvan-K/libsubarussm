#include "ECUPort.hh"
#include <iostream>
#include <assert.h>

using namespace SSM;

class ResultHandler {

public:

  ResultHandler(int max_call) : m_callcount(0), m_maxcall(max_call) {}

  int handle(const SSM::Observables& observables,
	     const SSM::Values& results)
  {
    for(int i(0); i<results.size(); i++)
      std::cout << results[i] << " " 
		<< observables[i]->unit() << std::endl;
    
    m_callcount +=1;

    if(m_callcount==m_maxcall)
      return 1;
    
    return 0;
  };

private: 
  
  int m_maxcall;
  int m_callcount;
};

int main(int argc, char** argv)
{
  ECUPort ECU("/dev/ttyUSB0");

  BatteryVoltage           battery_voltage;
  EngineSpeed              engine_speed;
  ManifoldRelativePressure manifold_pressure;
  ExhaustGasTemperature    exhaust_gas_temp;
  IntakeAirTemperature     intake_temp;
  AirFuelLeanCorrection    af_lean_corr;
  EngineLoad               en_load;
  ThrottlePedal            thr;
  KnockingCorrection       kn_corr;
  NeutralPositionSwitch    nt_sw;
  KnockSignal1             knock_1;
  KnockSignal2             knock_2;
  
  Observables observables { &battery_voltage,
			    &engine_speed,
			    &manifold_pressure,
			    &exhaust_gas_temp,
			    &intake_temp,
			    &af_lean_corr,
			    &en_load,
			    &thr,
			    &kn_corr,
			    &nt_sw,
			    &knock_1,
			    &knock_2 };

  ECU.singleRead_(BatteryVoltage(),
		  EngineSpeed(),
		  ManifoldRelativePressure(),
		  ExhaustGasTemperature(),
		  IntakeAirTemperature(),
		  AirFuelLeanCorrection(),
		  EngineLoad(),
		  ThrottlePedal(),
		  KnockingCorrection(),
		  NeutralPositionSwitch(),
		  KnockSignal1(),
		  KnockSignal2());

  Values values = ECU.singleRead(observables);
  assert(observables.size() == values.size());

  std::cout << "Single read results:" << std::endl;
  for(int i(0); i<values.size(); i++)
    std::cout << ECU.isAvailable(*observables[i])
  	      << " | " << values[i] << " "
   	      << observables[i]->unit() << std::endl;

  ResultHandler handler(100);
  auto callback = std::bind(&ResultHandler::handle, 
  			    &handler, 
  			    std::placeholders::_1,
   			    std::placeholders::_2);

  std::cout << "\nStarting continuous read:" << std::endl;
  ECU.continuousRead(observables, callback);
}

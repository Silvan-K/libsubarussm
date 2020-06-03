#include "ECUPort.hh"
#include <iostream>
#include <assert.h>

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
  SSM::ECUPort ECU("/dev/ttyUSB0");

  SSM::BatteryVoltage           battery_voltage;
  SSM::EngineSpeed              engine_speed;
  SSM::ManifoldRelativePressure manifold_pressure;
  SSM::ExhaustGasTemperature    exhaust_gas_temp;

  SSM::Observables observables { &battery_voltage,
				 &engine_speed,
				 &manifold_pressure,
				 &exhaust_gas_temp };

  SSM::Values values = ECU.singleRead(observables);
  assert(observables.size() == values.size());

  std::cout << "Single read results:" << std::endl;
  for(int i(0); i<values.size(); i++)
    std::cout << values[i] << " " << observables[i]->unit() << std::endl;

  ResultHandler handler(100);
  auto callback = std::bind(&ResultHandler::handle, 
			    &handler, 
			    std::placeholders::_1,
			    std::placeholders::_2);

  std::cout << "\nStarting continuous read:" << std::endl;
  ECU.continuousRead(observables, callback);
}

#include "TwiddleTuner.h"

TwiddleTuner::TwiddleTuner() {}

TwiddleTuner::~TwiddleTuner() {}

// Initialization
void TwiddleTuner::Init(double tolerance, int timesteps, std::vector<double**>& parameters) {
	
	// save inputs
	//tol = tolerance;
	//tim = timesteps;
	//par = parameters;
	
	// intialize trackers
	//round_count = 0;
	//current_score = 0.0;
	//first_round = true;
}

// Simulator restart helper
void TwiddleTuner::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

// Timestep update
void TwiddleTuner::Update(double score) {
	
	// Add a round count
	round_count++;
}


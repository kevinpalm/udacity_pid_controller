#ifndef TWIDDLETUNER_H
#define TWIDDLETUNER_H

#include <uWS/uWS.h>

class TwiddleTuner {
public:

  // Input variables
  double tol;
  int tim;
  std::vector<double*> par;
  
  // Tracker variables
  int round_count;
  double current_score;
  double past_score;
  double best_score;
  bool first_round;

  // Initilaizer
  void Init(double tolerance, int timesteps, std::vector<double**>& parameters);

  // Simulator restart helper
  void Restart(uWS::WebSocket<uWS::SERVER> websocket);
  
  // Timestep updater
  void Update(double score);
  
};

#endif /* TWIDDLETUNER_H */

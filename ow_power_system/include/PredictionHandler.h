// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef __PREDICTION_HANDLER_H__
#define __PREDICTION_HANDLER_H__

#include <vector>
#include <map>

#include "Messages/MessageBus.h"
#include "Messages/ProgEventMessage.h"
#include "Messages/ScalarMessage.h"
#include "ModelBasedAsyncPrognoserBuilder.h"

// The PredictionHandler class subscribes to the battery EoD event message and
// prints each event as it is received.
class PredictionHandler : public IMessageProcessor
{
public:
  PredictionHandler(double& rul, double& soc, double& temp, MessageBus& bus,
                    const std::string& src, int node_num);
  ~PredictionHandler();
  void processMessage(const std::shared_ptr<Message>& message) override;
private:
  double findMedian(std::vector<double> samples);

  // References used by other classes to process EoD predictions.
  double& rul_ref;
  double& soc_ref;
  double& temp_ref;
  MessageBus& bus;

  std::string identifier;
  int node_number;
};

#endif
